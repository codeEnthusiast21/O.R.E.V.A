"""YDLIDAR X2 wire-protocol decoding and stream framing.

The X2 emits scan data as a sequence of variable-length packets over a
115200-baud serial link.  This module is deliberately independent of any
serial library so the protocol logic can be tested from byte buffers.

Wire format (YDLIDAR X2 Development Manual, DOC#:01.13.000101)::

    PH(2) | CT(1) | LSN(1) | FSA(2) | LSA(2) | CS(2) | S1(2) ... Sn(2)

* ``PH`` is the little-endian value ``0x55AA``, transmitted as ``AA 55``.
* ``CT`` bit 0 marks the first packet of a revolution.  For a start packet,
  bits 7..1 encode the previous frame's scan frequency as ``CT >> 1 / 10``.
* ``LSN`` is the number of 2-byte samples in the packet.
* ``FSA`` and ``LSA`` are encoded angles: ``(raw >> 1) / 64`` degrees.
* ``CS`` is the XOR of the header word, FSA, every 16-bit sample, the
  ``(LSN << 8 | CT)`` word, and LSA.

The parser intentionally does not filter measurements.  Zero-distance samples
(no return) and out-of-range samples are retained so that a later validation
layer can make explicit, auditable decisions.  All timestamps are host-side
receive times; the X2 has no hardware timestamp in this packet format.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

HEADER_BYTES = b"\xAA\x55"
HEADER_WORD = 0x55AA
PACKET_HEADER_SIZE = 10
SAMPLE_SIZE = 2
MAX_SAMPLES_PER_PACKET = 255
MAX_PACKET_BYTES = PACKET_HEADER_SIZE + SAMPLE_SIZE * MAX_SAMPLES_PER_PACKET
ANGLE_UNITS_PER_DEGREE = 64.0
DISTANCE_RAW_UNITS_PER_METRE = 4000.0
X2_BAUD_RATE = 115200

# Triangulation correction constants from the X2 development manual.
_ANGLE_CORRECTION_A = 21.8
_ANGLE_CORRECTION_B = 155.3


class ParseErrorReason(str, Enum):
    """Machine-readable reasons a byte sequence could not become a packet."""

    BAD_HEADER = "bad_header"
    TRUNCATED_FRAME = "truncated_frame"
    FRAME_LENGTH_MISMATCH = "frame_length_mismatch"
    INVALID_LSN = "invalid_lsn"
    CHECKSUM_MISMATCH = "checksum_mismatch"
    INVALID_ANGLE = "invalid_angle"


class PacketDecodeError(ValueError):
    """Raised when a complete frame cannot be decoded as an X2 packet."""

    def __init__(self, reason: ParseErrorReason, detail: str) -> None:
        super().__init__(detail)
        self.reason = reason
        self.detail = detail


@dataclass(frozen=True, slots=True)
class LidarPoint:
    """A single X2 sample in the LiDAR's own polar frame.

    ``distance_raw`` is retained because it is the exact value carried by the
    protocol.  ``distance_m`` is ``distance_raw / 4000``.

    A zero distance means the sensor reported no valid return.  Such points
    are intentionally retained; filtering belongs to a later validation layer.
    """

    angle_deg: float
    distance_m: float
    distance_raw: int


@dataclass(frozen=True, slots=True)
class LidarPacket:
    """A decoded X2 point-cloud packet."""

    ct: int
    lsn: int
    fsa_deg: float
    lsa_deg: float
    is_start: bool
    scan_frequency_hz: float | None
    points: tuple[LidarPoint, ...]
    received_unix_ns: int
    received_mono_ns: int


@dataclass(frozen=True, slots=True)
class PacketError:
    """A framing or decoding error encountered while consuming a stream."""

    reason: ParseErrorReason
    stream_offset: int
    detail: str


@dataclass(frozen=True, slots=True)
class ParseBatch:
    """Result of feeding one byte chunk to :class:`PacketFramer`."""

    packets: tuple[LidarPacket, ...]
    errors: tuple[PacketError, ...]
    bytes_discarded: int


def compute_checksum(frame: bytes) -> int:
    """Compute the X2 checksum for a complete frame.

    ``frame`` must contain exactly the 10-byte header plus ``2 * LSN`` sample
    bytes.  :func:`decode_packet` performs the same length check before calling
    this function.
    """

    if len(frame) < PACKET_HEADER_SIZE:
        raise ValueError("frame is shorter than the X2 packet header")

    lsn = frame[3]
    expected_length = PACKET_HEADER_SIZE + SAMPLE_SIZE * lsn
    if len(frame) != expected_length:
        raise ValueError(
            f"LSN {lsn} requires {expected_length} bytes, got {len(frame)}"
        )

    checksum = HEADER_WORD
    checksum ^= int.from_bytes(frame[4:6], "little")  # FSA
    for index in range(lsn):
        offset = PACKET_HEADER_SIZE + index * SAMPLE_SIZE
        checksum ^= int.from_bytes(frame[offset : offset + SAMPLE_SIZE], "little")
    checksum ^= (lsn << 8) | frame[2]  # LSN and CT
    checksum ^= int.from_bytes(frame[6:8], "little")  # LSA
    return checksum & 0xFFFF


def _encoded_angle_to_degrees(raw: int) -> float:
    """Convert a raw FSA/LSA value to degrees.

    Bit 0 of the raw angle is the protocol's fixed check bit; the remaining
    bits carry the angle in 1/64-degree units.
    """

    angle = (raw >> 1) / ANGLE_UNITS_PER_DEGREE
    if angle > 360.0:
        raise PacketDecodeError(
            ParseErrorReason.INVALID_ANGLE,
            f"encoded angle {raw:#06x} maps to {angle:.3f} degrees",
        )
    if angle == 360.0:
        angle = 0.0
    return angle


def _normalise_angle(angle_deg: float) -> float:
    """Normalise an angle to the half-open interval [0, 360)."""

    return angle_deg % 360.0


def _angle_correction_deg(distance_mm: float) -> float:
    """Return the X2 triangulation angle correction in degrees."""

    if distance_mm <= 0.0:
        return 0.0
    tangent = _ANGLE_CORRECTION_A * (
        (_ANGLE_CORRECTION_B - distance_mm) / (_ANGLE_CORRECTION_B * distance_mm)
    )
    return math.degrees(math.atan(tangent))


def decode_packet(
    frame: bytes,
    *,
    received_unix_ns: int,
    received_mono_ns: int,
) -> LidarPacket:
    """Decode exactly one complete X2 packet.

    Args:
        frame: Complete packet bytes, including header, checksum, and samples.
        received_unix_ns: Host wall-clock receive time in nanoseconds.
        received_mono_ns: Host monotonic receive time in nanoseconds.

    Raises:
        PacketDecodeError: If the frame is not a well-formed X2 packet.

    The two timestamps describe when the host received the chunk containing
    the packet, not when the sensor acquired each sample.  The X2 protocol
    does not provide per-sample or per-packet sensor timestamps.
    """

    if len(frame) < PACKET_HEADER_SIZE:
        raise PacketDecodeError(
            ParseErrorReason.TRUNCATED_FRAME,
            f"frame has {len(frame)} bytes, need at least {PACKET_HEADER_SIZE}",
        )
    if frame[0:2] != HEADER_BYTES:
        raise PacketDecodeError(
            ParseErrorReason.BAD_HEADER,
            f"expected header {HEADER_BYTES.hex()}, got {frame[0:2].hex()}",
        )

    ct = frame[2]
    lsn = frame[3]
    if lsn == 0:
        raise PacketDecodeError(ParseErrorReason.INVALID_LSN, "LSN must be at least 1")

    expected_length = PACKET_HEADER_SIZE + SAMPLE_SIZE * lsn
    if len(frame) != expected_length:
        raise PacketDecodeError(
            ParseErrorReason.FRAME_LENGTH_MISMATCH,
            f"LSN {lsn} requires {expected_length} bytes, got {len(frame)}",
        )

    expected_checksum = compute_checksum(frame)
    actual_checksum = int.from_bytes(frame[8:10], "little")
    if expected_checksum != actual_checksum:
        raise PacketDecodeError(
            ParseErrorReason.CHECKSUM_MISMATCH,
            f"computed checksum {expected_checksum:#06x}, frame contains {actual_checksum:#06x}",
        )

    fsa_raw = int.from_bytes(frame[4:6], "little")
    lsa_raw = int.from_bytes(frame[6:8], "little")
    fsa_deg = _encoded_angle_to_degrees(fsa_raw)
    lsa_deg = _encoded_angle_to_degrees(lsa_raw)

    angle_span = _normalise_angle(lsa_deg - fsa_deg)
    points: list[LidarPoint] = []
    for index in range(lsn):
        offset = PACKET_HEADER_SIZE + index * SAMPLE_SIZE
        distance_raw = int.from_bytes(frame[offset : offset + SAMPLE_SIZE], "little")
        distance_mm = distance_raw / 4.0
        distance_m = distance_raw / DISTANCE_RAW_UNITS_PER_METRE

        if lsn == 1:
            angle_deg = fsa_deg
        else:
            angle_deg = fsa_deg + angle_span * index / (lsn - 1)

        angle_deg = _normalise_angle(angle_deg + _angle_correction_deg(distance_mm))
        points.append(
            LidarPoint(
                angle_deg=angle_deg,
                distance_m=distance_m,
                distance_raw=distance_raw,
            )
        )

    is_start = bool(ct & 0x01)
    reported_frequency = (ct >> 1) / 10.0 if is_start else None
    if reported_frequency == 0.0:
        reported_frequency = None

    return LidarPacket(
        ct=ct,
        lsn=lsn,
        fsa_deg=fsa_deg,
        lsa_deg=lsa_deg,
        is_start=is_start,
        scan_frequency_hz=reported_frequency,
        points=tuple(points),
        received_unix_ns=received_unix_ns,
        received_mono_ns=received_mono_ns,
    )


class PacketFramer:
    """Incrementally split an X2 byte stream into decoded packets.

    The framer handles arbitrary read-chunk boundaries, leading garbage,
    power-on ``A5 5A`` messages, false header sequences, checksum failures,
    and truncated data at end of stream.  After any rejected candidate it
    advances by one byte and searches for the next header, so a valid packet
    following corrupt data can still be recovered.

    All packets returned by one :meth:`feed` call share that call's timestamps.
    Callers that need finer timing should feed smaller chunks.  The class is
    stateful and not thread-safe; use one instance per input stream.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._buffer_start_offset = 0
        self._discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        """Number of bytes currently waiting for a complete packet."""

        return len(self._buffer)

    @property
    def discarded_bytes(self) -> int:
        """Cumulative bytes discarded while resynchronising."""

        return self._discarded_bytes

    def _consume(self, count: int) -> None:
        if count <= 0:
            return
        del self._buffer[:count]
        self._buffer_start_offset += count

    def _discard(self, count: int) -> None:
        if count <= 0:
            return
        self._discarded_bytes += count
        self._consume(count)

    def feed(
        self,
        chunk: bytes,
        *,
        received_unix_ns: int,
        received_mono_ns: int,
    ) -> ParseBatch:
        """Consume a chunk of bytes and return any packets completed by it."""

        packets: list[LidarPacket] = []
        errors: list[PacketError] = []
        discarded_before = self._discarded_bytes

        if chunk:
            self._buffer.extend(chunk)

        while self._buffer:
            header_index = self._buffer.find(HEADER_BYTES)
            if header_index < 0:
                # Preserve a trailing 0xAA so a header split across chunks is
                # not lost.  Everything else is unusable garbage.
                keep = 1 if self._buffer[-1] == HEADER_BYTES[0] else 0
                self._discard(len(self._buffer) - keep)
                break

            if header_index > 0:
                self._discard(header_index)
                continue

            if len(self._buffer) < PACKET_HEADER_SIZE:
                break

            lsn = self._buffer[3]
            if lsn == 0:
                errors.append(
                    PacketError(
                        reason=ParseErrorReason.INVALID_LSN,
                        stream_offset=self._buffer_start_offset,
                        detail="candidate header has LSN 0",
                    )
                )
                self._discard(1)
                continue

            frame_length = PACKET_HEADER_SIZE + SAMPLE_SIZE * lsn
            if len(self._buffer) < frame_length:
                break

            frame = bytes(self._buffer[:frame_length])
            try:
                packet = decode_packet(
                    frame,
                    received_unix_ns=received_unix_ns,
                    received_mono_ns=received_mono_ns,
                )
            except PacketDecodeError as error:
                errors.append(
                    PacketError(
                        reason=error.reason,
                        stream_offset=self._buffer_start_offset,
                        detail=error.detail,
                    )
                )
                # The candidate may have been a false header inside another
                # frame.  Advancing one byte preserves the ability to find a
                # real header that starts immediately afterwards.
                self._discard(1)
                continue

            self._consume(frame_length)
            packets.append(packet)

        return ParseBatch(
            packets=tuple(packets),
            errors=tuple(errors),
            bytes_discarded=self._discarded_bytes - discarded_before,
        )

    def finish(self) -> ParseBatch:
        """Flush end-of-stream state and report any truncated trailing data.

        A partial packet cannot be decoded.  It is reported as a
        ``TRUNCATED_FRAME`` error and discarded.  The framer can still be fed
        again afterwards, although that is not required for normal EOF use.
        """

        discarded_before = self._discarded_bytes
        if not self._buffer:
            return ParseBatch(packets=(), errors=(), bytes_discarded=0)

        error = PacketError(
            reason=ParseErrorReason.TRUNCATED_FRAME,
            stream_offset=self._buffer_start_offset,
            detail=f"stream ended with {len(self._buffer)} undecoded bytes",
        )
        self._discard(len(self._buffer))
        return ParseBatch(
            packets=(),
            errors=(error,),
            bytes_discarded=self._discarded_bytes - discarded_before,
        )
