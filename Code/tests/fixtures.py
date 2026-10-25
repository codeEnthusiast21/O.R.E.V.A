"""Deterministic fixtures for the OREVA LiDAR tests.

No fixture in this file is a hardware capture.  The packets are constructed
from the documented X2 wire format for simulation and test purposes, except
for the explicit manual vectors which are byte-for-byte examples published in
the YDLIDAR X2 development manual.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from oreva_lidar import LidarPacket, LidarPoint

# Official no-intensity example from the YDLIDAR X2 development manual:
# start packet, LSN=1, FSA=LSA=0xAE53, CS=0x54AB, sample=0.
MANUAL_START_PACKET = bytes.fromhex("AA 55 01 01 53 AE 53 AE AB 54 00 00")

# Power-on message emitted before scan data, per the X2 development manual.
POWER_ON_MESSAGE = bytes.fromhex("A5 5A 05 00 00 40 81")

MANUAL_ANGLE_FSA_RAW = 0x6FE5
MANUAL_ANGLE_LSA_RAW = 0x79BD
MANUAL_ANGLE_LSN = 40
MANUAL_ANGLE_FIRST_DISTANCE_MM = 1000.0
MANUAL_ANGLE_LAST_DISTANCE_MM = 8000.0


def encode_angle(angle_deg: float) -> int:
    """Encode degrees as the X2's 1/64-degree value with check bit set."""

    if not math.isfinite(angle_deg) or not 0.0 <= angle_deg <= 360.0:
        raise ValueError(f"angle must be in [0, 360], got {angle_deg!r}")
    if angle_deg == 360.0:
        angle_deg = 0.0
    quantized = round(angle_deg * 64.0)
    return ((quantized << 1) | 0x01) & 0xFFFF


def encode_distance(distance_m: float) -> int:
    """Encode metres as the X2's unsigned 16-bit quarter-millimetre sample."""

    if not math.isfinite(distance_m) or distance_m < 0.0:
        raise ValueError(f"distance must be finite and non-negative, got {distance_m!r}")
    raw = round(distance_m * 4000.0)
    if raw > 0xFFFF:
        raise ValueError(f"distance {distance_m!r} m does not fit in a 16-bit sample")
    return raw


def _fixture_checksum(ct: int, fsa_raw: int, lsa_raw: int, samples: Sequence[int]) -> int:
    """Independent implementation of the documented X2 checksum formula."""

    checksum = 0x55AA
    checksum ^= fsa_raw
    for sample in samples:
        checksum ^= sample
    checksum ^= (len(samples) << 8) | ct
    checksum ^= lsa_raw
    return checksum & 0xFFFF


def build_packet(
    *,
    ct: int,
    fsa_raw: int,
    lsa_raw: int,
    samples: Sequence[int],
) -> bytes:
    """Build an X2 packet from already-encoded fields."""

    if not 0 <= ct <= 0xFF:
        raise ValueError("ct must fit in one byte")
    if not 0 <= fsa_raw <= 0xFFFF or not 0 <= lsa_raw <= 0xFFFF:
        raise ValueError("encoded angles must fit in 16 bits")
    if len(samples) > 0xFF:
        raise ValueError("X2 LSN cannot exceed 255")
    for sample in samples:
        if not 0 <= sample <= 0xFFFF:
            raise ValueError("sample must fit in 16 bits")

    checksum = _fixture_checksum(ct, fsa_raw, lsa_raw, samples)
    frame = bytearray((0xAA, 0x55, ct, len(samples)))
    frame.extend(fsa_raw.to_bytes(2, "little"))
    frame.extend(lsa_raw.to_bytes(2, "little"))
    frame.extend(checksum.to_bytes(2, "little"))
    for sample in samples:
        frame.extend(sample.to_bytes(2, "little"))
    return bytes(frame)


def data_packet(
    *,
    start_deg: float,
    end_deg: float,
    distances_m: Sequence[float] | None = None,
    sample_count: int | None = None,
    ct: int = 0x00,
) -> bytes:
    """Build a normal point-cloud packet spanning ``start_deg`` to ``end_deg``."""

    if distances_m is None:
        if sample_count is None:
            raise ValueError("distances_m or sample_count is required")
        if sample_count < 1:
            raise ValueError("sample_count must be at least 1")
        distances = [0.0] * sample_count
    else:
        distances = list(distances_m)
        if sample_count is not None and sample_count != len(distances):
            raise ValueError("sample_count does not match distances_m")

    samples = [encode_distance(distance) for distance in distances]
    return build_packet(
        ct=ct,
        fsa_raw=encode_angle(start_deg),
        lsa_raw=encode_angle(end_deg),
        samples=samples,
    )


def start_packet(
    *,
    angle_deg: float = 0.0,
    frequency_hz: float | None = None,
) -> bytes:
    """Build an X2 start-of-revolution packet."""

    if frequency_hz is None:
        ct = 0x01
    else:
        if not math.isfinite(frequency_hz) or frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be positive")
        encoded = round(frequency_hz * 10.0)
        if not 1 <= encoded <= 0x7F:
            raise ValueError("frequency_hz is outside the X2 CT encoding range")
        ct = (encoded << 1) | 0x01
    encoded_angle = encode_angle(angle_deg)
    return build_packet(ct=ct, fsa_raw=encoded_angle, lsa_raw=encoded_angle, samples=[0])


def manual_angle_packet() -> bytes:
    """Packet used by the manual's interpolation and correction example."""

    samples = (
        [encode_distance(MANUAL_ANGLE_FIRST_DISTANCE_MM / 1000.0)]
        + [0] * (MANUAL_ANGLE_LSN - 2)
        + [encode_distance(MANUAL_ANGLE_LAST_DISTANCE_MM / 1000.0)]
    )
    return build_packet(
        ct=0x00,
        fsa_raw=MANUAL_ANGLE_FSA_RAW,
        lsa_raw=MANUAL_ANGLE_LSA_RAW,
        samples=samples,
    )


def point(angle_deg: float, distance_m: float = 1.0) -> LidarPoint:
    """Create a deterministic scan point for assembler tests."""

    distance_raw = encode_distance(distance_m)
    return LidarPoint(
        angle_deg=angle_deg,
        distance_m=distance_raw / 4000.0,
        distance_raw=distance_raw,
    )


def make_packet(
    *,
    fsa_deg: float,
    lsa_deg: float,
    points: Sequence[LidarPoint] | None = None,
    is_start: bool = False,
    scan_frequency_hz: float | None = None,
    received_unix_ns: int = 0,
    received_mono_ns: int = 0,
) -> LidarPacket:
    """Construct a decoded packet directly, bypassing the wire layer."""

    if points is None:
        points = [point(fsa_deg)]
    if not points:
        raise ValueError("packet must contain at least one point")
    if scan_frequency_hz is not None and not is_start:
        raise ValueError("scan_frequency_hz is only valid for a start packet")

    ct = 0x01 if is_start else 0x00
    return LidarPacket(
        ct=ct,
        lsn=len(points),
        fsa_deg=fsa_deg,
        lsa_deg=lsa_deg,
        is_start=is_start,
        scan_frequency_hz=scan_frequency_hz,
        points=tuple(points),
        received_unix_ns=received_unix_ns,
        received_mono_ns=received_mono_ns,
    )


def make_data_packet(
    *,
    angles_deg: Sequence[float],
    received_unix_ns: int = 0,
    received_mono_ns: int = 0,
) -> LidarPacket:
    """Construct a decoded non-start packet from a sequence of point angles."""

    if not angles_deg:
        raise ValueError("angles_deg must not be empty")
    return make_packet(
        fsa_deg=angles_deg[0],
        lsa_deg=angles_deg[-1],
        points=[point(angle) for angle in angles_deg],
        received_unix_ns=received_unix_ns,
        received_mono_ns=received_mono_ns,
    )


def make_start_packet(
    *,
    angle_deg: float = 0.0,
    scan_frequency_hz: float | None = None,
    received_unix_ns: int = 0,
    received_mono_ns: int = 0,
) -> LidarPacket:
    """Construct a decoded start-of-revolution packet."""

    return make_packet(
        fsa_deg=angle_deg,
        lsa_deg=angle_deg,
        points=[point(angle_deg)],
        is_start=True,
        scan_frequency_hz=scan_frequency_hz,
        received_unix_ns=received_unix_ns,
        received_mono_ns=received_mono_ns,
    )
