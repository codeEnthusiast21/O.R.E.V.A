"""Scan assembly for YDLIDAR X2 packets.

A packet covers a small angular interval; a usable 360-degree scan is built by
collecting packets until a revolution boundary is observed.  The X2 emits a
dedicated start-of-revolution packet (``CT`` bit 0), which is the primary
boundary.  Some captures omit start packets, so a conservative angle-wrap
fallback is also provided.

The assembler does not judge whether a scan is usable for mapping.  It records
how the scan ended and whether that boundary was natural.  Distance filtering,
minimum point counts, and health decisions belong to a later validation layer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .protocol import LidarPacket, LidarPoint

DEFAULT_MAX_POINTS_PER_SCAN = 4096
DEFAULT_MAX_SCAN_DURATION_S = 2.0
DEFAULT_WRAP_PREV_MIN_DEG = 300.0
DEFAULT_WRAP_NEXT_MAX_DEG = 60.0


class ScanEndReason(str, Enum):
    """Why the assembler emitted a scan."""

    START_PACKET = "start_packet"
    ANGLE_WRAP = "angle_wrap"
    MAX_POINTS = "max_points"
    MAX_DURATION = "max_duration"
    CLOCK_ANOMALY = "clock_anomaly"
    EOF = "eof"


@dataclass(frozen=True, slots=True)
class LidarScan:
    """A sequence of points collected between two revolution boundaries.

    ``complete`` means the scan ended at a natural revolution boundary
    (start packet or angle wrap).  It does not guarantee full angular
    coverage; inspect :attr:`angular_coverage_deg` and point count for that.
    """

    sequence: int
    points: tuple[LidarPoint, ...]
    packet_count: int
    start_unix_ns: int
    end_unix_ns: int
    start_mono_ns: int
    end_mono_ns: int
    end_reason: ScanEndReason
    scan_frequency_hz: float | None

    @property
    def complete(self) -> bool:
        """Whether the scan ended at a natural revolution boundary."""

        return self.end_reason in (
            ScanEndReason.START_PACKET,
            ScanEndReason.ANGLE_WRAP,
        )

    @property
    def duration_s(self) -> float:
        """Host-receive duration derived from the monotonic clock."""

        return (self.end_mono_ns - self.start_mono_ns) / 1_000_000_000.0

    @property
    def angular_coverage_deg(self) -> float:
        """Angular span covered by the points, ignoring density.

        The value is ``360 - largest_gap`` between sorted point angles.  A
        single point, or points at one angle, has zero coverage.  This is a
        geometric description only; it is not a usability threshold.
        """

        if len(self.points) < 2:
            return 0.0

        angles = sorted(point.angle_deg % 360.0 for point in self.points)
        gaps = [angles[index + 1] - angles[index] for index in range(len(angles) - 1)]
        gaps.append(angles[0] + 360.0 - angles[-1])
        return max(0.0, min(360.0, 360.0 - max(gaps)))


class ScanAssembler:
    """Collect decoded packets into :class:`LidarScan` objects.

    Args:
        max_points_per_scan: Safety bound on retained points.  The bound is
            checked before adding a packet, so a single packet can exceed it
            when a caller configures a limit smaller than one packet.
        max_scan_duration_s: Safety bound on one scan's monotonic duration.
        wrap_prev_min_deg: Minimum previous-packet LSA for the angle-wrap
            fallback.  The fallback only triggers when a scan crosses from the
            high 300s back to the low degrees of the circle.
        wrap_next_max_deg: Maximum current-packet FSA for the angle-wrap
            fallback.

    The assembler is stateful and not thread-safe.  Feed packets from one
    input stream in order, and call :meth:`finish` at end of stream.
    """

    def __init__(
        self,
        *,
        max_points_per_scan: int = DEFAULT_MAX_POINTS_PER_SCAN,
        max_scan_duration_s: float = DEFAULT_MAX_SCAN_DURATION_S,
        wrap_prev_min_deg: float = DEFAULT_WRAP_PREV_MIN_DEG,
        wrap_next_max_deg: float = DEFAULT_WRAP_NEXT_MAX_DEG,
    ) -> None:
        if max_points_per_scan < 1:
            raise ValueError("max_points_per_scan must be at least 1")
        if not math.isfinite(max_scan_duration_s) or max_scan_duration_s <= 0.0:
            raise ValueError("max_scan_duration_s must be a positive finite value")
        if not 0.0 <= wrap_prev_min_deg < 360.0:
            raise ValueError("wrap_prev_min_deg must be in [0, 360)")
        if not 0.0 <= wrap_next_max_deg < 360.0:
            raise ValueError("wrap_next_max_deg must be in [0, 360)")

        self._max_points_per_scan = max_points_per_scan
        self._max_scan_duration_ns = int(max_scan_duration_s * 1_000_000_000)
        self._wrap_prev_min_deg = wrap_prev_min_deg
        self._wrap_next_max_deg = wrap_next_max_deg

        self._points: list[LidarPoint] = []
        self._packet_count = 0
        self._has_pending_scan = False
        self._started_with_start_packet = False
        self._start_unix_ns = 0
        self._end_unix_ns = 0
        self._start_mono_ns = 0
        self._end_mono_ns = 0
        self._last_mono_ns = 0
        self._last_lsa_deg = 0.0
        self._sequence = 0

    @property
    def buffered_point_count(self) -> int:
        """Points accumulated in the scan currently being built."""

        return len(self._points)

    def add(self, packet: LidarPacket) -> LidarScan | None:
        """Add one packet, returning a scan if this packet closes one.

        Clock regression closes the current scan as ``CLOCK_ANOMALY`` rather
        than producing a scan with a negative duration.  A scan that has
        already reached the point limit or duration limit is closed before
        the packet is added, so that packet starts the next scan.
        """

        if len(packet.points) != packet.lsn:
            raise ValueError(
                f"packet LSN {packet.lsn} does not match {len(packet.points)} points"
            )
        if not packet.points:
            raise ValueError("cannot assemble a packet with no points")

        if not self._has_pending_scan:
            self._begin(packet)
            return None

        if packet.received_mono_ns < self._last_mono_ns:
            scan = self._close(ScanEndReason.CLOCK_ANOMALY)
            self._begin(packet)
            return scan

        if packet.is_start:
            scan = self._close(
                ScanEndReason.START_PACKET,
                scan_frequency_hz=packet.scan_frequency_hz,
            )
            self._begin(packet)
            return scan

        # A scan that began with a start packet already has a reliable
        # boundary for its end, so the heuristic is only used for scans that
        # began mid-revolution.
        if (
            not self._started_with_start_packet
            and self._last_lsa_deg >= self._wrap_prev_min_deg
            and packet.fsa_deg <= self._wrap_next_max_deg
        ):
            scan = self._close(ScanEndReason.ANGLE_WRAP)
            self._begin(packet)
            return scan

        if self._points and (
            len(self._points) + len(packet.points) > self._max_points_per_scan
        ):
            scan = self._close(ScanEndReason.MAX_POINTS)
            self._begin(packet)
            return scan

        if (
            packet.received_mono_ns - self._start_mono_ns
            >= self._max_scan_duration_ns
        ):
            scan = self._close(ScanEndReason.MAX_DURATION)
            self._begin(packet)
            return scan

        self._points.extend(packet.points)
        self._packet_count += 1
        self._end_unix_ns = packet.received_unix_ns
        self._end_mono_ns = packet.received_mono_ns
        self._last_mono_ns = packet.received_mono_ns
        self._last_lsa_deg = packet.lsa_deg
        return None

    def finish(self) -> LidarScan | None:
        """Emit the pending scan as incomplete, or return ``None`` if empty."""

        if not self._has_pending_scan:
            return None
        return self._close(ScanEndReason.EOF)

    def _begin(self, packet: LidarPacket) -> None:
        self._points = list(packet.points)
        self._packet_count = 1
        self._has_pending_scan = True
        self._started_with_start_packet = packet.is_start
        self._start_unix_ns = packet.received_unix_ns
        self._end_unix_ns = packet.received_unix_ns
        self._start_mono_ns = packet.received_mono_ns
        self._end_mono_ns = packet.received_mono_ns
        self._last_mono_ns = packet.received_mono_ns
        self._last_lsa_deg = packet.lsa_deg

    def _close(
        self,
        reason: ScanEndReason,
        *,
        scan_frequency_hz: float | None = None,
    ) -> LidarScan:
        self._sequence += 1
        scan = LidarScan(
            sequence=self._sequence,
            points=tuple(self._points),
            packet_count=self._packet_count,
            start_unix_ns=self._start_unix_ns,
            end_unix_ns=self._end_unix_ns,
            start_mono_ns=self._start_mono_ns,
            end_mono_ns=self._end_mono_ns,
            end_reason=reason,
            scan_frequency_hz=scan_frequency_hz,
        )
        self._points = []
        self._packet_count = 0
        self._has_pending_scan = False
        self._started_with_start_packet = False
        return scan
