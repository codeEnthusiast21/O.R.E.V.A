"""Conversion of assembled X2 scans into ROS 2 ``sensor_msgs/LaserScan`` geometry.

The OREVA core never imports ROS.  This module performs the geometry and
metadata work needed by a ROS 2 publisher and returns a plain dataclass, so
the conversion can be unit-tested without ``rclpy`` and reused by other
consumers.

Design choices
--------------

* **Native angular frame.**  ``angle_min`` is 0 and angles advance toward
  ``2 * pi``.  The X2's zero reference is arbitrary with respect to the rover;
  mounting rotation belongs in the URDF/TF tree, not in this conversion.
* **Uniform bins.**  ``sensor_msgs/LaserScan`` requires a constant
  ``angle_increment``, while X2 samples are not perfectly uniform.  Points are
  therefore assigned to a fixed grid of ``bin_count`` half-open bins
  ``[i * increment, (i + 1) * increment)``.  A point exactly on a boundary goes
  to the higher bin.  The default 720 bins give a 0.5-degree grid.
* **Conservative aggregation.**  When several points fall in one bin, the
  minimum valid range is kept.  This biases toward nearby obstacles, which is
  the safer choice for navigation, and the behaviour is explicit rather than
  an undocumented average.
* **No-return semantics.**  Empty bins, zero-distance samples, and samples
  outside ``[range_min_m, range_max_m]`` become ``inf``.  In ROS this means
  "no return" and is distinct from ``NaN`` ("invalid measurement").  The
  counts are reported so callers can audit what was discarded.
* **Timing.**  The X2 provides no per-ray or per-packet sensor timestamp.
  ``time_increment_s`` is therefore 0.0, ``scan_time_s`` is the host receive
  duration, and the scan stamp is the scan's start time.

Range limits are required arguments rather than hard-coded constants: the
decoder retains every raw sample, and the caller must choose limits that match
the datasheet and the rover's safety envelope.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .scan import LidarScan

DEFAULT_BIN_COUNT = 720
DEFAULT_FRAME_ID = "laser_link"


@dataclass(frozen=True, slots=True)
class LaserScanFrame:
    """A ROS 2 ``sensor_msgs/LaserScan`` payload without a ROS dependency.

    ``ranges_m`` has one entry per angular bin; its length is the configured
    ``bin_count``.  ``inf`` marks a bin with no valid return.
    ``angle_min_rad``, ``angle_increment_rad``, and the tuple index define each
    bin's angle: ``angle_min_rad + index * angle_increment_rad``.
    """

    stamp_unix_ns: int
    frame_id: str
    sequence: int
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float
    time_increment_s: float
    scan_time_s: float
    range_min_m: float
    range_max_m: float
    ranges_m: tuple[float, ...]
    points_used: int
    points_no_return: int
    points_out_of_range: int


def to_laser_scan_frame(
    scan: LidarScan,
    *,
    range_min_m: float,
    range_max_m: float,
    bin_count: int = DEFAULT_BIN_COUNT,
    frame_id: str = DEFAULT_FRAME_ID,
) -> LaserScanFrame:
    """Project a scan onto a uniform angular grid for ``sensor_msgs/LaserScan``.

    Args:
        scan: An assembled scan.  Points are expected to come from
            :func:`oreva_lidar.decode_packet` or :class:`ScanAssembler`.
        range_min_m: Smallest usable range in metres.  Samples closer than
            this are treated as no return.
        range_max_m: Largest usable range in metres.  Samples farther than
            this are treated as no return.
        bin_count: Number of uniform angular bins.  Must be at least 2.
        frame_id: TF frame for the scan, conventionally ``laser_link``.

    Raises:
        TypeError: If ``bin_count`` is not an integer or ``frame_id`` is not a
            string.
        ValueError: If a numeric limit or the frame name is invalid.

    Returns:
        A :class:`LaserScanFrame` whose ``ranges_m`` has exactly ``bin_count``
        entries.
    """

    if isinstance(bin_count, bool) or not isinstance(bin_count, int):
        raise TypeError("bin_count must be an integer")
    if bin_count < 2:
        raise ValueError("bin_count must be at least 2")
    if not math.isfinite(range_min_m) or range_min_m < 0.0:
        raise ValueError("range_min_m must be a finite non-negative value")
    if not math.isfinite(range_max_m) or range_max_m <= range_min_m:
        raise ValueError("range_max_m must be finite and greater than range_min_m")
    if not isinstance(frame_id, str):
        raise TypeError("frame_id must be a string")
    if not frame_id.strip():
        raise ValueError("frame_id must be a non-empty string")

    angle_increment_rad = math.tau / bin_count
    degrees_per_bin = 360.0 / bin_count
    ranges = [math.inf] * bin_count
    points_used = 0
    points_no_return = 0
    points_out_of_range = 0

    for point in scan.points:
        if point.distance_raw <= 0:
            points_no_return += 1
            continue
        if point.distance_m < range_min_m or point.distance_m > range_max_m:
            points_out_of_range += 1
            continue

        # Use degrees for binning so exact degree boundaries (for example
        # 0.5 degrees on a 720-bin grid) do not drift through the radians
        # conversion.  The modulo makes the 360-to-0 wrap explicit.
        angle_deg = point.angle_deg % 360.0
        index = int(angle_deg / degrees_per_bin)
        if index >= bin_count:
            # Floating-point guard for an angle that rounds to 360 after the
            # modulo; it belongs with the highest valid bin.
            index = bin_count - 1

        ranges[index] = min(ranges[index], point.distance_m)
        points_used += 1

    return LaserScanFrame(
        stamp_unix_ns=scan.start_unix_ns,
        frame_id=frame_id,
        sequence=scan.sequence,
        angle_min_rad=0.0,
        angle_max_rad=angle_increment_rad * (bin_count - 1),
        angle_increment_rad=angle_increment_rad,
        time_increment_s=0.0,
        scan_time_s=scan.duration_s,
        range_min_m=range_min_m,
        range_max_m=range_max_m,
        ranges_m=tuple(ranges),
        points_used=points_used,
        points_no_return=points_no_return,
        points_out_of_range=points_out_of_range,
    )
