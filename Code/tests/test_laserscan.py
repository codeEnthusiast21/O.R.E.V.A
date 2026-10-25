from __future__ import annotations

import math
import unittest

from oreva_lidar import (
    DEFAULT_BIN_COUNT,
    DEFAULT_FRAME_ID,
    LaserScanFrame,
    LidarScan,
    ScanEndReason,
    to_laser_scan_frame,
)

from .fixtures import point

RANGE_MIN_M = 0.15
RANGE_MAX_M = 8.0


def make_scan(
    points: list,
    *,
    sequence: int = 1,
    start_unix_ns: int = 1_000,
    duration_s: float = 0.1,
) -> LidarScan:
    start_mono_ns = 5_000_000_000
    duration_ns = int(duration_s * 1_000_000_000)
    return LidarScan(
        sequence=sequence,
        points=tuple(points),
        packet_count=1,
        start_unix_ns=start_unix_ns,
        end_unix_ns=start_unix_ns + duration_ns,
        start_mono_ns=start_mono_ns,
        end_mono_ns=start_mono_ns + duration_ns,
        end_reason=ScanEndReason.START_PACKET,
        scan_frequency_hz=None,
    )


def convert(scan: LidarScan, **overrides: object) -> LaserScanFrame:
    arguments: dict[str, object] = {
        "range_min_m": RANGE_MIN_M,
        "range_max_m": RANGE_MAX_M,
    }
    arguments.update(overrides)
    return to_laser_scan_frame(scan, **arguments)  # type: ignore[arg-type]


def finite_indexes(frame: LaserScanFrame) -> list[int]:
    return [index for index, value in enumerate(frame.ranges_m) if math.isfinite(value)]


class GridMetadataTests(unittest.TestCase):
    def test_default_grid(self) -> None:
        frame = convert(make_scan([point(0.0, 2.0)]))

        self.assertEqual(len(frame.ranges_m), DEFAULT_BIN_COUNT)
        self.assertEqual(frame.frame_id, DEFAULT_FRAME_ID)
        self.assertEqual(frame.angle_min_rad, 0.0)
        self.assertAlmostEqual(frame.angle_increment_rad, math.tau / DEFAULT_BIN_COUNT)
        self.assertAlmostEqual(
            frame.angle_max_rad,
            (math.tau / DEFAULT_BIN_COUNT) * (DEFAULT_BIN_COUNT - 1),
        )
        self.assertEqual(frame.time_increment_s, 0.0)
        self.assertEqual(frame.range_min_m, RANGE_MIN_M)
        self.assertEqual(frame.range_max_m, RANGE_MAX_M)

    def test_metadata_comes_from_the_scan(self) -> None:
        frame = convert(
            make_scan(
                [point(0.0, 1.0)],
                sequence=7,
                start_unix_ns=123_456,
                duration_s=0.125,
            )
        )

        self.assertEqual(frame.sequence, 7)
        self.assertEqual(frame.stamp_unix_ns, 123_456)
        self.assertAlmostEqual(frame.scan_time_s, 0.125)

    def test_custom_grid_and_frame(self) -> None:
        scan = make_scan(
            [point(0.0, 1.0), point(90.0, 1.0), point(180.0, 1.0), point(270.0, 1.0)]
        )
        frame = convert(scan, bin_count=4, frame_id="base_scan")

        self.assertEqual(len(frame.ranges_m), 4)
        self.assertEqual(frame.frame_id, "base_scan")
        self.assertAlmostEqual(frame.angle_increment_rad, math.tau / 4.0)
        self.assertAlmostEqual(frame.angle_max_rad, math.tau * 3.0 / 4.0)
        self.assertEqual(finite_indexes(frame), [0, 1, 2, 3])


class BinningTests(unittest.TestCase):
    def test_points_map_to_expected_bins(self) -> None:
        scan = make_scan(
            [
                point(0.0, 1.0),
                point(90.0, 1.0),
                point(180.0, 1.0),
                point(270.0, 1.0),
                point(359.9, 1.0),
            ]
        )
        frame = convert(scan)

        self.assertEqual(finite_indexes(frame), [0, 180, 360, 540, 719])
        self.assertEqual(frame.points_used, 5)

    def test_bin_boundaries_are_half_open(self) -> None:
        # With 720 bins each bin spans 0.5 degrees.
        frame = convert(make_scan([point(0.499, 1.0), point(0.5, 2.0)]))

        self.assertEqual(frame.ranges_m[0], 1.0)
        self.assertEqual(frame.ranges_m[1], 2.0)

    def test_angles_wrap_at_360(self) -> None:
        frame = convert(make_scan([point(359.999, 1.0), point(360.0, 2.0)]))

        self.assertEqual(frame.ranges_m[719], 1.0)
        self.assertEqual(frame.ranges_m[0], 2.0)
        self.assertEqual(frame.points_used, 2)

    def test_duplicate_bin_keeps_the_minimum_range(self) -> None:
        frame = convert(make_scan([point(10.0, 2.0), point(10.0, 1.0)]))

        self.assertEqual(finite_indexes(frame), [20])
        self.assertEqual(frame.ranges_m[20], 1.0)
        self.assertEqual(frame.points_used, 2)

    def test_out_of_range_point_does_not_lower_a_valid_bin(self) -> None:
        frame = convert(make_scan([point(10.0, 2.0), point(10.0, 8.5)]))

        self.assertEqual(frame.ranges_m[20], 2.0)
        self.assertEqual(frame.points_used, 1)
        self.assertEqual(frame.points_out_of_range, 1)

    def test_empty_bins_are_infinite(self) -> None:
        frame = convert(make_scan([point(0.0, 1.0)]))

        self.assertEqual(frame.ranges_m[0], 1.0)
        self.assertEqual(sum(1 for value in frame.ranges_m if value == math.inf), 719)
        self.assertEqual(frame.points_used, 1)


class RangeHandlingTests(unittest.TestCase):
    def test_zero_distance_is_a_no_return(self) -> None:
        frame = convert(make_scan([point(0.0, 0.0)]))

        self.assertTrue(all(value == math.inf for value in frame.ranges_m))
        self.assertEqual(frame.points_used, 0)
        self.assertEqual(frame.points_no_return, 1)
        self.assertEqual(frame.points_out_of_range, 0)

    def test_ranges_outside_limits_become_no_return(self) -> None:
        frame = convert(make_scan([point(0.0, 0.1), point(10.0, 8.5)]))

        self.assertTrue(all(value == math.inf for value in frame.ranges_m))
        self.assertEqual(frame.points_used, 0)
        self.assertEqual(frame.points_out_of_range, 2)

    def test_range_limits_are_inclusive(self) -> None:
        frame = convert(make_scan([point(0.0, RANGE_MIN_M), point(10.0, RANGE_MAX_M)]))

        self.assertEqual(frame.ranges_m[0], RANGE_MIN_M)
        self.assertEqual(frame.ranges_m[20], RANGE_MAX_M)
        self.assertEqual(frame.points_used, 2)
        self.assertEqual(frame.points_out_of_range, 0)

    def test_counts_are_reported_separately(self) -> None:
        scan = make_scan(
            [
                point(0.0, 0.0),  # no return
                point(10.0, 0.1),  # below range
                point(20.0, 8.5),  # above range
                point(30.0, 1.0),  # valid
            ]
        )
        frame = convert(scan)

        self.assertEqual(frame.points_used, 1)
        self.assertEqual(frame.points_no_return, 1)
        self.assertEqual(frame.points_out_of_range, 2)
        self.assertEqual(finite_indexes(frame), [60])

    def test_partial_scan_converts_without_filling_gaps(self) -> None:
        frame = convert(make_scan([point(100.0, 1.0), point(110.0, 1.0)]))

        self.assertEqual(finite_indexes(frame), [200, 220])
        self.assertEqual(frame.points_used, 2)


class ConfigurationValidationTests(unittest.TestCase):
    def test_rejects_invalid_bin_count(self) -> None:
        scan = make_scan([point(0.0, 1.0)])
        for bin_count in (0, 1, -5):
            with self.subTest(bin_count=bin_count), self.assertRaises(ValueError):
                convert(scan, bin_count=bin_count)
        for bin_count in (1.5, True, "720"):
            with self.subTest(bin_count=bin_count), self.assertRaises(TypeError):
                convert(scan, bin_count=bin_count)

    def test_rejects_invalid_ranges(self) -> None:
        scan = make_scan([point(0.0, 1.0)])
        invalid_pairs = (
            (-0.1, 8.0),
            (math.nan, 8.0),
            (math.inf, 8.0),
            (0.15, 0.15),
            (0.15, 0.1),
            (0.15, math.nan),
            (0.15, math.inf),
        )
        for range_min_m, range_max_m in invalid_pairs:
            with (
                self.subTest(range_min_m=range_min_m, range_max_m=range_max_m),
                self.assertRaises(ValueError),
            ):
                convert(scan, range_min_m=range_min_m, range_max_m=range_max_m)

    def test_rejects_invalid_frame_id(self) -> None:
        scan = make_scan([point(0.0, 1.0)])
        for frame_id in ("", "   "):
            with self.subTest(frame_id=frame_id), self.assertRaises(ValueError):
                convert(scan, frame_id=frame_id)
        with self.subTest(frame_id=None), self.assertRaises(TypeError):
            convert(scan, frame_id=None)


if __name__ == "__main__":
    unittest.main()
