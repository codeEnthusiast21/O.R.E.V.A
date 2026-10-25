from __future__ import annotations

import math
import unittest
from dataclasses import replace

from oreva_lidar import LidarScan, ScanAssembler, ScanEndReason

from .fixtures import make_data_packet, make_start_packet, point


class LidarScanGeometryTests(unittest.TestCase):
    def _scan(
        self,
        points: tuple,
        *,
        end_reason: ScanEndReason = ScanEndReason.START_PACKET,
        start_mono_ns: int = 0,
        end_mono_ns: int = 1_000_000_000,
    ) -> LidarScan:
        return LidarScan(
            sequence=1,
            points=points,
            packet_count=1,
            start_unix_ns=10,
            end_unix_ns=20,
            start_mono_ns=start_mono_ns,
            end_mono_ns=end_mono_ns,
            end_reason=end_reason,
            scan_frequency_hz=None,
        )

    def test_complete_only_for_natural_boundaries(self) -> None:
        for reason in (
            ScanEndReason.START_PACKET,
            ScanEndReason.ANGLE_WRAP,
        ):
            with self.subTest(reason=reason):
                self.assertTrue(self._scan((point(0.0),), end_reason=reason).complete)

        for reason in (
            ScanEndReason.MAX_POINTS,
            ScanEndReason.MAX_DURATION,
            ScanEndReason.CLOCK_ANOMALY,
            ScanEndReason.EOF,
        ):
            with self.subTest(reason=reason):
                self.assertFalse(self._scan((point(0.0),), end_reason=reason).complete)

    def test_duration_uses_monotonic_clock(self) -> None:
        scan = self._scan(
            (point(0.0),),
            start_mono_ns=1_000_000_000,
            end_mono_ns=2_500_000_000,
        )
        self.assertAlmostEqual(scan.duration_s, 1.5)

    def test_coverage_for_one_point_is_zero(self) -> None:
        self.assertEqual(self._scan((point(0.0),)).angular_coverage_deg, 0.0)

    def test_coverage_for_duplicate_angles_is_zero(self) -> None:
        scan = self._scan((point(0.0), point(0.0), point(0.0)))
        self.assertEqual(scan.angular_coverage_deg, 0.0)

    def test_coverage_uses_largest_gap(self) -> None:
        scan = self._scan((point(0.0), point(90.0), point(180.0), point(270.0)))
        self.assertAlmostEqual(scan.angular_coverage_deg, 270.0)

    def test_coverage_across_zero(self) -> None:
        scan = self._scan((point(350.0), point(0.0), point(10.0)))
        self.assertAlmostEqual(scan.angular_coverage_deg, 20.0)


class ScanAssemblerTests(unittest.TestCase):
    def test_start_packet_closes_previous_scan(self) -> None:
        assembler = ScanAssembler()
        start = make_start_packet(
            angle_deg=0.0,
            received_unix_ns=100,
            received_mono_ns=1_000,
        )
        first = make_data_packet(
            angles_deg=[10.0, 20.0],
            received_unix_ns=200,
            received_mono_ns=2_000,
        )
        second = make_data_packet(
            angles_deg=[20.0, 30.0],
            received_unix_ns=300,
            received_mono_ns=3_000,
        )
        closing = make_start_packet(
            angle_deg=0.0,
            scan_frequency_hz=6.0,
            received_unix_ns=400,
            received_mono_ns=4_000,
        )

        self.assertIsNone(assembler.add(start))
        self.assertIsNone(assembler.add(first))
        self.assertIsNone(assembler.add(second))
        scan = assembler.add(closing)

        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.sequence, 1)
        self.assertEqual(scan.packet_count, 3)
        self.assertEqual(len(scan.points), 5)
        self.assertEqual(scan.start_unix_ns, 100)
        self.assertEqual(scan.end_unix_ns, 300)
        self.assertEqual(scan.start_mono_ns, 1_000)
        self.assertEqual(scan.end_mono_ns, 3_000)
        self.assertEqual(scan.end_reason, ScanEndReason.START_PACKET)
        self.assertTrue(scan.complete)
        self.assertAlmostEqual(scan.scan_frequency_hz or 0.0, 6.0)
        self.assertEqual(assembler.buffered_point_count, 1)

        final = assembler.finish()
        self.assertIsNotNone(final)
        assert final is not None
        self.assertEqual(final.sequence, 2)
        self.assertEqual(final.end_reason, ScanEndReason.EOF)
        self.assertFalse(final.complete)
        self.assertIsNone(assembler.finish())

    def test_start_packet_without_frequency_reports_none(self) -> None:
        assembler = ScanAssembler()
        assembler.add(make_start_packet(angle_deg=0.0, received_mono_ns=1))
        scan = assembler.add(
            make_start_packet(angle_deg=0.0, scan_frequency_hz=None, received_mono_ns=2)
        )
        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertIsNone(scan.scan_frequency_hz)

    def test_angle_wrap_fallback_closes_scan(self) -> None:
        assembler = ScanAssembler()
        before_wrap = make_data_packet(
            angles_deg=[350.0, 355.0],
            received_unix_ns=10,
            received_mono_ns=20,
        )
        after_wrap = make_data_packet(
            angles_deg=[0.0, 5.0],
            received_unix_ns=30,
            received_mono_ns=40,
        )

        self.assertIsNone(assembler.add(before_wrap))
        scan = assembler.add(after_wrap)

        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.end_reason, ScanEndReason.ANGLE_WRAP)
        self.assertTrue(scan.complete)
        self.assertEqual(len(scan.points), 2)
        self.assertEqual(scan.end_unix_ns, 10)
        self.assertEqual(scan.end_mono_ns, 20)
        self.assertIsNone(scan.scan_frequency_hz)
        self.assertEqual(assembler.buffered_point_count, 2)

        final = assembler.finish()
        self.assertIsNotNone(final)
        assert final is not None
        self.assertEqual(final.end_reason, ScanEndReason.EOF)
        self.assertEqual(len(final.points), 2)

    def test_no_wrap_fallback_after_start_packet(self) -> None:
        assembler = ScanAssembler()
        start = make_start_packet(
            angle_deg=350.0,
            received_mono_ns=1,
        )
        data = make_data_packet(angles_deg=[0.0, 5.0], received_mono_ns=2)
        closing = make_start_packet(
            angle_deg=0.0,
            scan_frequency_hz=5.0,
            received_mono_ns=3,
        )

        self.assertIsNone(assembler.add(start))
        self.assertIsNone(assembler.add(data))
        scan = assembler.add(closing)

        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.end_reason, ScanEndReason.START_PACKET)
        self.assertEqual(len(scan.points), 3)
        self.assertAlmostEqual(scan.scan_frequency_hz or 0.0, 5.0)

    def test_partial_first_scan_keeps_low_coverage(self) -> None:
        assembler = ScanAssembler()
        partial = make_data_packet(angles_deg=[100.0, 110.0], received_mono_ns=1)
        closing = make_start_packet(angle_deg=0.0, received_mono_ns=2)

        self.assertIsNone(assembler.add(partial))
        scan = assembler.add(closing)

        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertTrue(scan.complete)
        self.assertAlmostEqual(scan.angular_coverage_deg, 10.0)

    def test_finish_on_empty_assembler_returns_none(self) -> None:
        self.assertIsNone(ScanAssembler().finish())

    def test_max_points_closes_before_adding_packet(self) -> None:
        assembler = ScanAssembler(max_points_per_scan=2)
        full = make_data_packet(angles_deg=[0.0, 10.0], received_mono_ns=1)
        extra = make_data_packet(angles_deg=[20.0], received_mono_ns=2)

        self.assertIsNone(assembler.add(full))
        scan = assembler.add(extra)

        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.end_reason, ScanEndReason.MAX_POINTS)
        self.assertFalse(scan.complete)
        self.assertEqual(len(scan.points), 2)
        self.assertEqual(assembler.buffered_point_count, 1)

    def test_exact_point_limit_is_allowed_until_next_packet(self) -> None:
        assembler = ScanAssembler(max_points_per_scan=2)
        full = make_data_packet(angles_deg=[0.0, 10.0], received_mono_ns=1)
        self.assertIsNone(assembler.add(full))
        scan = assembler.finish()
        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(len(scan.points), 2)
        self.assertEqual(scan.end_reason, ScanEndReason.EOF)

    def test_packet_larger_than_point_limit_is_bounded_by_packet_size(self) -> None:
        assembler = ScanAssembler(max_points_per_scan=1)
        packet = make_data_packet(angles_deg=[0.0, 10.0], received_mono_ns=1)
        self.assertIsNone(assembler.add(packet))
        scan = assembler.finish()
        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(len(scan.points), 2)

    def test_max_duration_closes_before_adding_packet(self) -> None:
        assembler = ScanAssembler(max_scan_duration_s=1.0)
        first = make_data_packet(angles_deg=[0.0], received_mono_ns=0)
        late = make_data_packet(angles_deg=[10.0], received_mono_ns=1_500_000_000)

        self.assertIsNone(assembler.add(first))
        scan = assembler.add(late)

        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.end_reason, ScanEndReason.MAX_DURATION)
        self.assertFalse(scan.complete)
        self.assertEqual(scan.duration_s, 0.0)
        self.assertEqual(assembler.buffered_point_count, 1)

    def test_exact_duration_limit_closes(self) -> None:
        assembler = ScanAssembler(max_scan_duration_s=1.0)
        first = make_data_packet(angles_deg=[0.0], received_mono_ns=0)
        boundary = make_data_packet(angles_deg=[10.0], received_mono_ns=1_000_000_000)
        self.assertIsNone(assembler.add(first))
        scan = assembler.add(boundary)
        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.end_reason, ScanEndReason.MAX_DURATION)

    def test_clock_regression_closes_current_scan(self) -> None:
        assembler = ScanAssembler()
        first = make_data_packet(angles_deg=[0.0], received_mono_ns=100)
        earlier = make_data_packet(angles_deg=[10.0], received_mono_ns=99)

        self.assertIsNone(assembler.add(first))
        scan = assembler.add(earlier)

        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.end_reason, ScanEndReason.CLOCK_ANOMALY)
        self.assertFalse(scan.complete)
        self.assertEqual(scan.duration_s, 0.0)
        self.assertEqual(assembler.buffered_point_count, 1)

    def test_equal_monotonic_timestamps_are_allowed(self) -> None:
        assembler = ScanAssembler()
        first = make_data_packet(angles_deg=[0.0], received_mono_ns=100)
        second = make_data_packet(angles_deg=[10.0], received_mono_ns=100)
        self.assertIsNone(assembler.add(first))
        self.assertIsNone(assembler.add(second))
        scan = assembler.finish()
        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.packet_count, 2)

    def test_small_angle_backstep_does_not_trigger_wrap(self) -> None:
        assembler = ScanAssembler()
        first = make_data_packet(angles_deg=[100.0, 110.0], received_mono_ns=1)
        second = make_data_packet(angles_deg=[105.0, 115.0], received_mono_ns=2)
        self.assertIsNone(assembler.add(first))
        self.assertIsNone(assembler.add(second))
        scan = assembler.finish()
        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.packet_count, 2)

    def test_wrap_thresholds_are_inclusive(self) -> None:
        assembler = ScanAssembler()
        first = make_data_packet(angles_deg=[300.0], received_mono_ns=1)
        second = make_data_packet(angles_deg=[60.0], received_mono_ns=2)
        self.assertIsNone(assembler.add(first))
        scan = assembler.add(second)
        self.assertIsNotNone(scan)
        assert scan is not None
        self.assertEqual(scan.end_reason, ScanEndReason.ANGLE_WRAP)

    def test_mismatched_lsn_is_rejected(self) -> None:
        assembler = ScanAssembler()
        packet = make_data_packet(angles_deg=[0.0], received_mono_ns=1)
        with self.assertRaises(ValueError):
            assembler.add(replace(packet, lsn=2))

    def test_empty_packet_is_rejected(self) -> None:
        assembler = ScanAssembler()
        packet = make_data_packet(angles_deg=[0.0], received_mono_ns=1)
        with self.assertRaises(ValueError):
            assembler.add(replace(packet, lsn=0, points=()))

    def test_buffered_point_count_tracks_scan(self) -> None:
        assembler = ScanAssembler()
        packet = make_data_packet(angles_deg=[0.0, 10.0], received_mono_ns=1)
        self.assertEqual(assembler.buffered_point_count, 0)
        assembler.add(packet)
        self.assertEqual(assembler.buffered_point_count, 2)
        assembler.finish()
        self.assertEqual(assembler.buffered_point_count, 0)


class ScanAssemblerConfigTests(unittest.TestCase):
    def test_rejects_invalid_limits(self) -> None:
        invalid_arguments = (
            {"max_points_per_scan": 0},
            {"max_points_per_scan": -1},
            {"max_scan_duration_s": 0.0},
            {"max_scan_duration_s": -1.0},
            {"max_scan_duration_s": math.inf},
            {"max_scan_duration_s": math.nan},
            {"wrap_prev_min_deg": -1.0},
            {"wrap_prev_min_deg": 360.0},
            {"wrap_next_max_deg": -1.0},
            {"wrap_next_max_deg": 360.0},
        )
        for arguments in invalid_arguments:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                ScanAssembler(**arguments)


if __name__ == "__main__":
    unittest.main()
