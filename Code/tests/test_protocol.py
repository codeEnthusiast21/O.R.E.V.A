from __future__ import annotations

import unittest

from oreva_lidar import (
    LidarPacket,
    PacketDecodeError,
    PacketFramer,
    ParseErrorReason,
    compute_checksum,
    decode_packet,
)

from .fixtures import (
    MANUAL_ANGLE_FIRST_DISTANCE_MM,
    MANUAL_ANGLE_FSA_RAW,
    MANUAL_ANGLE_LAST_DISTANCE_MM,
    MANUAL_ANGLE_LSA_RAW,
    MANUAL_START_PACKET,
    POWER_ON_MESSAGE,
    build_packet,
    data_packet,
    encode_angle,
    encode_distance,
    manual_angle_packet,
    start_packet,
)


def _signature(packet: LidarPacket) -> tuple[object, ...]:
    # Kept deliberately field-based so whole-stream and split-stream decoding
    # can be compared without depending on different chunk timestamps.
    return (
        packet.ct,
        packet.lsn,
        packet.fsa_deg,
        packet.lsa_deg,
        packet.is_start,
        packet.scan_frequency_hz,
        packet.points,
    )


class ManualVectorTests(unittest.TestCase):
    def test_manual_start_packet_decodes(self) -> None:
        packet = decode_packet(
            MANUAL_START_PACKET,
            received_unix_ns=1_000,
            received_mono_ns=2_000,
        )

        self.assertEqual(packet.lsn, 1)
        self.assertEqual(packet.ct, 0x01)
        self.assertTrue(packet.is_start)
        self.assertIsNone(packet.scan_frequency_hz)
        self.assertAlmostEqual(packet.fsa_deg, 348.640625)
        self.assertAlmostEqual(packet.lsa_deg, 348.640625)
        self.assertEqual(len(packet.points), 1)
        self.assertAlmostEqual(packet.points[0].angle_deg, 348.640625)
        self.assertEqual(packet.points[0].distance_raw, 0)
        self.assertEqual(packet.points[0].distance_m, 0.0)
        self.assertEqual(packet.received_unix_ns, 1_000)
        self.assertEqual(packet.received_mono_ns, 2_000)

    def test_manual_checksum(self) -> None:
        self.assertEqual(compute_checksum(MANUAL_START_PACKET), 0x54AB)

    def test_manual_interpolation_and_correction(self) -> None:
        packet = decode_packet(
            manual_angle_packet(),
            received_unix_ns=0,
            received_mono_ns=0,
        )

        self.assertEqual(packet.lsn, 40)
        self.assertEqual(len(packet.points), 40)
        self.assertAlmostEqual(packet.fsa_deg, (MANUAL_ANGLE_FSA_RAW >> 1) / 64.0)
        self.assertAlmostEqual(packet.lsa_deg, (MANUAL_ANGLE_LSA_RAW >> 1) / 64.0)

        first = packet.points[0]
        last = packet.points[-1]
        self.assertEqual(first.distance_raw, encode_distance(MANUAL_ANGLE_FIRST_DISTANCE_MM / 1000.0))
        self.assertEqual(last.distance_raw, encode_distance(MANUAL_ANGLE_LAST_DISTANCE_MM / 1000.0))
        self.assertAlmostEqual(first.distance_m, 1.0)
        self.assertAlmostEqual(last.distance_m, 8.0)

        # Values published in the X2 manual's second-level analysis example.
        # The manual rounds the starting angles to two decimals, so compare
        # with a tolerance that reflects that rounding rather than treating
        # the printed values as exact floating-point results.
        self.assertAlmostEqual(first.angle_deg, 217.0178, delta=0.002)
        self.assertAlmostEqual(last.angle_deg, 235.6326, delta=0.002)

        base_step = (packet.lsa_deg - packet.fsa_deg) / 39.0
        self.assertAlmostEqual(packet.points[1].angle_deg, packet.fsa_deg + base_step)

    def test_distance_conversion_uses_quarter_millimetres(self) -> None:
        frame = build_packet(
            ct=0x00,
            fsa_raw=encode_angle(0.0),
            lsa_raw=encode_angle(0.0),
            samples=[0x6FE5],
        )
        point = decode_packet(frame, received_unix_ns=0, received_mono_ns=0).points[0]
        self.assertEqual(point.distance_raw, 0x6FE5)
        self.assertAlmostEqual(point.distance_m, 7.16125)


class DecodeValidationTests(unittest.TestCase):
    def test_rejects_short_frame(self) -> None:
        with self.assertRaises(PacketDecodeError) as context:
            decode_packet(b"\xAA\x55", received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(context.exception.reason, ParseErrorReason.TRUNCATED_FRAME)

    def test_rejects_bad_header(self) -> None:
        frame = bytearray(data_packet(start_deg=0.0, end_deg=0.0, sample_count=1))
        frame[0] = 0x00
        with self.assertRaises(PacketDecodeError) as context:
            decode_packet(bytes(frame), received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(context.exception.reason, ParseErrorReason.BAD_HEADER)

    def test_rejects_zero_lsn(self) -> None:
        frame = build_packet(ct=0x00, fsa_raw=0x0001, lsa_raw=0x0001, samples=[])
        with self.assertRaises(PacketDecodeError) as context:
            decode_packet(frame, received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(context.exception.reason, ParseErrorReason.INVALID_LSN)

    def test_rejects_frame_length_mismatch(self) -> None:
        frame = data_packet(start_deg=0.0, end_deg=10.0, sample_count=2)
        for malformed in (frame[:-1], frame + b"\x00"):
            with self.subTest(length=len(malformed)):
                with self.assertRaises(PacketDecodeError) as context:
                    decode_packet(malformed, received_unix_ns=0, received_mono_ns=0)
                self.assertEqual(
                    context.exception.reason,
                    ParseErrorReason.FRAME_LENGTH_MISMATCH,
                )

    def test_rejects_bad_checksum(self) -> None:
        frame = bytearray(data_packet(start_deg=0.0, end_deg=10.0, sample_count=2))
        frame[-1] ^= 0xFF
        with self.assertRaises(PacketDecodeError) as context:
            decode_packet(bytes(frame), received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(context.exception.reason, ParseErrorReason.CHECKSUM_MISMATCH)

    def test_rejects_angle_above_360(self) -> None:
        # (23100 >> 1) / 64 = 360.9375 degrees, which cannot be a valid angle.
        invalid_raw = (23100 << 1) | 0x01
        frame = build_packet(
            ct=0x00,
            fsa_raw=invalid_raw,
            lsa_raw=encode_angle(10.0),
            samples=[0],
        )
        with self.assertRaises(PacketDecodeError) as context:
            decode_packet(frame, received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(context.exception.reason, ParseErrorReason.INVALID_ANGLE)

    def test_accepts_360_encoded_angle_as_zero(self) -> None:
        raw_360 = (23040 << 1) | 0x01
        frame = build_packet(
            ct=0x00,
            fsa_raw=raw_360,
            lsa_raw=encode_angle(30.0),
            samples=[0, 0, 0],
        )
        packet = decode_packet(frame, received_unix_ns=0, received_mono_ns=0)
        self.assertAlmostEqual(packet.fsa_deg, 0.0)
        self.assertAlmostEqual(packet.points[0].angle_deg, 0.0)

    def test_lsn_one_uses_start_angle(self) -> None:
        frame = build_packet(
            ct=0x00,
            fsa_raw=encode_angle(123.5),
            lsa_raw=encode_angle(10.0),
            samples=[0],
        )
        packet = decode_packet(frame, received_unix_ns=0, received_mono_ns=0)
        self.assertAlmostEqual(packet.points[0].angle_deg, 123.5)

    def test_start_flag_and_frequency(self) -> None:
        timed = decode_packet(
            start_packet(angle_deg=0.0, frequency_hz=6.0),
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertTrue(timed.is_start)
        self.assertAlmostEqual(timed.scan_frequency_hz, 6.0)

        no_frequency = decode_packet(
            start_packet(angle_deg=0.0),
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertTrue(no_frequency.is_start)
        self.assertIsNone(no_frequency.scan_frequency_hz)

        data = decode_packet(
            data_packet(start_deg=0.0, end_deg=10.0, sample_count=2),
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertFalse(data.is_start)
        self.assertIsNone(data.scan_frequency_hz)

    def test_angle_wrap_interpolation(self) -> None:
        packet = decode_packet(
            data_packet(start_deg=350.0, end_deg=10.0, sample_count=3),
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertEqual(len(packet.points), 3)
        self.assertAlmostEqual(packet.points[0].angle_deg, 350.0)
        self.assertAlmostEqual(packet.points[1].angle_deg, 0.0)
        self.assertAlmostEqual(packet.points[2].angle_deg, 10.0)

    def test_zero_distance_is_retained(self) -> None:
        packet = decode_packet(
            data_packet(
                start_deg=0.0,
                end_deg=1.0,
                distances_m=[0.0, 1.0],
            ),
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertEqual(packet.points[0].distance_raw, 0)
        self.assertEqual(packet.points[0].distance_m, 0.0)
        self.assertEqual(packet.points[1].distance_raw, 4000)
        self.assertAlmostEqual(packet.points[1].distance_m, 1.0)

    def test_checksum_helper_rejects_bad_length(self) -> None:
        frame = data_packet(start_deg=0.0, end_deg=10.0, sample_count=2)
        with self.assertRaises(ValueError):
            compute_checksum(b"\xAA\x55")
        with self.assertRaises(ValueError):
            compute_checksum(frame[:-1])
        with self.assertRaises(ValueError):
            compute_checksum(frame + b"\x00")


class FramerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.first = data_packet(start_deg=0.0, end_deg=10.0, sample_count=2)
        self.second = data_packet(start_deg=10.0, end_deg=20.0, sample_count=2)
        self.third = data_packet(start_deg=20.0, end_deg=30.0, sample_count=2)

    def test_multiple_packets_in_one_chunk(self) -> None:
        framer = PacketFramer()
        batch = framer.feed(
            self.first + self.second,
            received_unix_ns=100,
            received_mono_ns=200,
        )
        self.assertEqual(len(batch.packets), 2)
        self.assertEqual(batch.errors, ())
        self.assertEqual(batch.bytes_discarded, 0)
        for packet in batch.packets:
            self.assertEqual(packet.received_unix_ns, 100)
            self.assertEqual(packet.received_mono_ns, 200)

    def test_packet_decodes_across_every_split_point(self) -> None:
        stream = self.first + self.second + self.third
        framer = PacketFramer()
        expected = framer.feed(stream, received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(len(expected.packets), 3)
        self.assertEqual(expected.errors, ())

        expected_signatures = [_signature(packet) for packet in expected.packets]
        for split in range(1, len(stream)):
            with self.subTest(split=split):
                split_framer = PacketFramer()
                first_batch = split_framer.feed(
                    stream[:split],
                    received_unix_ns=10,
                    received_mono_ns=20,
                )
                second_batch = split_framer.feed(
                    stream[split:],
                    received_unix_ns=30,
                    received_mono_ns=40,
                )
                final_batch = split_framer.finish()
                packets = first_batch.packets + second_batch.packets
                self.assertEqual(
                    [_signature(packet) for packet in packets],
                    expected_signatures,
                )
                self.assertEqual(first_batch.errors, ())
                self.assertEqual(second_batch.errors, ())
                self.assertEqual(final_batch.errors, ())
                self.assertEqual(first_batch.bytes_discarded, 0)
                self.assertEqual(second_batch.bytes_discarded, 0)
                self.assertEqual(final_batch.bytes_discarded, 0)

    def test_garbage_and_power_on_message_are_skipped(self) -> None:
        framer = PacketFramer()
        garbage = b"\x00\x01garbage"
        batch = framer.feed(
            garbage + POWER_ON_MESSAGE + self.first,
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertEqual(len(batch.packets), 1)
        self.assertEqual(batch.errors, ())
        self.assertEqual(batch.bytes_discarded, len(garbage) + len(POWER_ON_MESSAGE))

    def test_false_header_inside_sample_data(self) -> None:
        # 0x55AA is stored little-endian as AA 55, the same bytes as the header.
        frame = build_packet(
            ct=0x00,
            fsa_raw=encode_angle(0.0),
            lsa_raw=encode_angle(0.0),
            samples=[0x55AA],
        )
        framer = PacketFramer()
        batch = framer.feed(frame, received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(len(batch.packets), 1)
        self.assertEqual(batch.errors, ())
        self.assertEqual(batch.packets[0].points[0].distance_raw, 0x55AA)

    def test_checksum_failure_recovers_to_next_packet(self) -> None:
        corrupt = bytearray(self.first)
        corrupt[-1] ^= 0xFF
        framer = PacketFramer()
        batch = framer.feed(
            bytes(corrupt) + self.second,
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertEqual(len(batch.packets), 1)
        self.assertEqual(len(batch.errors), 1)
        self.assertEqual(batch.errors[0].reason, ParseErrorReason.CHECKSUM_MISMATCH)
        self.assertEqual(batch.errors[0].stream_offset, 0)
        self.assertGreater(batch.bytes_discarded, 0)
        self.assertEqual(
            batch.packets[0].points[0].angle_deg,
            decode_packet(self.second, received_unix_ns=0, received_mono_ns=0)
            .points[0]
            .angle_deg,
        )

    def test_invalid_lsn_candidate_is_reported_and_skipped(self) -> None:
        bad_candidate = b"\xAA\x55" + (b"\x00" * 8)
        framer = PacketFramer()
        batch = framer.feed(
            bad_candidate + self.first,
            received_unix_ns=0,
            received_mono_ns=0,
        )
        self.assertEqual(len(batch.packets), 1)
        self.assertEqual(len(batch.errors), 1)
        self.assertEqual(batch.errors[0].reason, ParseErrorReason.INVALID_LSN)
        self.assertEqual(batch.errors[0].stream_offset, 0)

    def test_truncated_packet_reported_at_finish(self) -> None:
        frame = self.first + self.second
        framer = PacketFramer()
        batch = framer.feed(frame[:-5], received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(len(batch.packets), 1)
        self.assertEqual(batch.errors, ())
        self.assertEqual(framer.buffered_bytes, len(self.second) - 5)

        final_batch = framer.finish()
        self.assertEqual(final_batch.packets, ())
        self.assertEqual(len(final_batch.errors), 1)
        self.assertEqual(final_batch.errors[0].reason, ParseErrorReason.TRUNCATED_FRAME)
        self.assertEqual(final_batch.errors[0].stream_offset, len(self.first))
        self.assertEqual(framer.buffered_bytes, 0)
        self.assertEqual(framer.finish().errors, ())

    def test_partial_header_across_chunks(self) -> None:
        framer = PacketFramer()
        first_batch = framer.feed(self.first[:1], received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(first_batch.packets, ())
        self.assertEqual(framer.buffered_bytes, 1)
        second_batch = framer.feed(self.first[1:], received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(len(second_batch.packets), 1)
        self.assertEqual(second_batch.errors, ())

    def test_empty_chunk_and_finish(self) -> None:
        framer = PacketFramer()
        batch = framer.feed(b"", received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(batch.packets, ())
        self.assertEqual(batch.errors, ())
        self.assertEqual(framer.finish().packets, ())
        self.assertEqual(framer.finish().errors, ())

    def test_timestamps_follow_each_chunk(self) -> None:
        framer = PacketFramer()
        first_batch = framer.feed(self.first, received_unix_ns=111, received_mono_ns=222)
        second_batch = framer.feed(self.second, received_unix_ns=333, received_mono_ns=444)
        self.assertEqual(first_batch.packets[0].received_unix_ns, 111)
        self.assertEqual(first_batch.packets[0].received_mono_ns, 222)
        self.assertEqual(second_batch.packets[0].received_unix_ns, 333)
        self.assertEqual(second_batch.packets[0].received_mono_ns, 444)

    def test_invalid_angle_candidate_recovers(self) -> None:
        invalid = build_packet(
            ct=0x00,
            fsa_raw=(23100 << 1) | 0x01,
            lsa_raw=encode_angle(10.0),
            samples=[0],
        )
        framer = PacketFramer()
        batch = framer.feed(invalid + self.first, received_unix_ns=0, received_mono_ns=0)
        self.assertEqual(len(batch.packets), 1)
        self.assertEqual(len(batch.errors), 1)
        self.assertEqual(batch.errors[0].reason, ParseErrorReason.INVALID_ANGLE)


if __name__ == "__main__":
    unittest.main()
