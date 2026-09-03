"""Unit tests for the YDLIDAR X2 serial driver."""

import unittest
from unittest.mock import Mock, patch

from ydlidar_x2_ros2.driver import (
    ChecksumError,
    InvalidPacketError,
    ScanPacket,
    ScanPoint,
    SerialTimeoutError,
    YDLidarX2,
)


# Complete no-intensity example packet published in the YDLIDAR protocol.
MANUAL_EXAMPLE_PACKET = bytes.fromhex(
    "AA 55 01 01 53 AE 53 AE AB 54 00 00"
)


def build_packet(
    *,
    packet_type: int,
    start_angle_deg: int,
    end_angle_deg: int,
    raw_samples: tuple[int, ...],
) -> bytes:
    """Build a valid test packet from readable field values."""

    sample_count = len(raw_samples)
    raw_start_angle = (start_angle_deg * 64) << 1
    raw_end_angle = (end_angle_deg * 64) << 1

    checksum = YDLidarX2.HEADER_WORD
    checksum ^= (sample_count << 8) | packet_type
    checksum ^= raw_start_angle
    checksum ^= raw_end_angle

    for raw_sample in raw_samples:
        checksum ^= raw_sample

    sample_bytes = b"".join(
        raw_sample.to_bytes(YDLidarX2.SAMPLE_SIZE, "little")
        for raw_sample in raw_samples
    )

    return (
        YDLidarX2.HEADER_BYTES
        + bytes((packet_type, sample_count))
        + raw_start_angle.to_bytes(2, "little")
        + raw_end_angle.to_bytes(2, "little")
        + checksum.to_bytes(2, "little")
        + sample_bytes
    )


class FakeSerial:
    """Small in-memory serial port used to test reads without hardware."""

    def __init__(self, data: bytes, max_chunk_size: int | None = None):
        self.buffer = bytearray(data)
        self.max_chunk_size = max_chunk_size
        self.is_open = True

    def read(self, size: int) -> bytes:
        """Return up to size bytes, optionally simulating partial reads."""

        if not self.buffer:
            return b""

        if self.max_chunk_size is not None:
            size = min(size, self.max_chunk_size)

        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def close(self) -> None:
        """Mark this fake port as closed."""

        self.is_open = False


def driver_with_serial(serial_port: FakeSerial) -> YDLidarX2:
    """Create a driver around a fake port without opening real hardware."""

    driver = object.__new__(YDLidarX2)
    driver.serial = serial_port
    return driver


def decoded_packet(
    *,
    is_start: bool,
    points: tuple[ScanPoint, ...] = (),
) -> ScanPacket:
    """Create a decoded packet for testing full-scan collection."""

    return ScanPacket(
        packet_type=0x01 if is_start else 0x00,
        sample_count=max(1, len(points)),
        start_angle_deg=0.0,
        end_angle_deg=0.0,
        scan_frequency_hz=None,
        points=points,
    )


class DecodePacketTests(unittest.TestCase):
    """Tests for validating and decoding complete packet bytes."""

    def test_decodes_manual_example_packet(self):
        packet = YDLidarX2.decode_packet(MANUAL_EXAMPLE_PACKET)

        self.assertTrue(packet.is_start)
        self.assertEqual(packet.packet_type, 0x01)
        self.assertEqual(packet.sample_count, 1)
        self.assertAlmostEqual(packet.start_angle_deg, 348.640625)
        self.assertAlmostEqual(packet.end_angle_deg, 348.640625)
        self.assertIsNone(packet.scan_frequency_hz)
        self.assertEqual(len(packet.points), 1)
        self.assertAlmostEqual(packet.points[0].angle_deg, 348.640625)
        self.assertEqual(packet.points[0].distance_mm, 0.0)

    def test_rejects_packet_shorter_than_fixed_section(self):
        with self.assertRaises(InvalidPacketError):
            YDLidarX2.decode_packet(b"\xAA\x55")

    def test_rejects_invalid_header(self):
        packet = b"\x00\x55" + MANUAL_EXAMPLE_PACKET[2:]

        with self.assertRaises(InvalidPacketError):
            YDLidarX2.decode_packet(packet)

    def test_rejects_zero_samples(self):
        packet = bytes.fromhex("AA 55 00 00 00 00 00 00 00 00")

        with self.assertRaises(InvalidPacketError):
            YDLidarX2.decode_packet(packet)

    def test_rejects_incorrect_packet_length(self):
        with self.assertRaises(InvalidPacketError):
            YDLidarX2.decode_packet(MANUAL_EXAMPLE_PACKET[:-1])

    def test_rejects_bad_checksum(self):
        packet = bytearray(MANUAL_EXAMPLE_PACKET)
        packet[8] ^= 0x01

        with self.assertRaises(ChecksumError):
            YDLidarX2.decode_packet(bytes(packet))

    def test_decodes_samples_across_zero_degrees(self):
        packet_bytes = build_packet(
            packet_type=0x00,
            start_angle_deg=350,
            end_angle_deg=10,
            raw_samples=(4000, 8000, 12000),
        )

        packet = YDLidarX2.decode_packet(packet_bytes)

        self.assertFalse(packet.is_start)
        self.assertEqual(packet.sample_count, 3)
        self.assertEqual(
            [point.distance_mm for point in packet.points],
            [1000.0, 2000.0, 3000.0],
        )
        self.assertAlmostEqual(packet.points[0].angle_deg, 343.2378139784)
        self.assertAlmostEqual(packet.points[1].angle_deg, 352.6227564079)
        self.assertAlmostEqual(packet.points[2].angle_deg, 2.4181094719)

    def test_decodes_frequency_from_start_packet(self):
        # Frequency bits encode 50 tenths of a hertz; bit 0 marks the start.
        packet_type = (50 << 1) | 0x01
        packet_bytes = build_packet(
            packet_type=packet_type,
            start_angle_deg=0,
            end_angle_deg=0,
            raw_samples=(0,),
        )

        packet = YDLidarX2.decode_packet(packet_bytes)

        self.assertTrue(packet.is_start)
        self.assertEqual(packet.scan_frequency_hz, 5.0)


class ScanCollectionTests(unittest.TestCase):
    """Tests for assembling packet fragments into complete revolutions."""

    def test_consecutive_calls_return_consecutive_scans(self):
        point_a1 = ScanPoint(angle_deg=10.0, distance_mm=1000.0)
        point_a2 = ScanPoint(angle_deg=20.0, distance_mm=2000.0)
        point_b = ScanPoint(angle_deg=30.0, distance_mm=3000.0)

        packets = (
            decoded_packet(is_start=True),
            decoded_packet(is_start=False, points=(point_a1,)),
            decoded_packet(is_start=False, points=(point_a2,)),
            decoded_packet(is_start=True),
            decoded_packet(is_start=False, points=(point_b,)),
            decoded_packet(is_start=True),
        )

        # Prevent __init__ from opening a real serial device.
        with patch(
            "ydlidar_x2_ros2.driver.serial.Serial",
            return_value=FakeSerial(b""),
        ):
            driver = YDLidarX2("fake-port")

        driver.read_packet = Mock(side_effect=packets)

        first_scan = driver.get_scan()
        second_scan = driver.get_scan()

        self.assertEqual(first_scan, (point_a1, point_a2))
        self.assertEqual(second_scan, (point_b,))
        self.assertEqual(driver.read_packet.call_count, 6)


class SerialReadingTests(unittest.TestCase):
    """Tests for the serial framing helpers."""

    def test_read_exactly_accumulates_partial_reads(self):
        driver = driver_with_serial(
            FakeSerial(b"\x10\x20\x30", max_chunk_size=1)
        )

        self.assertEqual(driver._read_exactly(3), b"\x10\x20\x30")

    def test_read_exactly_raises_after_timeout(self):
        driver = driver_with_serial(FakeSerial(b"\x10"))

        with self.assertRaises(SerialTimeoutError):
            driver._read_exactly(2)

    def test_find_header_handles_noise_and_overlapping_aa(self):
        serial_port = FakeSerial(b"\x00\xAA\xAA\x55remaining")
        driver = driver_with_serial(serial_port)

        driver._find_header()

        self.assertEqual(bytes(serial_port.buffer), b"remaining")

    def test_read_packet_handles_noise_and_partial_reads(self):
        serial_port = FakeSerial(
            b"\x99\x00" + MANUAL_EXAMPLE_PACKET,
            max_chunk_size=1,
        )
        driver = driver_with_serial(serial_port)

        packet = driver.read_packet()

        self.assertTrue(packet.is_start)
        self.assertEqual(packet.sample_count, 1)
        self.assertEqual(packet.points[0].distance_mm, 0.0)


if __name__ == "__main__":
    unittest.main()
