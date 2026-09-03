"""Python serial driver for the YDLIDAR X2."""

from dataclasses import dataclass
import math
import serial


class YDLidarError(RuntimeError):
    """Base exception for YDLIDAR driver failures."""


class SerialTimeoutError(YDLidarError):
    """Raised when the serial port stops producing expected bytes."""


class InvalidPacketError(YDLidarError):
    """Raised when a packet has an invalid structure."""


class ChecksumError(InvalidPacketError):
    """Raised when a packet fails checksum validation."""


@dataclass(frozen=True)
class ScanPoint:
    """One polar-coordinate measurement from the LiDAR."""

    angle_deg: float
    distance_mm: float


@dataclass(frozen=True)
class ScanPacket:
    """One validated packet containing part of a 360-degree scan."""

    packet_type: int
    sample_count: int
    start_angle_deg: float
    end_angle_deg: float
    scan_frequency_hz: float | None
    points: tuple[ScanPoint, ...]

    @property
    def is_start(self) -> bool:
        """Whether this packet marks the beginning of a new scan."""

        return bool(self.packet_type & 0x01)


class YDLidarX2:
    """Read and decode scan packets from a YDLIDAR X2."""

    # The manual writes the header word as 0x55AA.
    # Because it is transmitted little-endian, the bytes arrive AA 55.
    HEADER_WORD = 0x55AA
    HEADER_BYTES = b"\xAA\x55"

    BAUD_RATE = 115200

    # After the two-byte header:
    # CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2) = 8 bytes.
    FIXED_BODY_SIZE = 8

    # The X2 provides one two-byte distance value per sample.
    SAMPLE_SIZE = 2

    def __init__(
        self,
        port: str,
        baudrate: int = BAUD_RATE,
        timeout: float = 1.0,
    ):
        """Open the X2 serial connection."""

        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
        )
        self._scan_synchronized = False

    def __enter__(self):
        """Allow use with Python's with statement."""

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Close the port when leaving a with block."""

        self.close()

    def close(self):
        """Release the serial device."""

        if self.serial.is_open:
            self.serial.close()

    def _read_exactly(self, size: int) -> bytes:
        """Read exactly size bytes or raise SerialTimeoutError."""

        read_bytes = bytearray()

        while len(read_bytes) < size:
            size_left = size - len(read_bytes)
            raw = self.serial.read(size_left)

            if raw == b'':
                raise SerialTimeoutError(f"Error at {len(read_bytes)} out of {size} bytes")

            read_bytes.extend(raw)

        return bytes(read_bytes)

    def _find_header(self) -> None:
        """Consume serial bytes until AA 55 is found."""
        previous = None
        while True:

            current = self._read_exactly(1)

            if current == b'\x55' and previous == b'\xAA':
                return

            previous = current

    def read_packet(self) -> ScanPacket:
        """Read and decode the next complete variable-length packet."""

        self._find_header()

        # Read CT through CS. LSN is body byte 1.
        body = self._read_exactly(self.FIXED_BODY_SIZE)
        sample_count = body[1]

        if sample_count == 0:
            raise InvalidPacketError("Packet contains zero samples")

        sample_bytes = self._read_exactly(
            sample_count * self.SAMPLE_SIZE
        )

        complete_packet = self.HEADER_BYTES + body + sample_bytes
        return self.decode_packet(complete_packet)

    @classmethod
    def decode_packet(cls, data: bytes) -> ScanPacket:
        """Validate and decode one complete X2 packet."""

        # Every packet has a 10-byte fixed section before its sample data:
        # PH(2) + CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2).
        if len(data) < 10:
            raise InvalidPacketError(
                f"Packet contains fewer than 10 bytes; received {len(data)}"
            )

        if data[:2] != cls.HEADER_BYTES:
            raise InvalidPacketError(
                f"Invalid packet header: {data[:2].hex(' ')}"
            )

        # CT contains the packet-type flag and, for a start packet, frequency.
        packet_type = data[2]

        if data[3] == 0:
            raise InvalidPacketError("Packet contains zero samples")
        sample_count = data[3]

        # Multi-byte values arrive least-significant byte first (little-endian).
        raw_starting_angle = int.from_bytes(data[4:6], "little")
        raw_end_angle = int.from_bytes(data[6:8], "little")
        received_checksum = int.from_bytes(data[8:10], "little")

        # LSN tells us exactly how many two-byte samples must follow CS.
        expected_length = 10 + sample_count * cls.SAMPLE_SIZE
        if len(data) != expected_length:
            raise InvalidPacketError(
                f"Expected {expected_length} bytes, received {len(data)}"
            )

        # Decode every sample as one unsigned, little-endian 16-bit value.
        raw_samples = []
        for i in range(sample_count):
            offset = 10 + i * cls.SAMPLE_SIZE
            raw = int.from_bytes(
                data[offset:offset + cls.SAMPLE_SIZE],
                "little",
            )
            raw_samples.append(raw)

        # The protocol checksum XORs 16-bit fields and samples. CS itself is
        # intentionally excluded because it is the value being verified.
        calculated_checksum = cls.HEADER_WORD
        calculated_checksum ^= (sample_count << 8) | packet_type
        calculated_checksum ^= raw_starting_angle
        calculated_checksum ^= raw_end_angle

        for sample in raw_samples:
            calculated_checksum ^= sample

        if calculated_checksum != received_checksum:
            raise ChecksumError(
                f"Checksum mismatch: calculated 0x{calculated_checksum:04X}, "
                f"received 0x{received_checksum:04X}"
            )

        # X2 triangle-LiDAR samples are encoded in quarter-millimeters.
        distances_mm = []
        for raw_sample in raw_samples:
            distance_mm = raw_sample / 4.0
            distances_mm.append(distance_mm)

        # First-level angle analysis: decode the packet boundary angles and
        # evenly interpolate one base angle for every distance sample.
        starting_angle = (raw_starting_angle >> 1) / 64.0
        end_angle = (raw_end_angle >> 1) / 64.0

        base_angles = []

        if sample_count == 1:
            base_angles.append(starting_angle)
        else:
            angle_span = end_angle - starting_angle

            # A packet can cross 0 degrees, for example from 350 to 10.
            if angle_span < 0:
                angle_span += 360.0

            angle_step = angle_span / (sample_count - 1)

            for i in range(sample_count):
                intermediate_angle = starting_angle + i * angle_step
                base_angles.append(intermediate_angle % 360.0)

        # Second-level analysis: compensate for the X2's triangle geometry.
        corrected_angles = []

        for i in range(sample_count):
            distance_mm = distances_mm[i]
            base_angle = base_angles[i]

            # A zero distance represents an invalid/no-return measurement.
            if distance_mm == 0:
                angle_correction = 0.0
            else:
                angle_correction = math.degrees(
                    math.atan(
                        21.8 * (155.3 - distance_mm)
                        / (155.3 * distance_mm)
                    )
                )

            corrected_angle = (base_angle + angle_correction) % 360.0
            corrected_angles.append(corrected_angle)

        # Pair each corrected angle with its matching distance. Build with a
        # mutable list, then freeze the completed collection as a tuple.
        points_list = []

        for angle, distance_mm in zip(corrected_angles, distances_mm):
            point = ScanPoint(
                angle_deg=angle,
                distance_mm=distance_mm,
            )
            points_list.append(point)

        points = tuple(points_list)

        # Bit 0 of CT marks the start/zero packet. The remaining bits encode
        # the previous revolution's frequency in tenths of a hertz. A decoded
        # value of zero means the device did not provide real-time frequency.
        scan_frequency_hz = None
        if packet_type & 0x01:
            decoded_frequency_hz = (packet_type >> 1) / 10.0
            if decoded_frequency_hz > 0:
                scan_frequency_hz = decoded_frequency_hz

        return ScanPacket(
            packet_type=packet_type,
            sample_count=sample_count,
            start_angle_deg=starting_angle,
            end_angle_deg=end_angle,
            scan_frequency_hz=scan_frequency_hz,
            points=points,
        )

    def get_scan(self) -> tuple[ScanPoint, ...]:
        """Collect packets belonging to one complete revolution."""

        # Only the first call needs to find an initial scan boundary.
        if not self._scan_synchronized:
            while True:
                packet = self.read_packet()

                if packet.is_start:
                    self._scan_synchronized = True
                    break

        scan_points = []

        # Collect until the boundary marking the next revolution.
        while True:
            packet = self.read_packet()

            if packet.is_start:
                return tuple(scan_points)

            scan_points.extend(packet.points)



