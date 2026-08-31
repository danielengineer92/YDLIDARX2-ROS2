# YDLIDAR X2 ROS 2 Driver

A pure-Python serial driver for the YDLIDAR X2 2D LiDAR. The driver reads the
X2's binary serial stream, validates packet checksums, decodes distances and
angles, and assembles packet fragments into complete 360-degree scans.

ROS 2 `sensor_msgs/msg/LaserScan` publishing is the next development stage.

## Current features

- Serial synchronization using the X2 `AA 55` packet header
- Safe handling of partial serial reads and timeouts
- Variable-length packet decoding
- 16-bit XOR checksum validation
- Distance decoding in millimeters
- First- and second-level angle analysis
- Scan-frequency decoding from start packets
- Complete-revolution assembly
- Hardware-free unit tests using a fake serial port

## Hardware tested

- YDLIDAR X2
- Silicon Labs CP210x USB-to-UART adapter
- Linux serial device `/dev/ttyUSB0`
- 115200 baud

The serial-device number can change after reconnecting the adapter. Check the
current port with:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Your user needs serial-port access. On Ubuntu, add the user to `dialout`, then
log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run the tests

The test suite does not require a connected LiDAR:

```bash
python -m unittest -v
```

## Read one complete scan

```python
from driver import YDLidarX2


with YDLidarX2("/dev/ttyUSB0", timeout=2.0) as lidar:
    scan = lidar.get_scan()

print(f"Received {len(scan)} points")

for point in scan[:10]:
    print(
        f"angle={point.angle_deg:.2f} deg, "
        f"distance={point.distance_mm:.1f} mm"
    )
```

## Verified hardware result

The current driver successfully decoded a real X2 revolution containing 396
points with angle coverage from approximately 0 to 360 degrees.

## Protocol reference

The decoder follows the official
[YDLIDAR SDK communication protocol](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md).

