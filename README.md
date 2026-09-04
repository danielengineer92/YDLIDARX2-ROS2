# YDLIDAR X2 ROS 2 Driver

A pure-Python ROS 2 driver for the YDLIDAR X2 2D LiDAR. It reads the X2 binary
serial stream, validates and decodes each packet, assembles complete
revolutions, and publishes standard `sensor_msgs/msg/LaserScan` messages on
`/scan`.

The included launch file starts the driver, publishes the transform needed for
standalone visualization, and opens RViz with a ready-to-use display.

This project was built and hardware-tested with:

- Ubuntu 24.04
- ROS 2 Jazzy
- YDLIDAR X2
- Silicon Labs CP210x USB-to-UART adapter
- 115200 baud serial communication

## What the project does

The data takes this path:

```text
YDLIDAR X2
    -> CP2102 serial adapter
    -> Python packet decoder
    -> complete raw revolution
    -> evenly spaced angular bins
    -> ROS 2 LaserScan on /scan
    -> RViz
```

Features include:

- synchronization on the X2 `AA 55` packet header
- safe handling of partial serial reads and timeouts
- variable-length packet decoding and 16-bit XOR checksum validation
- distance and first/second-level angle decoding
- complete-revolution assembly
- conversion from millimetres and degrees to ROS metres and radians
- configurable LaserScan bin count
- scan-quality diagnostics in the node log
- launch arguments, standalone TF, and a saved RViz configuration
- 13 hardware-free driver tests using a fake serial port

## 1. Install ROS 2 and dependencies

Install ROS 2 Jazzy using the official ROS documentation first. A desktop
installation is recommended because it includes RViz.

Then install the build tools used by this workspace:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Source ROS before using its commands:

```bash
source /opt/ros/jazzy/setup.bash
```

If `rosdep` has never been initialized on this computer, run:

```bash
sudo rosdep init
rosdep update
```

## 2. Clone the repository

```bash
cd ~/Documents
git clone https://github.com/danielengineer92/YDLIDARX2-ROS2.git
cd YDLIDARX2-ROS2
```

Install the dependencies declared by `package.xml`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 3. Connect the LiDAR

Plug in the X2 and find its serial device:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls -l /dev/serial/by-id/ 2>/dev/null
```

The tested CP2102 adapter appeared as `/dev/ttyUSB0`. A path under
`/dev/serial/by-id/` is more stable if the computer has multiple USB serial
devices.

Check whether your account belongs to the `dialout` group:

```bash
groups
```

If `dialout` is absent, add it and then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

The new group membership does not apply to terminals that were already open.

## 4. Build the ROS 2 workspace

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ydlidar_x2_ros2
source install/setup.bash
```

Sourcing has two layers:

- `/opt/ros/jazzy/setup.bash` makes the base ROS 2 installation available.
- `install/setup.bash` makes this newly built package available.

You must source both again in each new terminal. `--symlink-install` is useful
while developing Python because many source edits become visible without
copying the package files again. Rebuild after changing package metadata,
launch files, entry points, or installed data files.

## 5. Launch the complete stack

Make sure the LiDAR is connected, then run:

```bash
ros2 launch ydlidar_x2_ros2 ydlidar_x2.launch.py
```

That one command starts three ROS processes:

1. `ydlidar_x2_node` reads the serial stream and publishes `/scan`.
2. `ydlidar_x2_static_tf` connects `map` to `ydlidar_x2_link` for standalone
   visualization.
3. `rviz2` loads `config/ydlidar_x2.rviz` and displays the scan.

Press `Ctrl+C` in the launch terminal to stop the complete stack and release
the serial port.

## 6. Override launch settings

The default serial port is `/dev/ttyUSB0`, but launch arguments can override it
without changing source code:

```bash
ros2 launch ydlidar_x2_ros2 ydlidar_x2.launch.py port:=/dev/ttyUSB1
```

You can use a stable device path too:

```bash
ros2 launch ydlidar_x2_ros2 ydlidar_x2.launch.py \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Available launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | Serial device connected to the X2 |
| `frame_id` | `ydlidar_x2_link` | TF frame assigned to each scan |
| `world_frame` | `map` | Parent frame used by standalone RViz |
| `angle_bins` | `250` | Number of evenly spaced values in `ranges` |

For example, compare output density with:

```bash
ros2 launch ydlidar_x2_ros2 ydlidar_x2.launch.py angle_bins:=180
```

The bin count does not make the physical LiDAR produce more samples. It only
controls how the raw points are placed into the fixed-size LaserScan array.
Empty directions are published as `inf`; if multiple points land in one bin,
the nearest distance is retained.

## 7. Inspect the ROS data

Open another terminal and source both environments:

```bash
cd ~/Documents/YDLIDARX2-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

List the active topics and measure the scan rate:

```bash
ros2 topic list
ros2 topic hz /scan
```

Inspect one complete message or only its range array:

```bash
ros2 topic echo /scan --once
ros2 topic echo /scan --once --field ranges
```

Verify the transform used by RViz:

```bash
ros2 run tf2_ros tf2_echo map ydlidar_x2_link
```

During hardware testing, steady-state scans commonly contained about 246-249
raw points, 175-190 usable returns, and published at roughly 12.2 Hz. The exact
number of valid returns changes with the room, target material, distance, and
occlusion.

The node prints diagnostics similar to:

```text
raw=247, zero=62, usable=185, bins=181/250 (72.4%), collisions=4, range=0.29-5.35 m, rate=12.22 Hz
```

- `raw` is the number of decoded points in one physical revolution.
- `zero` counts no-return samples reported as zero by the sensor.
- `usable` counts samples inside the configured distance limits.
- `bins` reports how many LaserScan directions received a valid return.
- `collisions` counts extra points that landed in an already occupied bin.
- `rate` is the measured revolution/publish frequency.

The first revolution is intentionally discarded because startup data can span
more than one scan boundary.

## Run only the driver node

The launch file is the normal entry point. For debugging, the node can also run
without TF or RViz:

```bash
ros2 run ydlidar_x2_ros2 ydlidar_x2_node
```

Pass ROS parameters directly with:

```bash
ros2 run ydlidar_x2_ros2 ydlidar_x2_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p angle_bins:=250
```

## Run the hardware-free driver tests

These tests exercise serial synchronization, packet validation, checksums,
angle decoding, and revolution assembly without requiring a connected X2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/ydlidar_x2_ros2/test/test_driver.py -q
```

## Read one scan from plain Python

After building and sourcing the workspace:

```python
from ydlidar_x2_ros2.driver import YDLidarX2


with YDLidarX2('/dev/ttyUSB0', timeout=2.0) as lidar:
    scan = lidar.get_scan()

print(f'Received {len(scan)} points')

for point in scan[:10]:
    print(
        f'angle={point.angle_deg:.2f} deg, '
        f'distance={point.distance_mm:.1f} mm'
    )
```

## Troubleshooting

### `/dev/ttyUSB0` does not exist

Confirm the LiDAR adapter is plugged in and run the device-listing commands
again. Linux may assign a different number after reconnection. Override `port`
with the detected path.

### Permission denied while opening the serial port

Confirm `groups` contains `dialout`. If you just added the group, log out and
back in before trying again. Also ensure another driver process is not already
using the same port.

### `ros2` says `launch` is an invalid command

Install the Jazzy launch CLI extension and source ROS again:

```bash
sudo apt install ros-jazzy-ros2launch
source /opt/ros/jazzy/setup.bash
```

### The launch file cannot be found

Rebuild and source the workspace after changing launch or packaging files:

```bash
colcon build --symlink-install --packages-select ydlidar_x2_ros2
source install/setup.bash
```

Launch files must be installed by the `data_files` section in `setup.py`. This
project installs files matching `launch/*.launch.py`.

### RViz says the frame does not exist or drops messages

Use the included launch file and set RViz's Fixed Frame to `map`. A
`LaserScan.header.frame_id` labels the data but does not create a TF transform;
the launch file's static transform supplies that relationship for standalone
viewing.

### The scan contains many `inf` values or zero raw distances

`inf` means no valid return was assigned to that angular bin. Raw zero values
are no-return measurements from the sensor. Move the LiDAR away from nearby
clutter, give it a clear horizontal view, and compare several scans rather
than judging the startup revolution.

## Project layout

```text
YDLIDARX2-ROS2/
├── README.md
├── requirements.txt
└── src/ydlidar_x2_ros2/
    ├── config/ydlidar_x2.rviz
    ├── launch/ydlidar_x2.launch.py
    ├── package.xml
    ├── setup.py
    ├── test/test_driver.py
    └── ydlidar_x2_ros2/
        ├── driver.py
        └── node.py
```

`driver.py` is independent of ROS message types and handles the wire protocol.
`node.py` is the ROS adapter: it reads a revolution from the driver, converts
the points to a LaserScan, and publishes the result.

## Protocol reference

The decoder follows the official
[YDLIDAR SDK communication protocol](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/doc/YDLidar-SDK-Communication-Protocol.md).
