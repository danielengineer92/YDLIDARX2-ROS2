import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ydlidar_x2_ros2.driver import YDLidarX2


class YDLidarX2Node(Node):
    """Convert raw X2 revolutions into ROS 2 LaserScan messages."""

    def __init__(self):
        """Configure the hardware driver and ROS interfaces."""
        super().__init__("ydlidar_x2_node")

        # Parameters let launch files and command-line users change hardware
        # settings without editing this Python source file.
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("frame_id", "ydlidar_x2_link")
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("angle_bins", 250)

        self.port = self.get_parameter("port").value
        self.frame_id = self.get_parameter("frame_id").value
        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value
        self.angle_bins = self.get_parameter("angle_bins").value

        self.get_logger().info(f"Opening port {self.port}")

        self.lidar = YDLidarX2(
            port=self.port,
            timeout=2.0,
        )

        # A depth of 10 lets a slow subscriber briefly fall behind without
        # forcing the serial-reading callback to wait for it.
        self.scan_publisher = self.create_publisher(
            LaserScan,
            "scan",
            10,
        )

        # get_scan() blocks until one revolution is complete, so the LiDAR's
        # motor determines the real publish rate. This short timer simply asks
        # the executor to begin the next read as soon as it can.
        self.scan_timer = self.create_timer(
            0.01,
            self.publish_scan,
        )

        # The first read can contain startup data from more than one
        # revolution, so discard it before publishing regular scans.
        self.warmup_scans_remaining = 1

    def publish_scan(self):
        """Read one revolution, bin its points, and publish a LaserScan."""
        scan_start = self.get_clock().now()
        points = self.lidar.get_scan()
        if self.warmup_scans_remaining > 0:
            self.warmup_scans_remaining -= 1
            self.get_logger().info("Discarding warm-up scan")
            return

        scan_end = self.get_clock().now()
        scan_time = (scan_end - scan_start).nanoseconds / 1_000_000_000

        bin_count = int(self.angle_bins)

        if bin_count <= 0:
            self.get_logger().error("angle_bins must be greater than 0")
            return

        angle_min = -math.pi
        angle_increment = (2.0 * math.pi) / bin_count
        angle_max = angle_min + (bin_count - 1) * angle_increment

        # LaserScan requires one range per evenly spaced direction. Infinity
        # is ROS's conventional value for a direction with no valid return.
        ranges = [math.inf] * bin_count

        for point in points:
            # The X2 reports increasing angles clockwise, while ROS uses
            # counter-clockwise-positive angles. Negating removes the
            # left/right mirror; modulo keeps the result in 0-360 degrees.
            angle = math.radians((-point.angle_deg) % 360.0)

            # LaserScan will use the conventional ROS range of -π to +π.
            if angle >= math.pi:
                angle -= 2.0 * math.pi

            # Turn a continuous angle into the corresponding array position.
            index = int((angle - angle_min) / angle_increment)
            distance = point.distance_mm / 1000.0

            if (
                0 <= index < bin_count
                and self.range_min <= distance <= self.range_max
            ):
                # Multiple raw points can land in one bin. Keeping the nearest
                # return is the safest representation for obstacle detection.
                ranges[index] = min(ranges[index], distance)

        # The following values are diagnostics only; they make it easier to
        # compare bin counts and judge the physical scan quality.
        raw_distances = [
            point.distance_mm / 1000.0
            for point in points
        ]

        zero_count = sum(
            distance == 0.0
            for distance in raw_distances
        )

        usable_distances = [
            distance
            for distance in raw_distances
            if self.range_min <= distance <= self.range_max
        ]

        filled_bin_count = sum(
            math.isfinite(distance)
            for distance in ranges
        )

        collision_count = len(usable_distances) - filled_bin_count
        fill_percent = 100.0 * filled_bin_count / bin_count

        nearest = min(usable_distances, default=math.nan)
        farthest = max(usable_distances, default=math.nan)

        scan_frequency = (
            1.0 / scan_time
            if scan_time > 0.0
            else 0.0
        )

        self.get_logger().info(
            f"raw={len(points)}, "
            f"zero={zero_count}, "
            f"usable={len(usable_distances)}, "
            f"bins={filled_bin_count}/{bin_count} ({fill_percent:.1f}%), "
            f"collisions={collision_count}, "
            f"range={nearest:.2f}-{farthest:.2f} m, "
            f"rate={scan_frequency:.2f} Hz",
            throttle_duration_sec=2.0,
        )

        # LaserScan angles are radians, distances are metres, and the header
        # names the TF frame in which every measurement was taken.
        message = LaserScan()
        message.header.stamp = scan_start.to_msg()
        message.header.frame_id = self.frame_id

        message.angle_min = angle_min
        message.angle_max = angle_max
        message.angle_increment = angle_increment

        message.scan_time = scan_time
        message.time_increment = scan_time / bin_count

        message.range_min = self.range_min
        message.range_max = self.range_max
        message.ranges = ranges

        self.scan_publisher.publish(message)

    def destroy_node(self):
        """Release both LiDAR and ROS resources."""
        try:
            self.lidar.close()
        finally:
            super().destroy_node()


def main(args=None):
    """Run the node until ROS shuts down or the user presses Ctrl+C."""
    rclpy.init(args=args)

    node = None

    try:
        node = YDLidarX2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        # A launch shutdown may already have stopped the shared ROS context.
        # Checking first prevents a second shutdown call from raising an error.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
