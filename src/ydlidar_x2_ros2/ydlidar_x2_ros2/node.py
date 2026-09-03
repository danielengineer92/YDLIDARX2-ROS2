from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ydlidar_x2_ros2.driver import YDLidarX2


class YDLidarX2Node(Node):
    def __init__(self):
        super().__init__("ydlidar_x2_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("frame_id", "ydlidar_x2_link")
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 8.0)
        self.declare_parameter("angle_bins", 360)

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

        self.scan_publisher = self.create_publisher(
            LaserScan,
            "scan",
            10,
        )

        self.scan_timer = self.create_timer(
            0.01,
            self.publish_scan,
        )

    def publish_scan(self):
        scan_start = self.get_clock().now()
        points = self.lidar.get_scan()
        message = LaserScan()
        message.header.stamp = scan_start.to_msg()
        message.header.frame_id = self.frame_id
