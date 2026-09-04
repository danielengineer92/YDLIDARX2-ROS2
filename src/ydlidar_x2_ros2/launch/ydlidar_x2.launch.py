from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the driver, its standalone TF, and a configured RViz window."""
    # LaunchConfiguration values are resolved when the launch file runs. Each
    # one can be overridden on the command line, for example port:=/dev/ttyUSB1.
    port = LaunchConfiguration("port")
    frame_id = LaunchConfiguration("frame_id")
    world_frame = LaunchConfiguration("world_frame")
    angle_bins = LaunchConfiguration("angle_bins")
    # Find the installed package instead of hard-coding one user's source path.
    rviz_config = PathJoinSubstitution([
        FindPackageShare("ydlidar_x2_ros2"),
        "config",
        "ydlidar_x2.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="Serial device connected to the YDLIDAR X2",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="ydlidar_x2_link",
            description="Coordinate frame attached to the LiDAR",
        ),
        DeclareLaunchArgument(
            "world_frame",
            default_value="map",
            description="Parent frame used for standalone visualization",
        ),
        DeclareLaunchArgument(
            "angle_bins",
            default_value="250",
            description="Number of angular bins in each LaserScan",
        ),
        # Start the Python node that reads the serial port and publishes /scan.
        Node(
            package="ydlidar_x2_ros2",
            executable="ydlidar_x2_node",
            name="ydlidar_x2_node",
            output="screen",
            parameters=[{
                "port": port,
                "frame_id": frame_id,
                "angle_bins": ParameterValue(
                    angle_bins,
                    value_type=int,
                ),
            }],
        ),
        # Publish an identity transform for standalone viewing. A robot would
        # normally supply this transform from its URDF/robot_state_publisher.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="ydlidar_x2_static_tf",
            output="screen",
            arguments=[
                "--frame-id",
                world_frame,
                "--child-frame-id",
                frame_id,
            ],
        ),
        # -d tells RViz to restore the displays saved in this repository.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
