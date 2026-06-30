"""Vehicle-side launch: camera + low-level control ONLY.

Run this on the car. All perception/computation and visualization happen on the
PC (see driving.launch.py). The car publishes /camera/image/compressed and
subscribes to /control. Both machines must share the same ROS_DOMAIN_ID and LAN.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the camera and control nodes on the vehicle."""

    image_topic = LaunchConfiguration("image_topic")
    control_topic = LaunchConfiguration("control_topic")

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("control_topic", default_value="/control"),
        Node(
            package="camera",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "publish_topic": image_topic,
                "publish_hz": 30.0,
                "debug_log": False,
            }],
        ),
        Node(
            package="control",
            executable="control_node",
            name="control_node",
            output="screen",
            parameters=[{
                "control_topic": control_topic,
                "use_joystick_control": False,
                "command_hz": 10.0,
            }],
        ),
    ])
