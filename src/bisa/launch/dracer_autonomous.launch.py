from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_config_path():
    """Return the installed config path for the BISA autonomous parameters."""

    return str(Path(get_package_share_directory("bisa")) / "config" / "dracer_params.yaml")


def default_model_path():
    """Return the expected installed best.pth checkpoint path."""

    return str(Path(get_package_share_directory("bisa")) / "checkpoints" / "best.pth")


def generate_launch_description():
    """Launch camera, low-level control, and the BISA autonomous mission node."""

    route_mode = LaunchConfiguration("route_mode")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    control_topic = LaunchConfiguration("control_topic")
    debug_log = LaunchConfiguration("debug_log")

    return LaunchDescription([
        DeclareLaunchArgument("route_mode", default_value="OUT"),
        DeclareLaunchArgument("config_file", default_value=default_config_path()),
        DeclareLaunchArgument("model_path", default_value=default_model_path()),
        DeclareLaunchArgument("image_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("control_topic", default_value="/control"),
        DeclareLaunchArgument("debug_log", default_value="true"),
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
        Node(
            package="bisa",
            executable="bisa_autonomous_node",
            name="bisa_autonomous_node",
            output="screen",
            parameters=[{
                "route_mode": route_mode,
                "config_file": config_file,
                "model_path": model_path,
                "image_topic": image_topic,
                "control_topic": control_topic,
                "debug_log": debug_log,
            }],
        ),
    ])
