"""Vehicle-side launch: camera + low-level control + operator gamepad.

Run this on the car. All perception/computation and visualization happen on the
PC (see driving.launch.py). The car publishes /camera/image/compressed and
subscribes to /control. Both machines must share the same ROS_DOMAIN_ID and LAN.

The joystick_node runs HERE (not on the PC): the wireless pad's USB dongle plugs
into the car (the car has /dev/input/js0 and the I2C bus; a WSL PC has neither).
control_node arbitrates AUTO (/control from the PC) vs MANUAL (this joystick),
toggled at runtime with the pad's A button. E-stop = X button.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the camera, control, and joystick nodes on the vehicle."""

    image_topic = LaunchConfiguration("image_topic")
    control_topic = LaunchConfiguration("control_topic")
    enable_joystick = LaunchConfiguration("enable_joystick")

    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("control_topic", default_value="/control"),
        # Set false if no gamepad dongle is plugged into the car.
        DeclareLaunchArgument("enable_joystick", default_value="true"),
        Node(
            package="camera",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "publish_topic": image_topic,
                # PC consumes at most control_hz/inference_hz (=10Hz), so streaming
                # faster just burns the weak vehicle board's CPU + WiFi power for
                # frames nobody reads. Matched to 10 to cut capture/encode/stream ~3x.
                "publish_hz": 10.0,
                # Lower JPEG quality trims encode CPU and WiFi bytes (radio power).
                "jpeg_quality": 45,
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
            package="joystick",
            executable="joystick_node",
            name="joystick_node",
            output="screen",
            condition=IfCondition(enable_joystick),
            parameters=[{
                "calibration_mode": False,
                "start_in_manual": False,
            }],
        ),
        # Battery/power monitor. Publishes battery_status (%) plus the raw
        # /battery/voltage, /battery/current_ma, /battery/power_w topics. Runs on
        # the car (I2C bus 3). 5Hz: battery values change slowly, so a low rate
        # keeps the I2C read load off the board.
        Node(
            package="battery",
            executable="battery_node",
            name="battery_node",
            output="screen",
            parameters=[{
                "publish_hz": 5.0,
                "debug_log": False,
            }],
        ),
    ])
