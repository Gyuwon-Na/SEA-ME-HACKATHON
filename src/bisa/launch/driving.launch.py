"""PC-side launch: perception + mission FSM + control output + visualization + GUI.

Run this on the operator PC. It subscribes to the car's camera, does all the
computation, publishes /control back to the car, and opens the debug view and
the parameter-tuning GUI. Both machines must share ROS_DOMAIN_ID and LAN.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def default_config_path(power_safe=False):
    """Return the installed BISA params YAML path.

    When ``power_safe`` is True, points at the power-safe fork
    (``dracer_params_powersafe.yaml``) which lowers throttle.speed_max and
    softens throttle.ramp_up_per_cmd so motor current spikes don't sag the
    shared 2S 18650 pack and brown out the D3-G 5V rail / USB camera.
    """

    name = "dracer_params_powersafe.yaml" if power_safe else "dracer_params.yaml"
    return str(Path(get_package_share_directory("bisa")) / "config" / name)


def default_model_path():
    """Return the installed best.pt checkpoint path."""

    return str(Path(get_package_share_directory("bisa")) / "checkpoints" / "best.pt")


def generate_launch_description():
    """Launch the autonomous compute node, the viewer, and the param GUI on the PC."""

    route_mode = LaunchConfiguration("route_mode")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    control_topic = LaunchConfiguration("control_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    enable_gui = LaunchConfiguration("enable_gui")
    enable_viz = LaunchConfiguration("enable_viz")
    enable_power_gui = LaunchConfiguration("enable_power_gui")

    # NOTE: the operator gamepad + joystick_node run on the CAR (vehicle.launch),
    # not here. This PC is often WSL, which exposes neither /dev/input/js0 nor the
    # I2C bus, so control_node/joystick_node cannot run here. Plug the wireless
    # pad's USB dongle into the car; the wireless pad stays in the operator's hand.
    return LaunchDescription([
        DeclareLaunchArgument("route_mode", default_value="IN"),
        # --- power-safe mode (removable) -------------------------------------
        # true (default) loads dracer_params_powersafe.yaml: gentler throttle
        # ramp + lower speed_max so motor current spikes don't sag the shared 2S
        # pack and brown out the board's 5V rail / USB camera. Turn off at runtime
        # with `power_safe:=false`, or to fully remove this feature delete this
        # DeclareLaunchArgument and restore: default_value=default_config_path().
        DeclareLaunchArgument("power_safe", default_value="true"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PythonExpression([
                '"', default_config_path(power_safe=True), '" if "',
                LaunchConfiguration("power_safe"),
                '" == "true" else "', default_config_path(power_safe=False), '"',
            ]),
        ),
        DeclareLaunchArgument("model_path", default_value=default_model_path()),
        DeclareLaunchArgument("image_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("control_topic", default_value="/control"),
        DeclareLaunchArgument("debug_image_topic", default_value="/bisa/debug/image/compressed"),
        DeclareLaunchArgument("enable_gui", default_value="true"),
        DeclareLaunchArgument("enable_viz", default_value="true"),
        # Standalone tkinter power monitor (battery V/A/W + board-5V/motor
        # estimate + sag plot). Turn off with `enable_power_gui:=false`.
        DeclareLaunchArgument("enable_power_gui", default_value="true"),
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
                "debug_image_topic": debug_image_topic,
                "publish_debug_image": True,
                "debug_log": True,
            }],
        ),
        Node(
            package="bisa",
            executable="viz_node",
            name="bisa_viz_node",
            output="screen",
            condition=IfCondition(enable_viz),
            parameters=[{"debug_image_topic": debug_image_topic}],
        ),
        Node(
            package="bisa",
            executable="param_gui_node",
            name="bisa_param_gui_node",
            output="screen",
            condition=IfCondition(enable_gui),
            parameters=[{"target_node": "bisa_autonomous_node", "config_file": config_file}],
        ),
        Node(
            package="bisa",
            executable="power_gui_node",
            name="bisa_power_gui_node",
            output="screen",
            condition=IfCondition(enable_power_gui),
            parameters=[{
                "brownout_voltage": 6.0,        # 2S pack sag alarm line (~12.0 for 4S).
                "board_current_limit_a": 5.0,   # D3-G rated 5A.
            }],
        ),
    ])
