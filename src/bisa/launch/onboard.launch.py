"""All-on-vehicle launch: run the FULL self-driving stack on the car (no PC).

Use this when remote computation is not allowed (e.g. a competition rule) or
when you simply want to drive without any WiFi link. Everything — camera,
low-level control, operator gamepad, AND the perception + mission FSM + control
node — runs on the car's TOPST (aarch64 Cortex-A72, CPU-only) chip. Nodes talk
over localhost DDS, so NO network / WiFi dongle is required to drive.

Contrast:
  vehicle.launch.py  -> car streams camera to the PC, PC computes (driving.launch)
  onboard.launch.py  -> car computes everything itself (this file)

Because YOLO runs on the A72 CPU here (no GPU), the detector defaults are
overridden for the vehicle: smaller imgsz, a low inference-rate cap, device=cpu,
and the debug JPEG stream is off (the car is headless — nobody is watching it).
The sign-vote window that votes over detector frames is re-scaled from the 30 Hz
PC baseline to keep the same wall-clock timing at the lower on-car inference rate.

Watch the log line "YOLO infer: N ms/frame, M FPS effective" and tune imgsz /
inference_hz to what the chip actually delivers.

Usage (on the car):
  ros2 launch bisa onboard.launch.py                 # route_mode:=OUT
  ros2 launch bisa onboard.launch.py route_mode:=IN
  ros2 launch bisa onboard.launch.py inference_hz:=3.0 imgsz:=256
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_config_path():
    """Return the installed BISA params YAML path."""

    return str(Path(get_package_share_directory("bisa")) / "config" / "dracer_params.yaml")


def default_model_path():
    """Return the installed best.pt checkpoint path."""

    return str(Path(get_package_share_directory("bisa")) / "checkpoints" / "best.pt")


def generate_launch_description():
    """Launch camera, control, joystick, and the compute node — all on the car."""

    route_mode = LaunchConfiguration("route_mode")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    control_topic = LaunchConfiguration("control_topic")
    enable_joystick = LaunchConfiguration("enable_joystick")

    # CPU-only detector tuning (overridable on the command line). These become
    # flat dotted ROS parameters that override the shared YAML at launch —
    # dracer_params.yaml stays PC/GPU-tuned and is untouched. The config fields
    # are typed (imgsz int, inference_hz float), and a LaunchConfiguration
    # resolves to a string, so wrap with ParameterValue to coerce the type —
    # otherwise declare_parameter rejects the override on a type mismatch.
    imgsz = ParameterValue(LaunchConfiguration("imgsz"), value_type=int)
    inference_hz = ParameterValue(LaunchConfiguration("inference_hz"), value_type=float)
    # Off by default (headless car saves CPU). Turn on to stream the Detect /
    # Lane-mask JPEGs so a PC on the same LAN can watch with `ros2 run bisa
    # viz_node`. Only useful while the WiFi dongle is still in (tuning), not on
    # the dongle-out competition run.
    publish_debug_image = ParameterValue(
        LaunchConfiguration("publish_debug_image"), value_type=bool
    )

    return LaunchDescription([
        DeclareLaunchArgument("route_mode", default_value="OUT"),
        DeclareLaunchArgument("config_file", default_value=default_config_path()),
        DeclareLaunchArgument("model_path", default_value=default_model_path()),
        DeclareLaunchArgument("image_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("control_topic", default_value="/control"),
        # Set false if no gamepad dongle is plugged into the car.
        DeclareLaunchArgument("enable_joystick", default_value="true"),
        # Detector input resolution. Lower = faster on CPU, sees small/far
        # lights later. 320 is the vehicle default (PC uses 640 on GPU).
        DeclareLaunchArgument("imgsz", default_value="320"),
        # Inference-rate cap. The A72 will likely be slower than this anyway;
        # it bounds CPU use so the lane/control loop keeps a free core.
        DeclareLaunchArgument("inference_hz", default_value="4.0"),
        # Stream debug JPEGs for a PC viewer (tuning only; costs CPU). Default
        # off — the car is headless and normally runs dongle-out.
        DeclareLaunchArgument("publish_debug_image", default_value="false"),

        # --- camera (publishes /camera/image/compressed) ---------------------
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

        # --- low-level control (subscribes /control, drives the ESC/servo) ---
        Node(
            package="control",
            executable="control_node",
            name="control_node",
            output="screen",
            parameters=[{
                "control_topic": control_topic,
                "use_joystick_control": False,
                "command_hz": 10.0,
                # Board-priority voltage guard: motor + the D3-G 5V/5A regulator
                # share one 2S pack, so a sag browns out the board's 5V rail and
                # USB camera. The guard watches /battery/voltage and scales the
                # motor down before that happens (forward-only, floored at the
                # min rolling throttle). See control/power_guard.py.
                "voltage_guard_enabled": True,
                "guard_low_voltage": 6.5,
                "guard_critical_voltage": 6.2,
                "guard_min_scale": 0.75,
                "guard_floor_throttle": 0.20,
                "battery_voltage_topic": "/battery/voltage",
            }],
        ),

        # --- battery / power monitor (I2C INA219, publishes /battery/voltage) -
        # Feeds the control_node voltage guard. Without this node the guard has
        # no voltage data and stays idle (scale 1.0), so protection needs it
        # running. 5Hz: pack voltage changes slowly and a low rate keeps the
        # I2C read load off the board.
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

        # --- on-car system vitals (CPU / temp / memory) ----------------------
        # Headless car can't show a monitor, so it publishes its own vitals for
        # the PC power GUI to display. Dependency-free (/proc + /sys reads), 2Hz
        # — negligible load next to CPU-only YOLO. Watch these when the onboard
        # stack stalls: a saturated core or thermal throttle shows up here.
        Node(
            package="bisa",
            executable="system_telemetry_node",
            name="system_telemetry_node",
            output="screen",
            parameters=[{
                "publish_hz": 2.0,
            }],
        ),

        # --- operator gamepad (AUTO/MANUAL toggle = A, E-stop = X) -----------
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

        # --- perception + mission FSM + /control output (ON THE CAR) ---------
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
                # Headless car: off by default to save CPU; flip on with
                # publish_debug_image:=true to watch from a PC on the same LAN.
                "publish_debug_image": publish_debug_image,
                "debug_log": True,
                # --- CPU-only detector overrides (flat dotted config params) ---
                "detector.device": "cpu",
                "detector.imgsz": imgsz,
                "detector.inference_hz": inference_hz,
                # Sign vote window re-scaled 30 Hz -> ~4 Hz to preserve wall-clock:
                #   sign vote 9/15f@30Hz (~0.5s) -> 2/3f@4Hz (~0.75s)
                "detector.sign_vote_k": 2,
                "detector.sign_vote_n": 3,
            }],
        ),
    ])
