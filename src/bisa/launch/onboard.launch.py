"""All-on-vehicle launch: run the FULL self-driving stack on the car (no PC).

Use this when remote computation is not allowed (e.g. a competition rule) or
when you simply want to drive without any WiFi link. Everything — camera,
low-level control, operator gamepad, AND the perception + mission FSM + control
node — runs on the car's TOPST (aarch64 Cortex-A72 + PowerVR GPU) chip. Nodes
talk over localhost DDS, so NO network / WiFi dongle is required to drive.

Contrast:
  vehicle.launch.py  -> car streams camera to the PC, PC computes (driving.launch)
  onboard.launch.py  -> car computes everything itself (this file)

YOLO runs via the NCNN backend with Vulkan GPU acceleration on the PowerVR
GT9524 (TCC8050). Vehicle A/B measurements show Vulkan trades a little detector
FPS for substantially lower CPU load, leaving cores for the lane/control loop.
The debug JPEG stream is off by default (headless car).
The sign-vote window is scaled from the 30 Hz PC baseline to match the GPU
inference rate on this board.

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
    """Return the installed NCNN model dir (GPU-accelerated via Vulkan).

    The on-car launch runs YOLO through the NCNN backend with Vulkan GPU
    acceleration on the PowerVR GT9524. Vulkan leaves more CPU time for
    lane/control; use device:=cpu for the measured CPU A/B baseline.
    PC launches keep using best.pt.
    Note: this NCNN model is exported at imgsz=320 — keep imgsz:=320.
    """

    return str(Path(get_package_share_directory("bisa")) / "checkpoints" / "best_ncnn_model")


def generate_launch_description():
    """Launch camera, control, joystick, and the compute node — all on the car."""

    route_mode = LaunchConfiguration("route_mode")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    control_topic = LaunchConfiguration("control_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    enable_joystick = LaunchConfiguration("enable_joystick")
    enable_actuation = LaunchConfiguration("enable_actuation")

    # GPU-accelerated detector tuning (overridable on the command line). These
    # become flat dotted ROS parameters that override the shared YAML at launch
    # — dracer_params.yaml stays PC/GPU-tuned and is untouched. The config
    # fields are typed (imgsz int, inference_hz float), and a LaunchConfiguration
    # resolves to a string, so wrap with ParameterValue to coerce the type —
    # otherwise declare_parameter rejects the override on a type mismatch.
    pipeline_hz = ParameterValue(LaunchConfiguration("pipeline_hz"), value_type=float)
    imgsz = ParameterValue(LaunchConfiguration("imgsz"), value_type=int)
    inference_hz = ParameterValue(LaunchConfiguration("inference_hz"), value_type=float)
    camera_hz = ParameterValue(LaunchConfiguration("camera_hz"), value_type=float)
    ncnn_threads = ParameterValue(LaunchConfiguration("ncnn_threads"), value_type=int)
    opencv_threads = ParameterValue(LaunchConfiguration("opencv_threads"), value_type=int)
    # Off by default (headless car saves CPU/GPU). Turn on to stream the
    # Detect / Lane-mask JPEGs so a PC on the same LAN can watch with
    # `ros2 run bisa viz_node`. Only useful while the WiFi dongle is still
    # in (tuning), not on the dongle-out competition run.
    publish_debug_image = ParameterValue(
        LaunchConfiguration("publish_debug_image"), value_type=bool
    )
    debug_image_hz = ParameterValue(
        LaunchConfiguration("debug_image_hz"), value_type=float
    )

    return LaunchDescription([
        DeclareLaunchArgument("route_mode", default_value="OUT"),
        DeclareLaunchArgument("config_file", default_value=default_config_path()),
        DeclareLaunchArgument("model_path", default_value=default_model_path()),
        DeclareLaunchArgument("image_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("control_topic", default_value="/control"),
        DeclareLaunchArgument("detections_topic", default_value="/bisa/detections"),
        # Set false if no gamepad dongle is plugged into the car.
        DeclareLaunchArgument("enable_joystick", default_value="true"),
        # Safe integration/benchmark mode: false keeps the hardware PCA9685
        # control node completely absent while every perception/control topic is
        # still produced and can be measured. Production default remains true.
        DeclareLaunchArgument("enable_actuation", default_value="true"),
        # Camera, lane perception, command publication, and the low-level command
        # watchdog share one time base by default. The NCNN detector gets the same
        # cap but may run slower when inference time exceeds the period.
        DeclareLaunchArgument("pipeline_hz", default_value="20.0"),
        DeclareLaunchArgument("device", default_value="vulkan:0"),
        DeclareLaunchArgument(
            "camera_hz", default_value=LaunchConfiguration("pipeline_hz")
        ),
        DeclareLaunchArgument("ncnn_threads", default_value="2"),
        DeclareLaunchArgument("opencv_threads", default_value="1"),
        # Detector input resolution. Must match the NCNN export resolution.
        # The NCNN model was exported at 320; override on CLI if re-exported.
        DeclareLaunchArgument("imgsz", default_value="320"),
        # Inference-rate cap; effective FPS is reported from measured results.
        DeclareLaunchArgument(
            "inference_hz", default_value=LaunchConfiguration("pipeline_hz")
        ),
        # Stream debug JPEGs for a PC viewer (tuning only; costs CPU). Default
        # off — the car is headless and normally runs dongle-out.
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        DeclareLaunchArgument("debug_image_hz", default_value="20.0"),

        # --- camera (publishes /camera/image/compressed) ---------------------
        Node(
            package="camera",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "publish_topic": image_topic,
                "capture_hz": camera_hz,
                "publish_hz": camera_hz,
                "mjpg_passthrough": True,
                "require_mjpg_passthrough": True,
                "jpeg_quality": 90,
                "debug_log": False,
            }],
        ),

        # --- low-level control (subscribes /control, drives the ESC/servo) ---
        Node(
            package="control",
            executable="control_node",
            name="control_node",
            output="screen",
            condition=IfCondition(enable_actuation),
            parameters=[{
                "control_topic": control_topic,
                "use_joystick_control": False,
                "command_hz": pipeline_hz,
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

        # --- NCNN detector (isolated Python process; no control-loop GIL) ------
        Node(
            package="bisa",
            executable="bisa_detector_node",
            name="bisa_detector_node",
            output="screen",
            parameters=[{
                "config_file": config_file,
                "model_path": model_path,
                "image_topic": image_topic,
                "detections_topic": detections_topic,
                "opencv_num_threads": opencv_threads,
                "detector.device": LaunchConfiguration("device"),
                "detector.imgsz": imgsz,
                "detector.inference_hz": inference_hz,
                "detector.ncnn_threads": ncnn_threads,
                "detector.warmup_enabled": True,
            }],
        ),

        # --- C++ lane perception + mission FSM + deterministic 20 Hz control --
        Node(
            package="bisa_cpp",
            executable="bisa_autonomous_node",
            name="bisa_autonomous_node",
            output="screen",
            parameters=[{
                "route_mode": route_mode,
                "config_file": config_file,
                "image_topic": image_topic,
                "control_topic": control_topic,
                "detections_topic": detections_topic,
                "publish_debug_image": publish_debug_image,
                "debug_image_hz": debug_image_hz,
                "perception_hz": pipeline_hz,
                "control_hz": pipeline_hz,
                "detection_hz_target": inference_hz,
                "sign_vote_k": 6,
                "sign_vote_n": 10,
                "light_confirm_frames": 8,
            }],
        ),
    ])
