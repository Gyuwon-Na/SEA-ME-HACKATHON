"""All-on-vehicle launch: run the FULL self-driving stack on the car (no PC).

Use this when remote computation is not allowed (e.g. a competition rule) or
when you simply want to drive without any WiFi link. Everything — camera,
low-level control, operator gamepad, AND the perception + mission FSM + control
node — runs on the car's TOPST (aarch64 Cortex-A72 + PowerVR GPU) chip. Nodes
talk over localhost DDS, so NO network / WiFi dongle is required to drive.

This is the production launch: the car computes everything itself. A PC may
optionally run viz_node and param_gui_node over ROS for live tuning.

YOLO runs via the NCNN backend on the TOPST CPU. Vehicle same-frame A/B tests
showed that the PowerVR Vulkan path produced incorrect bounding-box coordinates,
while CPU inference produced stable detections with only a small FPS penalty.
The debug JPEG stream is off by default (headless car).
The sign-vote window is scaled from the 30 Hz PC baseline to match the CPU
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
    """Return the installed NCNN model directory.

    The on-car launch runs YOLO through the NCNN CPU backend because the tested
    PowerVR Vulkan path returned incorrect bounding boxes. ``device:=vulkan:0``
    remains available only for explicit backend experiments.
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
    mission_state_topic = LaunchConfiguration("mission_state_topic")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_joystick = LaunchConfiguration("enable_joystick")
    enable_actuation = LaunchConfiguration("enable_actuation")

    # CPU detector tuning (overridable on the command line). These
    # become flat dotted ROS parameters that override the shared YAML at launch
    # so every onboard entry point selects the same verified backend. The config
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
        DeclareLaunchArgument("mission_state_topic", default_value="/bisa/mission_state"),
        # Disable only for rosbag replay; production keeps the physical camera.
        DeclareLaunchArgument("enable_camera", default_value="true"),
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
        DeclareLaunchArgument("device", default_value="cpu"),
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
            condition=IfCondition(enable_camera),
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
                # battery_node is no longer part of the competition runtime;
                # keep the matching downstream guard explicitly disabled.
                "voltage_guard_enabled": False,
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
                "mission_state_topic": mission_state_topic,
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
