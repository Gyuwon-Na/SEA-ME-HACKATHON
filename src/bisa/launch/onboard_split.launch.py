"""분리+taskset 온보드 launch — YOLO를 별도 프로세스(전용코어)로 분리한 최적 구성.

onboard.launch.py(단일 프로세스, YOLO 내부 스레드)와 달리 검출을 detector_node로
떼어 **코어 2,3에 격리**하고 camera/제어/autonomous는 **코어 0,1**에 둠.
실측(hs/camera_opt_log.md): 검출 1.6→6.0FPS, 화면 1.65→4.6Hz, YOLO 439→117ms.
단순 분리는 4코어에서 경합으로 역효과였고 **taskset 코어격리가 결정적**이었음.

사용(차에서):
  ros2 launch bisa onboard_split.launch.py publish_debug_image:=True
  ros2 launch bisa onboard_split.launch.py enable_joystick:=false   # 게임패드 없을 때
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 코어 배분: 무거운 YOLO는 전용 2코어, 나머지 전부 나머지 2코어.
CORES_MAIN = "taskset -c 0,1"   # camera / control / autonomous / 보조노드
CORES_DET = "taskset -c 2,3"    # detector_node(YOLO NCNN) 전용


def default_config_path():
    """설치된 BISA params YAML 경로."""

    return str(Path(get_package_share_directory("bisa")) / "config" / "dracer_params.yaml")


def default_model_path():
    """설치된 NCNN 모델 디렉토리(온보드 CPU용). imgsz는 detector_node가 metadata에서 자동 강제."""

    return str(Path(get_package_share_directory("bisa")) / "checkpoints" / "best_ncnn_model")


def generate_launch_description():
    route_mode = LaunchConfiguration("route_mode")
    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    control_topic = LaunchConfiguration("control_topic")
    enable_joystick = LaunchConfiguration("enable_joystick")
    enable_detector = LaunchConfiguration("enable_detector")
    publish_debug_image = LaunchConfiguration("publish_debug_image")

    return LaunchDescription([
        DeclareLaunchArgument("route_mode", default_value="OUT"),
        DeclareLaunchArgument("config_file", default_value=default_config_path()),
        DeclareLaunchArgument("model_path", default_value=default_model_path()),
        DeclareLaunchArgument("image_topic", default_value="/camera/image/compressed"),
        DeclareLaunchArgument("control_topic", default_value="/control"),
        DeclareLaunchArgument("enable_joystick", default_value="true"),
        # false면 detector_node(YOLO)를 안 띄움 → 차선(LAB)만 주행. 나중에
        # detector_node를 따로 실행하면 신호등 검출이 붙음("YOLO 시간 두고 켜기").
        DeclareLaunchArgument("enable_detector", default_value="true"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),

        # --- camera (코어 0,1) — 화질유지 passthrough + 20fps ------------------
        Node(
            package="camera",
            executable="camera_node",
            name="camera_node",
            output="screen",
            prefix=CORES_MAIN,
            parameters=[{
                "publish_topic": image_topic,
                "publish_hz": 20.0,
                "mjpg_passthrough": True,
                "debug_log": False,
            }],
        ),

        # --- detector_node (코어 2,3 전용) — YOLO NCNN, GIL 탈출 --------------
        Node(
            package="bisa",
            executable="detector_node",
            name="bisa_detector_node",
            output="screen",
            prefix=CORES_DET,
            condition=IfCondition(enable_detector),
            parameters=[{
                "config_file": config_file,
                "model_path": model_path,
                "image_topic": image_topic,
                "detections_topic": "/bisa/detections",
            }],
        ),

        # --- low-level control (코어 0,1) ------------------------------------
        Node(
            package="control",
            executable="control_node",
            name="control_node",
            output="screen",
            prefix=CORES_MAIN,
            parameters=[{
                "control_topic": control_topic,
                "use_joystick_control": False,
                "command_hz": 10.0,
                "voltage_guard_enabled": True,
                "guard_low_voltage": 6.5,
                "guard_critical_voltage": 6.2,
                "guard_min_scale": 0.75,
                "guard_floor_throttle": 0.20,
                "battery_voltage_topic": "/battery/voltage",
            }],
        ),

        # --- battery monitor (코어 0,1) --------------------------------------
        Node(
            package="battery",
            executable="battery_node",
            name="battery_node",
            output="screen",
            prefix=CORES_MAIN,
            parameters=[{
                "publish_hz": 5.0,
                "debug_log": False,
            }],
        ),

        # --- system vitals (코어 0,1) ----------------------------------------
        Node(
            package="bisa",
            executable="system_telemetry_node",
            name="system_telemetry_node",
            output="screen",
            prefix=CORES_MAIN,
            parameters=[{
                "publish_hz": 2.0,
            }],
        ),

        # --- operator gamepad (코어 0,1) -------------------------------------
        Node(
            package="joystick",
            executable="joystick_node",
            name="joystick_node",
            output="screen",
            prefix=CORES_MAIN,
            condition=IfCondition(enable_joystick),
            parameters=[{
                "calibration_mode": False,
                "start_in_manual": False,
            }],
        ),

        # --- perception + mission FSM + /control (코어 0,1) ------------------
        # external_detector=true: 내부 YOLO 스레드 대신 detector_node의 검출을 구독.
        Node(
            package="bisa",
            executable="bisa_autonomous_node",
            name="bisa_autonomous_node",
            output="screen",
            prefix=CORES_MAIN,
            parameters=[{
                "route_mode": route_mode,
                "config_file": config_file,
                "model_path": model_path,
                "image_topic": image_topic,
                "control_topic": control_topic,
                "publish_debug_image": publish_debug_image,
                "debug_log": True,
                "external_detector": True,
                "detections_topic": "/bisa/detections",
            }],
        ),
    ])
