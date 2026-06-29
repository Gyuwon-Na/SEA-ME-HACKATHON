"""Single ROS2 node that runs D-Racer perception, mission FSM, and /control output."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from control_msgs.msg import Control
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from .dracer_config import load_config, resolve_package_relative_path
from .lane_perception import LaneObs, LanePerception, clamp
from .mission_controller import ControlCmd, LaneController, make_course_fsm
from .object_detector import BestPthDetector, DetectionBuffer


def as_bool(value) -> bool:
    """Converts launch string/bool parameter values into a Python bool."""

    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def get_default_config_path() -> str:
    """Finds the package-local YAML config in source or installed layouts."""

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("bisa")) / "config" / "dracer_params.yaml"
        if installed.exists():
            return str(installed)
    except Exception:
        pass

    for base_path in Path(__file__).resolve().parents:
        candidates = [
            base_path / "config" / "dracer_params.yaml",
            base_path / "share" / "bisa" / "config" / "dracer_params.yaml",
            base_path / "src" / "bisa" / "config" / "dracer_params.yaml",
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
    return "config/dracer_params.yaml"


class BisaAutonomousNode(Node):
    """ROS2 node that subscribes to camera images and publishes Control messages."""

    def __init__(self):
        """Declares parameters, loads config, and starts camera/control timers."""

        super().__init__("bisa_autonomous_node")
        self.declare_parameter("config_file", get_default_config_path())
        self.declare_parameter("route_mode", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("control_topic", "/control")
        self.declare_parameter("debug_log", True)

        config_file = str(self.get_parameter("config_file").value)
        route_mode = str(self.get_parameter("route_mode").value).strip()
        model_path = str(self.get_parameter("model_path").value).strip()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)
        self.debug_log = as_bool(self.get_parameter("debug_log").value)

        self.config = load_config(config_file)
        if route_mode:
            self.config.mission.route_mode = route_mode.upper()
        if model_path:
            self.config.detector.model_path = model_path
        resolved_model = resolve_package_relative_path(__file__, self.config.detector.model_path)

        self.lane_perception = LanePerception(self.config)
        self.det_buffer = DetectionBuffer(maxlen=40)
        self.detector = BestPthDetector(self.config, resolved_model, logger=self.get_logger())
        self.controller = LaneController(self.config)
        self.fsm = make_course_fsm(self.config, self.controller)
        self.latest_lane = LaneObs(valid=False, lost_reason="no_frame")
        self.last_frame = None
        self.last_log_time = 0.0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, self.image_topic, self.image_callback, image_qos)
        self.control_pub = self.create_publisher(Control, self.control_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.config.mission.control_hz, 1.0), self.control_loop)

        self.get_logger().info(
            "BISA autonomous node started: "
            f"route={self.config.mission.route_mode}, image_topic={self.image_topic}, "
            f"control_topic={self.control_topic}, model={resolved_model}, config={config_file}"
        )

    def decode_image(self, msg: CompressedImage) -> np.ndarray | None:
        """Decodes a ROS CompressedImage into a BGR OpenCV frame."""

        raw_data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image")
        return frame

    def image_callback(self, msg: CompressedImage) -> None:
        """Processes the newest camera frame for lane and async object perception."""

        frame = self.decode_image(msg)
        if frame is None:
            return
        self.last_frame = frame
        self.latest_lane = self.lane_perception.compute_lane_obs(frame)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        previous_infer_time = self.detector.last_infer_time
        detections = self.detector.infer(frame, now_sec)
        if detections or self.detector.last_infer_time != previous_infer_time:
            self.det_buffer.push(detections)

    def publish_control(self, cmd: ControlCmd) -> None:
        """Publishes clamped normalized throttle/steering to the /control topic."""

        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steering = float(clamp(cmd.steering, -1.0, 1.0))
        msg.throttle = float(clamp(cmd.throttle, 0.0, self.config.throttle.max))
        self.control_pub.publish(msg)

    def control_loop(self) -> None:
        """Runs the mission FSM at a fixed rate and publishes one control command."""

        now_sec = self.get_clock().now().nanoseconds / 1e9
        cmd = self.fsm.step(self.latest_lane, self.det_buffer, now_sec)
        self.publish_control(cmd)
        self.log_status(now_sec, cmd)

    def log_status(self, now_sec: float, cmd: ControlCmd) -> None:
        """Logs compact state/perception/control diagnostics at debug_log_hz."""

        if not self.debug_log:
            return
        period = 1.0 / max(self.config.mission.debug_log_hz, 0.1)
        if now_sec - self.last_log_time < period:
            return
        self.last_log_time = now_sec
        lane = self.latest_lane
        self.get_logger().info(
            f"state={self.fsm.state} throttle={cmd.throttle:.2f} steering={cmd.steering:.2f} "
            f"lane_valid={lane.valid} err={lane.center_error:.2f} curv={lane.curvature:.2f} "
            f"fork={lane.fork_seen} rotary={lane.rotary_seen}/{lane.rotary_exit_seen}"
        )


def main(args=None) -> None:
    """Initializes rclpy and spins the BISA autonomous node."""

    rclpy.init(args=args)
    node = BisaAutonomousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
