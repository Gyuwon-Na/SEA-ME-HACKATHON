"""Single ROS2 node that runs D-Racer perception, mission FSM, and /control output."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from control_msgs.msg import Control
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32, String

from . import traffic_light, visualization
from .aruco_detector import ArucoDetector
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
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/bisa/debug/image/compressed")

        config_file = str(self.get_parameter("config_file").value)
        route_mode = str(self.get_parameter("route_mode").value).strip()
        model_path = str(self.get_parameter("model_path").value).strip()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)
        self.debug_log = as_bool(self.get_parameter("debug_log").value)
        self.publish_debug_image = as_bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self.config = load_config(config_file)
        if route_mode:
            self.config.mission.route_mode = route_mode.upper()
        if model_path:
            self.config.detector.model_path = model_path
        resolved_model = resolve_package_relative_path(__file__, self.config.detector.model_path)

        self.lane_perception = LanePerception(self.config)
        self.det_buffer = DetectionBuffer(maxlen=40)
        self.detector = BestPthDetector(self.config, resolved_model, logger=self.get_logger())
        self.light_analyzer = traffic_light.TrafficLightAnalyzer(self.config)
        self.aruco_detector = ArucoDetector(self.config)
        self.controller = LaneController(self.config)
        self.fsm = make_course_fsm(self.config, self.controller)
        self.latest_lane = LaneObs(valid=False, lost_reason="no_frame")
        self.latest_detections = []
        self.latest_markers = []
        self.last_cmd = ControlCmd(0.0, 0.0)
        self.last_frame = None
        self.last_log_time = 0.0

        # YOLO is CPU-only on this vehicle and a single predict() can exceed the
        # camera frame interval. Running it inside image_callback would block the
        # single-threaded executor and freeze the camera/debug stream and control
        # loop, so inference runs on a background thread that always processes the
        # most recent frame and drops anything it could not keep up with.
        self._infer_lock = threading.Lock()
        self._infer_frame = None
        self._infer_stop = False
        self._infer_thread = threading.Thread(
            target=self._inference_worker, name="bisa_inference", daemon=True
        )

        # BEST_EFFORT + shallow queue: always process the newest camera frame and
        # drop stale ones, matching the camera publisher and avoiding WiFi lag.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, self.image_topic, self.image_callback, image_qos)
        self.control_pub = self.create_publisher(Control, self.control_topic, 10)

        # Mission/detection status topics for PC-side `ros2 topic echo`.
        self.detect_green_pub = self.create_publisher(Bool, "detect_green", 10)
        self.detect_red_pub = self.create_publisher(Bool, "detect_red", 10)
        self.detect_sign_pub = self.create_publisher(String, "detect_sign", 10)
        self.detect_aruco_pub = self.create_publisher(String, "detect_aruco", 10)
        # Mission FSM + rotary diagnostics for `ros2 topic echo` while manually
        # driving the car through the rotary to watch the state machine advance.
        self.mission_state_pub = self.create_publisher(String, "/mission/state", 10)
        self.rotary_progress_pub = self.create_publisher(Float32, "/mission/rotary_progress", 10)
        self.exit_line_count_pub = self.create_publisher(Int32, "/mission/exit_line_count", 10)
        self.rotary_exit_seen_pub = self.create_publisher(Bool, "/mission/rotary_exit_seen", 10)
        # Debug JPEG stream reuses the camera's BEST_EFFORT/depth-1 profile so it
        # drops stale frames instead of triggering RELIABLE retransmits, which
        # caused WiFi head-of-line blocking against the viz/monitor subscribers.
        self.debug_image_pub = self.create_publisher(
            CompressedImage, self.debug_image_topic, image_qos
        )

        # Expose tunable config as flat dotted ROS parameters and apply live edits.
        self._declare_tuning_params()
        self.add_on_set_parameters_callback(self._on_set_tuning_params)

        self.timer = self.create_timer(1.0 / max(self.config.mission.control_hz, 1.0), self.control_loop)
        self._infer_thread.start()

        self.get_logger().info(
            "BISA autonomous node started: "
            f"route={self.config.mission.route_mode}, image_topic={self.image_topic}, "
            f"control_topic={self.control_topic}, model={resolved_model}, config={config_file}"
        )

    def _flatten_config(self) -> list[tuple[str, object]]:
        """Flattens the config dataclasses into (dotted_name, value) tuples."""

        from dataclasses import fields, is_dataclass

        pairs: list[tuple[str, object]] = []

        def walk(prefix: str, obj) -> None:
            if is_dataclass(obj):
                for f in fields(obj):
                    walk(f"{prefix}.{f.name}" if prefix else f.name, getattr(obj, f.name))
            elif isinstance(obj, dict):
                for key, value in obj.items():
                    if isinstance(value, (bool, int, float, str)):
                        pairs.append((f"{prefix}.{key}", value))
            elif isinstance(obj, (list, tuple)):
                if obj and all(isinstance(v, bool) for v in obj):
                    pairs.append((prefix, [bool(v) for v in obj]))
                elif obj and all(isinstance(v, int) and not isinstance(v, bool) for v in obj):
                    pairs.append((prefix, [int(v) for v in obj]))
                elif obj and all(isinstance(v, (int, float)) for v in obj):
                    pairs.append((prefix, [float(v) for v in obj]))
            elif isinstance(obj, (bool, int, float, str)):
                pairs.append((prefix, obj))

        walk("", self.config)
        return pairs

    def _declare_tuning_params(self) -> None:
        """Declares every flat config value as a ROS parameter for live tuning."""

        self._tuning_names = set()
        for name, value in self._flatten_config():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)
            self._tuning_names.add(name)

    def _on_set_tuning_params(self, params) -> SetParametersResult:
        """Applies parameter updates to the shared config so they take effect live."""

        from dataclasses import is_dataclass

        for param in params:
            name = param.name
            if name not in getattr(self, "_tuning_names", set()):
                continue
            parts = name.split(".")
            obj = getattr(self.config, parts[0], None)
            ok = obj is not None
            for key in parts[1:-1]:
                if isinstance(obj, dict):
                    obj = obj.get(key)
                elif is_dataclass(obj):
                    obj = getattr(obj, key, None)
                else:
                    obj = None
                if obj is None:
                    ok = False
                    break
            if not ok:
                continue
            leaf = parts[-1]
            value = param.value
            if isinstance(obj, dict):
                if leaf in obj:
                    obj[leaf] = self._cast_like(obj[leaf], value)
            elif is_dataclass(obj) and hasattr(obj, leaf):
                setattr(obj, leaf, self._cast_like(getattr(obj, leaf), value))
        return SetParametersResult(successful=True)

    @staticmethod
    def _cast_like(current, value):
        """Coerces an incoming ROS parameter value to the current field's type."""

        if isinstance(current, bool):
            return bool(value)
        if isinstance(current, int) and not isinstance(current, bool):
            return int(value)
        if isinstance(current, float):
            return float(value)
        if isinstance(current, str):
            return str(value)
        if isinstance(current, list):
            return list(value)
        return value

    def publish_detection_status(self) -> None:
        """Publishes compact detection flags for PC-side echo monitoring."""

        classes = {det.cls for det in self.latest_detections}
        self.detect_green_pub.publish(Bool(data="traffic_green" in classes))
        self.detect_red_pub.publish(Bool(data="traffic_red" in classes))
        left = int("sign_left" in classes)
        right = int("sign_right" in classes)
        self.detect_sign_pub.publish(String(data=f"left={left} right={right}"))
        ids = sorted({m.id for m in self.latest_markers})
        self.detect_aruco_pub.publish(String(data=f"ids={ids}" if ids else "none"))

    def publish_mission_status(self) -> None:
        """Publishes FSM state and rotary diagnostics for PC-side echo monitoring."""

        self.mission_state_pub.publish(String(data=self.fsm.state))
        self.rotary_progress_pub.publish(Float32(data=float(getattr(self.fsm, "rotary_progress", 0.0))))
        self.exit_line_count_pub.publish(Int32(data=int(getattr(self.fsm, "exit_line_count", 0))))
        self.rotary_exit_seen_pub.publish(Bool(data=bool(self.latest_lane.rotary_exit_seen)))

    def publish_debug_overlay(self) -> None:
        """Draws the overlay on the latest frame and publishes it as JPEG."""

        if self.last_frame is None:
            return
        light_roi = (
            self.config.roi.detector_light if self.config.traffic_light.enabled else None
        )
        overlay = visualization.draw_overlay(
            self.last_frame,
            self.lane_perception.last_viz,
            self.latest_detections,
            self.latest_markers,
            self.last_cmd,
            self.fsm.state,
            target_id=self.config.aruco.target_id,
            light_roi=light_roi,
            light_states=self.light_analyzer.last_states,
        )
        ok, encoded = cv2.imencode(".jpg", overlay)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.debug_image_pub.publish(msg)

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
        self.latest_lane = self.lane_perception.compute_lane_obs(
            frame, collect_viz=self.publish_debug_image
        )
        # Hand the newest frame to the background inference thread, replacing any
        # frame it has not consumed yet so heavy YOLO never blocks this callback.
        with self._infer_lock:
            self._infer_frame = frame

        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.latest_markers = self.aruco_detector.detect(frame, now_sec)

    def _inference_worker(self) -> None:
        """Runs YOLO off the executor thread so camera/control stay responsive."""

        while not self._infer_stop:
            with self._infer_lock:
                frame = self._infer_frame
                self._infer_frame = None
            if frame is None:
                time.sleep(0.005)
                continue
            # Optional CLAHE/color-correction preprocessing shared by the YOLO
            # detector and the HSV traffic-light analyzer (no-op when disabled).
            proc = traffic_light.preprocess_frame(frame, self.config.color_correction)
            now_sec = self.get_clock().now().nanoseconds / 1e9
            previous_infer_time = self.detector.last_infer_time
            detections = self.detector.infer(proc, now_sec)
            if self.detector.last_infer_time != previous_infer_time:
                # Inference actually ran; refresh buffer and status snapshot.
                # Fold in HSV traffic-light detections so they share the vote path.
                detections = detections + self.light_analyzer.detect(proc)
                self.det_buffer.push(detections)
                self.latest_detections = detections

    def publish_control(self, cmd: ControlCmd) -> None:
        """Publishes clamped normalized throttle/steering to the /control topic."""

        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steering = float(clamp(cmd.steering, -1.0, 1.0))
        msg.throttle = float(clamp(cmd.throttle, 0.0, self.config.throttle.speed_max))
        self.control_pub.publish(msg)

    def target_marker_visible(self) -> bool:
        """Returns True while the configured target ArUco marker is in view."""

        if not self.config.aruco.enabled:
            return False
        target = self.config.aruco.target_id
        return any(marker.id == target for marker in self.latest_markers)

    def control_loop(self) -> None:
        """Runs the mission FSM at a fixed rate and publishes one control command."""

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.target_marker_visible():
            # Target ArUco marker in view: stop immediately and hold the FSM in
            # place. The mission resumes from the same state once it disappears.
            self.controller.prev_throttle = 0.0
            cmd = ControlCmd(0.0, self.last_cmd.steering)
        else:
            cmd = self.fsm.step(self.latest_lane, self.det_buffer, now_sec)
        self.last_cmd = cmd
        self.publish_control(cmd)
        # Status flags and the debug overlay are published here at control_hz,
        # not once per camera frame. Detections/markers only refresh at
        # inference/aruco rate, so per-frame publishing re-sent identical data
        # (and re-encoded the overlay JPEG) ~3x, starving the single-threaded
        # executor and the control loop.
        self.publish_detection_status()
        self.publish_mission_status()
        if self.publish_debug_image:
            self.publish_debug_overlay()
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
        rotary_extra = ""
        if getattr(self.fsm, "exit_line_count", None) is not None:
            rotary_extra = (
                f" prog={getattr(self.fsm, 'rotary_progress', 0.0):.2f} "
                f"lines={getattr(self.fsm, 'exit_line_count', 0)} "
                f"lock={int(getattr(self.fsm, 'line_lockout', False))}"
            )
        self.get_logger().info(
            f"state={self.fsm.state} throttle={cmd.throttle:.2f} steering={cmd.steering:.2f} "
            f"lane_valid={lane.valid} err={lane.center_error:.2f} curv={lane.curvature:.2f} "
            f"fork={lane.fork_seen} rotary={lane.rotary_seen}/{lane.rotary_exit_seen}{rotary_extra}"
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
        node._infer_stop = True
        node._infer_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
