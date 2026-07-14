"""Single ROS2 node that runs D-Racer perception, mission FSM, and /control output."""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from multiprocessing import shared_memory
from pathlib import Path

import cv2
import numpy as np
import rclpy
from control_msgs.msg import Control
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from . import traffic_light, visualization
from .aruco_detector import ArucoDetector
from .dracer_config import load_config, resolve_package_relative_path
from .inference_process import run_inference_process
from .lane_perception import LaneObs, LanePerception, clamp
from .mission_controller import (
    ControlCmd,
    LaneController,
    fresh_light_state,
    make_course_fsm,
)
from .object_detector import DetectionBuffer


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
        self.declare_parameter("lane_mask_topic", "/bisa/debug/lane_mask/compressed")
        self.declare_parameter("debug_image_hz", 5.0)
        self.declare_parameter("debug_jpeg_quality", 70)
        self.declare_parameter("opencv_num_threads", 1)

        config_file = str(self.get_parameter("config_file").value)
        route_mode = str(self.get_parameter("route_mode").value).strip()
        model_path = str(self.get_parameter("model_path").value).strip()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)
        self.debug_log = as_bool(self.get_parameter("debug_log").value)
        self.publish_debug_image = as_bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.lane_mask_topic = str(self.get_parameter("lane_mask_topic").value)
        self.debug_image_hz = min(
            5.0, max(0.1, float(self.get_parameter("debug_image_hz").value))
        )
        self.debug_jpeg_quality = max(
            1, min(100, int(self.get_parameter("debug_jpeg_quality").value))
        )
        self.opencv_num_threads = max(
            1, int(self.get_parameter("opencv_num_threads").value)
        )
        cv2.setNumThreads(self.opencv_num_threads)

        self.config = load_config(config_file)
        if route_mode:
            self.config.mission.route_mode = route_mode.upper()
        if model_path:
            self.config.detector.model_path = model_path
        resolved_model = resolve_package_relative_path(__file__, self.config.detector.model_path)
        self.resolved_model = resolved_model

        self.lane_perception = LanePerception(self.config)
        # 30 Hz inference with up to 36-frame consecutive windows needs >36
        # frames of history; 90 frames = 3 s of headroom.
        self.det_buffer = DetectionBuffer(maxlen=90)
        self.aruco_detector = ArucoDetector(self.config)
        self.controller = LaneController(self.config)
        self.fsm = make_course_fsm(self.config, self.controller)
        self.latest_lane = LaneObs(valid=False)
        self._lane_reset_requested = False
        # Phase-gated detections that feed the FSM vote buffer and status topics.
        self.fsm_detections = []
        # Pre-gate snapshot for the Detect View only: shows every model
        # detection (incl. traffic lights the FSM is currently ignoring) so
        # the model's raw behavior stays visible while tuning. Never feeds
        # the vote buffer or the status topics.
        self.raw_detections = []
        # Raw classify_light state ("green"/"red"/None) on the latest frame's
        # light boxes, ignoring mission phase — drives the /detect_green and
        # /detect_red monitor topics exactly as the tuner reports it.
        self.light_state = None
        # Atomic tuple: (state, unique inference sequence, ROS timestamp sec).
        # Consumers never observe a state from one inference with the sequence
        # or timestamp from another.
        self._light_snapshot = (None, 0, 0.0)
        self._inference_seq = 0
        # Atomic tuple: (ready, human-readable backend status).
        self._detector_status = (False, "starting")
        self.latest_markers = []
        self.last_cmd = ControlCmd(0.0, 0.0)
        self.last_frame = None
        self.last_log_time = 0.0
        self.last_debug_error_log_time = 0.0
        self._next_viz_collect_time = 0.0
        self.has_started = False
        self.final_red_stop_latched = False
        self._final_red_streak = 0
        self._last_final_red_seq = None
        self.aruco_target_visible = False
        self.aruco_seen_streak = 0
        self.aruco_clear_streak = 0
        self.aruco_stop_active = False
        self._last_aruco_seq = 0
        self._aruco_pause_start = None

        # NCNN's Python binding holds the GIL during the ~31 s first Vulkan build.
        # A thread therefore freezes every ROS Python callback despite the
        # MultiThreadedExecutor. Keep all model work in a spawned process and
        # exchange only the newest frame through one fixed shared-memory slot.
        self._mp_context = mp.get_context("spawn")
        self._infer_process = None
        self._infer_shm = None
        self._infer_array = None
        self._infer_shape = None
        self._infer_frame_lock = None
        self._infer_frame_sequence = None
        self._infer_frame_stamp = None
        self._infer_stop_event = None
        self._infer_result_queue = None
        self._infer_config_queue = None
        self._infer_last_start = 0.0

        # Separate callback groups so image processing and control run in
        # parallel threads under MultiThreadedExecutor. Each group is
        # MutuallyExclusive so the heavy image_callback never re-enters itself,
        # and likewise for control_loop — but image and control CAN overlap.
        self._cb_image = MutuallyExclusiveCallbackGroup()
        self._cb_control = MutuallyExclusiveCallbackGroup()
        self._cb_debug = MutuallyExclusiveCallbackGroup()

        # BEST_EFFORT + shallow queue: always process the newest camera frame and
        # drop stale ones, matching the camera publisher and avoiding WiFi lag.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, self.image_topic, self.image_callback, image_qos,
                                 callback_group=self._cb_image)
        self.control_pub = self.create_publisher(Control, self.control_topic, 10)

        # Mission/detection status topics for PC-side `ros2 topic echo`.
        self.detect_green_pub = self.create_publisher(Bool, "detect_green", 10)
        self.detect_red_pub = self.create_publisher(Bool, "detect_red", 10)
        self.detect_sign_pub = self.create_publisher(String, "detect_sign", 10)
        self.detect_aruco_pub = self.create_publisher(String, "detect_aruco", 10)
        self.detector_ready_pub = self.create_publisher(
            Bool, "/bisa/detector/ready", 10
        )
        self.detector_status_pub = self.create_publisher(
            String, "/bisa/detector/status", 10
        )
        self.status_pub = self.create_publisher(String, "/bisa/status", 10)
        # Debug JPEG stream reuses the camera's BEST_EFFORT/depth-1 profile so it
        # drops stale frames instead of triggering RELIABLE retransmits, which
        # caused WiFi head-of-line blocking against the viz/monitor subscribers.
        self.debug_image_pub = self.create_publisher(
            CompressedImage, self.debug_image_topic, image_qos
        )
        # Second debug stream: the ROI-sized binarized lane mask with lane
        # overlays, for threshold/ROI tuning in its own window on the PC.
        self.lane_mask_pub = self.create_publisher(
            CompressedImage, self.lane_mask_topic, image_qos
        )

        # Expose tunable config as flat dotted ROS parameters and apply live edits.
        self._declare_tuning_params()
        self.add_on_set_parameters_callback(self._on_set_tuning_params)

        self.timer = self.create_timer(1.0 / max(self.config.mission.control_hz, 1.0), self.control_loop,
                                       callback_group=self._cb_control)
        self.debug_timer = None
        if self.publish_debug_image:
            self.debug_timer = self.create_timer(
                1.0 / self.debug_image_hz,
                self.debug_loop,
                callback_group=self._cb_debug,
            )
        self.get_logger().info(
            "BISA autonomous node started: "
            f"route={self.config.mission.route_mode}, image_topic={self.image_topic}, "
            f"control_topic={self.control_topic}, model={resolved_model}, config={config_file}, "
            f"opencv_threads={cv2.getNumThreads()}"
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
            if self.has_parameter(name):
                self._tuning_names.add(name)
                continue
            param = self.declare_parameter(name, value)
            self._tuning_names.add(name)
            # A launch/CLI override resolves declare_parameter to a value that
            # differs from the config default. The on-set callback is not
            # registered yet at declaration time, so it never fires for these
            # initial overrides; push them into self.config here. This is what
            # makes onboard.launch's CPU overrides (imgsz/inference_hz/device/…)
            # actually take effect without editing the shared YAML.
            if param.value != value:
                self._apply_tuning_value(name, param.value)

    def _apply_tuning_value(self, name: str, value) -> None:
        """Writes one flat dotted config value back into the shared config."""

        from dataclasses import is_dataclass

        if name not in getattr(self, "_tuning_names", set()):
            return
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
            return
        leaf = parts[-1]
        if isinstance(obj, dict):
            if leaf in obj:
                obj[leaf] = self._cast_like(obj[leaf], value)
        elif is_dataclass(obj) and hasattr(obj, leaf):
            setattr(obj, leaf, self._cast_like(getattr(obj, leaf), value))

    def _on_set_tuning_params(self, params) -> SetParametersResult:
        """Applies parameter updates to the shared config so they take effect live."""

        for param in params:
            self._apply_tuning_value(param.name, param.value)
        if self._infer_config_queue is not None:
            try:
                while True:
                    self._infer_config_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._infer_config_queue.put_nowait(self.config)
            except queue.Full:
                pass
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

    def _fresh_light_snapshot(self, now_sec: float) -> tuple[str | None, int]:
        """Returns the latest light verdict only while it is detector-fresh."""

        return fresh_light_state(
            self._light_snapshot,
            now_sec,
            self.config.detector.light_stale_sec,
        )

    def _set_detector_status(self, ready: bool, status: str) -> None:
        """Atomically updates readiness consumed by the ROS status publishers."""

        self._detector_status = (bool(ready), str(status))

    def publish_detection_status(self, now_sec: float) -> None:
        """Publishes compact detection flags for PC-side echo monitoring."""

        # Traffic light: the raw classify_light state, mutually exclusive —
        # GREEN -> green=True/red=False, RED -> green=False/red=True, else both
        # False. Independent of the mission-phase gating used for driving.
        state, _ = self._fresh_light_snapshot(now_sec)
        self.detect_green_pub.publish(Bool(data=(state == "green")))
        self.detect_red_pub.publish(Bool(data=(state == "red")))
        classes = {det.cls for det in self.fsm_detections}
        left = int("sign_left" in classes)
        right = int("sign_right" in classes)
        self.detect_sign_pub.publish(String(data=f"left={left} right={right}"))
        ids = sorted({m.id for m in self.latest_markers})
        self.detect_aruco_pub.publish(String(data=f"ids={ids}" if ids else "none"))
        ready, status = self._detector_status
        self.detector_ready_pub.publish(Bool(data=ready))
        self.detector_status_pub.publish(String(data=status))
        self.status_pub.publish(String(data=self._status_text(now_sec, self.last_cmd)))

    def _publish_jpeg(self, publisher, image) -> None:
        """Encodes one BGR image as JPEG and publishes it on the given topic."""

        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.debug_jpeg_quality],
        )
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        publisher.publish(msg)

    def publish_debug_overlay(self) -> None:
        """Publishes the full-frame detect view and the ROI lane-mask view.

        The detect view is drawn on the color-corrected frame (the same
        preprocessing the detector sees), so the CLAHE/saturation/brightness
        sliders change the picture live. The lane-mask view shows the binarized
        ROI the lane pipeline works on, for threshold and ROI-size tuning.
        """

        if self.last_frame is None:
            return
        # Detect View always shows the corrected frame so the CLAHE/saturation/
        # brightness sliders are visible live, regardless of whether the
        # detector is actually consuming the correction (cfg.enabled).
        frame = traffic_light.apply_correction_chain(self.last_frame, self.config.color_correction)
        # Per-box classify_light verdict so the overlay colors traffic-light
        # boxes exactly like the tuner (by verdict, not the raw YOLO class).
        light_verdicts = {
            id(det): traffic_light.classify_light(frame, det.bbox, self.config)[0]
            for det in self.raw_detections
            if det.cls in ("traffic_green", "traffic_red")
        }
        overlay = visualization.draw_overlay(
            frame,
            self.lane_perception.last_viz,
            self.raw_detections,
            self.latest_markers,
            self.last_cmd,
            self.fsm.state,
            target_id=self.config.aruco.target_id,
            light_roi=self.config.roi.detector_light,
            light_verdicts=light_verdicts,
            accepted_ids={id(d) for d in self.fsm_detections},
            cc_active=bool(self.config.color_correction.enabled),
        )
        # The live traffic-light state driving /detect_green|/detect_red, so the
        # operator can see exactly what the mission is reacting to.
        now_sec = self.get_clock().now().nanoseconds / 1e9
        state, _ = self._fresh_light_snapshot(now_sec)
        vtxt = {"green": "GREEN", "red": "RED"}.get(state, "none")
        vcolor = {"green": (0, 255, 0), "red": (0, 0, 255)}.get(state, (170, 170, 170))
        cv2.putText(overlay, f"light: {vtxt}", (8, overlay.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, f"light: {vtxt}", (8, overlay.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, vcolor, 2, cv2.LINE_AA)
        self._publish_jpeg(self.debug_image_pub, overlay)

        mask_view = visualization.draw_lane_mask_view(
            self.lane_perception.last_viz, self.last_cmd
        )
        if mask_view is not None:
            self._publish_jpeg(self.lane_mask_pub, mask_view)

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
        now_sec = self.get_clock().now().nanoseconds / 1e9
        collect_viz = (
            self.publish_debug_image and now_sec >= self._next_viz_collect_time
        )
        if collect_viz:
            self._next_viz_collect_time = now_sec + 1.0 / self.debug_image_hz
        if self._lane_reset_requested:
            self.lane_perception.reset_fork_history()
            self._lane_reset_requested = False
        self.latest_lane = self.lane_perception.compute_lane_obs(
            frame, collect_viz=collect_viz
        )
        self._submit_inference_frame(frame, now_sec)
        self.latest_markers = self.aruco_detector.detect(frame, now_sec)

    def _gate_classified_detections(self, detections):
        """Apply mission-phase gates to light labels classified in the child."""

        listen_green = self.fsm.state.endswith("WAIT_GREEN")
        listen_red = self.has_started and not self.final_red_stop_latched
        kept = []
        for detection in detections:
            if detection.cls == "traffic_green" and not listen_green:
                continue
            if detection.cls == "traffic_red" and not listen_red:
                continue
            kept.append(detection)
        return kept

    def _create_inference_resources(self, frame: np.ndarray) -> None:
        """Allocate one fixed latest-frame shared-memory slot and IPC controls."""

        self._infer_shape = tuple(frame.shape)
        self._infer_shm = shared_memory.SharedMemory(create=True, size=frame.nbytes)
        self._infer_array = np.ndarray(
            self._infer_shape, dtype=np.uint8, buffer=self._infer_shm.buf
        )
        self._infer_frame_lock = self._mp_context.Lock()
        self._infer_frame_sequence = self._mp_context.Value("Q", 0, lock=False)
        self._infer_frame_stamp = self._mp_context.Value("d", 0.0, lock=False)
        self._infer_stop_event = self._mp_context.Event()
        self._infer_result_queue = self._mp_context.Queue(maxsize=8)
        self._infer_config_queue = self._mp_context.Queue(maxsize=1)

    def _start_inference_process(self) -> None:
        """Spawn or restart the isolated detector against existing shared memory."""

        now = time.monotonic()
        if now - self._infer_last_start < 2.0:
            return
        self._infer_last_start = now
        if self._infer_stop_event.is_set():
            self._infer_stop_event.clear()
        self._infer_process = self._mp_context.Process(
            target=run_inference_process,
            name="bisa_inference",
            daemon=True,
            args=(
                self._infer_shm.name,
                self._infer_shape,
                self._infer_frame_lock,
                self._infer_frame_sequence,
                self._infer_frame_stamp,
                self._infer_stop_event,
                self._infer_result_queue,
                self._infer_config_queue,
                self.config,
                self.resolved_model,
                self.opencv_num_threads,
            ),
        )
        self._infer_process.start()
        self._set_detector_status(
            False, f"process starting pid={self._infer_process.pid}"
        )

    def _submit_inference_frame(self, frame: np.ndarray, stamp: float) -> None:
        """Overwrite the shared slot and ensure a detector process is alive."""

        if self._infer_shm is None:
            self._create_inference_resources(frame)
        elif tuple(frame.shape) != self._infer_shape:
            self.shutdown_inference_process()
            self._create_inference_resources(frame)
        with self._infer_frame_lock:
            np.copyto(self._infer_array, frame)
            self._infer_frame_stamp.value = float(stamp)
            self._infer_frame_sequence.value += 1
        if self._infer_process is None or not self._infer_process.is_alive():
            if self._infer_process is not None:
                self._set_detector_status(
                    False, f"process exited code={self._infer_process.exitcode}; restarting"
                )
            self._start_inference_process()

    def _drain_inference_results(self) -> None:
        """Consume detector status/results and monitor unexpected process exits."""

        if self._infer_result_queue is not None:
            while True:
                try:
                    message = self._infer_result_queue.get_nowait()
                except queue.Empty:
                    break
                message_type = message.get("type")
                if message_type == "status":
                    self._set_detector_status(message["ready"], message["status"])
                elif message_type == "metrics" and self.debug_log:
                    self.get_logger().info(
                        f"YOLO infer: {message['average_ms']:.0f} ms/frame, "
                        f"{message['fps']:.1f} FPS effective "
                        f"(imgsz={message['imgsz']}, hz_cap={message['hz_cap']}, "
                        f"device={message['device']})"
                    )
                elif message_type == "result":
                    self.raw_detections = message["raw_detections"]
                    gated = self._gate_classified_detections(
                        message["classified_detections"]
                    )
                    self.det_buffer.push(gated)
                    self.fsm_detections = gated
                    self.light_state = message["light_state"]
                    self._inference_seq = int(message["sequence"])
                    self._light_snapshot = (
                        self.light_state,
                        self._inference_seq,
                        float(message["stamp"]),
                    )
        if (
            self._infer_process is not None
            and not self._infer_process.is_alive()
            and self._infer_process.exitcode is not None
        ):
            self.light_state = None
            self._light_snapshot = (None, self._inference_seq, 0.0)
            self._set_detector_status(
                False, f"process exited code={self._infer_process.exitcode}"
            )

    def shutdown_inference_process(self) -> None:
        """Stop the detector process and release all shared-memory resources."""

        if self._infer_stop_event is not None:
            self._infer_stop_event.set()
        if self._infer_process is not None:
            self._infer_process.join(timeout=1.0)
            if self._infer_process.is_alive():
                self._infer_process.terminate()
                self._infer_process.join(timeout=1.0)
            if self._infer_process.is_alive():
                self._infer_process.kill()
                self._infer_process.join(timeout=1.0)
            self._infer_process.close()
        if self._infer_result_queue is not None:
            self._infer_result_queue.close()
        if self._infer_config_queue is not None:
            self._infer_config_queue.close()
        if self._infer_shm is not None:
            self._infer_shm.close()
            try:
                self._infer_shm.unlink()
            except FileNotFoundError:
                pass
        self._infer_process = None
        self._infer_shm = None
        self._infer_array = None
        self._infer_shape = None
        self._infer_frame_lock = None
        self._infer_frame_sequence = None
        self._infer_frame_stamp = None
        self._infer_stop_event = None
        self._infer_result_queue = None
        self._infer_config_queue = None

    def publish_control(self, cmd: ControlCmd) -> None:
        """Publishes clamped normalized throttle/steering to the /control topic."""

        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steering = float(clamp(cmd.steering, -1.0, 1.0))
        msg.throttle = float(clamp(cmd.throttle, 0.0, self.config.throttle.speed_max))
        self.control_pub.publish(msg)

    def _target_marker_visible(self, markers) -> bool:
        """Returns True while the configured target ArUco marker is in view."""

        if not self.config.aruco.enabled:
            return False
        target = self.config.aruco.target_id
        return any(marker.id == target for marker in markers)

    def update_global_aruco_stop(self, markers, marker_seq: int, now_sec: float) -> bool:
        """Debounces target-marker visibility into a global pause command."""

        visible = self._target_marker_visible(markers)
        self.aruco_target_visible = visible
        if marker_seq != self._last_aruco_seq:
            self._last_aruco_seq = marker_seq
            if visible:
                self.aruco_seen_streak = min(
                    self.aruco_seen_streak + 1, self.config.aruco.confirm_frames
                )
                self.aruco_clear_streak = 0
            else:
                self.aruco_clear_streak = min(
                    self.aruco_clear_streak + 1, self.config.aruco.clear_frames
                )
                self.aruco_seen_streak = 0

        if not self.aruco_stop_active and self.aruco_seen_streak >= self.config.aruco.confirm_frames:
            self.aruco_stop_active = True
            self._aruco_pause_start = now_sec
        elif self.aruco_stop_active and self.aruco_clear_streak >= self.config.aruco.clear_frames:
            self.aruco_stop_active = False
            if self._aruco_pause_start is not None:
                self.fsm.enter_t += max(0.0, now_sec - self._aruco_pause_start)
            self._aruco_pause_start = None
        return self.aruco_stop_active

    def update_final_red_stop(self, light_state, light_seq) -> bool:
        """Latches a final red stop after the vehicle has started once."""

        if self.final_red_stop_latched:
            return True
        if not self.has_started:
            self._final_red_streak = 0
            return False
        if light_state != "red":
            self._final_red_streak = 0
            return False
        if light_seq == self._last_final_red_seq:
            return False
        self._last_final_red_seq = light_seq
        self._final_red_streak += 1
        if self._final_red_streak >= self.config.detector.light_confirm_frames:
            self.final_red_stop_latched = True
        return self.final_red_stop_latched

    def control_loop(self) -> None:
        """Runs the mission FSM at a fixed rate and publishes one control command."""

        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._drain_inference_results()
        # Snapshot shared state written by image_callback on another thread.
        # Python reference assignment is atomic, so these reads are safe without
        # a lock — the worst case is reading the previous frame's value.
        lane = self.latest_lane
        markers = self.latest_markers
        light_state, light_seq = self._fresh_light_snapshot(now_sec)
        final_red_stop = self.update_final_red_stop(light_state, light_seq)
        aruco_stop = self.update_global_aruco_stop(
            markers, self.aruco_detector.last_sequence, now_sec
        )

        if final_red_stop:
            self.controller.prev_throttle = 0.0
            cmd = ControlCmd(0.0, self.last_cmd.steering)
        elif aruco_stop:
            self.controller.prev_throttle = 0.0
            cmd = ControlCmd(0.0, self.last_cmd.steering)
        else:
            cmd = self.fsm.step(lane, self.det_buffer, now_sec,
                                 light_state=light_state, light_seq=light_seq)
            if not self.has_started and not self.fsm.state.endswith("WAIT_GREEN"):
                self.has_started = True
            if self.fsm.consume_lane_reset_request():
                self._lane_reset_requested = True
        self.last_cmd = cmd
        self.publish_control(cmd)
        # Keep control deterministic: visualization and its two JPEG encodes run
        # on a separate low-rate timer/callback group.
        self.publish_detection_status(now_sec)
        self.log_status(now_sec, cmd)

    def debug_loop(self) -> None:
        """Publishes tuning images without blocking the 20 Hz control loop."""

        if self.publish_debug_image:
            try:
                self.publish_debug_overlay()
            except (cv2.error, TypeError, ValueError) as exc:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if now_sec - self.last_debug_error_log_time >= 2.0:
                    self.last_debug_error_log_time = now_sec
                    self.get_logger().warning(
                        f"Skipping debug frame after visualization error: {exc}"
                    )

    def log_status(self, now_sec: float, cmd: ControlCmd) -> None:
        """Logs compact state/perception/control diagnostics at debug_log_hz."""

        if not self.debug_log:
            return
        period = 1.0 / max(self.config.mission.debug_log_hz, 0.1)
        if now_sec - self.last_log_time < period:
            return
        self.last_log_time = now_sec
        lane = self.latest_lane
        # Raw (ROI-gated) traffic-light boxes the detector currently sees, so it
        # is obvious whether a missing verdict is "no detection" vs "wrong color".
        n_light = sum(
            1 for d in self.raw_detections
            if d.cls in ("traffic_green", "traffic_red")
        )
        light_state, _ = self._fresh_light_snapshot(now_sec)
        ready, detector_status = self._detector_status
        self.get_logger().info(
            f"{self._status_text(now_sec, cmd)} detector_ready={ready} "
            f"lights={n_light} "
            f"lane_valid={lane.valid} err={lane.center_error:.2f} "
            f"detector_status='{detector_status}'"
        )

    def _status_text(self, now_sec: float, cmd: ControlCmd) -> str:
        light_state, _ = self._fresh_light_snapshot(now_sec)
        return (
            f"current_fsm_state={self.fsm.state} "
            f"aruco_target_visible={self.aruco_target_visible} "
            f"aruco_seen_streak={self.aruco_seen_streak} "
            f"aruco_clear_streak={self.aruco_clear_streak} "
            f"aruco_stop_active={self.aruco_stop_active} "
            f"traffic_light_state={light_state} "
            f"final_red_stop_latched={self.final_red_stop_latched} "
            f"last_control_throttle={cmd.throttle:.2f} "
            f"last_control_steering={cmd.steering:.2f}"
        )


def main(args=None) -> None:
    """Initializes rclpy and spins the BISA autonomous node."""

    rclpy.init(args=args)
    node = BisaAutonomousNode()
    # MultiThreadedExecutor runs image_callback and control_loop on separate
    # threads so lane perception never blocks the 10 Hz control output.
    # Thread 1: _cb_image (decode + lane + ArUco)
    # Thread 2: _cb_control (FSM + /control publish + debug overlay)
    # Thread 3: _cb_debug (low-rate overlay + JPEG encode)
    # Thread 4: default callback group (parameter services, ROS internals)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_inference_process()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
