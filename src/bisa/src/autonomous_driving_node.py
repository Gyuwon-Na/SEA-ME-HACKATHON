"""Single ROS2 node that runs D-Racer perception, mission FSM, and /control output."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import rclpy
from control_msgs.msg import Control
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from . import traffic_light, visualization
from .aruco_detector import ArucoDetector
from .dracer_config import load_config, resolve_package_relative_path
from .lane_perception import LaneObs, LanePerception, clamp
from .mission_controller import ControlCmd, LaneController, make_course_fsm
from .object_detector import BestPthDetector, Detection, DetectionBuffer


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
        # debug 뷰 JPEG 품질(0-100). WiFi 전송용으로 낮게(기본 50) — 프레임 크기↓
        self.declare_parameter("debug_jpeg_quality", 50)
        # 외부 detector_node(별도 프로세스) 사용 여부 — true면 내부 YOLO 스레드 대신
        # /bisa/detections 구독(GIL 탈출). 기본 false로 기존 스레드 방식 그대로 보존.
        self.declare_parameter("external_detector", False)
        self.declare_parameter("detections_topic", "/bisa/detections")
        # raw Image 토픽 구독 여부 (True면 sensor_msgs/Image, False면 CompressedImage)
        self.declare_parameter("use_raw_image", False)

        config_file = str(self.get_parameter("config_file").value)
        route_mode = str(self.get_parameter("route_mode").value).strip()
        model_path = str(self.get_parameter("model_path").value).strip()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)
        self.debug_log = as_bool(self.get_parameter("debug_log").value)
        self.publish_debug_image = as_bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.lane_mask_topic = str(self.get_parameter("lane_mask_topic").value)
        self.debug_jpeg_quality = int(self.get_parameter("debug_jpeg_quality").value)
        self.external_detector = as_bool(self.get_parameter("external_detector").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.use_raw_image = as_bool(self.get_parameter("use_raw_image").value)

        self.config = load_config(config_file)
        if route_mode:
            self.config.mission.route_mode = route_mode.upper()
        if model_path:
            self.config.detector.model_path = model_path
        resolved_model = resolve_package_relative_path(__file__, self.config.detector.model_path)

        self.lane_perception = LanePerception(self.config)
        # 30 Hz inference with up to 36-frame consecutive windows needs >36
        # frames of history; 90 frames = 3 s of headroom.
        self.det_buffer = DetectionBuffer(maxlen=90)
        self.detector = BestPthDetector(self.config, resolved_model, logger=self.get_logger())
        self.aruco_detector = ArucoDetector(self.config)
        self.controller = LaneController(self.config)
        self.fsm = make_course_fsm(self.config, self.controller)
        self.latest_lane = LaneObs(valid=False)
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
        self.latest_markers = []
        self.last_cmd = ControlCmd(0.0, 0.0)
        self.last_frame = None
        self.last_log_time = 0.0

        # 2단 파이프라인(더블 버퍼링):
        #   Stage1(_preprocess_worker): raw 프레임 → CLAHE/색보정 → proc 버퍼
        #   Stage2(_inference_worker):  proc 버퍼 → Vulkan 추론(GPU)
        # ncnn Vulkan predict는 GPU 연산 대기 중 GIL을 해제하므로, Stage2가
        # 추론(104ms)하는 동안 Stage1이 다음 프레임 CLAHE(30ms)를 겹쳐서 처리함
        # → 사이클이 max(104,30)≈104ms로 단축(순차 135ms 대비). 각 버퍼는 최신
        # 1개만 유지하고 밀린 프레임은 버려 항상 최신을 처리.
        self._infer_stop = False
        # Stage1 입력: 최신 raw 프레임 (image_callback이 write)
        self._raw_lock = threading.Lock()
        self._raw_frame = None
        # Stage1→Stage2 더블 버퍼: (proc, raw) 최신 1개
        self._proc_lock = threading.Lock()
        self._proc_buf = None
        self._preprocess_thread = threading.Thread(
            target=self._preprocess_worker, name="bisa_preprocess", daemon=True
        )
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
        if self.use_raw_image:
            self.create_subscription(Image, self.image_topic, self.image_callback_raw, image_qos)
        else:
            self.create_subscription(CompressedImage, self.image_topic, self.image_callback, image_qos)
        self.control_pub = self.create_publisher(Control, self.control_topic, 10)

        # Mission/detection status topics for PC-side `ros2 topic echo`.
        self.detect_green_pub = self.create_publisher(Bool, "detect_green", 10)
        self.detect_red_pub = self.create_publisher(Bool, "detect_red", 10)
        self.detect_sign_pub = self.create_publisher(String, "detect_sign", 10)
        self.detect_aruco_pub = self.create_publisher(String, "detect_aruco", 10)
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

        self.timer = self.create_timer(1.0 / max(self.config.mission.control_hz, 1.0), self.control_loop)
        # 외부 detector 모드면 내부 YOLO 스레드 대신 검출 토픽을 구독(GIL 탈출).
        if self.external_detector:
            self.create_subscription(
                String, self.detections_topic, self.detections_callback, 10
            )
            self.get_logger().info(
                f"External detector mode ON: subscribing {self.detections_topic} "
                "(internal YOLO thread disabled)"
            )
        else:
            self._preprocess_thread.start()
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

        # Traffic light: the raw classify_light state, mutually exclusive —
        # GREEN -> green=True/red=False, RED -> green=False/red=True, else both
        # False. Independent of the mission-phase gating used for driving.
        state = self.light_state
        self.detect_green_pub.publish(Bool(data=(state == "green")))
        self.detect_red_pub.publish(Bool(data=(state == "red")))
        classes = {det.cls for det in self.fsm_detections}
        left = int("sign_left" in classes)
        right = int("sign_right" in classes)
        self.detect_sign_pub.publish(String(data=f"left={left} right={right}"))
        ids = sorted({m.id for m in self.latest_markers})
        self.detect_aruco_pub.publish(String(data=f"ids={ids}" if ids else "none"))

    def _publish_jpeg(self, publisher, image) -> None:
        """Encodes one BGR image as JPEG and publishes it on the given topic.

        debug_jpeg_quality로 인코딩 품질 조절(기본 50). WiFi로 PC 뷰어에 보낼
        때 quality를 낮추면 프레임 크기가 ~1/3로 줄어 드롭이 크게 감소함.
        검출은 보드에서 원본 프레임으로 하므로 이 압축은 뷰 전송에만 영향.
        """
        ok, encoded = cv2.imencode(
            ".jpg", image,
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
        state = self.light_state
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

    def _process_frame(self, frame: np.ndarray) -> None:
        """공통 프레임 처리 — compressed/raw 콜백 양쪽에서 호출."""

        self.last_frame = frame
        self.latest_lane = self.lane_perception.compute_lane_obs(
            frame, collect_viz=self.publish_debug_image
        )
        # Stage1(전처리 스레드)에 최신 raw 프레임 전달. 밀린 프레임은 덮어써
        # 항상 최신만 처리(더블 버퍼링의 입력 슬롯).
        with self._raw_lock:
            self._raw_frame = frame
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.latest_markers = self.aruco_detector.detect(frame, now_sec)

    def image_callback(self, msg: CompressedImage) -> None:
        """Processes the newest camera frame for lane and async object perception."""

        frame = self.decode_image(msg)
        if frame is None:
            return
        self._process_frame(frame)

    def image_callback_raw(self, msg: Image) -> None:
        """sensor_msgs/Image (bgr8) 수신 — JPEG 디코딩 없이 바로 numpy 배열로 변환."""

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding != "bgr8":
            self.get_logger().warning(f"Unsupported image encoding: {msg.encoding}")
            return
        self._process_frame(frame)

    def _classify_and_gate(self, detections, frame):
        """Classifies every light box once, returning ``(light_state, gated)``.

        The YOLO class of a light box is unreliable (an unlit red lens still
        looks red), so the model only localizes: :func:`classify_light` splits
        the box into rows and the LIT row decides the class (top=red,
        bottom=green), overriding the YOLO label. That single verdict feeds two
        outputs so ``classify_light`` runs only once per box:

        * ``light_state`` — the raw verdict ("green"/"red"/None) ignoring
          mission phase, mirroring the tuner; drives /detect_green|/detect_red.
          Red wins if both somehow appear (fail-safe toward "stop").
        * ``gated`` — the FSM-facing detection list. Boxes with no clearly lit
          row are dropped; green only survives while waiting to launch, red only
          after the finish window opened, so mid-course false positives never
          reach the vote buffer. Sign/other classes pass through untouched.

        Reads FSM state from the inference thread; a one-frame-stale value is
        harmless here.
        """

        listen_green = self.fsm.state.endswith("WAIT_GREEN")
        listen_red = bool(self.fsm.finish_crossed)
        seen_green = seen_red = False
        kept = []
        for det in detections:
            if det.cls in ("traffic_green", "traffic_red"):
                row_cls, _scores = traffic_light.classify_light(
                    frame, det.bbox, self.config
                )
                if row_cls is None:
                    continue
                seen_green = seen_green or row_cls == "traffic_green"
                seen_red = seen_red or row_cls == "traffic_red"
                det = replace(det, cls=row_cls)
                if det.cls == "traffic_green" and not listen_green:
                    continue
                if det.cls == "traffic_red" and not listen_red:
                    continue
            kept.append(det)
        light_state = "red" if seen_red else "green" if seen_green else None
        return light_state, kept

    def detections_callback(self, msg: String) -> None:
        """외부 detector_node의 검출 결과(JSON)를 받아 색판별·gating 적용.

        내부 `_inference_worker`의 검출-이후 로직과 동일하되, 검출은 별도
        프로세스가 이미 끝냈으므로 여기선 최신 프레임으로 색판별·gating만 함.
        (external_detector=true일 때만 구독됨.)
        """

        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        dets = [
            Detection(
                cls=d["cls"], conf=float(d["conf"]),
                bbox=tuple(d["bbox"]), cx=float(d["cx"]), cy=float(d["cy"]),
            )
            for d in payload.get("dets", [])
        ]
        frame = self.last_frame
        if frame is None:
            return
        # worker와 동일하게 전처리 프레임에 색판별(1~2프레임 오차는 신호등에 무해).
        proc = traffic_light.preprocess_frame(frame, self.config.color_correction)
        self.raw_detections = dets
        self.light_state, gated = self._classify_and_gate(dets, proc)
        self.det_buffer.push(gated)
        self.fsm_detections = gated

    def _preprocess_worker(self) -> None:
        """Stage1: raw 프레임 → CLAHE/색보정 → proc 더블버퍼.

        Stage2가 Vulkan 추론(GPU, GIL 해제) 중일 때 이 CLAHE가 겹쳐 실행됨.
        입력·출력 모두 최신 1개만 유지하고 밀린 프레임은 버림.
        """
        while not self._infer_stop:
            with self._raw_lock:
                frame = self._raw_frame
                self._raw_frame = None
            if frame is None:
                time.sleep(0.002)
                continue
            # CLAHE/색보정 (color_correction 비활성 시 near-noop).
            proc = traffic_light.preprocess_frame(frame, self.config.color_correction)
            with self._proc_lock:
                self._proc_buf = proc

    def _inference_worker(self) -> None:
        """Stage2: proc 버퍼 → Vulkan 추론. GPU 대기 중 GIL 해제로 Stage1과 겹침."""

        # Latency accounting so the CPU-only vehicle's real YOLO throughput is
        # visible in the log — the numbers to tune imgsz/inference_hz against.
        infer_count = 0
        infer_time_sum = 0.0
        last_report = time.perf_counter()
        while not self._infer_stop:
            with self._proc_lock:
                proc = self._proc_buf
                self._proc_buf = None
            if proc is None:
                time.sleep(0.002)
                continue
            now_sec = self.get_clock().now().nanoseconds / 1e9
            previous_infer_time = self.detector.last_infer_time
            infer_start = time.perf_counter()
            detections = self.detector.infer(proc, now_sec)
            if self.detector.last_infer_time != previous_infer_time:
                # Inference actually ran; refresh buffer and status snapshot.
                infer_count += 1
                infer_time_sum += time.perf_counter() - infer_start
                self.raw_detections = detections
                # One classify_light pass yields both the raw monitor state and
                # the phase-gated FSM detections.
                self.light_state, gated = self._classify_and_gate(detections, proc)
                self.det_buffer.push(gated)
                self.fsm_detections = gated
            # 추론 성능 로그 (Vulkan 검증용 — 확인 후 다시 주석 처리)
            if infer_count > 0:
                report_elapsed = time.perf_counter() - last_report
                if report_elapsed >= 5.0:
                    avg_ms = 1000.0 * infer_time_sum / infer_count
                    fps = infer_count / report_elapsed
                    self.get_logger().info(
                        f"YOLO infer: {avg_ms:.0f} ms/frame, {fps:.1f} FPS "
                        f"(imgsz={self.config.detector.imgsz}, "
                        f"hz_cap={self.config.detector.inference_hz}, "
                        f"device={self.detector.device})"
                    )
                    infer_count = 0
                    infer_time_sum = 0.0
                    last_report = time.perf_counter()

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
            cmd = self.fsm.step(self.latest_lane, self.det_buffer, now_sec,
                                 light_state=self.light_state)
        self.last_cmd = cmd
        self.publish_control(cmd)
        # Status flags and the debug overlay are published here at control_hz,
        # not once per camera frame. Detections/markers only refresh at
        # inference/aruco rate, so per-frame publishing re-sent identical data
        # (and re-encoded the overlay JPEG) ~3x, starving the single-threaded
        # executor and the control loop.
        self.publish_detection_status()
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
        # Raw (ROI-gated) traffic-light boxes the detector currently sees, so it
        # is obvious whether a missing verdict is "no detection" vs "wrong color".
        n_light = sum(
            1 for d in self.raw_detections
            if d.cls in ("traffic_green", "traffic_red")
        )
        green_streak = getattr(self.fsm, "_green_streak", 0)
        red_streak = getattr(self.fsm, "_red_streak", 0)
        # self.get_logger().info(
        #     f"state={self.fsm.state} light={self.light_state} "
        #     f"lights={n_light} g_streak={green_streak} r_streak={red_streak} "
        #     f"throttle={cmd.throttle:.2f} steering={cmd.steering:.2f} "
        #     f"lane_valid={lane.valid} err={lane.center_error:.2f}"
        # )


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
        if node._preprocess_thread.is_alive():
            node._preprocess_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()
