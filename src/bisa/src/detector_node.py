"""Process-isolated ROS detector feeding the C++ autonomous core."""

from __future__ import annotations

import copy
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float64MultiArray, String

from .dracer_config import load_config
from .inference_process import _classify_lights
from .object_detector import BestPthDetector
from .traffic_light import preprocess_frame


CLASS_IDS = {
    "traffic_red": 0,
    "traffic_green": 1,
    "sign_left": 2,
    "sign_right": 3,
}

LIVE_PARAMETER_PATHS = {
    "detector.imgsz": ("detector", "imgsz"),
    "detector.inference_hz": ("detector", "inference_hz"),
    "detector.conf.traffic_green": ("detector", "conf", "traffic_green"),
    "detector.conf.traffic_red": ("detector", "conf", "traffic_red"),
    "detector.conf.sign_left": ("detector", "conf", "sign_left"),
    "detector.conf.sign_right": ("detector", "conf", "sign_right"),
    "color_correction.enabled": ("color_correction", "enabled"),
    "color_correction.clahe_clip": ("color_correction", "clahe_clip"),
    "color_correction.saturation_boost": ("color_correction", "saturation_boost"),
    "color_correction.brightness": ("color_correction", "brightness"),
    "color_correction.contrast": ("color_correction", "contrast"),
    "color_correction.saturation": ("color_correction", "saturation"),
    "color_correction.gamma": ("color_correction", "gamma"),
    "traffic_light.green_h_lo": ("traffic_light", "green_h_lo"),
    "traffic_light.green_h_hi": ("traffic_light", "green_h_hi"),
    "traffic_light.green_s_min": ("traffic_light", "green_s_min"),
    "traffic_light.green_v_min": ("traffic_light", "green_v_min"),
    "traffic_light.red_h1_hi": ("traffic_light", "red_h1_hi"),
    "traffic_light.red_h2_lo": ("traffic_light", "red_h2_lo"),
    "traffic_light.red_s_min": ("traffic_light", "red_s_min"),
    "traffic_light.red_v_min": ("traffic_light", "red_v_min"),
    "traffic_light.row_min_ratio": ("traffic_light", "row_min_ratio"),
    "traffic_light.row_lit_v_min": ("traffic_light", "row_lit_v_min"),
    "traffic_light.row_white_s_max": ("traffic_light", "row_white_s_max"),
    "traffic_light.lab_a_red_min": ("traffic_light", "lab_a_red_min"),
    "traffic_light.lab_a_green_max": ("traffic_light", "lab_a_green_max"),
    "traffic_light.lab_l_min": ("traffic_light", "lab_l_min"),
}


def _default_config() -> str:
    return str(Path(get_package_share_directory("bisa")) / "config" / "dracer_params.yaml")


def _default_model() -> str:
    return str(
        Path(get_package_share_directory("bisa"))
        / "checkpoints"
        / "best_ncnn_model"
    )


class BisaDetectorNode(Node):
    """Runs NCNN in its own process and publishes compact detection packets."""

    def __init__(self) -> None:
        super().__init__("bisa_detector_node")
        self.declare_parameter("config_file", _default_config())
        self.declare_parameter("model_path", _default_model())
        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("detections_topic", "/bisa/detections")
        self.declare_parameter("detector.device", "vulkan:0")
        self.declare_parameter("detector.imgsz", 320)
        self.declare_parameter("detector.inference_hz", 20.0)
        self.declare_parameter("detector.ncnn_threads", 2)
        self.declare_parameter("detector.warmup_enabled", True)
        self.declare_parameter("opencv_num_threads", 1)

        config_file = str(self.get_parameter("config_file").value)
        model_path = str(self.get_parameter("model_path").value)
        self.config = load_config(config_file)
        self.config.detector.device = str(self.get_parameter("detector.device").value)
        self.config.detector.imgsz = int(self.get_parameter("detector.imgsz").value)
        self.config.detector.inference_hz = float(
            self.get_parameter("detector.inference_hz").value
        )
        self.config.detector.ncnn_threads = int(
            self.get_parameter("detector.ncnn_threads").value
        )
        self.config.detector.warmup_enabled = bool(
            self.get_parameter("detector.warmup_enabled").value
        )
        self._declare_live_parameters()
        cv2.setNumThreads(max(1, int(self.get_parameter("opencv_num_threads").value)))

        self.detector = BestPthDetector(self.config, model_path, logger=self.get_logger())
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.sequence = 0
        self.infer_count = 0
        self.infer_time_sum = 0.0
        self.report_started = time.perf_counter()
        self._status = (False, "waiting for first camera frame")

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.packet_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("detections_topic").value),
            1,
        )
        self.ready_pub = self.create_publisher(Bool, "/bisa/detector/ready", 1)
        self.status_pub = self.create_publisher(String, "/bisa/detector/status", 1)
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            image_qos,
        )
        self.create_timer(1.0, self._republish_status)
        self._publish_status(False, "waiting for first camera frame")

    @staticmethod
    def _config_value(config, path):
        value = config
        for part in path:
            value = value[part] if isinstance(value, dict) else getattr(value, part)
        return value

    @staticmethod
    def _set_config_value(config, path, value) -> None:
        target = config
        for part in path[:-1]:
            target = target[part] if isinstance(target, dict) else getattr(target, part)
        if isinstance(target, dict):
            target[path[-1]] = value
        else:
            setattr(target, path[-1], value)

    def _declare_live_parameters(self) -> None:
        """Declares GUI-visible detector parameters from the loaded YAML."""

        already_declared = {"detector.imgsz", "detector.inference_hz"}
        for name, path in LIVE_PARAMETER_PATHS.items():
            if name in already_declared:
                continue
            initial = self._config_value(self.config, path)
            value = self.declare_parameter(name, initial).value
            self._set_config_value(self.config, path, value)

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        """Atomically applies GUI tuning values before the next inference frame."""

        candidate = copy.deepcopy(self.config)
        changed = []
        for parameter in parameters:
            path = LIVE_PARAMETER_PATHS.get(parameter.name)
            if path is None:
                continue
            self._set_config_value(candidate, path, parameter.value)
            changed.append(parameter.name)

        detector = candidate.detector
        color = candidate.color_correction
        light = candidate.traffic_light
        valid = (
            detector.imgsz >= 32
            and detector.inference_hz > 0.0
            and all(0.0 <= float(value) <= 1.0 for value in detector.conf.values())
            and color.clahe_clip >= 0.0
            and color.gamma > 0.0
            and 0 <= light.green_h_lo <= light.green_h_hi <= 180
            and 0 <= light.red_h1_hi <= 180
            and 0 <= light.red_h2_lo <= 180
            and 0.0 <= light.row_min_ratio <= 1.0
        )
        if not valid:
            return SetParametersResult(
                successful=False,
                reason="invalid detector, color-correction, or traffic-light range",
            )
        if (
            "detector.imgsz" in changed
            and self.detector._is_ncnn_model()
            and detector.imgsz != self.config.detector.imgsz
        ):
            return SetParametersResult(
                successful=False,
                reason="the installed NCNN model has a fixed export size; re-export before changing imgsz",
            )

        self.config = candidate
        self.detector.config = candidate
        if changed:
            self.get_logger().info("applied live parameters: %s" % ", ".join(changed))
        return SetParametersResult(successful=True)

    def _publish_status(self, ready: bool, status: str) -> None:
        self._status = (bool(ready), str(status))
        self.ready_pub.publish(Bool(data=bool(ready)))
        self.status_pub.publish(String(data=str(status)))

    def _republish_status(self) -> None:
        self._publish_status(*self._status)

    def image_callback(self, msg: CompressedImage) -> None:
        """Consumes only the newest frame; this node may block without control jitter."""

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self._publish_status(False, "camera JPEG decode failed")
            return
        if not self.detector.ready:
            self._publish_status(False, "warming detector")
            if not self.detector.warmup(tuple(frame.shape)):
                self._publish_status(False, self.detector.last_error or "warmup failed")
                return
            self._publish_status(
                True,
                f"ready device={self.detector.device} "
                f"ncnn_threads={self.config.detector.ncnn_threads}",
            )

        now_sec = self.get_clock().now().nanoseconds / 1e9
        processed = preprocess_frame(frame, self.config.color_correction)
        previous_infer_time = self.detector.last_infer_time
        started = time.perf_counter()
        detections = self.detector.infer(processed, now_sec)
        if self.detector.last_infer_time == previous_infer_time:
            return
        elapsed = time.perf_counter() - started
        light_state, classified = _classify_lights(
            detections, processed, self.config
        )

        self.sequence += 1
        light_code = 1.0 if light_state == "green" else 2.0 if light_state == "red" else 0.0
        records = [det for det in classified if det.cls in CLASS_IDS]
        # Preserve the source camera timestamp so the C++ debug renderer can
        # draw detections on the exact frame that produced them.  Float64 is
        # exact for ROS sec/nanosec integer fields; Float32 is not.
        payload = [
            float(self.sequence),
            float(msg.header.stamp.sec),
            float(msg.header.stamp.nanosec),
            light_code,
            float(len(records)),
        ]
        for detection in records:
            payload.extend(
                [
                    float(CLASS_IDS[detection.cls]),
                    float(detection.conf),
                    *[float(value) for value in detection.bbox],
                ]
            )
        self.packet_pub.publish(Float64MultiArray(data=payload))

        self.infer_count += 1
        self.infer_time_sum += elapsed
        report_elapsed = time.perf_counter() - self.report_started
        if report_elapsed >= 3.0:
            self.get_logger().info(
                "YOLO infer: %.0f ms/frame, %.1f FPS effective "
                "(imgsz=%d, hz_cap=%.1f, device=%s)"
                % (
                    1000.0 * self.infer_time_sum / max(self.infer_count, 1),
                    self.infer_count / report_elapsed,
                    self.config.detector.imgsz,
                    self.config.detector.inference_hz,
                    self.detector.device,
                )
            )
            self.infer_count = 0
            self.infer_time_sum = 0.0
            self.report_started = time.perf_counter()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BisaDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
