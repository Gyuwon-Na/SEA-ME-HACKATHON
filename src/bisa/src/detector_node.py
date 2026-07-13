"""Process-isolated ROS detector feeding the C++ autonomous core."""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
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
        cv2.setNumThreads(max(1, int(self.get_parameter("opencv_num_threads").value)))

        self.detector = BestPthDetector(self.config, model_path, logger=self.get_logger())
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
