"""독립 프로세스 YOLO 검출 노드 — GIL 탈출용.

`autonomous_driving_node`의 `_inference_worker`는 파이썬 스레드라서 메인 스레드의
lane/aruco(numpy/opencv)가 GIL을 잡으면 YOLO 추론이 굶어 유효 FPS가 떨어짐.
이 노드는 YOLO를 **별도 프로세스**로 떼어 전용 코어에서 돌리고, 검출 결과만
`/bisa/detections`(JSON String)로 내보냄. 검출 알고리즘(BestPthDetector.infer)은
그대로 재사용하며 실행 위치만 옮김 — 근본 로직은 침범하지 않음.

색판별(classify_light)은 FSM 상태에 의존하므로 여기서 하지 않고, 원본 검출만
보냄. autonomous_node가 자기 최신 프레임으로 기존 `_classify_and_gate`를 적용함
(신호등 색은 느리게 변해 1~2프레임 차이는 무해).
"""

from __future__ import annotations

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from . import traffic_light
from .dracer_config import load_config, resolve_package_relative_path
from .object_detector import BestPthDetector

# autonomous_driving_node와 동일한 기본 config 탐색 로직 재사용
from .autonomous_driving_node import get_default_config_path, as_bool


class DetectorNode(Node):
    """이미지를 구독해 YOLO 검출만 수행하고 결과를 토픽으로 발행하는 노드."""

    def __init__(self):
        super().__init__("bisa_detector_node")
        self.declare_parameter("config_file", get_default_config_path())
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("detections_topic", "/bisa/detections")
        self.declare_parameter("debug_log", True)

        config_file = str(self.get_parameter("config_file").value)
        model_path = str(self.get_parameter("model_path").value).strip()
        self.image_topic = str(self.get_parameter("image_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        self.debug_log = as_bool(self.get_parameter("debug_log").value)

        self.config = load_config(config_file)
        if model_path:
            self.config.detector.model_path = model_path
        resolved_model = resolve_package_relative_path(__file__, self.config.detector.model_path)
        # NCNN export는 고정 해상도(예: 320)라 config의 640(PC/GPU기준)으로 돌리면
        # 검출이 깨짐 — metadata에서 고정 imgsz를 읽어 강제(best.pt면 None → config 유지).
        # onboard.launch가 autonomous에 imgsz:=320 주입하던 역할을 이 노드가 직접 수행.
        export_imgsz = traffic_light.ncnn_export_imgsz(resolved_model)
        if export_imgsz:
            self.config.detector.imgsz = export_imgsz
            self.get_logger().info(f"NCNN fixed imgsz={export_imgsz} applied")
        self.detector = BestPthDetector(self.config, resolved_model, logger=self.get_logger())

        # 카메라와 동일한 BEST_EFFORT/depth-1 — 항상 최신 프레임만, 밀린 건 버림.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, self.image_topic, self.image_callback, image_qos)
        self.det_pub = self.create_publisher(String, detections_topic, image_qos)

        # 유효 FPS 로깅용 카운터 (worker와 동일 포맷)
        self._infer_count = 0
        self._infer_time_sum = 0.0
        self._last_report = time.perf_counter()

        self.get_logger().info(
            f"BISA detector node started: image_topic={self.image_topic}, "
            f"detections_topic={detections_topic}, model={resolved_model}"
        )

    def decode_image(self, msg: CompressedImage):
        """CompressedImage → BGR ndarray (autonomous_node와 동일 방식)."""

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    def image_callback(self, msg: CompressedImage) -> None:
        """최신 프레임에 YOLO를 돌려 검출 결과를 JSON으로 발행."""

        frame = self.decode_image(msg)
        if frame is None:
            return
        # YOLO 검출기와 HSV 분석기가 공유하는 전처리(비활성 시 no-op).
        proc = traffic_light.preprocess_frame(frame, self.config.color_correction)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        prev_infer = self.detector.last_infer_time
        infer_start = time.perf_counter()
        detections = self.detector.infer(proc, now_sec)
        # should_run(inference_hz 캡)으로 스킵된 프레임은 발행하지 않음 —
        # last_infer_time 변화로 '실제 추론했는지'를 판별(worker와 동일).
        if self.detector.last_infer_time == prev_infer:
            return

        self._infer_count += 1
        self._infer_time_sum += time.perf_counter() - infer_start

        payload = {
            "stamp": now_sec,
            "dets": [
                {"cls": d.cls, "conf": d.conf, "bbox": list(d.bbox), "cx": d.cx, "cy": d.cy}
                for d in detections
            ],
        }
        out = String()
        out.data = json.dumps(payload)
        self.det_pub.publish(out)

        if self.debug_log and self._infer_count > 0:
            elapsed = time.perf_counter() - self._last_report
            if elapsed >= 3.0:
                avg_ms = 1000.0 * self._infer_time_sum / self._infer_count
                fps = self._infer_count / elapsed
                self.get_logger().info(
                    f"YOLO infer: {avg_ms:.0f} ms/frame, {fps:.1f} FPS effective "
                    f"(imgsz={self.config.detector.imgsz}, "
                    f"hz_cap={self.config.detector.inference_hz}, "
                    f"device={self.detector.device}) [detached process]"
                )
                self._infer_count = 0
                self._infer_time_sum = 0.0
                self._last_report = time.perf_counter()


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
