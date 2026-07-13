"""YOLO best.pth detector wrapper and temporal filtering utilities."""

from __future__ import annotations

import os
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from .dracer_config import AutonomousConfig


@dataclass
class Detection:
    """Represents one gated detector result in original image coordinates."""

    cls: str
    conf: float
    bbox: tuple[float, float, float, float]
    cx: float
    cy: float


class DetectionBuffer:
    """Stores recent detections for vote and consecutive-frame decisions."""

    def __init__(self, maxlen: int = 30):
        """Initializes the rolling temporal buffer."""

        self.frames: deque[list[Detection]] = deque(maxlen=maxlen)

    def push(self, detections: Iterable[Detection]) -> None:
        """Adds one detector frame to the history."""

        self.frames.append(list(detections))

    def count(self, cls: str, last_n: int) -> int:
        """Counts how many of the latest frames contained a class."""

        recent = list(self.frames)[-last_n:]
        return sum(1 for frame in recent if any(det.cls == cls for det in frame))

    def stable_seen(self, cls: str, k: int, n: int) -> bool:
        """Returns true when a class appears in at least k of the last n frames."""

        return self.count(cls, n) >= k



class BestPthDetector:
    """Runs the fine-tuned best.pt model when ultralytics is available."""

    # Checkpoint label names differ from the mission class names the FSM
    # expects (and differ between retrained models). Mapping by NAME through
    # this table — instead of by class index — means a retrained model with a
    # different index order can never silently swap red and green.
    NAME_ALIASES = {
        "green_light": "traffic_green",
        "red_light": "traffic_red",
        "light_green": "traffic_green",
        "light_red": "traffic_red",
        "left_turn": "sign_left",
        "right_turn": "sign_right",
    }

    def __init__(self, config: AutonomousConfig, model_path: str, logger=None):
        """Initializes lazy model loading so syntax tests do not require torch."""

        self.config = config
        self.model_path = str(Path(model_path).expanduser())
        self.logger = logger
        self.model = None
        self.task = "detect"
        self.device = "cpu"
        self.last_infer_time = 0.0

    def _resolve_device(self) -> str:
        """Picks the inference device.

        "cpu"          → CPU (기본)
        "cuda"/"cuda:0" → NVIDIA GPU (PC용)
        "vulkan:N"     → NCNN Vulkan 백엔드 GPU N번
                         (PowerVR/Mali/Adreno 등 비-CUDA GPU, D3-G 보드용)
        "auto"         → CUDA 있으면 cuda:0, 없으면 cpu
        """
        preference = str(getattr(self.config.detector, "device", "auto")).lower()
        if preference == "cpu":
            return "cpu"
        if preference in ("cuda", "gpu", "0", "cuda:0"):
            return "cuda:0"
        # Vulkan GPU (NCNN Vulkan 백엔드) — "vulkan" 또는 "vulkan:N"
        if preference.startswith("vulkan"):
            gpu_id = preference.split(":")[-1] if ":" in preference else "0"
            return f"vulkan:{gpu_id}"
        try:  # 'auto'
            import torch

            if torch.cuda.is_available():
                return "cuda:0"
        except Exception:  # pragma: no cover - depends on target env.
            pass
        return "cpu"

    def _log_warn(self, message: str) -> None:
        """Writes warnings through ROS logger when available."""

        if self.logger is not None:
            self.logger.warning(message)

    def load_model(self) -> bool:
        """Loads the YOLO model from checkpoints/best.pth on first use."""

        if self.model is not None:
            return True
        if not self.config.detector.enabled:
            return False
        if not Path(self.model_path).exists():
            self._log_warn(f"Detector model not found: {self.model_path}")
            return False
        try:
            import torch
            from ultralytics import YOLO

            raw_device = self._resolve_device()
            if raw_device == "cpu":
                # CPU 스레드 수 제한: YOLO 추론이 모든 코어를 독점하지 않도록
                torch.set_num_threads(max(1, min(4, (os.cpu_count() or 4) - 1)))

            self.model = YOLO(self.model_path)
            self.task = str(getattr(self.model, "task", "detect"))

            if raw_device.startswith("vulkan"):
                # 워밍업: Vulkan 셰이더(SPIR-V) 컴파일을 로드 시점에 1회 수행.
                # 안 하면 첫 실추론이 ~38초 걸림(셰이더 JIT). 여기서 미리 태움.
                # 핵심: predict에 매번 raw_device("vulkan:N")를 그대로 전달해야
                # NCNN net이 use_vulkan_compute=True를 유지 → 컨볼루션이 PowerVR
                # GPU에서 실행되고 A72 CPU는 거의 안 씀(측정: 370%→18% 점유).
                # "cpu"를 넘기면 백엔드가 CPU로 리로드되어 오프로드가 사라짐.
                imgsz = int(self.config.detector.imgsz)
                dummy = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
                self.model.predict(source=dummy, imgsz=imgsz, device=raw_device, verbose=False)
            self.device = raw_device

            if self.logger is not None:
                self.logger.info(
                    f"Detector loaded: raw_device={raw_device}, "
                    f"predict_device={self.device}, task={self.task}"
                )
            return True
        except Exception as exc:  # pragma: no cover - depends on target vehicle env.
            self._log_warn(f"Failed to load detector model: {exc}")
            return False

    def should_run(self, now_sec: float) -> bool:
        """Rate-limits inference so lane following remains lightweight."""

        hz = max(float(self.config.detector.inference_hz), 0.1)
        if now_sec - self.last_infer_time < 1.0 / hz:
            return False
        self.last_infer_time = now_sec
        return True

    def infer(self, frame_bgr: np.ndarray, now_sec: float) -> list[Detection]:
        """Runs model inference and returns confidence/ROI-gated detections."""

        if not self.should_run(now_sec):
            return []
        if not self.load_model():
            return []

        results = self.model.predict(
            source=frame_bgr,
            imgsz=int(self.config.detector.imgsz),
            device=self.device,
            verbose=False,
        )
        if not results:
            return []

        if self.task == "classify":
            return self._classify_detections(
                results[0], frame_bgr.shape[1], frame_bgr.shape[0]
            )

        detections: list[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_index = int(box.cls[0].item())
                cls_name = self._class_name_from_index(cls_index)
                conf = float(box.conf[0].item())
                if conf < self.config.detector.conf.get(cls_name, 0.55):
                    continue
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                det = Detection(
                    cls=cls_name,
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cx=(x1 + x2) / 2.0,
                    cy=(y1 + y2) / 2.0,
                )
                if self.in_expected_roi(det, frame_bgr.shape[1], frame_bgr.shape[0]):
                    detections.append(det)
        return detections

    def _classify_detections(self, result, width: int, height: int) -> list[Detection]:
        """Turns a whole-frame classification result into one mission detection."""

        probs = getattr(result, "probs", None)
        if probs is None:
            return []
        top1 = int(probs.top1)
        conf = float(probs.top1conf)
        model_names = getattr(self.model, "names", {}) or {}
        raw_name = str(model_names.get(top1, top1))
        cls_name = self.NAME_ALIASES.get(raw_name, raw_name)
        if conf < self.config.detector.conf.get(cls_name, 0.55):
            return []
        return [
            Detection(
                cls=cls_name,
                conf=conf,
                bbox=(0.0, 0.0, float(width), float(height)),
                cx=width / 2.0,
                cy=height / 2.0,
            )
        ]

    def _class_name_from_index(self, cls_index: int) -> str:
        """Maps model class IDs to mission class names via model names + aliases.

        The model's own ``names`` metadata is authoritative; the config
        ``class_map`` is only a fallback for checkpoints that lack names.
        """

        model_names = getattr(self.model, "names", {}) or {}
        raw_name = model_names.get(cls_index)
        if raw_name is not None:
            raw_name = str(raw_name)
            return self.NAME_ALIASES.get(raw_name, raw_name)
        for name, index in self.config.detector.class_map.items():
            if int(index) == cls_index:
                return name
        return str(cls_index)

    def in_expected_roi(self, detection: Detection, width: int, height: int) -> bool:
        """Applies mission-specific ROI gating before temporal voting."""

        if detection.cls.startswith("traffic"):
            roi = self.config.roi.detector_light
        elif detection.cls.startswith("sign"):
            roi = self.config.roi.detector_sign
        else:
            return True

        x0, y0, x1, y1 = roi
        return (
            x0 * width <= detection.cx <= x1 * width
            and y0 * height <= detection.cy <= y1 * height
        )
