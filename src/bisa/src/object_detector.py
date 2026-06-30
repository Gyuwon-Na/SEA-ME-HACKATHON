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
    area: float


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

    def consecutive_count(self, cls: str) -> int:
        """Counts class detections in consecutive frames ending at the latest frame."""

        total = 0
        for frame in reversed(self.frames):
            if any(det.cls == cls for det in frame):
                total += 1
            else:
                break
        return total

    def stable_consecutive(self, cls: str, frames: int) -> bool:
        """Returns true when a class appears for the requested consecutive frames."""

        return self.consecutive_count(cls) >= frames

    def not_seen_consecutive(self, cls: str, frames: int) -> bool:
        """Returns true when a class is absent for the requested consecutive frames."""

        if len(self.frames) < frames:
            return False
        recent = list(self.frames)[-frames:]
        return all(not any(det.cls == cls for det in frame) for frame in recent)

    def best(self, cls: str) -> Detection | None:
        """Returns the highest-confidence latest detection for the requested class."""

        if not self.frames:
            return None
        candidates = [det for det in self.frames[-1] if det.cls == cls]
        if not candidates:
            return None
        return max(candidates, key=lambda item: item.conf)


class BestPthDetector:
    """Runs the fine-tuned best.pt model when ultralytics is available."""

    # The shipped best.pt is a 4-class image classifier whose label names differ
    # from the mission class names the FSM expects. Translate them here so a
    # classification checkpoint drops straight into the detection/vote pipeline.
    CLASSIFY_NAME_ALIASES = {
        "green_light": "traffic_green",
        "red_light": "traffic_red",
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
        self.last_infer_time = 0.0
        self.load_error: Exception | None = None

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

            # This vehicle has no CUDA/Vulkan compute device (Telechips TCC8050 /
            # PowerVR), so inference is CPU-only. Cap threads to leave headroom
            # for the ROS control loop on the quad-core CPU.
            torch.set_num_threads(max(1, min(4, (os.cpu_count() or 4) - 1)))
            self.model = YOLO(self.model_path)
            self.task = str(getattr(self.model, "task", "detect"))
            return True
        except Exception as exc:  # pragma: no cover - depends on target vehicle env.
            self.load_error = exc
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
            device="cpu",
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
                    area=max(0.0, (x2 - x1) * (y2 - y1)),
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
        cls_name = self.CLASSIFY_NAME_ALIASES.get(raw_name, raw_name)
        if conf < self.config.detector.conf.get(cls_name, 0.55):
            return []
        return [
            Detection(
                cls=cls_name,
                conf=conf,
                bbox=(0.0, 0.0, float(width), float(height)),
                cx=width / 2.0,
                cy=height / 2.0,
                area=float(width * height),
            )
        ]

    def _class_name_from_index(self, cls_index: int) -> str:
        """Maps model class IDs to mission class names via CLASS_MAP."""

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
        elif detection.cls == "dynamic_marker":
            roi = self.config.roi.detector_dynamic
        else:
            return True

        x0, y0, x1, y1 = roi
        return (
            x0 * width <= detection.cx <= x1 * width
            and y0 * height <= detection.cy <= y1 * height
        )
