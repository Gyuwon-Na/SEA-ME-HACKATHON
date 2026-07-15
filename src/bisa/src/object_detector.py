"""YOLO best.pth detector wrapper and temporal filtering utilities."""

from __future__ import annotations

import math
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

    def clear(self) -> None:
        """Drops old frames before starting a fresh stationary vote."""

        self.frames.clear()

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
        "left_sign": "sign_left",
        "right_sign": "sign_right",
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
        self.ready = False
        self.last_error = ""
        self._runtime_config_key = None

    def _resolve_device(self) -> str:
        """Picks the inference device: CUDA GPU, Vulkan GPU, or CPU.

        Priority for 'auto': CUDA (PC GPU) → Vulkan (embedded GPU, e.g.
        TOPST D3-G PowerVR via NCNN) → CPU fallback.
        """

        preference = str(getattr(self.config.detector, "device", "auto")).lower()
        if preference in ("cpu",):
            return "cpu"
        if preference in ("cuda", "cuda:0"):
            return "cuda:0"
        if preference.startswith("vulkan"):
            # Accept 'vulkan', 'vulkan:0', 'vulkan:1', etc.
            requested = preference if ":" in preference else "vulkan:0"
            return requested if self._vulkan_device_available(requested) else "cpu"
        if preference in ("gpu", "0"):
            # Generic 'gpu' — try CUDA first, then Vulkan.
            try:
                import torch
                if torch.cuda.is_available():
                    return "cuda:0"
            except Exception:
                pass
            return "vulkan:0" if self._vulkan_device_available("vulkan:0") else "cpu"
        # 'auto' — cascade CUDA → Vulkan → CPU.
        try:
            import torch
            if torch.cuda.is_available():
                return "cuda:0"
        except Exception:  # pragma: no cover - depends on target env.
            pass
        # On embedded boards with Vulkan-capable GPUs (PowerVR, Mali, etc.),
        # NCNN can use Vulkan compute. Ultralytics NCNN models accept
        # device='vulkan:0' to offload inference to the GPU.
        if self._is_ncnn_model() and self._vulkan_device_available("vulkan:0"):
            return "vulkan:0"
        return "cpu"

    def _vulkan_device_available(self, device: str) -> bool:
        """Checks that NCNN sees a real Vulkan GPU at the requested index."""

        if not self._is_ncnn_model():
            self._log_warn("Vulkan requested for a non-NCNN model; falling back to CPU")
            return False
        try:
            import ncnn

            index = int(str(device).split(":", 1)[1]) if ":" in str(device) else 0
            if index < 0 or index >= int(ncnn.get_gpu_count()):
                self._log_warn(
                    f"Vulkan GPU index {index} unavailable; falling back to CPU"
                )
                return False
            name = str(ncnn.get_gpu_info(index).device_name())
            if "llvmpipe" in name.lower():
                self._log_warn(
                    f"Vulkan device {index} is software renderer {name}; falling back to CPU"
                )
                return False
            if self.logger is not None:
                self.logger.info(f"Vulkan device verified: index={index}, name={name}")
            return True
        except Exception as exc:
            self._log_warn(f"Vulkan runtime check failed: {exc}; falling back to CPU")
            return False

    def _log_warn(self, message: str) -> None:
        """Writes warnings through ROS logger when available."""

        if self.logger is not None:
            self.logger.warning(message)

    def _is_ncnn_model(self) -> bool:
        """Returns True when the configured model path points to an NCNN export."""

        p = Path(self.model_path)
        # NCNN exports are directories containing .param + .bin files, or the
        # path ends with '_ncnn_model' by ultralytics convention.
        if p.is_dir():
            return (p / "model.ncnn.param").exists() or str(p).endswith("_ncnn_model")
        return False

    def load_model(self) -> bool:
        """Loads the YOLO model from checkpoints on first use."""

        if self.model is not None:
            return True
        if not self.config.detector.enabled:
            return False
        if not Path(self.model_path).exists():
            self._log_warn(f"Detector model not found: {self.model_path}")
            return False
        try:
            from ultralytics import YOLO

            self.device = self._resolve_device()
            self.model = YOLO(
                self.model_path,
                task="detect" if self._is_ncnn_model() else None,
            )
            self.task = str(getattr(self.model, "task", "detect"))
            if self.logger is not None:
                backend = "NCNN" if self._is_ncnn_model() else "PyTorch"
                self.logger.info(
                    f"Detector loaded: backend={backend}, device={self.device} "
                    f"(task={self.task})"
                )
            return True
        except Exception as exc:  # pragma: no cover - depends on target vehicle env.
            self.last_error = str(exc)
            self._log_warn(f"Failed to load detector model: {exc}")
            return False

    def _configure_ncnn_runtime(self) -> None:
        """Caps NCNN CPU workers after AutoBackend has created its Net."""

        if not self._is_ncnn_model() or self.model is None:
            return
        predictor = getattr(self.model, "predictor", None)
        backend = getattr(getattr(predictor, "model", None), "backend", None)
        net = getattr(backend, "net", None)
        if net is None:
            return
        threads = max(1, int(getattr(self.config.detector, "ncnn_threads", 2)))
        key = (threads, bool(net.opt.use_vulkan_compute))
        if self._runtime_config_key == key:
            return
        net.opt.num_threads = threads
        self._runtime_config_key = key
        if self.logger is not None:
            self.logger.info(
                f"NCNN runtime configured: threads={net.opt.num_threads}, "
                f"vulkan={bool(net.opt.use_vulkan_compute)}"
            )

    def _predict(self, source):
        """Run predict, configuring NCNN threads before its Net loads weights."""

        kwargs = {
            "source": source,
            "imgsz": int(self.config.detector.imgsz),
            "device": self.device,
            "conf": min(float(value) for value in self.config.detector.conf.values()),
            "verbose": False,
        }
        # Ultralytics creates ncnn.Net lazily inside the first predict() and
        # exposes no thread option. Setting net.opt afterwards is too late for
        # convolution pipelines, which retain their load-time value. Temporarily
        # wrap the constructor so num_threads is already set before load_param /
        # load_model. The detector runs in its own process, so this module-local
        # patch cannot race another model.
        if self._is_ncnn_model() and getattr(self.model, "predictor", None) is None:
            import ncnn

            original_net = ncnn.Net
            threads = max(1, int(getattr(self.config.detector, "ncnn_threads", 2)))

            def configured_net(*args, **net_kwargs):
                net = original_net(*args, **net_kwargs)
                net.opt.num_threads = threads
                return net

            ncnn.Net = configured_net
            try:
                return self.model.predict(**kwargs)
            finally:
                ncnn.Net = original_net
        return self.model.predict(**kwargs)

    def warmup(self, frame_shape: tuple[int, int, int] = (480, 640, 3)) -> bool:
        """Loads the backend and builds kernels before detector results are used."""

        if not self.config.detector.enabled:
            self.ready = True
            return True
        if not self.load_model():
            return False
        try:
            if bool(getattr(self.config.detector, "warmup_enabled", True)):
                dummy = np.zeros(frame_shape, dtype=np.uint8)
                self._predict(dummy)
            self._configure_ncnn_runtime()
            self.ready = True
            self.last_error = ""
            # Warmup is not part of the detector-rate clock.
            self.last_infer_time = 0.0
            return True
        except Exception as exc:  # pragma: no cover - target runtime dependent.
            self.ready = False
            self.last_error = str(exc)
            self._log_warn(f"Detector warmup failed on {self.device}: {exc}")
            return False

    def reset_device(self, device: str) -> None:
        """Drops the current backend and selects a safe device for retry."""

        self.model = None
        self.device = "cpu"
        self.ready = False
        self.last_infer_time = 0.0
        self._runtime_config_key = None
        self.config.detector.device = str(device)

    def should_run(self, now_sec: float) -> bool:
        """Rate-limits inference so lane following remains lightweight."""

        hz = max(float(self.config.detector.inference_hz), 0.1)
        if now_sec - self.last_infer_time < 1.0 / hz:
            return False
        self.last_infer_time = now_sec
        return True

    def infer(
        self,
        frame_bgr: np.ndarray,
        now_sec: float,
        inference_roi: list[float] | None = None,
    ) -> list[Detection]:
        """Runs inference on the light ROI and returns full-frame detections."""

        if not self.load_model():
            return []
        if not self.should_run(now_sec):
            return []

        x0, y0, x1, y1 = self.roi_bounds(
            frame_bgr.shape,
            inference_roi or self.config.roi.detector_light,
        )
        results = self._predict(frame_bgr[y0:y1, x0:x1])
        self._configure_ncnn_runtime()
        self.ready = True
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
                bx1, by1, bx2, by2 = [float(v) for v in box.xyxy[0].tolist()]
                bx1, bx2 = bx1 + x0, bx2 + x0
                by1, by2 = by1 + y0, by2 + y0
                det = Detection(
                    cls=cls_name,
                    conf=conf,
                    bbox=(bx1, by1, bx2, by2),
                    cx=(bx1 + bx2) / 2.0,
                    cy=(by1 + by2) / 2.0,
                )
                if self.in_expected_roi(det, frame_bgr.shape[1], frame_bgr.shape[0]):
                    detections.append(det)
        return detections

    def light_roi_bounds(self, frame_shape: tuple[int, ...]) -> tuple[int, int, int, int]:
        """Converts the configured normalized ROI into a non-empty image crop."""

        return self.roi_bounds(frame_shape, self.config.roi.detector_light)

    def inference_roi(self, mission_state: str) -> list[float]:
        """Uses the light crop before launch and one full-frame pass afterwards."""

        roi = (
            self.config.roi.detector_light
            if mission_state.endswith("WAIT_GREEN")
            else self.config.roi.detector_sign
        )
        return list(roi)

    @staticmethod
    def roi_bounds(
        frame_shape: tuple[int, ...], roi: list[float]
    ) -> tuple[int, int, int, int]:
        """Converts one normalized ROI into a non-empty image crop."""

        height, width = frame_shape[:2]
        if height < 1 or width < 1:
            raise ValueError("frame must have a positive width and height")
        left, top, right, bottom = roi
        x0 = min(max(math.floor(float(left) * width), 0), width - 1)
        y0 = min(max(math.floor(float(top) * height), 0), height - 1)
        x1 = min(max(math.ceil(float(right) * width), x0 + 1), width)
        y1 = min(max(math.ceil(float(bottom) * height), y0 + 1), height)
        return x0, y0, x1, y1

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
