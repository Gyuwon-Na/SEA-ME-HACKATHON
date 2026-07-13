"""Process-isolated YOLO worker using a latest-frame shared-memory buffer."""

from __future__ import annotations

import queue
import time
from dataclasses import replace
from multiprocessing import shared_memory

import cv2
import numpy as np

from . import traffic_light
from .object_detector import BestPthDetector


def _put_latest(result_queue, message) -> None:
    """Put a message without ever blocking inference on a slow consumer."""

    try:
        result_queue.put_nowait(message)
        return
    except queue.Full:
        pass
    try:
        result_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        result_queue.put_nowait(message)
    except queue.Full:
        pass


def _classify_lights(detections, frame, config):
    """Replace YOLO light labels with the configured row-color verdict."""

    seen_green = False
    seen_red = False
    classified = []
    for detection in detections:
        if detection.cls in ("traffic_green", "traffic_red"):
            row_class, _ = traffic_light.classify_light(
                frame, detection.bbox, config
            )
            if row_class is None:
                continue
            seen_green = seen_green or row_class == "traffic_green"
            seen_red = seen_red or row_class == "traffic_red"
            detection = replace(detection, cls=row_class)
        classified.append(detection)
    light_state = "red" if seen_red else "green" if seen_green else None
    return light_state, classified


def _prepare_detector(detector, result_queue, opencv_num_threads: int) -> bool:
    """Warm the requested backend and fall back from Vulkan to NCNN CPU."""

    if not detector.config.detector.enabled:
        _put_latest(result_queue, {"type": "status", "ready": True, "status": "disabled"})
        return True
    requested = str(detector.config.detector.device)
    _put_latest(
        result_queue,
        {"type": "status", "ready": False, "status": f"warming requested={requested}"},
    )
    if detector.warmup():
        _put_latest(
            result_queue,
            {
                "type": "status",
                "ready": True,
                "status": (
                    f"ready device={detector.device} "
                    f"ncnn_threads={detector.config.detector.ncnn_threads} "
                    f"opencv_threads={opencv_num_threads}"
                ),
            },
        )
        return True
    if detector.device.startswith("vulkan"):
        detector.reset_device("cpu")
        _put_latest(
            result_queue,
            {"type": "status", "ready": False, "status": "warming fallback=cpu"},
        )
        if detector.warmup():
            _put_latest(
                result_queue,
                {
                    "type": "status",
                    "ready": True,
                    "status": (
                        "ready device=cpu "
                        f"ncnn_threads={detector.config.detector.ncnn_threads} "
                        f"opencv_threads={opencv_num_threads}"
                    ),
                },
            )
            return True
    error = detector.last_error or "detector unavailable"
    _put_latest(
        result_queue,
        {"type": "status", "ready": False, "status": f"failed: {error}"},
    )
    return False


def run_inference_process(
    shm_name,
    frame_shape,
    frame_lock,
    frame_sequence,
    frame_stamp,
    stop_event,
    result_queue,
    config_queue,
    config,
    model_path,
    opencv_num_threads,
):
    """Attach to the latest-frame buffer and run all model work in this process."""

    cv2.setNumThreads(max(1, int(opencv_num_threads)))
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_frame = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
    detector = BestPthDetector(config, model_path, logger=None)
    prepared = False
    last_sequence = -1
    infer_count = 0
    infer_time_sum = 0.0
    last_report = time.perf_counter()
    try:
        while not stop_event.is_set():
            try:
                while True:
                    config = config_queue.get_nowait()
                    detector.config = config
            except queue.Empty:
                pass

            if not prepared:
                prepared = _prepare_detector(
                    detector, result_queue, max(1, int(opencv_num_threads))
                )
                if not prepared:
                    stop_event.wait(1.0)
                    continue
                infer_count = 0
                infer_time_sum = 0.0
                last_report = time.perf_counter()

            with frame_lock:
                sequence = int(frame_sequence.value)
                if sequence == last_sequence:
                    frame = None
                    stamp = 0.0
                else:
                    frame = shared_frame.copy()
                    stamp = float(frame_stamp.value)
            if frame is None:
                stop_event.wait(0.005)
                continue
            last_sequence = sequence

            try:
                processed = traffic_light.preprocess_frame(
                    frame, detector.config.color_correction
                )
                previous_infer_time = detector.last_infer_time
                infer_start = time.perf_counter()
                detections = detector.infer(processed, stamp)
                if detector.last_infer_time == previous_infer_time:
                    continue
                infer_count += 1
                infer_time_sum += time.perf_counter() - infer_start
                light_state, classified = _classify_lights(
                    detections, processed, detector.config
                )
                _put_latest(
                    result_queue,
                    {
                        "type": "result",
                        "sequence": sequence,
                        "stamp": stamp,
                        "raw_detections": detections,
                        "classified_detections": classified,
                        "light_state": light_state,
                    },
                )
                report_elapsed = time.perf_counter() - last_report
                if report_elapsed >= 3.0 and infer_count:
                    _put_latest(
                        result_queue,
                        {
                            "type": "metrics",
                            "average_ms": 1000.0 * infer_time_sum / infer_count,
                            "fps": infer_count / report_elapsed,
                            "imgsz": detector.config.detector.imgsz,
                            "hz_cap": detector.config.detector.inference_hz,
                            "device": detector.device,
                        },
                    )
                    infer_count = 0
                    infer_time_sum = 0.0
                    last_report = time.perf_counter()
            except Exception as exc:
                _put_latest(
                    result_queue,
                    {"type": "status", "ready": False, "status": f"error: {exc}"},
                )
                if detector.device.startswith("vulkan"):
                    detector.reset_device("cpu")
                prepared = False
    finally:
        shm.close()
