import numpy as np

from bisa.dracer_config import AutonomousConfig
from bisa.object_detector import BestPthDetector


class _Box:
    cls = np.array([3])
    conf = np.array([0.9])
    xyxy = np.array([[10.0, 20.0, 30.0, 40.0]])


class _Result:
    boxes = [_Box()]


class _Model:
    task = "detect"
    names = {3: "green_light"}


def test_light_roi_is_cropped_and_detection_returns_to_full_frame():
    config = AutonomousConfig()
    detector = BestPthDetector(config, "unused")
    detector.model = _Model()
    sources = []
    detector._predict = lambda source: sources.append(source) or [_Result()]

    detections = detector.infer(np.zeros((480, 640, 3), dtype=np.uint8), 1.0)

    assert sources[0].shape == (408, 512, 3)
    assert detections[0].bbox == (10.0, 20.0, 30.0, 40.0)


def test_light_roi_bounds_stay_inside_frame_and_non_empty():
    config = AutonomousConfig()
    config.roi.detector_light = [-1.0, 0.25, 2.0, 0.25]

    assert BestPthDetector(config, "unused").light_roi_bounds((480, 640, 3)) == (
        0, 120, 640, 121
    )


def test_inference_roi_switches_from_light_crop_to_full_frame():
    detector = BestPthDetector(AutonomousConfig(), "unused")

    assert detector.inference_roi("OUT_WAIT_GREEN") == [0.00, 0.00, 0.80, 0.85]
    assert detector.inference_roi("OUT_TO_FORK") == [0.00, 0.00, 1.00, 1.00]
