"""Regression tests for detector/control timing boundaries."""

import cv2
import numpy as np

from bisa.dracer_config import AutonomousConfig
from bisa.lane_perception import LaneObs, LanePerception
from bisa.mission_controller import (
    LaneController,
    fresh_light_state,
    make_course_fsm,
)
from bisa.object_detector import DetectionBuffer


def test_light_vote_counts_unique_inference_frames_only():
    config = AutonomousConfig()
    config.detector.light_confirm_frames = 3
    fsm = make_course_fsm(config, LaneController(config))
    lane = LaneObs(valid=False)
    detections = DetectionBuffer()

    for now in (1.0, 1.1, 1.2):
        fsm.step(lane, detections, now, light_state="green", light_seq=7)

    assert fsm._green_streak == 1
    assert not fsm.green_confirmed()

    fsm.step(lane, detections, 1.3, light_state="green", light_seq=8)
    fsm.step(lane, detections, 1.4, light_state="green", light_seq=9)
    assert fsm.green_confirmed()


def test_missing_or_stale_light_clears_vote_streak():
    config = AutonomousConfig()
    fsm = make_course_fsm(config, LaneController(config))
    lane = LaneObs(valid=False)
    detections = DetectionBuffer()

    fsm.step(lane, detections, 1.0, light_state="red", light_seq=1)
    assert fsm._red_streak == 1
    fsm.step(lane, detections, 1.1, light_state=None, light_seq=1)
    assert fsm._red_streak == 0

    assert fresh_light_state(("green", 4, 10.0), 10.5, 0.75) == ("green", 4)
    assert fresh_light_state(("green", 4, 10.0), 10.8, 0.75) == (None, 4)
    assert fresh_light_state(("red", 5, 11.0), 10.0, 0.75) == (None, 5)


def test_lane_pipeline_prepares_lab_once_per_frame(monkeypatch):
    """Mask and Hough must reuse one LAB conversion and one CLAHE result."""

    config = AutonomousConfig()
    perception = LanePerception(config)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    original = cv2.cvtColor
    calls = 0

    def counted(*args, **kwargs):
        nonlocal calls
        calls += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(cv2, "cvtColor", counted)
    perception.compute_lane_obs(frame)
    assert calls == 1
