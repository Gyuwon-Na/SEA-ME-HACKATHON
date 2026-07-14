"""Regression tests for detector/control timing boundaries."""

import cv2
import numpy as np
import pytest

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


def test_lane_mask_keeps_white_and_yellow_but_rejects_road_and_skin():
    """The steering mask is paint-only, not the separate dark-road mask."""

    config = AutonomousConfig()
    config.lane.morph_open_kernel = 1
    config.lane.morph_close_kernel = 1
    perception = LanePerception(config)
    prepared_lab = np.array([[
        [230, 128, 128],  # white paint
        [210, 127, 190],  # yellow paint
        [90, 128, 135],   # dark road
        [180, 132, 150],  # skin-like obstacle
    ]], dtype=np.uint8)
    frame = np.zeros((1, 4, 3), dtype=np.uint8)

    mask = perception.build_lane_mask(frame, prepared_lab=prepared_lab)

    assert mask.tolist() == [[255, 255, 0, 0]]


def test_hough_selects_only_nearest_lane_on_each_side(monkeypatch):
    """Two markings on the left must not be averaged into the steering target."""

    config = AutonomousConfig()
    perception = LanePerception(config)
    frame = np.zeros((240, 640, 3), dtype=np.uint8)
    lane_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    lines = np.array([
        [[70, 239, 160, 108]],   # outer left marking
        [[210, 239, 280, 108]],  # nearest left marking (must win)
        [[550, 239, 370, 108]],  # nearest right marking
    ], dtype=np.int32)
    monkeypatch.setattr(cv2, "HoughLinesP", lambda *args, **kwargs: lines)
    viz = {}

    center, left_present, right_present = perception.average_hough_lanes(
        frame, record=True, viz_out=viz, prepared_l_channel=lane_mask, vehicle_x=320.0
    )

    assert left_present and right_present
    assert center == pytest.approx(325.0, abs=1.0)
    assert len(viz["hough_selected_segments"]) == 2
    assert viz["hough_left_curve"][0][0] == pytest.approx(280.0, abs=1.0)
    assert viz["hough_right_curve"][0][0] == pytest.approx(370.0, abs=1.0)


def test_hough_keeps_one_sided_driving(monkeypatch):
    """When all visible markings are left of center, use the nearest one only."""

    config = AutonomousConfig()
    perception = LanePerception(config)
    frame = np.zeros((240, 640, 3), dtype=np.uint8)
    lines = np.array([
        [[70, 239, 160, 108]],
        [[210, 239, 280, 108]],
    ], dtype=np.int32)
    monkeypatch.setattr(cv2, "HoughLinesP", lambda *args, **kwargs: lines)

    center, left_present, right_present = perception.average_hough_lanes(
        frame, prepared_l_channel=np.zeros(frame.shape[:2], dtype=np.uint8), vehicle_x=320.0
    )

    assert left_present and not right_present
    expected = 280.0 + 640 * config.lane.assumed_lane_width_ratio / 2.0
    assert center == pytest.approx(expected, abs=1.0)


def test_hough_curve_fit_is_second_order():
    """The selected lane model must retain measurable quadratic curvature."""

    top_y, height, width = 100, 241, 640
    bottom_y = height - 1
    ys = np.linspace(top_y, bottom_y, 7)
    t = (ys - top_y) / (bottom_y - top_y)
    xs = 24.0 * t * t + 55.0 * t + 120.0
    coefficients = LanePerception._fit_hough_curve(
        np.column_stack((xs, ys)), top_y, height, width
    )

    assert coefficients is not None
    assert coefficients == pytest.approx([24.0, 55.0, 120.0], abs=1e-6)
