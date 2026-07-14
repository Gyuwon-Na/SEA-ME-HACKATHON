"""Regression tests for detector/control timing boundaries."""

import cv2
import numpy as np
import pytest

from bisa.dracer_config import AutonomousConfig
from bisa.lane_perception import LaneObs, LanePerception, PathCandidate
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


def test_throttle_starts_at_min_then_reaches_max_on_straight():
    config = AutonomousConfig()
    controller = LaneController(config)

    values = [
        controller.throttle_scheduler(config.throttle.speed_max, 0.0, 0.0)
        for _ in range(20)
    ]

    assert values[0] == pytest.approx(config.throttle.speed_min)
    assert all(
        config.throttle.speed_min <= value <= config.throttle.speed_max
        for value in values
    )
    assert values[-1] == pytest.approx(config.throttle.speed_max)


def test_throttle_ignores_straight_noise_and_slows_for_curve():
    config = AutonomousConfig()
    controller = LaneController(config)
    controller.prev_throttle = config.throttle.speed_max

    straight = controller.throttle_scheduler(
        config.throttle.speed_max,
        config.throttle.straight_steer_deadband,
        config.throttle.straight_curvature_deadband,
    )
    curve = controller.throttle_scheduler(config.throttle.speed_max, 0.45, 0.50)

    assert straight == pytest.approx(config.throttle.speed_max)
    assert config.throttle.speed_min <= curve < config.throttle.speed_max


def test_fork_sections_keep_safe_speed_caps():
    config = AutonomousConfig()
    throttle = config.throttle

    assert throttle.fork_approach_cap == pytest.approx(0.24)
    assert throttle.fork_commit_cap == pytest.approx(0.22)
    assert throttle.post_fork_cap == pytest.approx(throttle.speed_max)
    assert all(
        throttle.speed_min <= cap <= throttle.speed_max
        for cap in (
            throttle.fork_approach_cap,
            throttle.fork_commit_cap,
            throttle.post_fork_cap,
        )
    )


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

    mask = perception.build_lane_mask(
        frame, prepared_lab=prepared_lab, include_yellow=True
    )

    assert mask.tolist() == [[255, 255, 0, 0]]


def test_out_lane_mask_excludes_yellow_split_line():
    config = AutonomousConfig()
    config.mission.route_mode = "OUT"
    config.lane.morph_open_kernel = 1
    config.lane.morph_close_kernel = 1
    perception = LanePerception(config)
    prepared_lab = np.array([[
        [230, 128, 128],  # white OUT boundary
        [210, 127, 190],  # yellow IN split
    ]], dtype=np.uint8)

    mask = perception.build_lane_mask(
        np.zeros((1, 2, 3), dtype=np.uint8), prepared_lab=prepared_lab
    )

    assert mask.tolist() == [[255, 0]]


class _SignDetector:
    def __init__(self, direction=None):
        self.direction = direction

    def count(self, name, _window):
        return 99 if name == self.direction else 0


def test_out_fsm_curve_follows_until_sign_then_resumes_after_fork():
    config = AutonomousConfig()
    config.detector.light_confirm_frames = 1
    config.mission.fork_sign_advance_sec = 0.2
    config.mission.fork_commit_min_sec = 0.1
    config.mission.fork_commit_timeout_sec = 0.3
    controller = LaneController(config)
    fsm = make_course_fsm(config, controller)
    lane = LaneObs(valid=True)
    no_sign = _SignDetector()

    fsm.step(lane, no_sign, 1.0, light_state="green", light_seq=1)
    assert fsm.state == "OUT_TO_FORK"
    # Elapsed time and S-shaped curvature never advance a mission state.
    curved_lane = LaneObs(valid=True, curvature=0.8, signed_curvature=0.5)
    cmd = fsm.step(curved_lane, no_sign, 20.0)
    assert fsm.state == "OUT_TO_FORK"
    assert config.throttle.speed_min <= cmd.throttle <= config.throttle.speed_max

    left = _SignDetector("sign_left")
    fsm.step(lane, left, 20.1)
    assert fsm.state == "OUT_FORK_SIGN_ADVANCE"
    advance_cmd = fsm.step(lane, left, 20.15)
    assert advance_cmd.steering > 0.0
    fsm.step(lane, left, 20.4)
    assert fsm.state == "OUT_FORK_COMMIT"

    fork_lane = LaneObs(
        valid=True,
        fork_seen=True,
        left_branch=PathCandidate(0.55, 0.1, 100.0),
    )
    cmd = fsm.step(fork_lane, left, 20.5)
    assert cmd.steering > 0.0
    for now in (20.6, 20.7, 20.8):
        fsm.step(lane, left, now)
    assert fsm.state == "OUT_RESUME"
    assert fsm.consume_lane_reset_request()

    # ArUco pause and final red stop are handled above the FSM by the autonomous
    # node, so these observations must not create hidden mission-state stops.
    fsm.step(lane, no_sign, 20.9, light_state="red", light_seq=2)
    assert fsm.state == "OUT_RESUME"
    fsm.step(lane, no_sign, 21.0, marker_visible=True)
    fsm.step(lane, no_sign, 21.1, marker_visible=True)
    assert fsm.state == "OUT_RESUME"


@pytest.mark.parametrize(
    ("sign_name", "expected_sign"),
    [("sign_left", 1.0), ("sign_right", -1.0)],
)
def test_out_sign_advance_forces_direction_before_x_is_visible(sign_name, expected_sign):
    """A confirmed sign must steer immediately even with no X branch contour."""

    config = AutonomousConfig()
    config.detector.light_confirm_frames = 1
    config.steering.rate_limit_per_cmd = 1.0
    fsm = make_course_fsm(config, LaneController(config))
    lane = LaneObs(valid=True, center_error=0.0, fork_seen=False)
    no_sign = _SignDetector()

    fsm.step(lane, no_sign, 1.0, light_state="green", light_seq=1)
    fsm.step(lane, _SignDetector(sign_name), 1.1)
    cmd = fsm.step(lane, _SignDetector(sign_name), 1.15)

    assert fsm.state == "OUT_FORK_SIGN_ADVANCE"
    assert cmd.steering * expected_sign > 0.0
    assert abs(cmd.steering) > 0.05


def test_lane_perception_reset_clears_all_pre_fork_fit_history():
    perception = LanePerception(AutonomousConfig())
    perception.prev_center_error = 0.6
    perception.prev_near_center = 100.0
    perception.prev_left_target = 50.0
    perception.prev_right_target = 350.0
    perception.prev_single_is_left = True
    perception.filtered_curvature = 0.8
    perception.tracked_lane_width = 300.0

    perception.reset_fork_history()

    assert perception.prev_center_error == 0.0
    assert perception.prev_near_center is None
    assert perception.prev_left_target is None
    assert perception.prev_right_target is None
    assert perception.prev_single_is_left is None
    assert perception.filtered_curvature == 0.0
    assert perception.tracked_lane_width is None


def test_out_startup_crawls_when_white_lane_is_not_yet_visible():
    config = AutonomousConfig()
    config.detector.light_confirm_frames = 1
    fsm = make_course_fsm(config, LaneController(config))
    no_sign = _SignDetector()

    fsm.step(LaneObs(valid=False), no_sign, 1.0, light_state="green", light_seq=1)
    cmd = fsm.step(LaneObs(valid=False), no_sign, 1.1)

    assert fsm.state == "OUT_TO_FORK"
    assert cmd.throttle == pytest.approx(config.throttle.speed_min)
    assert cmd.steering == pytest.approx(0.0)


def test_in_route_never_enters_out_states():
    config = AutonomousConfig()
    config.mission.route_mode = "IN"
    config.detector.light_confirm_frames = 1
    fsm = make_course_fsm(config, LaneController(config))

    fsm.step(
        LaneObs(valid=True),
        _SignDetector(),
        1.0,
        light_state="green",
        light_seq=1,
    )

    assert fsm.state == "IN_ENTRY"


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


def test_single_lane_keeps_tracked_right_identity_across_camera_center(monkeypatch):
    """A right boundary crossing image center must not become a left lane."""

    config = AutonomousConfig()
    perception = LanePerception(config)
    frame = np.zeros((240, 640, 3), dtype=np.uint8)
    lane_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    both_lines = np.array([
        [[210, 239, 280, 108]],
        [[550, 239, 370, 108]],
    ], dtype=np.int32)
    monkeypatch.setattr(cv2, "HoughLinesP", lambda *args, **kwargs: both_lines)
    first_center, _, _ = perception.average_hough_lanes(
        frame, prepared_l_channel=lane_mask, vehicle_x=320.0
    )

    # Bottom reference x=300 is camera-left, but the curve target remains near
    # the previously tracked right boundary.  Old code classified it as left.
    single_right = np.array([[[300, 239, 350, 108]]], dtype=np.int32)
    monkeypatch.setattr(cv2, "HoughLinesP", lambda *args, **kwargs: single_right)
    center, left_present, right_present = perception.average_hough_lanes(
        frame,
        prepared_l_channel=lane_mask,
        vehicle_x=320.0,
        previous_center=first_center,
    )

    assert not left_present and right_present
    assert perception.tracked_lane_width == pytest.approx(90.0, abs=1.0)
    assert center == pytest.approx(305.0, abs=1.0)


def test_hough_fit_range_shortens_with_curvature(monkeypatch):
    """Maximum curvature keeps only the configured lower fitting region."""

    config = AutonomousConfig()
    perception = LanePerception(config)
    frame = np.zeros((240, 640, 3), dtype=np.uint8)
    lines = np.array([[[210, 239, 280, 108]]], dtype=np.int32)
    monkeypatch.setattr(cv2, "HoughLinesP", lambda *args, **kwargs: lines)
    viz = {}

    perception.average_hough_lanes(
        frame,
        record=True,
        viz_out=viz,
        prepared_l_channel=np.zeros(frame.shape[:2], dtype=np.uint8),
        vehicle_x=320.0,
        curvature_hint=1.0,
    )

    assert viz["hough_top_y"] == int(config.lane.hough_curve_top_ratio * 240)


def test_curve_shortens_controller_lookahead_for_more_steering():
    """Equal lateral error must command more steering on a sharp curve."""

    config = AutonomousConfig()
    assert config.steering.wheelbase_m == pytest.approx(0.17)
    config.steering.rate_limit_per_cmd = 1.0
    straight_controller = LaneController(config)
    curve_controller = LaneController(config)

    straight = straight_controller.steering_from_lane(
        LaneObs(valid=True, center_error=0.6, curvature=0.0), steer_limit=1.0
    )
    curve = curve_controller.steering_from_lane(
        LaneObs(valid=True, center_error=0.6, curvature=1.0), steer_limit=1.0
    )

    assert abs(curve) > abs(straight)

    no_boost_config = AutonomousConfig()
    no_boost_config.steering.rate_limit_per_cmd = 1.0
    no_boost_config.steering.curve_steer_boost = 0.0
    no_boost = LaneController(no_boost_config).steering_from_lane(
        LaneObs(valid=True, center_error=0.6, curvature=1.0), steer_limit=1.0
    )
    assert abs(curve) > abs(no_boost)

    full_curve_controller = LaneController(config)
    full_curve = full_curve_controller.steering_from_lane(
        LaneObs(valid=True, center_error=1.0, curvature=1.0),
        steer_limit=config.steering.s_curve_limit,
    )
    assert 0.85 <= abs(full_curve) <= config.steering.s_curve_limit


def test_fork_curve_scale_prevents_false_curvature_cancellation():
    """Fork far-ROI curvature must not cancel a strong near-center error."""

    config = AutonomousConfig()
    config.steering.rate_limit_per_cmd = 1.0
    lane = LaneObs(
        valid=True,
        center_error=0.60,
        curvature=1.0,
        signed_curvature=-0.60,
    )

    unscaled = LaneController(config).steering_from_lane(
        lane,
        steer_limit=1.0,
        curve_scale=1.0,
    )
    fork_scaled = LaneController(config).steering_from_lane(
        lane,
        steer_limit=1.0,
        curve_scale=config.steering.fork_curve_scale,
    )

    assert unscaled == pytest.approx(0.0, abs=1e-6)
    assert fork_scaled > 0.0


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
