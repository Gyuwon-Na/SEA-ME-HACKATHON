"""Configuration helpers for the BISA D-Racer autonomous node."""

from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Mapping

try:
    import yaml
except ModuleNotFoundError:  # pragma: no cover - target ROS images usually include PyYAML.
    yaml = None


@dataclass
class RoiConfig:
    """Stores image ROI ratios used by the classical vision pipeline."""

    near_y0: float = 0.70
    mid_y0: float = 0.52
    mid_y1: float = 0.75
    far_y0: float = 0.32
    far_y1: float = 0.58
    detector_light: list[float] = field(default_factory=lambda: [0.20, 0.00, 0.80, 0.55])
    detector_sign: list[float] = field(default_factory=lambda: [0.10, 0.05, 0.90, 0.70])


@dataclass
class LaneRoiConfig:
    """Pixel-space rectangular ROI that limits the classical lane pipeline only.

    The full camera frame is still used for traffic-light, sign, and ArUco
    detection; cropping happens solely for lane finding so the lane search area
    can stay small (e.g. 640x240) while the camera keeps its native size.

    ``width``/``height`` are live-tunable via sliders (rqt_reconfigure /
    ``ros2 param set``). A negative ``x_offset`` centers the box horizontally and
    a negative ``y_offset`` anchors it to the bottom of the frame, so by default
    the ROI sits on the bottom strip where the lane appears.
    """

    enabled: bool = True
    width: int = 640
    height: int = 240
    x_offset: int = -1
    y_offset: int = -1


@dataclass
class LaneVisionConfig:
    """Stores tunable parameters for road mask and Hough fallback lane detection."""

    # LAB road mask: the mask comes from cv2.inRange over the CLAHE-equalized
    # LAB image using these per-channel min/max bounds. Tune them live from the
    # "Lane ROI Mask" debug window; OpenCV 8-bit LAB ranges are L 0..255 and
    # a/b 0..255 with 128 as the neutral (gray) point.
    lab_l_min: int = 20
    lab_l_max: int = 205
    lab_a_min: int = 112
    lab_a_max: int = 145
    lab_b_min: int = 122
    lab_b_max: int = 148
    white_l_min: int = 200
    white_l_max: int = 255
    white_a_min: int = 115
    white_a_max: int = 140
    white_b_min: int = 115
    white_b_max: int = 145
    yellow_l_min: int = 80
    yellow_l_max: int = 255
    yellow_a_min: int = 100
    yellow_a_max: int = 140
    yellow_b_min: int = 165
    yellow_b_max: int = 255
    # CLAHE applied to the L channel before LAB thresholding AND before the
    # Canny/Hough lane-line pass (previously hardcoded clip=2.0 tile=8).
    lab_clahe_clip: float = 2.0
    lab_clahe_tile: int = 8
    morph_open_kernel: int = 3
    morph_close_kernel: int = 5
    min_component_area_ratio: float = 0.006
    fork_area_ratio: float = 0.035
    hough_roi_top_ratio: float = 0.45
    hough_curve_top_ratio: float = 0.62
    hough_curvature_smoothing: float = 0.25
    hough_canny_low: int = 50
    hough_canny_high: int = 150
    hough_threshold: int = 38
    hough_min_line_length: int = 30
    hough_max_line_gap: int = 290
    hough_slope_min_abs: float = 0.22
    assumed_lane_width_ratio: float = 0.62
    lane_width_min_ratio: float = 0.10
    lane_width_max_ratio: float = 0.70
    lane_width_smoothing: float = 0.20
    single_lane_switch_margin_ratio: float = 0.08
    max_center_jump: float = 0.35


@dataclass
class DetectorConfig:
    """Stores model, class, confidence, ROI, and temporal filter parameters."""

    model_path: str = "checkpoints/best.pt"
    enabled: bool = True
    imgsz: int = 640
    inference_hz: float = 30.0
    # NCNN defaults to every CPU core. On the 4-core TOPST that starves camera,
    # lane, and ROS callbacks, so reserve cores for the rest of the stack.
    ncnn_threads: int = 2
    # Build the NCNN/Vulkan pipelines in the background before mission use.
    warmup_enabled: bool = True
    # Inference device: 'auto' uses CUDA when available (PC GPU) else CPU. The
    # heavy detector runs on the operator PC, so a GPU there offloads the weak
    # vehicle board entirely. Set 'cpu'/'cuda' to force one.
    device: str = "auto"
    # Fallback only: class names normally come from the model's own metadata
    # (BestPthDetector.NAME_ALIASES). Matches the deployed 2026-07-08 model:
    # 0=left_sign, 1=right_sign, 2=red_light, 3=green_light.
    class_map: dict[str, int] = field(default_factory=lambda: {
        "sign_left": 0,
        "sign_right": 1,
        "traffic_red": 2,
        "traffic_green": 3,
    })
    conf: dict[str, float] = field(default_factory=lambda: {
        "traffic_green": 0.40,
        "traffic_red": 0.40,
        "sign_left": 0.65,
        "sign_right": 0.65,
    })
    sign_vote_k: int = 9
    sign_vote_n: int = 15
    # Traffic light: the mission launches on green and stops on red directly from
    # the classify_light verdict (the value the tuner shows). To reject a single
    # glitch frame, the same verdict must hold for this many consecutive control
    # ticks before it acts (control_hz=10 => 3 ticks = 0.3 s).
    light_confirm_frames: int = 3
    # A detector verdict older than this is never reused by the control FSM.
    light_stale_sec: float = 0.75


@dataclass
class ThrottleConfig:
    """Stores normalized throttle caps and ramp behavior for each mission state.

    ``speed_min``/``speed_max`` are the single source of truth for the moving
    speed band: every commanded throttle (while not stopped) is mapped into
    ``[speed_min, speed_max]``. Change these two to retune the overall speed.
    The per-state ``*_cap`` values only shape the *relative* speed inside that
    band; they are clamped into it.
    """

    speed_min: float = 0.20
    speed_max: float = 0.30
    launch_cap: float = 0.32
    s_curve_cap: float = 0.24
    fork_approach_cap: float = 0.20
    fork_commit_cap: float = 0.25
    post_fork_cap: float = 0.30
    post_fork_min: float = 0.25
    ramp_up_per_cmd: float = 0.03
    steer_slowdown: float = 0.22
    curvature_slowdown: float = 0.08


@dataclass
class SteeringConfig:
    """Stores pure-pursuit steering geometry, sign, and per-state clamp values.

    Steering is computed with pure pursuit: the normalized lane center error is
    mapped to a lateral offset of ``lateral_scale_m`` meters (at error 1.0) at a
    target point ``lookahead_m`` ahead, then the bicycle-model steering angle
    ``atan2(2 * wheelbase_m * sin(alpha), lookahead_m)`` is normalized into the
    [-1, 1] command range by ``max_steer_deg``. ``curve_blend`` shifts the aim
    point along the road's bend using the signed curvature cue.
    """

    steer_sign: int = 1
    lookahead_m: float = 0.60
    curve_lookahead_min_m: float = 0.38
    curve_response_power: float = 0.65
    curve_steer_boost: float = 0.20
    fork_curve_scale: float = 0.25
    wheelbase_m: float = 0.17
    lateral_scale_m: float = 0.30
    max_steer_deg: float = 30.0
    pp_gain: float = 1.0
    curve_blend: float = 1.0
    rate_limit_per_cmd: float = 0.12
    straight_limit: float = 0.60
    s_curve_limit: float = 0.95
    fork_approach_limit: float = 0.75
    fork_limit: float = 0.95
    post_fork_limit: float = 0.90
    lost_decay: float = 0.70


@dataclass
class ArucoConfig:
    """Stores ArUco marker detection parameters for the PC-side visualization."""

    enabled: bool = True
    dictionary: str = "DICT_6X6_50"
    target_id: int = 3
    detect_hz: float = 10.0


@dataclass
class ColorCorrectionConfig:
    """Frame preprocessing shared before detection (traffic_light_processor.py parity).

    Off by default so on-vehicle runs pay nothing. When ``enabled`` the frame fed
    to the detector/traffic-light analyzer goes through CLAHE -> saturation boost
    -> brightness/contrast/saturation/gamma, mirroring the reference pipeline. The
    lane and debug-overlay frames keep using the raw image.
    """

    enabled: bool = False
    clahe_clip: float = 2.0
    clahe_tile: int = 8
    saturation_boost: float = 1.5
    brightness: int = 0        # additive, -50..50 (convertScaleAbs beta)
    contrast: float = 1.0      # multiplicative, 0..3 (convertScaleAbs alpha)
    saturation: float = 1.0    # HSV S scale, 0..3
    gamma: float = 1.0         # gamma LUT, 0.1..3


@dataclass
class TrafficLightConfig:
    """Red/green color classification of a YOLO-detected traffic-light box.

    The YOLO box is split into vertical thirds; the top (red) and bottom (green)
    thirds are scored and the higher wins. The amber middle third is ignored
    (mission is red/green only; the competition light tapes it over). HSV bounds
    are scalars so each maps to one tuning slider.
    """

    red_h1_hi: int = 10        # upper hue of the low-red band [0, red_h1_hi]
    red_h2_lo: int = 170       # lower hue of the high-red band [red_h2_lo, 180]
    red_s_min: int = 120
    red_v_min: int = 100
    green_h_lo: int = 40
    green_h_hi: int = 90
    green_s_min: int = 50
    green_v_min: int = 50
    # Winning third must have at least this fraction of its pixels match (color
    # classifier) or lit (lit classifier), else the box is dropped.
    row_min_ratio: float = 0.02
    # Used only by the "lit" classifier: a pixel counts as lit when it is bright
    # (V >= row_lit_v_min) AND inside the row color mask, or near-white
    # (S <= row_white_s_max — an emitting lamp's blown-out core) next to such
    # bright colored pixels, so a bright background wall never scores.
    row_lit_v_min: int = 220
    row_white_s_max: int = 80
    # Used only by the "lab" classifier. OpenCV 8-bit LAB centers the green<->red
    # ``a`` channel at 128: a >= lab_a_red_min is red, a <= lab_a_green_max is
    # green; only pixels with L >= lab_l_min count (drops dark background).
    lab_a_red_min: int = 150
    lab_a_green_max: int = 110
    lab_l_min: int = 40
    # Which classifier the driving pipeline uses to read a light box:
    #   "color" - reference pure per-row color-pixel ratio (no brightness gate).
    #             DEFAULT: the competition light has its inactive lamps covered
    #             with black tape, so the only colored region is the active lamp
    #             and pure-color counting cannot be fooled by an unlit lens.
    #             No strict brightness gate, so it also misses a dim / sun-washed
    #             lamp less often. Field-proven outdoors.
    #   "lit"   - brightness-gated emitting detection (V>=row_lit_v_min +
    #             white-core-near-ring). Needed only when unlit lenses stay
    #             colored (no tape); adds a strict V>=220 gate that can miss a
    #             dim lamp.
    #   "lab"   - LAB a-channel (red = a high, green = a low). No hue wraparound,
    #             steadier under changing light, and catches a cyan-ish green
    #             lamp the HSV green range can clip. Flip here for a venue A/B.
    classifier: str = "color"


@dataclass
class MissionConfig:
    """Stores mission timing and route-selection parameters."""

    route_mode: str = "OUT"
    control_hz: float = 10.0
    launch_min_sec: float = 1.0
    fork_commit_min_sec: float = 0.8
    fork_commit_timeout_sec: float = 1.8
    finish_min_elapsed_sec: float = 8.0
    debug_log_hz: float = 1.0


@dataclass
class AutonomousConfig:
    """Top-level config object shared by perception, detection, and control."""

    roi: RoiConfig = field(default_factory=RoiConfig)
    lane_roi: LaneRoiConfig = field(default_factory=LaneRoiConfig)
    lane: LaneVisionConfig = field(default_factory=LaneVisionConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    throttle: ThrottleConfig = field(default_factory=ThrottleConfig)
    steering: SteeringConfig = field(default_factory=SteeringConfig)
    aruco: ArucoConfig = field(default_factory=ArucoConfig)
    color_correction: ColorCorrectionConfig = field(default_factory=ColorCorrectionConfig)
    traffic_light: TrafficLightConfig = field(default_factory=TrafficLightConfig)
    mission: MissionConfig = field(default_factory=MissionConfig)


def deep_update_dataclass(target: Any, values: Mapping[str, Any]) -> None:
    """Recursively copies mapping values into an existing dataclass instance."""

    if not is_dataclass(target):
        raise TypeError("target must be a dataclass instance")

    valid_names = {item.name for item in fields(target)}
    for key, value in values.items():
        if key not in valid_names:
            continue
        current_value = getattr(target, key)
        if is_dataclass(current_value) and isinstance(value, Mapping):
            deep_update_dataclass(current_value, value)
        else:
            setattr(target, key, value)


def load_config(config_path: str | None = None) -> AutonomousConfig:
    """Loads YAML configuration and merges it over conservative defaults."""

    config = AutonomousConfig()
    if not config_path:
        return config
    if yaml is None:
        return config

    path = Path(config_path).expanduser()
    if not path.exists():
        return config

    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if isinstance(data, Mapping):
        deep_update_dataclass(config, data)
    return config


def resolve_package_relative_path(base_file: str, maybe_relative_path: str) -> str:
    """Resolves package-relative paths such as checkpoints/best.pth."""

    path = Path(maybe_relative_path).expanduser()
    if path.is_absolute():
        return str(path)

    package_root = Path(base_file).resolve().parents[1]
    return str((package_root / maybe_relative_path).resolve())
