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
    optional_birdeye_y0: float = 0.35
    detector_light: list[float] = field(default_factory=lambda: [0.20, 0.00, 0.80, 0.55])
    detector_sign: list[float] = field(default_factory=lambda: [0.10, 0.05, 0.90, 0.70])
    detector_dynamic: list[float] = field(default_factory=lambda: [0.00, 0.35, 1.00, 1.00])


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

    black_hsv_lower: list[int] = field(default_factory=lambda: [0, 0, 0])
    black_hsv_upper: list[int] = field(default_factory=lambda: [180, 90, 90])
    morph_open_kernel: int = 3
    morph_close_kernel: int = 5
    min_component_area_ratio: float = 0.006
    fork_area_ratio: float = 0.035
    rotary_area_ratio: float = 0.08
    rotary_circularity_min: float = 0.18
    hough_roi_top_ratio: float = 0.58
    hough_canny_low: int = 50
    hough_canny_high: int = 150
    hough_threshold: int = 45
    hough_min_line_length: int = 45
    hough_max_line_gap: int = 290
    hough_slope_min_abs: float = 0.40
    assumed_lane_width_ratio: float = 0.62
    max_center_jump: float = 0.45
    # White/yellow lane-line detection (primary center source on this track:
    # dark mat with bright boundary lines). White is thresholded adaptively so
    # it survives lighting changes; yellow is a fixed HSV wedge.
    white_v_min: int = 140          # absolute floor for "white" brightness (V)
    white_v_percentile: float = 80.0  # adaptive floor: this percentile of ROI V
    white_s_max: int = 90           # white lines have low saturation
    line_yellow_h_lo: int = 15
    line_yellow_h_hi: int = 42
    line_yellow_s_min: int = 70
    line_yellow_v_min: int = 80
    line_col_row_frac: float = 0.10  # a column must be lit in >= this frac of band rows to count as a line
    lane_half_width_ratio: float = 0.30  # single-line offset (frac of ROI width) toward the missing line


@dataclass
class DetectorConfig:
    """Stores model, class, confidence, ROI, and temporal filter parameters."""

    model_path: str = "checkpoints/best.pth"
    enabled: bool = True
    imgsz: int = 320
    inference_hz: float = 10.0
    # Inference device: 'auto' uses CUDA when available (PC GPU) else CPU. The
    # heavy detector runs on the operator PC, so a GPU there offloads the weak
    # vehicle board entirely. Set 'cpu'/'cuda' to force one.
    device: str = "auto"
    class_map: dict[str, int] = field(default_factory=lambda: {
        "traffic_green": 0,
        "traffic_red": 1,
        "sign_left": 2,
        "sign_right": 3,
        "dynamic_marker": 4,
        "finish_line": 5,
    })
    conf: dict[str, float] = field(default_factory=lambda: {
        "traffic_green": 0.60,
        "traffic_red": 0.60,
        "sign_left": 0.65,
        "sign_right": 0.65,
        "dynamic_marker": 0.60,
        "finish_line": 0.55,
    })
    green_consecutive: int = 3
    green_vote_k: int = 3
    green_vote_n: int = 6
    sign_vote_k: int = 3
    sign_vote_n: int = 5
    dynamic_detect_consecutive: int = 2
    dynamic_clear_consecutive: int = 12
    red_consecutive_after_finish: int = 3


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
    rotary_approach_cap: float = 0.20
    rotary_inside_cap: float = 0.18
    dynamic_approach_cap: float = 0.18
    resume_cap: float = 0.30
    finish_cap: float = 0.34
    ramp_up_per_cmd: float = 0.03
    steer_slowdown: float = 0.22
    curvature_slowdown: float = 0.08


@dataclass
class SteeringConfig:
    """Stores normalized steering gains, sign, and per-state clamp values."""

    steer_sign: int = 1
    kp: float = 1.20
    ki: float = 0.0
    kd: float = 0.24
    kcurv: float = 0.30
    rate_limit_per_cmd: float = 0.10
    straight_limit: float = 0.45
    s_curve_limit: float = 0.80
    fork_approach_limit: float = 0.55
    fork_limit: float = 0.85
    post_fork_limit: float = 0.65
    dynamic_limit: float = 0.45
    rotary_limit: float = 0.85
    resume_limit: float = 0.60
    finish_limit: float = 0.55
    lost_decay: float = 0.70


@dataclass
class RotaryConfig:
    """Stores shortcut-course rotary progress and exit timing parameters."""

    direction: str = "CCW"
    min_rotation_time_sec: float = 4.5
    progress_threshold: float = 0.22
    exit_stable_frames: int = 3
    enter_ff: float = 0.35
    circulate_ff: float = 0.45
    exit_bias: float = -0.18


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
    """HSV color analysis of the traffic-light ROI (traffic_light_processor.py parity).

    Off by default. When ``enabled`` the analyzer runs the same red/yellow/green
    HSV masks (with HoughCircles confirmation for red/yellow) as the reference,
    over the ``roi.detector_light`` region, and emits traffic_green/traffic_red
    detections that feed the existing vote/FSM path. HSV bounds are stored as
    scalars so each maps to one tuning slider.
    """

    enabled: bool = False
    red_h1_hi: int = 10        # upper hue of the low-red band [0, red_h1_hi]
    red_h2_lo: int = 170       # lower hue of the high-red band [red_h2_lo, 180]
    red_s_min: int = 120
    red_v_min: int = 100
    yellow_h_lo: int = 15
    yellow_h_hi: int = 32
    yellow_s_min: int = 30
    yellow_v_min: int = 30
    green_h_lo: int = 40
    green_h_hi: int = 90
    green_s_min: int = 40
    green_v_min: int = 40
    # Fraction of one vertical band (1/3 of the light ROI) that must match its
    # color to count that lamp lit. Slider-tunable live.
    min_on_ratio: float = 0.01
    hough_min_dist: int = 20
    hough_param1: int = 50
    hough_param2: int = 10
    hough_min_radius: int = 2
    hough_max_radius: int = 30


@dataclass
class MissionConfig:
    """Stores mission timing and route-selection parameters."""

    route_mode: str = "OUT"
    control_hz: float = 10.0
    launch_min_sec: float = 1.0
    fork_commit_min_sec: float = 0.8
    fork_commit_timeout_sec: float = 1.8
    dynamic_stop_hold_sec: float = 0.2
    resume_min_sec: float = 1.0
    finish_min_elapsed_sec: float = 8.0
    dynamic_zone_elapsed_sec: float = 4.0
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
    rotary: RotaryConfig = field(default_factory=RotaryConfig)
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
