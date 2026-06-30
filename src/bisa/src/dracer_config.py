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


@dataclass
class DetectorConfig:
    """Stores model, class, confidence, ROI, and temporal filter parameters."""

    model_path: str = "checkpoints/best.pth"
    enabled: bool = True
    imgsz: int = 320
    inference_hz: float = 10.0
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
    sign_vote_k: int = 3
    sign_vote_n: int = 5
    dynamic_detect_consecutive: int = 2
    dynamic_clear_consecutive: int = 12
    red_consecutive_after_finish: int = 3


@dataclass
class ThrottleConfig:
    """Stores normalized throttle caps and ramp behavior for each mission state."""

    max: float = 0.40
    min_moving: float = 0.12
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
    lane: LaneVisionConfig = field(default_factory=LaneVisionConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    throttle: ThrottleConfig = field(default_factory=ThrottleConfig)
    steering: SteeringConfig = field(default_factory=SteeringConfig)
    rotary: RotaryConfig = field(default_factory=RotaryConfig)
    aruco: ArucoConfig = field(default_factory=ArucoConfig)
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
