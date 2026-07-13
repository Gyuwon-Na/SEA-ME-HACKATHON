"""Traffic-light color classification and shared frame preprocessing.

Pipeline: YOLO (``best.pt``) localizes the traffic-light box; its class label is
discarded. :func:`classify_light` then splits the box into vertical thirds and
decides the mission color from WHICH third is showing the lamp — top third =
red, bottom third = green. The middle (amber) third is ignored: the mission
only cares about red/green and the competition light has its amber lamp taped
over. Two scoring strategies are selectable via ``traffic_light.classifier``:

* ``"color"`` (:func:`classify_light_rows_color`, default) — the reference
  ``traffic_light_processor.py`` algorithm: pure per-row color-pixel ratio, no
  brightness gate. Robust when the inactive lamps are taped black (nothing else
  is colored) and it never misses a dim / sun-washed lamp for lack of bright-
  ness.
* ``"lit"`` (:func:`classify_light_rows`) — brightness-gated: a pixel counts
  only when it is bright (V >= ``row_lit_v_min``) and either matches the row
  color or is a near-white lamp core next to that color. Kept as a toggle for
  hardware where unlit lenses stay colored (no tape).
* ``"lab"`` (:func:`classify_light_rows_lab`) — LAB a-channel: red = a high,
  green = a low. ``a`` IS the green<->red axis, so one threshold per side
  replaces the two-band red hue range and is steadier under changing light.

The CLAHE -> saturation-boost -> brightness/contrast/gamma correction chain
(:func:`apply_correction_chain` / :func:`preprocess_frame`) mirrors the
reference's pre-detection enhancement and can run before YOLO.

This module has two parts:

* Sections 1-2 (preprocessing + classification) — pure OpenCV / NumPy, the code
  the driving pipeline calls every frame. Importing this module runs none of the
  ROS/GUI code below, so the classifiers stay cheap to import.
* Section 3 — the standalone tuning tool (:class:`TlTunerNode`, ``main``), merged
  in from the former ``traffic_light_tuner.py``. It reuses the very functions in
  sections 1-2, so every value dialed in maps 1:1 onto ``dracer_params.yaml``.
  Its ROS node and OpenCV windows start only when ``main`` is invoked
  (``ros2 run bisa traffic_light_tuner``).
"""

from __future__ import annotations

import threading
import time
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from .dracer_config import (
    AutonomousConfig,
    ColorCorrectionConfig,
    load_config,
    resolve_package_relative_path,
)
from .object_detector import BestPthDetector


_opencv_cache = threading.local()


def _cached_clahe(clip: float, tile: int):
    """Caches one CLAHE instance per thread and current tuning values."""

    key = (max(float(clip), 0.01), max(1, int(tile)))
    if getattr(_opencv_cache, "clahe_key", None) != key:
        _opencv_cache.clahe = cv2.createCLAHE(
            clipLimit=key[0], tileGridSize=(key[1], key[1])
        )
        _opencv_cache.clahe_key = key
    return _opencv_cache.clahe


@lru_cache(maxsize=32)
def _gamma_lut(gamma: float) -> np.ndarray:
    """Builds and caches the 256-entry gamma LUT used by live tuning."""

    inv_gamma = 1.0 / max(float(gamma), 0.1)
    return np.array(
        [((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8
    )


def apply_clahe(bgr: np.ndarray, clip: float = 2.0, tile: int = 8) -> np.ndarray:
    """Equalizes the LAB L-channel with CLAHE (reference apply_clahe)."""

    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    l_ch = _cached_clahe(clip, tile).apply(l_ch)
    return cv2.cvtColor(cv2.merge((l_ch, a_ch, b_ch)), cv2.COLOR_LAB2BGR)


def adjust_saturation(bgr: np.ndarray, factor: float = 1.0) -> np.ndarray:
    """Scales HSV saturation by ``factor`` (reference adjust_saturation)."""

    if abs(float(factor) - 1.0) < 1e-3:
        return bgr
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h_ch, s_ch, v_ch = cv2.split(hsv)
    s_ch = np.clip(s_ch.astype(np.float32) * float(factor), 0, 255).astype(np.uint8)
    return cv2.cvtColor(cv2.merge((h_ch, s_ch, v_ch)), cv2.COLOR_HSV2BGR)


def apply_color_correction(
    bgr: np.ndarray,
    brightness: int = 0,
    contrast: float = 1.0,
    saturation: float = 1.0,
    gamma: float = 1.0,
) -> np.ndarray:
    """Applies brightness/contrast/saturation/gamma (reference apply_color_correction)."""

    out = cv2.convertScaleAbs(bgr, alpha=float(contrast), beta=float(brightness))
    if abs(float(saturation) - 1.0) >= 1e-3:
        hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)
        h_ch, s_ch, v_ch = cv2.split(hsv)
        s_ch = np.clip(s_ch * float(saturation), 0, 255)
        out = cv2.cvtColor(cv2.merge((h_ch, s_ch, v_ch)).astype(np.uint8), cv2.COLOR_HSV2BGR)
    gamma = max(float(gamma), 0.1)
    if abs(gamma - 1.0) >= 1e-3:
        out = cv2.LUT(out, _gamma_lut(round(gamma, 4)))
    return out


def apply_correction_chain(bgr: np.ndarray | None, cfg: ColorCorrectionConfig) -> np.ndarray | None:
    """Runs the CLAHE -> saturation -> brightness/contrast/gamma chain unconditionally.

    Ignores ``cfg.enabled`` so the PC Detect View can always show the current
    slider settings for live tuning, even when the detector itself is fed the
    raw frame.
    """

    if bgr is None:
        return bgr
    out = apply_clahe(bgr, cfg.clahe_clip, cfg.clahe_tile)
    out = adjust_saturation(out, cfg.saturation_boost)
    out = apply_color_correction(out, cfg.brightness, cfg.contrast, cfg.saturation, cfg.gamma)
    return out


def preprocess_frame(bgr: np.ndarray | None, cfg: ColorCorrectionConfig) -> np.ndarray | None:
    """Runs the correction chain for the DETECTOR path when enabled; input otherwise."""

    if bgr is None or not getattr(cfg, "enabled", False):
        return bgr
    return apply_correction_chain(bgr, cfg)


def _hsv_ranges(tl) -> dict:
    """Builds the red/green (lower, upper) HSV bounds from scalar config fields."""

    return {
        "red1": (np.array([0, tl.red_s_min, tl.red_v_min]),
                 np.array([tl.red_h1_hi, 255, 255])),
        "red2": (np.array([tl.red_h2_lo, tl.red_s_min, tl.red_v_min]),
                 np.array([180, 255, 255])),
        "green": (np.array([tl.green_h_lo, tl.green_s_min, tl.green_v_min]),
                  np.array([tl.green_h_hi, 255, 255])),
    }


def _color_mask(hsv: np.ndarray, color: str, rng: dict) -> np.ndarray:
    """Builds the HSV mask for one color (red spans two hue bands)."""

    if color == "red":
        return cv2.bitwise_or(
            cv2.inRange(hsv, rng["red1"][0], rng["red1"][1]),
            cv2.inRange(hsv, rng["red2"][0], rng["red2"][1]),
        )
    return cv2.inRange(hsv, rng[color][0], rng[color][1])


def clamp_i(value, low, high):
    """Clamps a numeric value into an inclusive integer-friendly range."""

    return max(low, min(high, value))


def _light_bands(frame_bgr, bbox, config: AutonomousConfig):
    """Crops a light box and returns ``(top_bgr, bottom_bgr, tl)``.

    The box is split into vertical thirds; the top third (red lamp) and bottom
    third (green lamp) are kept as BGR crops (each classifier converts to its
    own color space), the middle (amber) third is dropped. Returns ``None``
    when the box is empty/degenerate.
    """

    if frame_bgr is None:
        return None
    height, width = frame_bgr.shape[:2]
    x1 = int(clamp_i(bbox[0], 0, width - 1))
    y1 = int(clamp_i(bbox[1], 0, height - 1))
    x2 = int(clamp_i(bbox[2], 0, width))
    y2 = int(clamp_i(bbox[3], 0, height))
    if x2 <= x1 or y2 <= y1:
        return None
    crop = frame_bgr[y1:y2, x1:x2]
    band_h = max(1, crop.shape[0] // 3)
    return crop[:band_h], crop[2 * band_h:], config.traffic_light


def _decide(red_score: float, green_score: float, tl) -> str | None:
    """Picks the higher-scoring lamp if it clears ``row_min_ratio``, else None."""

    if max(red_score, green_score) < float(tl.row_min_ratio):
        return None
    return "traffic_red" if red_score >= green_score else "traffic_green"


def classify_light(
    frame_bgr, bbox, config: AutonomousConfig
) -> tuple[str | None, tuple[float, float]]:
    """Dispatches to the row classifier named by ``traffic_light.classifier``.

    ``"color"`` (default) = reference pure-color HSV ratio; ``"lit"`` =
    brightness-gated HSV; ``"lab"`` = LAB a-channel (red = a high, green = a
    low). One switch point so the driving pipeline and the tuner stay in sync.
    Returns ``(mission_cls, (red_score, green_score))``; ``mission_cls`` is
    ``traffic_red``/``traffic_green`` or None.
    """

    mode = str(getattr(config.traffic_light, "classifier", "color")).lower()
    if mode == "lit":
        return classify_light_rows(frame_bgr, bbox, config)
    if mode == "lab":
        return classify_light_rows_lab(frame_bgr, bbox, config)
    return classify_light_rows_color(frame_bgr, bbox, config)


def classify_light_rows_color(
    frame_bgr, bbox, config: AutonomousConfig
) -> tuple[str | None, tuple[float, float]]:
    """Reference algorithm: pure per-row COLOR pixel ratio, NO brightness gate.

    Ports ``traffic_light_processor.analyze_traffic_light``: the top third is
    scored by its red-mask pixel fraction, the bottom third by its green-mask
    fraction, and the higher wins if it clears ``row_min_ratio``. With the
    competition light's inactive lamps taped black, the only colored region is
    the active lamp, so pure color counting cannot be fooled by an unlit lens.
    """

    bands = _light_bands(frame_bgr, bbox, config)
    if bands is None:
        return None, (0.0, 0.0)
    top, bottom, tl = bands
    rng = _hsv_ranges(tl)

    def color_ratio(band_bgr, color):
        if band_bgr.size == 0:
            return 0.0
        hsv = cv2.cvtColor(band_bgr, cv2.COLOR_BGR2HSV)
        matched = _color_mask(hsv, color, rng) > 0
        return float(np.count_nonzero(matched)) / float(band_bgr.shape[0] * band_bgr.shape[1])

    red_score = color_ratio(top, "red")
    green_score = color_ratio(bottom, "green")
    return _decide(red_score, green_score, tl), (red_score, green_score)


def classify_light_rows(
    frame_bgr, bbox, config: AutonomousConfig
) -> tuple[str | None, tuple[float, float]]:
    """Brightness-gated classifier: only LIT (emitting) pixels score.

    Same top=red / bottom=green split as the color classifier, but a pixel
    counts only when bright (V >= ``row_lit_v_min``) and either matches the row
    color or is a near-white lamp core (S <= ``row_white_s_max``) NEXT TO such
    bright colored pixels. Rejects an unlit-but-colored lens and a bright
    background wall leaking into the box. Selected by ``classifier: lit`` for
    hardware whose inactive lenses are not taped over.
    """

    bands = _light_bands(frame_bgr, bbox, config)
    if bands is None:
        return None, (0.0, 0.0)
    top, bottom, tl = bands
    rng = _hsv_ranges(tl)

    def lit_ratio(band_bgr, color):
        if band_bgr.size == 0:
            return 0.0
        band = cv2.cvtColor(band_bgr, cv2.COLOR_BGR2HSV)
        bright = band[:, :, 2] >= int(tl.row_lit_v_min)
        colored_lit = (bright & (_color_mask(band, color, rng) > 0)).astype(np.uint8)
        # The white core only counts within one dilation step of the bright
        # colored ring, so a bright background wall (white, no color ring)
        # never scores as a lit lamp.
        kernel = max(3, band.shape[0] // 3)
        near_ring = cv2.dilate(colored_lit, np.ones((kernel, kernel), np.uint8)) > 0
        whitish = bright & (band[:, :, 1] <= int(tl.row_white_s_max))
        lit = (colored_lit > 0) | (whitish & near_ring)
        return float(np.count_nonzero(lit)) / float(lit.shape[0] * lit.shape[1])

    red_score = lit_ratio(top, "red")
    green_score = lit_ratio(bottom, "green")
    return _decide(red_score, green_score, tl), (red_score, green_score)


def classify_light_rows_lab(
    frame_bgr, bbox, config: AutonomousConfig
) -> tuple[str | None, tuple[float, float]]:
    """LAB a-channel classifier: red = a high, green = a low.

    OpenCV 8-bit LAB centers the green<->red ``a`` channel at 128. The top
    third scores the fraction of pixels with ``a >= lab_a_red_min`` (reddish),
    the bottom third the fraction with ``a <= lab_a_green_max`` (greenish);
    only pixels with ``L >= lab_l_min`` count, so dark background is ignored.
    Because ``a`` IS the red-green axis, one threshold each side replaces the
    two-band red hue range and is steadier under changing light. Selected by
    ``classifier: lab``.
    """

    bands = _light_bands(frame_bgr, bbox, config)
    if bands is None:
        return None, (0.0, 0.0)
    top, bottom, tl = bands

    def lab_ratio(band_bgr, want):
        if band_bgr.size == 0:
            return 0.0
        lab = cv2.cvtColor(band_bgr, cv2.COLOR_BGR2LAB)
        bright = lab[:, :, 0] >= int(tl.lab_l_min)
        a_ch = lab[:, :, 1]
        if want == "red":
            matched = bright & (a_ch >= int(tl.lab_a_red_min))
        else:
            matched = bright & (a_ch <= int(tl.lab_a_green_max))
        return float(np.count_nonzero(matched)) / float(band_bgr.shape[0] * band_bgr.shape[1])

    red_score = lab_ratio(top, "red")
    green_score = lab_ratio(bottom, "green")
    return _decide(red_score, green_score, tl), (red_score, green_score)


# ============================================================================
# Section 3: standalone tuning tool (PC only) — merged from traffic_light_tuner.
#
# Verifies red/green detection live and tunes the values that affect it. It runs
# the exact pipeline path above — apply_correction_chain -> BestPthDetector ->
# classify_light (honoring ``traffic_light.classifier``) — so every value dialed
# in maps 1:1 onto ``dracer_params.yaml``. Nothing here runs on ``import``; the
# ROS node and OpenCV windows start only in :func:`main`.
#
# One "Detect" window shows the color-corrected frame the detector sees, each
# YOLO light box drawn and colored by the classifier verdict. The "TL Controls"
# window holds the sliders of the ACTIVE classifier (cycle with 'm'):
#   * color correction + row_min_ratio + conf + imgsz are always shown;
#   * color/lit modes add the green/red HSV bounds (lit adds two more);
#   * lab mode adds the LAB a-channel / L thresholds INSTEAD of HSV.
# Keys: 'm' cycles color->lit->lab (rebuilding sliders), 's' dumps a YAML block,
# 'q' quits.
#
# Run:  ros2 run bisa traffic_light_tuner
#       ros2 run bisa traffic_light_tuner --ros-args -p video_path:=/path/clip.mp4
# ============================================================================

CONTROLS = "TL Controls"
WIN_DETECT = "Detect"

# (trackbar label, config path, kind, lo, hi)
#   kind 'x10'/'x100' store value*10 or *100; 'boff' stores value+50 (bipolar);
#   'imgsz32' stores imgsz/32; 'raw' stores the integer directly.
# Color correction runs before the detector so boosting saturation / CLAHE makes
# the lit lamp pop and detect better — the Detect window ALWAYS shows the
# corrected frame, so the effect is visible live.
CC_SLIDERS = [
    ("clahe_clip x10", "color_correction.clahe_clip", "x10", 0, 80),
    ("clahe_tile", "color_correction.clahe_tile", "raw", 1, 16),
    ("sat_boost x10", "color_correction.saturation_boost", "x10", 0, 30),
    ("brightness +50", "color_correction.brightness", "boff", 0, 100),
    ("contrast x10", "color_correction.contrast", "x10", 0, 30),
    ("saturation x10", "color_correction.saturation", "x10", 0, 30),
    ("gamma x10", "color_correction.gamma", "x10", 1, 30),
]
# HSV bounds for the color / lit classifiers.
HSV_SLIDERS = [
    ("green H lo", "traffic_light.green_h_lo", "raw", 0, 180),
    ("green H hi", "traffic_light.green_h_hi", "raw", 0, 180),
    ("green S min", "traffic_light.green_s_min", "raw", 0, 255),
    ("green V min", "traffic_light.green_v_min", "raw", 0, 255),
    ("red H1 hi", "traffic_light.red_h1_hi", "raw", 0, 180),
    ("red H2 lo", "traffic_light.red_h2_lo", "raw", 0, 180),
    ("red S min", "traffic_light.red_s_min", "raw", 0, 255),
    ("red V min", "traffic_light.red_v_min", "raw", 0, 255),
]
# Extra brightness gates the lit classifier adds on top of HSV_SLIDERS.
LIT_SLIDERS = [
    ("row lit V min", "traffic_light.row_lit_v_min", "raw", 0, 255),
    ("row white S max", "traffic_light.row_white_s_max", "raw", 0, 255),
]
# LAB a-channel / L thresholds for the lab classifier (replace HSV_SLIDERS).
LAB_SLIDERS = [
    ("lab a red min", "traffic_light.lab_a_red_min", "raw", 128, 255),
    ("lab a green max", "traffic_light.lab_a_green_max", "raw", 0, 128),
    ("lab L min", "traffic_light.lab_l_min", "raw", 0, 255),
]
TAIL_SLIDERS = [
    ("row min ratio x100", "traffic_light.row_min_ratio", "x100", 0, 50),
    ("conf x100", "detector.conf", "x100", 1, 100),
    ("imgsz /32", "detector.imgsz", "imgsz32", 5, 30),
]
CLASSIFIER_CYCLE = ("color", "lit", "lab")


def sliders_for(mode: str) -> list:
    """Returns the trackbar set for a classifier: LAB gets LAB filters, HSV modes
    get the green/red bounds (lit adds two), plus the always-on CC/tail sliders."""

    sliders = list(CC_SLIDERS)
    if str(mode).lower() == "lab":
        sliders += LAB_SLIDERS
    else:
        sliders += HSV_SLIDERS
        if str(mode).lower() == "lit":
            sliders += LIT_SLIDERS
    return sliders + TAIL_SLIDERS


def _find_package_file(rel_path: str, fallback: str) -> str:
    """Finds a package data file: installed share dir first, then source tree.

    Shared by the config- and model-path lookups so the two stay identical.
    """

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("bisa")) / rel_path
        if installed.exists():
            return str(installed)
    except Exception:
        pass
    for base in Path(__file__).resolve().parents:
        for candidate in (base / rel_path, base / "src" / "bisa" / rel_path):
            if candidate.exists():
                return str(candidate)
    return fallback


def default_config_path() -> str:
    """Finds the installed or source dracer_params.yaml for initial values."""

    return _find_package_file("config/dracer_params.yaml", "")


def default_model_path() -> str:
    """Finds the NCNN model dir (CPU-optimized for the on-car A72), falling
    back to the best.pt checkpoint if the NCNN export is absent.

    NCNN inference is ~2.4x faster than torch best.pt at the same imgsz on the
    A72 (~67 ms vs ~160 ms standalone), which keeps this live tuner GUI
    responsive instead of starving the event loop. The NCNN model is exported
    at imgsz=320 (fixed) — keep the imgsz control at 320.
    """

    ncnn = _find_package_file("checkpoints/best_ncnn_model", "")
    if ncnn:
        return ncnn
    return _find_package_file("checkpoints/best.pt", "checkpoints/best.pt")


def ncnn_export_imgsz(model_path: str):
    """Returns the fixed inference size of an NCNN export, or ``None``.

    An NCNN export is baked at a single imgsz (``metadata.yaml``: ``imgsz``);
    running it at any other size degrades or breaks detection. ``onboard.launch``
    injects ``detector.imgsz:=320`` for the on-car node, but the tuner loads the
    shared YAML directly (``imgsz: 640``, sized for the PC GPU + best.pt), so it
    must read the export size back here instead. Returns ``None`` for a plain
    ``.pt`` checkpoint (torch runs fine at any imgsz).
    """

    meta = Path(model_path) / "metadata.yaml"
    if not meta.is_file():
        return None
    try:
        import yaml

        data = yaml.safe_load(meta.read_text()) or {}
        size = data.get("imgsz")
        if isinstance(size, (list, tuple)):
            size = size[0]
        return int(size) if size is not None else None
    except Exception:
        return None


class TlTunerNode(Node):
    """Reads camera frames, runs the light detector, and shows tuning views."""

    def __init__(self):
        """Loads config/model, builds the sliders, and opens the input source."""

        super().__init__("bisa_traffic_light_tuner")
        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("config_file", default_config_path())
        self.declare_parameter("model_path", default_model_path())
        self.declare_parameter("video_path", "")

        cfg_file = str(self.get_parameter("config_file").value)
        self.config = load_config(cfg_file)
        # Force per-frame inference with no rate limit / ROI gating for tuning.
        self.config.detector.inference_hz = 1000.0
        model_path = str(self.get_parameter("model_path").value)
        model_path = resolve_package_relative_path(__file__, model_path)
        # NCNN is exported at a fixed imgsz (320); the shared YAML the tuner just
        # loaded says 640 (PC GPU + best.pt). Feeding 640 to the 320 export
        # degrades/breaks detection, so pin imgsz to the export size — mirroring
        # onboard.launch's imgsz:=320 override that this direct-load path skips.
        export_imgsz = ncnn_export_imgsz(model_path)
        if export_imgsz and export_imgsz != self.config.detector.imgsz:
            self.get_logger().info(
                f"NCNN model is fixed at imgsz={export_imgsz}; overriding "
                f"config imgsz {self.config.detector.imgsz} -> {export_imgsz}"
            )
            self.config.detector.imgsz = export_imgsz
        self.detector = BestPthDetector(self.config, model_path, logger=self.get_logger())
        self.detector.load_model()

        # Active slider set follows the classifier; 'm' cycles it and rebuilds.
        self.sliders = sliders_for(self.config.traffic_light.classifier)
        self._build_windows()

        # Latest frame is stored by the input source and drawn by a steady GUI
        # timer. Decoupling the two means the windows keep repainting (waitKey
        # pumps every tick) and show a "waiting" placeholder even before any
        # camera frame arrives, instead of freezing blank until the first frame.
        self.latest_frame = None
        self.frame_count = 0

        # YOLO detection runs on a background thread so a single ~hundreds-of-ms
        # inference never blocks the GUI's waitKey pump (that starvation is what
        # made the window show "not responding"). The thread publishes just the
        # light boxes; the GUI timer overlays them and re-runs the fast,
        # slider-tuned classify_light so colour sliders still react instantly.
        self._detect_lock = threading.Lock()
        self._latest_boxes: list = []
        self._infer_stop = False
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

        video_path = str(self.get_parameter("video_path").value).strip()
        self.capture = None
        if video_path:
            self.capture = cv2.VideoCapture(int(video_path) if video_path.isdigit() else video_path)
            if not self.capture.isOpened():
                self.get_logger().error(f"Could not open video source: {video_path}")
            self.get_logger().info(f"TL tuner reading from video: {video_path}")
        else:
            topic = str(self.get_parameter("image_topic").value)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CompressedImage, topic, self._on_image, qos)
            self.get_logger().info(f"TL tuner subscribing to '{topic}' (focus a window, 's'=dump 'q'=quit)")

        # GUI runs on its own ~20 Hz timer, independent of frame arrival.
        self.create_timer(1.0 / 20.0, self._render)

    # ----- setup -------------------------------------------------------------

    def _build_windows(self) -> None:
        """Creates the image + control windows and the initial slider set."""

        cv2.namedWindow(WIN_DETECT, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_DETECT, 640, 480)
        # Spread the windows so WSLg does not stack them on top of each other
        # (a common "the window opened but I can't see it" cause).
        cv2.moveWindow(WIN_DETECT, 20, 20)
        self._build_controls()

    def _build_controls(self) -> None:
        """(Re)creates the controls window with the trackbars for ``self.sliders``."""

        cv2.namedWindow(CONTROLS, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CONTROLS, 460, 720)
        cv2.moveWindow(CONTROLS, 680, 20)
        for label, path, kind, lo, hi in self.sliders:
            init = self._encode(path, kind)
            init = max(lo, min(hi, init))
            cv2.createTrackbar(label, CONTROLS, init, hi, lambda _v: None)
            if lo > 0:
                cv2.setTrackbarMin(label, CONTROLS, lo)

    def _cycle_classifier(self) -> None:
        """Advances the classifier (color->lit->lab) and rebuilds the sliders."""

        cur = str(self.config.traffic_light.classifier).lower()
        idx = CLASSIFIER_CYCLE.index(cur) if cur in CLASSIFIER_CYCLE else 0
        self.config.traffic_light.classifier = CLASSIFIER_CYCLE[(idx + 1) % len(CLASSIFIER_CYCLE)]
        self.sliders = sliders_for(self.config.traffic_light.classifier)
        cv2.destroyWindow(CONTROLS)
        self._build_controls()
        self.get_logger().info(f"classifier -> {self.config.traffic_light.classifier}")

    def _cfg_get(self, path: str):
        """Reads a dotted config path; ``detector.conf`` returns the light conf."""

        if path == "detector.conf":
            return self.config.detector.conf.get("traffic_red", 0.4)
        obj = self.config
        for part in path.split("."):
            obj = getattr(obj, part)
        return obj

    def _encode(self, path: str, kind: str) -> int:
        """Converts a config value into its integer trackbar position."""

        val = self._cfg_get(path)
        if kind == "x10":
            return int(round(float(val) * 10))
        if kind == "x100":
            return int(round(float(val) * 100))
        if kind == "boff":
            return int(round(float(val))) + 50
        if kind == "imgsz32":
            return int(round(int(val) / 32))
        return int(bool(val)) if isinstance(val, bool) else int(val)

    # ----- per-frame ---------------------------------------------------------

    def _read_sliders(self) -> None:
        """Pulls every trackbar position back into the shared config object."""

        for label, path, kind, _lo, _hi in self.sliders:
            pos = cv2.getTrackbarPos(label, CONTROLS)
            if kind == "x10":
                value = pos / 10.0
            elif kind == "x100":
                value = pos / 100.0
            elif kind == "boff":
                value = pos - 50
            elif kind == "imgsz32":
                value = max(1, pos) * 32
            else:
                value = pos
            self._assign(path, value)

    def _assign(self, path: str, value) -> None:
        """Writes a value back to the config, matching the field's type."""

        if path == "detector.conf":
            # One slider tunes both light thresholds; signs keep their config value.
            self.config.detector.conf["traffic_green"] = float(value)
            self.config.detector.conf["traffic_red"] = float(value)
            return
        parts = path.split(".")
        obj = self.config
        for part in parts[:-1]:
            obj = getattr(obj, part)
        current = getattr(obj, parts[-1])
        setattr(obj, parts[-1], type(current)(value))

    def _detect_lights(self, frame_bgr: np.ndarray) -> list:
        """Runs the model and returns [(cls, conf, (x1,y1,x2,y2)), ...] for lights."""

        if self.detector.model is None:
            return []
        conf = min(self.config.detector.conf.get("traffic_green", 0.4),
                   self.config.detector.conf.get("traffic_red", 0.4))
        results = self.detector.model.predict(
            source=frame_bgr,
            imgsz=int(self.config.detector.imgsz),
            device=self.detector.device,
            conf=float(conf),
            verbose=False,
        )
        out = []
        if not results:
            return out
        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return out
        for box in boxes:
            cls_name = self.detector._class_name_from_index(int(box.cls[0].item()))
            if cls_name not in ("traffic_green", "traffic_red"):
                continue
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            out.append((cls_name, float(box.conf[0].item()), (x1, y1, x2, y2)))
        return out

    def _infer_loop(self) -> None:
        """Background YOLO loop: detects light boxes off the GUI thread.

        Runs continuously on the newest frame and stores only the boxes; the
        heavy predict() call therefore never sits between two GUI waitKey pumps,
        which is what previously froze the window. Colour classification stays on
        the GUI thread (see :meth:`_process`) so sliders keep reacting live.
        """

        while not self._infer_stop:
            frame = self.latest_frame
            if frame is None or self.detector.model is None:
                time.sleep(0.02)
                continue
            t0 = time.perf_counter()
            try:
                detect_input = apply_correction_chain(frame, self.config.color_correction)
                boxes = [bbox for _cls, _conf, bbox in self._detect_lights(detect_input)]
            except Exception as exc:  # pragma: no cover - depends on target env.
                self.get_logger().warning(f"detector thread error: {exc}")
                boxes = []
            with self._detect_lock:
                self._latest_boxes = boxes
            # Sleep ~as long as the inference took (≈50% duty cycle) so this
            # CPU-bound loop hands the GUI thread uncontended windows to redraw
            # and pump waitKey. ~1-2 Hz detection is plenty for tuning, and the
            # gap between key pumps stays well under any WM "not responding"
            # watchdog. Bounded so a one-off slow inference can't stall updates.
            elapsed = time.perf_counter() - t0
            time.sleep(min(0.4, max(0.03, elapsed)))

    def _process(self, frame: np.ndarray) -> None:
        """Applies filters, overlays the latest detections, and refreshes windows.

        Detection boxes come from the background inference thread; only the fast,
        slider-tuned classify_light runs here so the GUI stays responsive. Key
        handling and waitKey live in :meth:`_render`.
        """

        self._read_sliders()
        # Always show the corrected frame so the color-correction sliders have a
        # visible effect; the pipeline applies the same chain (enabled by default).
        detect_input = apply_correction_chain(frame, self.config.color_correction)

        detect_view = detect_input.copy()
        n_on = 0
        with self._detect_lock:
            boxes = list(self._latest_boxes)
        for bbox in boxes:
            x1, y1, x2, y2 = (int(v) for v in bbox)
            # The pipeline's classifier (config.classifier) decides the color;
            # only the box + GREEN/RED verdict is drawn.
            verdict, _scores = classify_light(detect_input, bbox, self.config)
            n_on += int(verdict is not None)
            color = self._verdict_color(verdict)
            cv2.rectangle(detect_view, (x1, y1), (x2, y2), color, 2 if verdict else 1)
            cv2.putText(detect_view, self._verdict_text(verdict),
                        (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

        self._hud(detect_view, [
            f"classifier={self.config.traffic_light.classifier}  lights={n_on}",
            "m=cycle classifier  s=dump params  q=quit",
        ])

        cv2.imshow(WIN_DETECT, detect_view)

    @staticmethod
    def _verdict_text(cls) -> str:
        """Maps a mission class to a short verdict label for the overlay."""

        return {"traffic_green": "GREEN", "traffic_red": "RED"}.get(cls, "none")

    @staticmethod
    def _verdict_color(cls) -> tuple:
        """BGR color for a verdict (green/red lamp, grey when undecided)."""

        return {"traffic_green": (0, 255, 0), "traffic_red": (0, 0, 255)}.get(cls, (160, 160, 160))

    @staticmethod
    def _hud(frame, lines) -> None:
        """Draws a small stacked HUD in the top-left corner."""

        y = 18
        for text in lines:
            cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y += 20

    def _dump_yaml(self) -> None:
        """Prints a copy-paste-ready YAML block of the tuned values."""

        cc, tl, det = self.config.color_correction, self.config.traffic_light, self.config.detector
        print("\n# ---- paste into dracer_params.yaml ----")
        print("color_correction:")
        print(f"  enabled: {str(cc.enabled).lower()}")
        for f in ("clahe_clip", "clahe_tile", "saturation_boost", "brightness",
                  "contrast", "saturation", "gamma"):
            print(f"  {f}: {getattr(cc, f)}")
        print("traffic_light:")
        print(f"  classifier: {tl.classifier}")
        fields = ["green_h_lo", "green_h_hi", "green_s_min", "green_v_min",
                  "red_h1_hi", "red_h2_lo", "red_s_min", "red_v_min", "row_min_ratio"]
        mode = str(tl.classifier).lower()
        if mode == "lit":
            fields += ["row_lit_v_min", "row_white_s_max"]
        elif mode == "lab":
            fields += ["lab_a_red_min", "lab_a_green_max", "lab_l_min"]
        for f in fields:
            print(f"  {f}: {getattr(tl, f)}")
        print("detector:")
        print(f"  imgsz: {det.imgsz}")
        print("  conf:")
        print(f"    traffic_green: {det.conf['traffic_green']}")
        print(f"    traffic_red: {det.conf['traffic_red']}")
        print("# ---------------------------------------\n")

    # ----- input callbacks ---------------------------------------------------

    def _on_image(self, msg: CompressedImage) -> None:
        """Stores the newest ROS frame; drawing happens in the GUI timer."""

        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.latest_frame = frame
            self.frame_count += 1

    def _render(self) -> None:
        """Steady GUI tick: draws the latest frame (or a placeholder) and pumps keys.

        Runs regardless of frame arrival so the windows never freeze blank. In
        video mode it also pulls the next frame from the capture here.
        """

        if self.capture is not None:
            ok, frame = self.capture.read()
            if ok:
                self.latest_frame = frame
                self.frame_count += 1
            else:
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

        if self.latest_frame is None:
            placeholder = np.full((480, 640, 3), 40, dtype=np.uint8)
            self._hud(placeholder, [
                "Waiting for camera frames...",
                "check: ros2 topic hz /camera/image/compressed",
                "same ROS_DOMAIN_ID as the car?  q=quit",
            ])
            cv2.imshow(WIN_DETECT, placeholder)
        else:
            self._process(self.latest_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rclpy.shutdown()
        elif key == ord("s"):
            self._dump_yaml()
        elif key == ord("m"):
            self._cycle_classifier()


def main(args=None) -> None:
    """Initializes rclpy and spins the traffic-light tuner node."""

    rclpy.init(args=args)
    node = TlTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._infer_stop = True
        node._infer_thread.join(timeout=1.0)
        if node.capture is not None:
            node.capture.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
