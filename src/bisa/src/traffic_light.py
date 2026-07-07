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
reference's pre-detection enhancement and can run before YOLO. Pure OpenCV /
NumPy, no ROS, so it stays importable in the syntax tests.
"""

from __future__ import annotations

import cv2
import numpy as np

from .dracer_config import AutonomousConfig, ColorCorrectionConfig


def apply_clahe(bgr: np.ndarray, clip: float = 2.0, tile: int = 8) -> np.ndarray:
    """Equalizes the LAB L-channel with CLAHE (reference apply_clahe)."""

    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=float(clip), tileGridSize=(int(tile), int(tile)))
    l_ch = clahe.apply(l_ch)
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
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype(np.uint8)
        out = cv2.LUT(out, table)
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
