"""HSV traffic-light color analysis and shared frame preprocessing.

Ports the ideas from the reference ``traffic_light_processor.py`` into the
D-Racer pipeline:

* a CLAHE -> saturation-boost -> brightness/contrast/saturation/gamma
  correction stage (:func:`preprocess_frame`) that can run before detection, and
* an HSV mask analyzer (:class:`TrafficLightAnalyzer`) that reports the lit color
  inside the traffic-light ROI, using HoughCircles to confirm red/yellow.

The reference light is a 1x4 HORIZONTAL bar split into four vertical sections.
THIS car's light is a 3x1 VERTICAL stack: top lamp = RED, middle = YELLOW,
bottom = GREEN. So we split the light ROI into three HORIZONTAL bands and check
each against only its expected color (top->red, middle->yellow, bottom->green),
exactly mirroring the reference's per-section color mapping but re-oriented. A
lamp lit in the top band means red; the bottom band means green.

We have no per-light bounding box (the shipped best.pt is a whole-frame
classifier), so the analysis runs over ``roi.detector_light`` -- keep that ROI
tight around the traffic light for the section mapping to line up. Pure OpenCV /
NumPy, no ROS, so it stays importable in the syntax tests.
"""

from __future__ import annotations

import cv2
import numpy as np

from .dracer_config import AutonomousConfig, ColorCorrectionConfig
from .object_detector import Detection


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


def preprocess_frame(bgr: np.ndarray | None, cfg: ColorCorrectionConfig) -> np.ndarray | None:
    """Runs the full correction chain when enabled; returns the input otherwise."""

    if bgr is None or not getattr(cfg, "enabled", False):
        return bgr
    out = apply_clahe(bgr, cfg.clahe_clip, cfg.clahe_tile)
    out = adjust_saturation(out, cfg.saturation_boost)
    out = apply_color_correction(out, cfg.brightness, cfg.contrast, cfg.saturation, cfg.gamma)
    return out


class TrafficLightAnalyzer:
    """Reports the lit traffic-light color from HSV masks inside the light ROI.

    Vertical 3x1 layout (top row index 0 -> bottom row index 2). Each entry is
    (color_name, row_index, needs_hough_circle).
    """

    # Top lamp = red, middle = yellow, bottom = green. Red/yellow are confirmed
    # with HoughCircles (round lamp), green is accepted on area alone -- matching
    # the reference, which only circle-checked the red/yellow lamps.
    SECTION_LAYOUT = (
        ("red", 0, True),
        ("yellow", 1, True),
        ("green", 2, False),
    )

    def __init__(self, config: AutonomousConfig):
        """Keeps a config reference and the most recent color decision."""

        self.config = config
        self.last_states = {"red": False, "yellow": False, "green": False}

    def _ranges(self) -> dict:
        """Builds the four (lower, upper) HSV bounds from scalar config fields."""

        tl = self.config.traffic_light
        return {
            "red1": (np.array([0, tl.red_s_min, tl.red_v_min]),
                     np.array([tl.red_h1_hi, 255, 255])),
            "red2": (np.array([tl.red_h2_lo, tl.red_s_min, tl.red_v_min]),
                     np.array([180, 255, 255])),
            "yellow": (np.array([tl.yellow_h_lo, tl.yellow_s_min, tl.yellow_v_min]),
                       np.array([tl.yellow_h_hi, 255, 255])),
            "green": (np.array([tl.green_h_lo, tl.green_s_min, tl.green_v_min]),
                      np.array([tl.green_h_hi, 255, 255])),
        }

    def roi_rect(self, width: int, height: int) -> tuple[int, int, int, int]:
        """Returns the pixel (x0, y0, x1, y1) of the traffic-light detection ROI."""

        x0, y0, x1, y1 = self.config.roi.detector_light
        return (int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height))

    def _color_mask(self, hsv: np.ndarray, color: str, rng: dict) -> np.ndarray:
        """Builds the HSV mask for one color (red spans two hue bands)."""

        if color == "red":
            return cv2.bitwise_or(
                cv2.inRange(hsv, rng["red1"][0], rng["red1"][1]),
                cv2.inRange(hsv, rng["red2"][0], rng["red2"][1]),
            )
        return cv2.inRange(hsv, rng[color][0], rng[color][1])

    def analyze(self, roi_bgr: np.ndarray) -> dict:
        """Returns {red, yellow, green} booleans for one 3x1 vertical light crop.

        Splits the crop into three horizontal bands and checks each band only
        against its expected color, so a lit top band => red and a lit bottom
        band => green (never confused for one another).
        """

        states = {"red": False, "yellow": False, "green": False}
        if roi_bgr is None or roi_bgr.size == 0:
            return states
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        rng = self._ranges()
        tl = self.config.traffic_light
        hough = dict(
            minDist=max(1, int(tl.hough_min_dist)),
            param1=max(1, int(tl.hough_param1)),
            param2=max(1, int(tl.hough_param2)),
            minRadius=max(0, int(tl.hough_min_radius)),
            maxRadius=max(0, int(tl.hough_max_radius)),
        )
        height = roi_bgr.shape[0]
        band_h = max(1, height // 3)
        for color, row, needs_circle in self.SECTION_LAYOUT:
            y0 = row * band_h
            y1 = (row + 1) * band_h if row < 2 else height
            band = self._color_mask(hsv, color, rng)[y0:y1, :]
            if band.size == 0:
                continue
            min_on = max(20, int(band.size * float(tl.min_on_ratio)))
            if cv2.countNonZero(band) <= min_on:
                continue
            if needs_circle:
                circles = cv2.HoughCircles(band, cv2.HOUGH_GRADIENT, 1, **hough)
                states[color] = circles is not None
            else:
                states[color] = True
        return states

    def detect(self, frame_bgr: np.ndarray | None) -> list[Detection]:
        """Analyzes the light ROI and returns mission Detection objects.

        Emits at most one ``traffic_green`` and one ``traffic_red`` so the result
        drops straight into the existing DetectionBuffer vote pipeline. Returns an
        empty list (and touches nothing) when the analyzer is disabled.
        """

        if frame_bgr is None or not self.config.traffic_light.enabled:
            return []
        height, width = frame_bgr.shape[:2]
        x0, y0, x1, y1 = self.roi_rect(width, height)
        x0c, y0c = max(0, x0), max(0, y0)
        x1c, y1c = min(width, x1), min(height, y1)
        roi = frame_bgr[y0c:y1c, x0c:x1c]
        states = self.analyze(roi)
        self.last_states = states

        detections: list[Detection] = []
        cx = (x0 + x1) / 2.0
        cy = (y0 + y1) / 2.0
        bbox = (float(x0), float(y0), float(x1), float(y1))
        area = max(0.0, (x1 - x0) * (y1 - y0))
        for color, cls in (("green", "traffic_green"), ("red", "traffic_red")):
            if states.get(color):
                detections.append(
                    Detection(cls=cls, conf=1.0, bbox=bbox, cx=cx, cy=cy, area=area)
                )
        return detections
