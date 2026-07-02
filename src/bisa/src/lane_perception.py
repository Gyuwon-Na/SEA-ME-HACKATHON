"""Classical lane and road perception for the D-Racer camera feed."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional

import cv2
import numpy as np

from .dracer_config import AutonomousConfig


@dataclass
class PathCandidate:
    """Represents a left/right/exit road branch target in normalized error space."""

    target_error: float
    area_ratio: float
    center_x: float


@dataclass
class LaneObs:
    """Carries the compact lane observation consumed by the mission FSM."""

    valid: bool
    center_error: float = 0.0
    curvature: float = 0.0
    signed_curvature: float = 0.0
    fork_seen: bool = False
    left_branch: Optional[PathCandidate] = None
    right_branch: Optional[PathCandidate] = None
    rotary_seen: bool = False
    rotary_exit_seen: bool = False
    lost_reason: str = ""

    def with_center_error(self, center_error: float) -> "LaneObs":
        """Returns a copy with a different target error for virtual branch control."""

        return replace(self, center_error=center_error, valid=True)


def clamp(value: float, low: float, high: float) -> float:
    """Clamps a numeric value to the configured inclusive range."""

    return max(low, min(high, value))


class LanePerception:
    """Builds road masks, lane centers, fork candidates, and rotary hints."""

    def __init__(self, config: AutonomousConfig):
        """Initializes kernels and keeps the last valid center for dropout recovery."""

        self.config = config
        self.prev_center_error = 0.0
        self.prev_near_center: Optional[float] = None
        # Visualization scratch data, populated only when compute_lane_obs is
        # called with collect_viz=True (PC-side debug overlay). Kept off the hot
        # path so on-vehicle runs pay nothing for it.
        self.last_viz: dict = {}

    def build_road_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Extracts the black track area with HSV thresholding and morphology."""

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower = np.array(self.config.lane.black_hsv_lower, dtype=np.uint8)
        upper = np.array(self.config.lane.black_hsv_upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        open_size = max(1, int(self.config.lane.morph_open_kernel))
        close_size = max(1, int(self.config.lane.morph_close_kernel))
        open_kernel = np.ones((open_size, open_size), dtype=np.uint8)
        close_kernel = np.ones((close_size, close_size), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        return mask

    def get_l_channel(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Applies the ERP reference CLAHE L-channel transform for robust edges."""

        try:
            lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2Lab)
            l_channel, _, _ = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            return clahe.apply(l_channel)
        except cv2.error:
            return np.zeros(frame_bgr.shape[:2], dtype=np.uint8)

    def lane_roi_rect(self, frame_width: int, frame_height: int) -> tuple[int, int, int, int]:
        """Returns the (x0, y0, w, h) lane ROI box in full-frame pixel coords.

        Width/height come from the live-tunable config; negative offsets center
        the box horizontally and anchor it to the bottom of the frame.
        """

        cfg = self.config.lane_roi
        if not getattr(cfg, "enabled", True):
            return 0, 0, frame_width, frame_height
        roi_w = int(clamp(float(cfg.width), 1.0, float(frame_width)))
        roi_h = int(clamp(float(cfg.height), 1.0, float(frame_height)))
        if cfg.x_offset < 0:
            x0 = (frame_width - roi_w) // 2
        else:
            x0 = int(clamp(float(cfg.x_offset), 0.0, float(frame_width - roi_w)))
        if cfg.y_offset < 0:
            y0 = frame_height - roi_h
        else:
            y0 = int(clamp(float(cfg.y_offset), 0.0, float(frame_height - roi_h)))
        return x0, y0, roi_w, roi_h

    def region_of_interest(self, image: np.ndarray, y0_ratio: float, y1_ratio: float) -> np.ndarray:
        """Returns a vertical ROI slice using ratios from the camera frame height."""

        height = image.shape[0]
        y0 = int(clamp(y0_ratio, 0.0, 1.0) * height)
        y1 = int(clamp(y1_ratio, 0.0, 1.0) * height)
        return image[y0:y1, :]

    def contour_center_x(self, roi_mask: np.ndarray, min_area_ratio: float) -> Optional[float]:
        """Finds the weighted center x of the largest valid road component."""

        if roi_mask.size == 0:
            return None
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        min_area = roi_mask.shape[0] * roi_mask.shape[1] * min_area_ratio
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]
        if not valid_contours:
            return None
        merged = np.vstack(valid_contours)
        moments = cv2.moments(merged)
        if moments["m00"] <= 0.0:
            return None
        return float(moments["m10"] / moments["m00"])

    def _line_endpoints(self, slope: float, intercept: float, top_y: int, height: int):
        """Returns ((x_top, top_y), (x_bot, height)) for a fitted lane line."""

        if abs(slope) < 1e-6:
            return None
        x_top = (top_y - intercept) / slope
        x_bot = (height - intercept) / slope
        return ((float(x_top), int(top_y)), (float(x_bot), int(height)))

    def average_hough_lanes(
        self, frame_bgr: np.ndarray, record: bool = False
    ) -> tuple[Optional[float], bool, bool]:
        """Computes lane center from Canny/Hough lines as in the ERP reference code."""

        height, width = frame_bgr.shape[:2]
        top_y = int(self.config.lane.hough_roi_top_ratio * height)
        if record:
            self.last_viz["hough_left"] = None
            self.last_viz["hough_right"] = None
            self.last_viz["hough_segments"] = []
            self.last_viz["hough_top_y"] = top_y
        l_channel = self.get_l_channel(frame_bgr)
        blur = cv2.GaussianBlur(l_channel, (5, 5), 0)
        edges = cv2.Canny(
            blur,
            int(self.config.lane.hough_canny_low),
            int(self.config.lane.hough_canny_high),
        )
        roi = np.zeros_like(edges)
        vertices = np.array([[(0, height), (0, top_y), (width, top_y), (width, height)]])
        cv2.fillPoly(roi, vertices, 255)
        roi_edges = cv2.bitwise_and(edges, roi)
        lines = cv2.HoughLinesP(
            roi_edges,
            1,
            np.pi / 180,
            int(self.config.lane.hough_threshold),
            minLineLength=int(self.config.lane.hough_min_line_length),
            maxLineGap=int(self.config.lane.hough_max_line_gap),
        )
        if lines is None:
            return None, False, False

        left_fit: list[tuple[float, float]] = []
        right_fit: list[tuple[float, float]] = []
        for raw_line in lines:
            x1, y1, x2, y2 = raw_line.reshape(4)
            if x1 == x2:
                continue
            slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)
            if slope < -self.config.lane.hough_slope_min_abs:
                left_fit.append((float(slope), float(intercept)))
                if record:
                    self.last_viz["hough_segments"].append((int(x1), int(y1), int(x2), int(y2)))
            elif slope > self.config.lane.hough_slope_min_abs:
                right_fit.append((float(slope), float(intercept)))
                if record:
                    self.last_viz["hough_segments"].append((int(x1), int(y1), int(x2), int(y2)))

        left_present = bool(left_fit)
        right_present = bool(right_fit)
        if not left_present and not right_present:
            return None, False, False

        assumed_width = width * self.config.lane.assumed_lane_width_ratio
        x_targets = []
        if left_present:
            slope, intercept = np.average(left_fit, axis=0)
            x_targets.append((top_y - intercept) / slope)
            if record:
                self.last_viz["hough_left"] = self._line_endpoints(slope, intercept, top_y, height)
        if right_present:
            slope, intercept = np.average(right_fit, axis=0)
            x_targets.append((top_y - intercept) / slope)
            if record:
                self.last_viz["hough_right"] = self._line_endpoints(slope, intercept, top_y, height)

        if left_present and right_present:
            center_x = float(np.average(x_targets))
        elif left_present:
            center_x = float(x_targets[0] + assumed_width / 2.0)
        else:
            center_x = float(x_targets[0] - assumed_width / 2.0)
        return center_x, left_present, right_present

    def build_line_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Extracts bright white + yellow lane boundary lines from the ROI.

        The track is a dark mat whose brightness drifts with lighting, so the
        drivable area cannot be thresholded reliably. The white/yellow boundary
        lines are far more distinctive: white is bright + low-saturation and
        yellow is a fixed hue wedge. White uses an adaptive brightness floor
        (max of an absolute floor and a high ROI percentile) so it tracks the
        exposure of each frame instead of a brittle fixed value.
        """

        cfg = self.config.lane
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v_floor = max(int(cfg.white_v_min), int(np.percentile(v, cfg.white_v_percentile)))
        white = (v >= v_floor) & (s <= int(cfg.white_s_max))
        yellow = (
            (h >= int(cfg.line_yellow_h_lo))
            & (h <= int(cfg.line_yellow_h_hi))
            & (s >= int(cfg.line_yellow_s_min))
            & (v >= int(cfg.line_yellow_v_min))
        )
        mask = (white | yellow).astype(np.uint8) * 255
        open_size = max(1, int(cfg.morph_open_kernel))
        close_size = max(1, int(cfg.morph_close_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((close_size, close_size), np.uint8))
        return mask

    def _band_line_centers(self, band_mask: np.ndarray) -> tuple[Optional[float], Optional[float]]:
        """Returns (left_line_x, right_line_x) in band-local x, or None if absent.

        Splits the band at its horizontal midpoint and takes the intensity-
        weighted centroid of lit columns on each side, but only when at least one
        column is lit across ``line_col_row_frac`` of the band's rows -- this
        rejects diffuse background speckle (which never forms a tall column) while
        keeping thin, near-vertical lane lines.
        """

        rows, width = band_mask.shape[:2]
        if rows == 0 or width == 0:
            return None, None
        col_counts = (band_mask > 0).sum(axis=0).astype(np.float64)
        min_col = max(3.0, rows * float(self.config.lane.line_col_row_frac))
        half = width // 2

        def side_center(counts: np.ndarray, base: int) -> Optional[float]:
            if counts.size == 0 or counts.max() < min_col:
                return None
            strong = counts >= min_col
            weights = counts * strong
            total = weights.sum()
            if total <= 0.0:
                return None
            xs = np.arange(counts.size, dtype=np.float64)
            return base + float((xs * weights).sum() / total)

        left_x = side_center(col_counts[:half], 0)
        right_x = side_center(col_counts[half:], half)
        return left_x, right_x

    def _lane_center_from_band(
        self, band_mask: np.ndarray, roi_width: int
    ) -> tuple[Optional[float], bool, bool]:
        """Combines the band's left/right lines into one lane-center x (ROI-local)."""

        left_x, right_x = self._band_line_centers(band_mask)
        half_width = float(self.config.lane.lane_half_width_ratio) * float(roi_width)
        if left_x is not None and right_x is not None:
            self._lane_half_width = max(20.0, (right_x - left_x) / 2.0)
            return (left_x + right_x) / 2.0, True, True
        span = getattr(self, "_lane_half_width", half_width)
        if left_x is not None:
            return left_x + span, True, False
        if right_x is not None:
            return right_x - span, False, True
        return None, False, False

    def detect_fork_candidates(self, mask: np.ndarray) -> tuple[bool, Optional[PathCandidate], Optional[PathCandidate]]:
        """Splits the far ROI into left/right road masses for fork decisions."""

        height, width = mask.shape[:2]
        far = self.region_of_interest(mask, self.config.roi.far_y0, self.config.roi.far_y1)
        if far.size == 0:
            return False, None, None

        half = width // 2
        min_ratio = self.config.lane.fork_area_ratio
        left_area = float(cv2.countNonZero(far[:, :half])) / float(far.size)
        right_area = float(cv2.countNonZero(far[:, half:])) / float(far.size)
        left = self._candidate_from_roi(far[:, :half], 0, width, left_area) if left_area >= min_ratio else None
        right = self._candidate_from_roi(far[:, half:], half, width, right_area) if right_area >= min_ratio else None
        return left is not None and right is not None, left, right

    def _candidate_from_roi(
        self,
        roi_mask: np.ndarray,
        x_offset: int,
        full_width: int,
        area_ratio: float,
    ) -> Optional[PathCandidate]:
        """Creates a path candidate from ROI image moments."""

        moments = cv2.moments(roi_mask)
        if moments["m00"] <= 0.0:
            return None
        center_x = x_offset + float(moments["m10"] / moments["m00"])
        target_error = (full_width / 2.0 - center_x) / (full_width / 2.0)
        return PathCandidate(clamp(target_error, -1.0, 1.0), area_ratio, center_x)

    def detect_rotary_candidates(self, mask: np.ndarray) -> tuple[bool, bool]:
        """Estimates rotary presence and exit branches from contour shape cues."""

        height, width = mask.shape[:2]
        mid = self.region_of_interest(mask, self.config.roi.mid_y0, 1.0)
        if mid.size == 0:
            return False, False

        contours, _ = cv2.findContours(mid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, False
        largest = max(contours, key=cv2.contourArea)
        area_ratio = cv2.contourArea(largest) / float(mid.size)
        perimeter = max(cv2.arcLength(largest, True), 1.0)
        circularity = 4.0 * np.pi * cv2.contourArea(largest) / (perimeter * perimeter)
        rotary_seen = (
            area_ratio >= self.config.lane.rotary_area_ratio
            and circularity >= self.config.lane.rotary_circularity_min
        )

        far = self.region_of_interest(mask, self.config.roi.far_y0, self.config.roi.far_y1)
        left_pixels = cv2.countNonZero(far[:, : width // 3]) if far.size else 0
        right_pixels = cv2.countNonZero(far[:, 2 * width // 3 :]) if far.size else 0
        exit_seen = rotary_seen and max(left_pixels, right_pixels) > far.size * 0.03
        return rotary_seen, exit_seen

    def _record_obs_viz(self, width, height, near_center, mid_center, far_center,
                        lane_center, center_error, roi_rect) -> None:
        """Stores ROI bands and centers for the PC debug overlay.

        Coordinates are recorded in ROI-local space; ``roi_offset`` lets the
        overlay translate them back onto the full camera frame.
        """

        def band(y0_ratio, y1_ratio):
            return (int(clamp(y0_ratio, 0.0, 1.0) * height), int(clamp(y1_ratio, 0.0, 1.0) * height))

        x0, y0, roi_w, roi_h = roi_rect
        self.last_viz.update({
            "width": int(width),
            "height": int(height),
            "near_band": band(self.config.roi.near_y0, 1.0),
            "mid_band": band(self.config.roi.mid_y0, self.config.roi.mid_y1),
            "far_band": band(self.config.roi.far_y0, self.config.roi.far_y1),
            "near_center": near_center,
            "mid_center": mid_center,
            "far_center": far_center,
            "lane_center_x": lane_center,
            "center_error": center_error,
            "roi_rect": (int(x0), int(y0), int(roi_w), int(roi_h)),
            "roi_offset": (int(x0), int(y0)),
        })

    def compute_lane_obs(self, frame_bgr: np.ndarray, collect_viz: bool = False) -> LaneObs:
        """Computes a full lane observation from one decoded BGR image."""

        full_height, full_width = frame_bgr.shape[:2]
        x0, y0, roi_w, roi_h = self.lane_roi_rect(full_width, full_height)
        roi_rect = (x0, y0, roi_w, roi_h)
        # Crop the lane search to the ROI box only; light/sign/aruco detection
        # keeps using the full frame elsewhere in the pipeline.
        roi_frame = frame_bgr[y0:y0 + roi_h, x0:x0 + roi_w]

        # Primary detection is the white/yellow boundary-line mask: on this track
        # the drivable mat is dark grey whose brightness drifts with lighting, so
        # a "black road" threshold is unreliable, while the bright boundary lines
        # are stable. Near/far lane centers both come from this one mask so the
        # curvature sign is consistent with the steering error.
        line_mask = self.build_line_mask(roi_frame)
        height, width = line_mask.shape[:2]
        near_band = self.region_of_interest(line_mask, self.config.roi.near_y0, 1.0)
        mid_band = self.region_of_interest(line_mask, self.config.roi.mid_y0, self.config.roi.mid_y1)
        far_band = self.region_of_interest(line_mask, self.config.roi.far_y0, self.config.roi.far_y1)

        near_center, near_left, near_right = self._lane_center_from_band(near_band, width)
        mid_center, _, _ = self._lane_center_from_band(mid_band, width)
        far_center, _, _ = self._lane_center_from_band(far_band, width)

        if near_center is None:
            # No boundary line near the vehicle: fall back to the last known
            # center so a momentary dropout coasts instead of snapping to zero.
            if collect_viz:
                self._record_line_viz(width, height, None, mid_center, far_center,
                                      None, near_left, near_right, roi_rect)
            return LaneObs(valid=False, center_error=self.prev_center_error, lost_reason="no_line_center")

        near_center = clamp(near_center, 0.0, float(width))
        # Steering error is measured against the full-frame center (vehicle
        # heading), so translate the ROI-local center back into frame x.
        near_center_full = near_center + x0
        if self.prev_near_center is not None:
            jump = abs(near_center_full - self.prev_near_center) / max(float(full_width), 1.0)
            if jump > self.config.lane.max_center_jump:
                near_center_full = self.prev_near_center
                near_center = near_center_full - x0

        self.prev_near_center = near_center_full
        center_error = (full_width / 2.0 - near_center_full) / (full_width / 2.0)
        # Look-ahead center for curvature: prefer far, then mid, then near.
        ahead = far_center if far_center is not None else mid_center
        if ahead is None:
            ahead = near_center
        signed_curv = (near_center - ahead) / max(float(full_width), 1.0)
        curvature = clamp(abs(ahead - near_center) / max(float(full_width), 1.0) * 2.5, 0.0, 1.0)
        if mid_center is not None:
            curvature = max(curvature, clamp(abs(mid_center - near_center) / max(float(full_width), 1.0) * 2.0, 0.0, 1.0))

        # Fork/rotary geometric cues still come from the dark-road area mask; they
        # only matter in the OUT/IN mission modes, not pure lane following.
        road_mask = self.build_road_mask(roi_frame)
        fork_seen, left_branch, right_branch = self.detect_fork_candidates(road_mask)
        rotary_seen, rotary_exit_seen = self.detect_rotary_candidates(road_mask)
        self.prev_center_error = clamp(center_error, -1.0, 1.0)
        if collect_viz:
            self._record_line_viz(width, height, near_center, mid_center, far_center,
                                  near_center, near_left, near_right, roi_rect)
        return LaneObs(
            valid=True,
            center_error=self.prev_center_error,
            curvature=curvature,
            signed_curvature=clamp(signed_curv, -1.0, 1.0),
            fork_seen=fork_seen,
            left_branch=left_branch,
            right_branch=right_branch,
            rotary_seen=rotary_seen,
            rotary_exit_seen=rotary_exit_seen,
        )

    def _record_line_viz(self, width, height, near_center, mid_center, far_center,
                         lane_center, near_left, near_right, roi_rect) -> None:
        """Stores line-detection ROI bands and centers for the PC debug overlay."""

        def band(y0_ratio, y1_ratio):
            return (int(clamp(y0_ratio, 0.0, 1.0) * height), int(clamp(y1_ratio, 0.0, 1.0) * height))

        near_b = band(self.config.roi.near_y0, 1.0)
        far_b = band(self.config.roi.far_y0, self.config.roi.far_y1)
        x0, y0, roi_w, roi_h = roi_rect
        # Draw detected boundary lines as short vertical markers at their x within
        # the near band so the operator can see left/right line pickups.
        segs = {"hough_left": None, "hough_right": None}
        if near_left and near_center is not None:
            segs["hough_left"] = ((float(near_center), near_b[0]), (float(near_center), near_b[1]))
        self.last_viz.update({
            "width": int(width),
            "height": int(height),
            "near_band": near_b,
            "mid_band": band(self.config.roi.mid_y0, self.config.roi.mid_y1),
            "far_band": far_b,
            "near_center": near_center,
            "mid_center": mid_center,
            "far_center": far_center,
            "lane_center_x": lane_center,
            "center_error": self.prev_center_error,
            "roi_rect": (int(x0), int(y0), int(roi_w), int(roi_h)),
            "roi_offset": (int(x0), int(y0)),
            "hough_left": None,
            "hough_right": None,
            "hough_segments": [],
        })
