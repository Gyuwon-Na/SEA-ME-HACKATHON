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

    def with_center_error(self, center_error: float) -> "LaneObs":
        """Returns a copy with a different target error for virtual branch control."""

        return replace(self, center_error=center_error, valid=True)


def clamp(value: float, low: float, high: float) -> float:
    """Clamps a numeric value to the configured inclusive range."""

    return max(low, min(high, value))


class LanePerception:
    """Builds road masks, lane centers, and fork candidates."""

    def __init__(self, config: AutonomousConfig):
        """Initializes kernels and keeps the last valid center for dropout recovery."""

        self.config = config
        self.prev_center_error = 0.0
        self.prev_near_center: Optional[float] = None
        self._clahe_key = None
        self._clahe = None
        self._morph_kernels: dict[int, np.ndarray] = {}
        # Visualization scratch data, populated only when compute_lane_obs is
        # called with collect_viz=True (PC-side debug overlay). Kept off the hot
        # path so on-vehicle runs pay nothing for it.
        self.last_viz: dict = {}

    def _get_clahe(self):
        """Returns a cached CLAHE object, rebuilding only after live tuning."""

        cfg = self.config.lane
        key = (
            max(float(cfg.lab_clahe_clip), 0.01),
            max(1, int(cfg.lab_clahe_tile)),
        )
        if key != self._clahe_key:
            self._clahe = cv2.createCLAHE(clipLimit=key[0], tileGridSize=(key[1], key[1]))
            self._clahe_key = key
        return self._clahe

    def _get_morph_kernel(self, size: int) -> np.ndarray:
        """Returns an immutable square morphology kernel from a tiny cache."""

        size = max(1, int(size))
        kernel = self._morph_kernels.get(size)
        if kernel is None:
            kernel = np.ones((size, size), dtype=np.uint8)
            self._morph_kernels[size] = kernel
        return kernel

    def prepare_lane_lab(self, frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Returns one CLAHE-equalized LAB image and its L channel.

        Road masking and white/yellow paint masking consume the same enhanced
        LAB image. Preparing it once avoids a second BGR->LAB conversion, split,
        and CLAHE pass for every camera frame.
        """

        lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
        l_channel = self._get_clahe().apply(lab[:, :, 0])
        lab[:, :, 0] = l_channel
        return lab, l_channel

    def build_road_mask(
        self, frame_bgr: np.ndarray, prepared_lab: np.ndarray | None = None
    ) -> np.ndarray:
        """Extracts the drivable road area with LAB thresholding and morphology."""

        cfg = self.config.lane
        lab = prepared_lab
        if lab is None:
            lab, _ = self.prepare_lane_lab(frame_bgr)
        lower = np.array([cfg.lab_l_min, cfg.lab_a_min, cfg.lab_b_min], dtype=np.uint8)
        upper = np.array([cfg.lab_l_max, cfg.lab_a_max, cfg.lab_b_max], dtype=np.uint8)
        mask = cv2.inRange(lab, lower, upper)
        open_kernel = self._get_morph_kernel(self.config.lane.morph_open_kernel)
        close_kernel = self._get_morph_kernel(self.config.lane.morph_close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        return mask

    def build_lane_mask(
        self, frame_bgr: np.ndarray, prepared_lab: np.ndarray | None = None
    ) -> np.ndarray:
        """Extracts white and yellow lane paint as one binary steering mask."""

        cfg = self.config.lane
        lab = prepared_lab
        if lab is None:
            lab, _ = self.prepare_lane_lab(frame_bgr)
        white = cv2.inRange(
            lab,
            np.array([cfg.white_l_min, cfg.white_a_min, cfg.white_b_min], dtype=np.uint8),
            np.array([cfg.white_l_max, cfg.white_a_max, cfg.white_b_max], dtype=np.uint8),
        )
        yellow = cv2.inRange(
            lab,
            np.array([cfg.yellow_l_min, cfg.yellow_a_min, cfg.yellow_b_min], dtype=np.uint8),
            np.array([cfg.yellow_l_max, cfg.yellow_a_max, cfg.yellow_b_max], dtype=np.uint8),
        )
        mask = cv2.bitwise_or(white, yellow)
        open_kernel = self._get_morph_kernel(cfg.morph_open_kernel)
        close_kernel = self._get_morph_kernel(cfg.morph_close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        return mask

    def get_l_channel(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Applies the ERP reference CLAHE L-channel transform for robust edges."""

        try:
            lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2Lab)
            l_channel, _, _ = cv2.split(lab)
            return self._get_clahe().apply(l_channel)
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
        self,
        frame_bgr: np.ndarray,
        record: bool = False,
        viz_out: dict | None = None,
        prepared_l_channel: np.ndarray | None = None,
    ) -> tuple[Optional[float], bool, bool]:
        """Computes lane center from Canny/Hough lines as in the ERP reference code."""

        height, width = frame_bgr.shape[:2]
        top_y = int(self.config.lane.hough_roi_top_ratio * height)
        if record and viz_out is not None:
            viz_out["hough_left"] = None
            viz_out["hough_right"] = None
            viz_out["hough_segments"] = []
            viz_out["hough_top_y"] = top_y
        l_channel = prepared_l_channel
        if l_channel is None:
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
        if record and viz_out is not None:
            # Shown in the lane mask debug view so the canny/hough sliders have
            # a visible effect (this is exactly what HoughLinesP consumes).
            viz_out["edges"] = roi_edges
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

        pts = lines.reshape(-1, 4).astype(float)
        x1, y1, x2, y2 = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]

        valid = x1 != x2
        x1, y1, x2, y2 = x1[valid], y1[valid], x2[valid], y2[valid]

        slopes = (y2 - y1) / (x2 - x1)
        intercepts = y1 - slopes * x1

        min_abs = self.config.lane.hough_slope_min_abs
        left_mask = slopes < -min_abs
        right_mask = slopes > min_abs

        left_slopes, left_intercepts = slopes[left_mask], intercepts[left_mask]
        right_slopes, right_intercepts = slopes[right_mask], intercepts[right_mask]

        if record and viz_out is not None:
            for m_mask in (left_mask, right_mask):
                for ix in np.where(m_mask)[0]:
                    viz_out["hough_segments"].append(
                        (int(x1[ix]), int(y1[ix]), int(x2[ix]), int(y2[ix]))
                    )

        left_fit = list(zip(left_slopes.tolist(), left_intercepts.tolist()))
        right_fit = list(zip(right_slopes.tolist(), right_intercepts.tolist()))

        left_present = len(left_fit) > 0
        right_present = len(right_fit) > 0
        if not left_present and not right_present:
            return None, False, False

        assumed_width = width * self.config.lane.assumed_lane_width_ratio
        x_targets = []
        if left_present:
            slope, intercept = np.average(left_fit, axis=0)
            x_targets.append((top_y - intercept) / slope)
            if record and viz_out is not None:
                viz_out["hough_left"] = self._line_endpoints(slope, intercept, top_y, height)
        if right_present:
            slope, intercept = np.average(right_fit, axis=0)
            x_targets.append((top_y - intercept) / slope)
            if record and viz_out is not None:
                viz_out["hough_right"] = self._line_endpoints(slope, intercept, top_y, height)

        if left_present and right_present:
            center_x = float(np.average(x_targets))
        elif left_present:
            center_x = float(x_targets[0] + assumed_width / 2.0)
        else:
            center_x = float(x_targets[0] - assumed_width / 2.0)
        return center_x, left_present, right_present

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

    def _record_obs_viz(self, width, height, near_center, mid_center, far_center,
                        lane_center, center_error, roi_rect, viz_out: dict | None = None) -> None:
        """Stores ROI bands and centers for the PC debug overlay.

        Coordinates are recorded in ROI-local space; ``roi_offset`` lets the
        overlay translate them back onto the full camera frame.
        """

        target = viz_out if viz_out is not None else self.last_viz

        def band(y0_ratio, y1_ratio):
            return (int(clamp(y0_ratio, 0.0, 1.0) * height), int(clamp(y1_ratio, 0.0, 1.0) * height))

        x0, y0, roi_w, roi_h = roi_rect
        target.update({
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

        # Thread safety: build viz data in a local dict, then swap atomically
        # at the end so control_loop never sees a half-built dict.
        viz_tmp: dict = {} if collect_viz else self.last_viz

        full_height, full_width = frame_bgr.shape[:2]
        x0, y0, roi_w, roi_h = self.lane_roi_rect(full_width, full_height)
        roi_rect = (x0, y0, roi_w, roi_h)
        # Crop the lane search to the ROI box only; light/sign/aruco detection
        # keeps using the full frame elsewhere in the pipeline.
        roi_frame = frame_bgr[y0:y0 + roi_h, x0:x0 + roi_w]

        try:
            prepared_lab, prepared_l_channel = self.prepare_lane_lab(roi_frame)
        except cv2.error:
            prepared_lab = None
            prepared_l_channel = np.zeros(roi_frame.shape[:2], dtype=np.uint8)
        road_mask = self.build_road_mask(roi_frame, prepared_lab=prepared_lab)
        lane_mask = self.build_lane_mask(roi_frame, prepared_lab=prepared_lab)
        if collect_viz:
            # The operator view shows only the paint mask used by Canny/Hough.
            # Road remains separate for centroid fallback and fork decisions.
            viz_tmp["lane_mask"] = lane_mask
            viz_tmp["frame_size"] = (int(full_width), int(full_height))
        height, width = road_mask.shape[:2]
        mid = self.region_of_interest(road_mask, self.config.roi.mid_y0, self.config.roi.mid_y1)
        far = self.region_of_interest(road_mask, self.config.roi.far_y0, self.config.roi.far_y1)

        # Primary lane center uses the color-gated white|yellow paint mask. This
        # rejects unrelated brightness edges (people, arms, furniture) before
        # Canny/Hough while retaining both lane colors used on the course.
        hough_center, _, _ = self.average_hough_lanes(
            roi_frame,
            record=collect_viz,
            viz_out=viz_tmp,
            prepared_l_channel=lane_mask,
        )
        near_center = hough_center
        if near_center is None:
            # Fall back to the black-road-mask centroid only when no lane line was
            # found, so a fully black frame still yields a usable center.
            near = self.region_of_interest(road_mask, self.config.roi.near_y0, 1.0)
            near_center = self.contour_center_x(near, self.config.lane.min_component_area_ratio)

        # Far/mid centroids stay mask-based; they feed curvature and fork cues
        # the reference never computes but the mission FSM depends on.
        mid_center = self.contour_center_x(mid, self.config.lane.min_component_area_ratio)
        far_center = self.contour_center_x(far, self.config.lane.min_component_area_ratio)

        if near_center is None:
            if collect_viz:
                self._record_obs_viz(width, height, None, mid_center, far_center,
                                     None, self.prev_center_error, roi_rect, viz_out=viz_tmp)
                self.last_viz = viz_tmp
            return LaneObs(valid=False, center_error=self.prev_center_error)

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
        far_or_near = far_center if far_center is not None else near_center
        mid_or_near = mid_center if mid_center is not None else near_center
        signed_curv = (near_center - far_or_near) / max(float(full_width), 1.0)
        curvature = clamp(abs(far_or_near - near_center) / max(float(full_width), 1.0) * 2.5, 0.0, 1.0)
        if mid_center is not None:
            curvature = max(curvature, clamp(abs(mid_or_near - near_center) / max(float(full_width), 1.0) * 2.0, 0.0, 1.0))

        fork_seen, left_branch, right_branch = self.detect_fork_candidates(road_mask)
        self.prev_center_error = clamp(center_error, -1.0, 1.0)
        if collect_viz:
            self._record_obs_viz(width, height, near_center, mid_center, far_center,
                                 near_center, self.prev_center_error, roi_rect, viz_out=viz_tmp)
            self.last_viz = viz_tmp  # atomic reference swap
        return LaneObs(
            valid=True,
            center_error=self.prev_center_error,
            curvature=curvature,
            signed_curvature=clamp(signed_curv, -1.0, 1.0),
            fork_seen=fork_seen,
            left_branch=left_branch,
            right_branch=right_branch,
        )
