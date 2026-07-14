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
        self.prev_left_target: Optional[float] = None
        self.prev_right_target: Optional[float] = None
        self.tracked_lane_width: Optional[float] = None
        self.prev_single_is_left: Optional[bool] = None
        self.filtered_curvature = 0.0
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
        self,
        frame_bgr: np.ndarray,
        prepared_lab: np.ndarray | None = None,
        include_yellow: bool | None = None,
    ) -> np.ndarray:
        """Builds the steering mask, excluding yellow for OUT by default."""

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
        open_kernel = self._get_morph_kernel(cfg.morph_open_kernel)
        close_kernel = self._get_morph_kernel(cfg.morph_close_kernel)
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, open_kernel)
        white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, close_kernel)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, open_kernel)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, close_kernel)
        if include_yellow is None:
            route = self.config.mission.route_mode.upper()
            include_yellow = route != "OUT" or not cfg.out_white_only
        mask = cv2.bitwise_or(white, yellow) if include_yellow else white
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

    @staticmethod
    def _evaluate_curve(coefficients: np.ndarray, y: float, top_y: int, bottom_y: int) -> float:
        """Evaluates normalized ``x(y) = ay^2 + by + c`` coefficients."""

        t = (float(y) - top_y) / max(float(bottom_y - top_y), 1.0)
        return float(coefficients[0] * t * t + coefficients[1] * t + coefficients[2])

    @classmethod
    def _curve_points(
        cls, coefficients: np.ndarray, top_y: int, height: int, step: int = 4
    ) -> list[tuple[float, int]]:
        """Samples a fitted curve for debug rendering."""

        bottom_y = height - 1
        return [
            (cls._evaluate_curve(coefficients, y, top_y, bottom_y), y)
            for y in range(top_y, bottom_y + 1, max(1, step))
        ]

    @classmethod
    def _fit_hough_curve(
        cls, points: np.ndarray, top_y: int, height: int, width: int
    ) -> Optional[np.ndarray]:
        """Fits one robust quadratic ``x(y)`` curve, with a linear sparse fallback."""

        if points.shape[0] < 2:
            return None
        bottom_y = height - 1
        span = max(float(bottom_y - top_y), 1.0)
        unique_y = np.unique(np.round(points[:, 1], 1))
        quadratic = bool(
            unique_y.size >= 3
            and float(np.ptp(points[:, 1])) >= 0.15 * span
        )

        def solve(samples: np.ndarray) -> np.ndarray:
            t = (samples[:, 1] - top_y) / span
            if quadratic:
                design = np.column_stack((t * t, t, np.ones_like(t)))
                return np.linalg.lstsq(design, samples[:, 0], rcond=None)[0]
            design = np.column_stack((t, np.ones_like(t)))
            linear = np.linalg.lstsq(design, samples[:, 0], rcond=None)[0]
            return np.array([0.0, linear[0], linear[1]], dtype=float)

        return solve(points)

    def average_hough_lanes(
        self,
        frame_bgr: np.ndarray,
        record: bool = False,
        viz_out: dict | None = None,
        prepared_l_channel: np.ndarray | None = None,
        vehicle_x: float | None = None,
        curvature_hint: float = 0.0,
        previous_center: float | None = None,
    ) -> tuple[Optional[float], bool, bool]:
        """Fits lane curves and preserves their left/right identity over time.

        A tight bend can move the only visible boundary across the camera center,
        so image-half classification is only an initial hint.  When one curve is
        left, the previous boundary tracks and lane center choose its identity.
        """

        height, width = frame_bgr.shape[:2]
        base_top = float(self.config.lane.hough_roi_top_ratio)
        curve_top = max(base_top, float(self.config.lane.hough_curve_top_ratio))
        adaptive_top = base_top + (curve_top - base_top) * clamp(curvature_hint, 0.0, 1.0)
        top_y = int(clamp(adaptive_top, 0.0, 0.90) * height)
        if record and viz_out is not None:
            viz_out["hough_left_curve"] = None
            viz_out["hough_right_curve"] = None
            viz_out["hough_segments"] = []
            viz_out["hough_selected_segments"] = []
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

        dx = x2 - x1
        dy = y2 - y1
        min_abs = self.config.lane.hough_slope_min_abs
        valid = (np.abs(dy) > 1e-6) & (np.abs(dy) >= min_abs * np.abs(dx))
        x1, y1, x2, y2 = x1[valid], y1[valid], x2[valid], y2[valid]
        if x1.size == 0:
            return None, False, False

        dx = x2 - x1
        dy = y2 - y1
        bottom_y = height - 1
        reference_x = x1 + (bottom_y - y1) * dx / dy
        in_bounds = (
            np.isfinite(reference_x)
            & (reference_x >= -0.25 * width)
            & (reference_x <= 1.25 * width)
        )
        x1, y1, x2, y2, reference_x = (
            values[in_bounds] for values in (x1, y1, x2, y2, reference_x)
        )
        if reference_x.size == 0:
            return None, False, False

        camera_x = width / 2.0 if vehicle_x is None else float(vehicle_x)
        left_indices = np.where(reference_x < camera_x)[0]
        right_indices = np.where(reference_x > camera_x)[0]
        cluster_limit = max(16.0, width * 0.12)

        if record and viz_out is not None:
            for ix in range(reference_x.size):
                viz_out["hough_segments"].append(
                    (int(x1[ix]), int(y1[ix]), int(x2[ix]), int(y2[ix]))
                )

        def select_curve(indices: np.ndarray, is_left: bool) -> Optional[np.ndarray]:
            if indices.size == 0:
                return None
            side_x = reference_x[indices]
            seed_index = indices[int(np.argmax(side_x) if is_left else np.argmin(side_x))]
            selected = indices[np.abs(side_x - reference_x[seed_index]) <= cluster_limit]
            ratios = np.linspace(0.0, 1.0, 8)
            points = np.vstack([
                np.column_stack((
                    x1[ix] + ratios * (x2[ix] - x1[ix]),
                    y1[ix] + ratios * (y2[ix] - y1[ix]),
                ))
                for ix in selected
            ])
            if record and viz_out is not None:
                for ix in selected:
                    viz_out["hough_selected_segments"].append(
                        (int(x1[ix]), int(y1[ix]), int(x2[ix]), int(y2[ix]))
                    )
            return self._fit_hough_curve(points, top_y, height, width)

        left_curve = select_curve(left_indices, True)
        right_curve = select_curve(right_indices, False)
        left_present = left_curve is not None
        right_present = right_curve is not None
        if not left_present and not right_present:
            return None, False, False

        if record and viz_out is not None:
            if left_curve is not None:
                viz_out["hough_left_curve"] = self._curve_points(left_curve, top_y, height)
            if right_curve is not None:
                viz_out["hough_right_curve"] = self._curve_points(right_curve, top_y, height)

        left_target = self._evaluate_curve(left_curve, top_y, top_y, bottom_y) \
            if left_curve is not None else None
        right_target = self._evaluate_curve(right_curve, top_y, top_y, bottom_y) \
            if right_curve is not None else None
        fallback_width = width * self.config.lane.assumed_lane_width_ratio
        lane_width = self.tracked_lane_width or fallback_width
        if left_target is not None and right_target is not None:
            observed_width = right_target - left_target
            minimum = width * self.config.lane.lane_width_min_ratio
            maximum = width * self.config.lane.lane_width_max_ratio
            if minimum <= observed_width <= maximum:
                smoothing = clamp(self.config.lane.lane_width_smoothing, 0.0, 1.0)
                if self.tracked_lane_width is None:
                    self.tracked_lane_width = observed_width
                else:
                    self.tracked_lane_width += smoothing * (
                        observed_width - self.tracked_lane_width
                    )
                lane_width = self.tracked_lane_width
            self.prev_left_target = left_target
            self.prev_right_target = right_target
            self.prev_single_is_left = None
            center_x = (left_target + right_target) / 2.0
        else:
            target = left_target if left_target is not None else right_target
            raw_is_left = left_target is not None
            left_center = target + lane_width / 2.0
            right_center = target - lane_width / 2.0

            def identity_score(is_left: bool) -> float | None:
                evidence = []
                previous_boundary = self.prev_left_target if is_left else self.prev_right_target
                if previous_boundary is not None:
                    evidence.append(abs(target - previous_boundary))
                if previous_center is not None:
                    hypothesis = left_center if is_left else right_center
                    evidence.append(abs(hypothesis - previous_center))
                return sum(evidence) / len(evidence) if evidence else None

            left_score = identity_score(True)
            right_score = identity_score(False)
            if left_score is None or right_score is None:
                is_left = raw_is_left
            else:
                is_left = left_score <= right_score
                margin = width * self.config.lane.single_lane_switch_margin_ratio
                if self.prev_single_is_left is not None and is_left != self.prev_single_is_left:
                    previous_score = left_score if self.prev_single_is_left else right_score
                    alternative_score = right_score if self.prev_single_is_left else left_score
                    if previous_score <= alternative_score + margin:
                        is_left = self.prev_single_is_left

            center_x = left_center if is_left else right_center
            self.prev_single_is_left = is_left
            if record and viz_out is not None and is_left != raw_is_left:
                points_key = "hough_left_curve" if raw_is_left else "hough_right_curve"
                moved_points = viz_out.get(points_key)
                viz_out[points_key] = None
                viz_out["hough_left_curve" if is_left else "hough_right_curve"] = moved_points
            if is_left:
                self.prev_left_target = target
            else:
                self.prev_right_target = target
            left_present, right_present = is_left, not is_left
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
        near = self.region_of_interest(road_mask, self.config.roi.near_y0, 1.0)
        mid = self.region_of_interest(road_mask, self.config.roi.mid_y0, self.config.roi.mid_y1)
        far = self.region_of_interest(road_mask, self.config.roi.far_y0, self.config.roi.far_y1)

        # The road mask supplies a cheap curvature hint before Hough runs.  Its
        # EMA controls how much of the far fit is discarded on a tight bend.
        road_near_center = self.contour_center_x(
            near, self.config.lane.min_component_area_ratio
        )
        mid_center = self.contour_center_x(mid, self.config.lane.min_component_area_ratio)
        far_center = self.contour_center_x(far, self.config.lane.min_component_area_ratio)
        rough_curvature = self.filtered_curvature
        if road_near_center is not None:
            rough_curvature = 0.0
            if far_center is not None:
                rough_curvature = max(
                    rough_curvature,
                    clamp(abs(far_center - road_near_center) / max(float(full_width), 1.0) * 2.5, 0.0, 1.0),
                )
            if mid_center is not None:
                rough_curvature = max(
                    rough_curvature,
                    clamp(abs(mid_center - road_near_center) / max(float(full_width), 1.0) * 2.0, 0.0, 1.0),
                )
        smoothing = clamp(self.config.lane.hough_curvature_smoothing, 0.0, 1.0)
        self.filtered_curvature += smoothing * (rough_curvature - self.filtered_curvature)

        # Primary lane center uses the color-gated white|yellow paint mask. This
        # rejects unrelated brightness edges (people, arms, furniture) before
        # Canny/Hough while retaining both lane colors used on the course.
        hough_center, _, _ = self.average_hough_lanes(
            roi_frame,
            record=collect_viz,
            viz_out=viz_tmp,
            prepared_l_channel=lane_mask,
            vehicle_x=full_width / 2.0 - x0,
            curvature_hint=self.filtered_curvature,
            previous_center=(
                self.prev_near_center - x0 if self.prev_near_center is not None else None
            ),
        )
        near_center = hough_center
        if near_center is None:
            # Fall back to the black-road-mask centroid only when no lane line was
            # found, so a fully black frame still yields a usable center.
            near_center = road_near_center

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
