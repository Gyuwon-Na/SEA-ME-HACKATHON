"""Pure OpenCV overlay drawing for the PC-side D-Racer debug view.

No ROS dependencies so it can be unit-tested and reused. Every function takes a
BGR frame and draws in place (or on a copy supplied by the caller).
"""

from __future__ import annotations

import math

import cv2
import numpy as np

# BGR colors.
COLOR_LANE = (0, 255, 0)        # detected lane lines (solid)
COLOR_LANE_RAW = (0, 160, 0)    # raw hough segments (faint)
COLOR_LANE_SELECTED = (0, 200, 255)  # segments supporting the selected curves
COLOR_CENTER_LINE = (255, 255, 0)   # image vertical center line
COLOR_LANE_CENTER = (0, 255, 0)     # perceived lane center marker (reference green)
COLOR_VEHICLE_CENTER = (0, 0, 255)  # vehicle center marker (reference red)
COLOR_STEER = (0, 0, 255)       # steering angle line
COLOR_ARUCO = (255, 0, 255)     # aruco marker
COLOR_ARUCO_TARGET = (0, 255, 255)  # target id marker
COLOR_ROI = (255, 128, 0)       # lane-detection ROI box
COLOR_LIGHT_ROI = (0, 255, 255)     # traffic-light analysis ROI box
COLOR_HUD = (255, 255, 255)
COLOR_EDGES = (255, 120, 0)     # canny edge pixels on the mask view
COLOR_BAND_NEAR = (0, 255, 255)
COLOR_BAND_MID = (0, 200, 255)
COLOR_BAND_FAR = (255, 200, 0)

DET_COLORS = {
    "traffic_green": (0, 255, 0),
    "traffic_red": (0, 0, 255),
    "sign_left": (255, 200, 0),
    "sign_right": (0, 200, 255),
}

MAX_STEER_DEG = 50.0


def _finite(value) -> bool:
    try:
        return np.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _ipt(value) -> int | None:
    if not _finite(value):
        return None
    return int(round(float(value)))


def _clip(value, upper: int) -> int | None:
    ivalue = _ipt(value)
    if ivalue is None:
        return None
    return max(0, min(ivalue, max(int(upper) - 1, 0)))


def _point(x, y, width: int, height: int):
    px = _clip(x, width)
    py = _clip(y, height)
    if px is None or py is None:
        return None
    return (px, py)


def _rect(frame, bbox):
    height, width = frame.shape[:2]
    if len(bbox) != 4 or not all(_finite(v) for v in bbox):
        return None
    x1, y1, x2, y2 = bbox
    p0 = _point(min(x1, x2), min(y1, y2), width, height)
    p1 = _point(max(x1, x2), max(y1, y2), width, height)
    if p0 is None or p1 is None or p0 == p1:
        return None
    return p0, p1


def _poly(points, ox=0, oy=0):
    safe = []
    for x, y in points:
        px = _ipt(x)
        py = _ipt(y)
        if px is None or py is None:
            return None
        safe.append([px + int(ox), py + int(oy)])
    return np.array(safe, dtype=np.int32)


def draw_lane_roi(frame, lane_viz) -> None:
    """Draws the lane-detection ROI box on the full camera frame."""

    if not lane_viz:
        return
    rect = lane_viz.get("roi_rect")
    if not rect:
        return
    safe = _rect(frame, (rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3]))
    if safe is None:
        return
    (x0, y0), (x1, y1) = safe
    cv2.rectangle(frame, (x0, y0), (x1, y1), COLOR_ROI, 2)
    cv2.putText(frame, f"lane ROI {x1 - x0}x{y1 - y0}", (x0 + 4, max(14, y0 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_ROI, 1, cv2.LINE_AA)


def draw_lanes(frame, lane_viz) -> None:
    """Draws selected quadratic lanes plus lane-center vs vehicle-center markers.

    Mirrors the reference ``lane_detector_1029.py`` visualization: bold green
    averaged lane lines blended translucently over the frame (display_lines +
    addWeighted), a green lane-center dot, and a red vehicle-center dot so the
    steering error is visible at a glance. Perception runs in ROI-local
    coordinates, so points are translated by ``roi_offset`` onto the full frame.
    """

    if not lane_viz:
        return
    height_f, width_f = frame.shape[:2]
    ox, oy = lane_viz.get("roi_offset", (0, 0))

    # Bold selected lane curves on a separate layer, then blend.
    curves = []
    for key in ("hough_left_curve", "hough_right_curve"):
        curve = lane_viz.get(key)
        if curve:
            poly = _poly(curve, ox, oy)
            if poly is not None:
                curves.append(poly)
    # Backward compatibility for visualization dictionaries recorded before
    # quadratic fitting was introduced.
    if not curves:
        for key in ("hough_left", "hough_right"):
            line = lane_viz.get(key)
            if line:
                poly = _poly(line, ox, oy)
                if poly is not None:
                    curves.append(poly)
    if curves:
        layer = np.zeros_like(frame)
        for curve in curves:
            cv2.polylines(layer, [curve], False, COLOR_LANE, 6, cv2.LINE_AA)
        cv2.addWeighted(frame, 0.8, layer, 1.0, 0.0, dst=frame)

    # Lane-center (green) vs vehicle-center (red) comparison at the near band.
    near_band = lane_viz.get("near_band")
    near_center = lane_viz.get("near_center")
    if near_band is not None:
        y_marker = _clip((near_band[0] + near_band[1]) / 2 + oy, height_f)
        if y_marker is None:
            return
        cv2.circle(frame, (width_f // 2, y_marker), 8, COLOR_VEHICLE_CENTER, -1)
        if near_center is not None:
            point = _point(float(near_center) + ox, y_marker, width_f, height_f)
            if point is not None:
                cv2.circle(frame, point, 8, COLOR_LANE_CENTER, -1)


def draw_light_roi(frame, light_roi) -> None:
    """Draws the traffic-light detection ROI box (where light boxes are kept)."""

    if not light_roi:
        return
    height, width = frame.shape[:2]
    x0, y0, x1, y1 = light_roi
    p0 = _point(x0 * width, y0 * height, width, height)
    p1 = _point(x1 * width, y1 * height, width, height)
    if p0 is None or p1 is None or p0 == p1:
        return
    cv2.rectangle(frame, p0, p1, COLOR_LIGHT_ROI, 2)
    cv2.putText(frame, "light ROI", (p0[0] + 4, max(14, p0[1] + 18)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_LIGHT_ROI, 1, cv2.LINE_AA)


def draw_center_and_steering(frame, steering: float) -> None:
    """Draws the image vertical center line and the steering-angle indicator.

    Pipeline convention: positive steering steers LEFT (center_error > 0 when
    the lane center is left of the vehicle), so the arrow tips toward -x.
    """

    height, width = frame.shape[:2]
    cx = width // 2
    cv2.line(frame, (cx, 0), (cx, height), COLOR_CENTER_LINE, 1)

    theta = math.radians(max(-1.0, min(1.0, steering)) * MAX_STEER_DEG)
    length = int(height * 0.5)
    tip = _point(cx - length * math.sin(theta), height - length * math.cos(theta), width, height)
    if tip is not None:
        cv2.arrowedLine(frame, (cx, height - 1), tip, COLOR_STEER, 3, tipLength=0.15)


def draw_detections(frame, detections, accepted_ids=None, light_verdicts=None) -> None:
    """Draws best.pt detection bounding boxes with class labels.

    Traffic-light boxes follow the SAME visualization as the tuner: colored by
    the classify_light verdict in ``light_verdicts`` (id(det) -> "traffic_green"
    / "traffic_red" / None) and labeled GREEN / RED / none, never by the
    unreliable raw YOLO class. Signs and other classes color by their detected
    class; when ``accepted_ids`` is given, non-accepted ones are drawn thin with
    a "(gated)" suffix so the model's raw output stays visible.
    """

    light_verdicts = light_verdicts or {}
    for det in detections or []:
        rect = _rect(frame, det.bbox)
        if rect is None:
            continue
        (x1, y1), (x2, y2) = rect
        if det.cls in ("traffic_green", "traffic_red"):
            verdict = light_verdicts.get(id(det))
            color = {"traffic_green": (0, 255, 0), "traffic_red": (0, 0, 255)}.get(
                verdict, (170, 170, 170))
            label = {"traffic_green": "GREEN", "traffic_red": "RED"}.get(verdict, "none")
            thickness = 2 if verdict else 1
        else:
            color = DET_COLORS.get(det.cls, (200, 200, 200))
            accepted = accepted_ids is None or id(det) in accepted_ids
            thickness = 2 if accepted else 1
            label = f"{det.cls} {det.conf:.2f}" + ("" if accepted else " (gated)")
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(frame, label, (x1, max(12, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def draw_aruco(frame, markers, target_id: int) -> None:
    """Draws a bounding box and ID label for each detected ArUco marker."""

    for marker in markers or []:
        rect = _rect(frame, marker.bbox)
        if rect is None:
            continue
        (x1, y1), (x2, y2) = rect
        is_target = marker.id == target_id
        color = COLOR_ARUCO_TARGET if is_target else COLOR_ARUCO
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        label = f"ID {marker.id}" + (" *" if is_target else "")
        cv2.putText(frame, label, (x1, max(14, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)


def draw_hud(frame, lines) -> None:
    """Draws a stacked text HUD in the top-left corner."""

    y = 20
    for text in lines:
        cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    COLOR_HUD, 1, cv2.LINE_AA)
        y += 22


def draw_lane_mask_view(lane_viz, cmd):
    """Renders the ROI-sized binarized lane mask with every lane-tuning overlay.

    Unlike :func:`draw_overlay` (raw camera frame), this view shows the exact
    mask the lane pipeline thresholds — so the LAB threshold, morphology,
    and canny/hough sliders all have an immediately visible effect. On top of
    the mask it draws the Canny edges, raw and averaged Hough lines, the
    near/mid/far ROI bands with their detected centers, the vehicle-center
    line, and the steering arrow, all in ROI-local coordinates. Returns None
    until the perception pipeline has recorded a mask.
    """

    if not lane_viz:
        return None
    mask = lane_viz.get("lane_mask")
    if mask is None:
        # Backward compatibility with older recorded visualization dictionaries.
        mask = lane_viz.get("road_mask")
    if mask is None:
        return None
    view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    height, width = view.shape[:2]

    edges = lane_viz.get("edges")
    if edges is not None and edges.shape[:2] == (height, width):
        view[edges > 0] = COLOR_EDGES

    # ROI bands (horizontal guide lines) with their mask centroids.
    for band_key, center_key, color in (
        ("near_band", "near_center", COLOR_BAND_NEAR),
        ("mid_band", "mid_center", COLOR_BAND_MID),
        ("far_band", "far_center", COLOR_BAND_FAR),
    ):
        band = lane_viz.get(band_key)
        if not band:
            continue
        y0, y1 = band
        y0 = _clip(y0, height)
        y1 = _clip(y1 - 1, height)
        if y0 is None or y1 is None:
            continue
        cv2.line(view, (0, y0), (width - 1, y0), color, 1)
        cv2.line(view, (0, y1), (width - 1, y1), color, 1)
        center = lane_viz.get(center_key)
        if center is not None:
            point = _point(center, (y0 + y1) / 2, width, height)
            if point is not None:
                cv2.circle(view, point, 6, color, -1)

    # Raw proposals (faint), selected clusters (amber), then fitted curves (bold).
    for x1, y1, x2, y2 in lane_viz.get("hough_segments") or []:
        p0 = _point(x1, y1, width, height)
        p1 = _point(x2, y2, width, height)
        if p0 is not None and p1 is not None:
            cv2.line(view, p0, p1, COLOR_LANE_RAW, 1)
    for x1, y1, x2, y2 in lane_viz.get("hough_selected_segments") or []:
        p0 = _point(x1, y1, width, height)
        p1 = _point(x2, y2, width, height)
        if p0 is not None and p1 is not None:
            cv2.line(view, p0, p1, COLOR_LANE_SELECTED, 2)
    curves = []
    for key in ("hough_left_curve", "hough_right_curve"):
        curve = lane_viz.get(key)
        if curve:
            poly = _poly(curve)
            if poly is not None:
                curves.append(poly)
    if not curves:
        for key in ("hough_left", "hough_right"):
            line = lane_viz.get(key)
            if line:
                poly = _poly(line)
                if poly is not None:
                    curves.append(poly)
    for curve in curves:
        cv2.polylines(view, [curve], False, COLOR_LANE, 2, cv2.LINE_AA)
    top_y = lane_viz.get("hough_top_y")
    if top_y is not None:
        top_y = _clip(top_y, height)
        if top_y is not None:
            cv2.line(view, (0, top_y), (width - 1, top_y), COLOR_LANE_RAW, 1)

    # Vehicle center (full-frame center translated into ROI coordinates) and
    # the lane-center marker the controller actually steers against.
    ox, _ = lane_viz.get("roi_offset", (0, 0))
    frame_w = lane_viz.get("frame_size", (width, height))[0]
    vehicle_x = frame_w // 2 - ox
    vehicle_x = _clip(vehicle_x, width)
    if vehicle_x is None:
        return view
    cv2.line(view, (vehicle_x, 0), (vehicle_x, height - 1), COLOR_CENTER_LINE, 1)
    near_band = lane_viz.get("near_band")
    near_center = lane_viz.get("near_center")
    if near_band and near_center is not None:
        y_marker = _clip((near_band[0] + near_band[1]) / 2, height)
        if y_marker is None:
            return view
        cv2.circle(view, (vehicle_x, y_marker), 6, COLOR_VEHICLE_CENTER, -1)
        point = _point(near_center, y_marker, width, height)
        if point is not None:
            cv2.circle(view, point, 6, COLOR_LANE_CENTER, -1)

    # Steering arrow from the bottom of the ROI. Pipeline convention: positive
    # steering steers LEFT, so the arrow tips toward -x (same as the full view).
    steering = float(getattr(cmd, "steering", 0.0))
    theta = math.radians(max(-1.0, min(1.0, steering)) * MAX_STEER_DEG)
    length = int(height * 0.45)
    tip = _point(vehicle_x - length * math.sin(theta), height - length * math.cos(theta), width, height)
    if tip is not None:
        cv2.arrowedLine(view, (vehicle_x, height - 1), tip, COLOR_STEER, 2, tipLength=0.2)

    err = lane_viz.get("center_error", 0.0)
    draw_hud(view, [
        f"mask=WHITE|YELLOW {width}x{height}",
        f"err={err:+.2f} steer={steering:+.2f}",
    ])
    return view


def draw_overlay(frame, lane_viz, detections, markers, cmd, state, target_id=3,
                 light_roi=None, light_verdicts=None, accepted_ids=None, cc_active=None):
    """Draws the full debug overlay on a copy of the frame and returns it."""

    out = frame.copy()
    draw_lane_roi(out, lane_viz)
    draw_lanes(out, lane_viz)
    draw_center_and_steering(out, float(getattr(cmd, "steering", 0.0)))
    draw_detections(out, detections, accepted_ids, light_verdicts)
    # Keep the traffic-light ROI gate active in object_detector.py, but do not
    # clutter the operator view with its yellow guide rectangle.
    draw_aruco(out, markers, target_id)

    det_classes = sorted({det.cls for det in (detections or [])})
    aruco_ids = sorted({m.id for m in (markers or [])})
    hud = [
        f"state={state}",
        f"thr={getattr(cmd, 'throttle', 0.0):.2f} steer={getattr(cmd, 'steering', 0.0):+.2f}",
        f"det={','.join(det_classes) if det_classes else '-'}",
        f"aruco={aruco_ids if aruco_ids else '-'}",
    ]
    if cc_active is not None:
        # Clarify whether the color correction shown here is also fed to the
        # detector (enabled) or is view-only (tuning preview).
        hud.append("cc=ON(detector)" if cc_active else "cc=view-only")
    draw_hud(out, hud)
    return out
