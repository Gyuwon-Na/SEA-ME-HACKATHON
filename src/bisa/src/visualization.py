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
COLOR_CENTER_LINE = (255, 255, 0)   # image vertical center line
COLOR_LANE_CENTER = (0, 255, 0)     # perceived lane center marker (reference green)
COLOR_VEHICLE_CENTER = (0, 0, 255)  # vehicle center marker (reference red)
COLOR_STEER = (0, 0, 255)       # steering angle line
COLOR_ARUCO = (255, 0, 255)     # aruco marker
COLOR_ARUCO_TARGET = (0, 255, 255)  # target id marker
COLOR_ROI = (255, 128, 0)       # lane-detection ROI box
COLOR_LIGHT_ROI = (0, 255, 255)     # traffic-light analysis ROI box
COLOR_HUD = (255, 255, 255)

DET_COLORS = {
    "traffic_green": (0, 255, 0),
    "traffic_red": (0, 0, 255),
    "sign_left": (255, 200, 0),
    "sign_right": (0, 200, 255),
    "dynamic_marker": (255, 0, 0),
    "finish_line": (200, 200, 200),
}

MAX_STEER_DEG = 50.0


def _ipt(value) -> int:
    return int(round(value))


def draw_lane_roi(frame, lane_viz) -> None:
    """Draws the lane-detection ROI box on the full camera frame."""

    if not lane_viz:
        return
    rect = lane_viz.get("roi_rect")
    if not rect:
        return
    x0, y0, w, h = rect
    cv2.rectangle(frame, (x0, y0), (x0 + w, y0 + h), COLOR_ROI, 2)
    cv2.putText(frame, f"lane ROI {w}x{h}", (x0 + 4, max(14, y0 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_ROI, 1, cv2.LINE_AA)


def draw_lanes(frame, lane_viz) -> None:
    """Draws averaged lane lines plus lane-center vs vehicle-center markers.

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

    # Bold averaged lane lines on a separate layer, then blend (reference look).
    segments = []
    for key in ("hough_left", "hough_right"):
        line = lane_viz.get(key)
        if line:
            (x1, y1), (x2, y2) = line
            segments.append(((_ipt(x1) + ox, _ipt(y1) + oy), (_ipt(x2) + ox, _ipt(y2) + oy)))
    if segments:
        layer = np.zeros_like(frame)
        for point_a, point_b in segments:
            cv2.line(layer, point_a, point_b, COLOR_LANE, 6)
        cv2.addWeighted(frame, 0.8, layer, 1.0, 0.0, dst=frame)

    # Lane-center (green) vs vehicle-center (red) comparison at the near band.
    near_band = lane_viz.get("near_band")
    near_center = lane_viz.get("near_center")
    if near_band is not None:
        y_marker = (near_band[0] + near_band[1]) // 2 + oy
        cv2.circle(frame, (width_f // 2, y_marker), 8, COLOR_VEHICLE_CENTER, -1)
        if near_center is not None:
            cv2.circle(frame, (_ipt(near_center) + ox, y_marker), 8, COLOR_LANE_CENTER, -1)


def draw_light_roi(frame, light_roi, light_states=None) -> None:
    """Draws the traffic-light HSV ROI box and its detected color state."""

    if not light_roi:
        return
    height, width = frame.shape[:2]
    x0, y0, x1, y1 = light_roi
    p0 = (_ipt(x0 * width), _ipt(y0 * height))
    p1 = (_ipt(x1 * width), _ipt(y1 * height))
    cv2.rectangle(frame, p0, p1, COLOR_LIGHT_ROI, 2)
    active = [name.upper() for name, on in (light_states or {}).items() if on]
    label = "light: " + (", ".join(active) if active else "-")
    cv2.putText(frame, label, (p0[0] + 4, max(14, p0[1] + 18)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_LIGHT_ROI, 1, cv2.LINE_AA)


def draw_center_and_steering(frame, steering: float) -> None:
    """Draws the image vertical center line and the steering-angle indicator."""

    height, width = frame.shape[:2]
    cx = width // 2
    cv2.line(frame, (cx, 0), (cx, height), COLOR_CENTER_LINE, 1)

    theta = math.radians(max(-1.0, min(1.0, steering)) * MAX_STEER_DEG)
    length = int(height * 0.5)
    tip = (_ipt(cx + length * math.sin(theta)), _ipt(height - length * math.cos(theta)))
    cv2.arrowedLine(frame, (cx, height - 1), tip, COLOR_STEER, 3, tipLength=0.15)


def draw_detections(frame, detections) -> None:
    """Draws best.pt detection bounding boxes with class labels."""

    for det in detections or []:
        x1, y1, x2, y2 = (int(v) for v in det.bbox)
        color = DET_COLORS.get(det.cls, (200, 200, 200))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{det.cls} {det.conf:.2f}", (x1, max(12, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def draw_aruco(frame, markers, target_id: int) -> None:
    """Draws a bounding box and ID label for each detected ArUco marker."""

    for marker in markers or []:
        x1, y1, x2, y2 = (int(v) for v in marker.bbox)
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


def draw_overlay(frame, lane_viz, detections, markers, cmd, state, target_id=3,
                 light_roi=None, light_states=None):
    """Draws the full debug overlay on a copy of the frame and returns it."""

    out = frame.copy()
    draw_lane_roi(out, lane_viz)
    draw_lanes(out, lane_viz)
    draw_center_and_steering(out, float(getattr(cmd, "steering", 0.0)))
    draw_detections(out, detections)
    draw_light_roi(out, light_roi, light_states)
    draw_aruco(out, markers, target_id)

    det_classes = sorted({det.cls for det in (detections or [])})
    aruco_ids = sorted({m.id for m in (markers or [])})
    draw_hud(out, [
        f"state={state}",
        f"thr={getattr(cmd, 'throttle', 0.0):.2f} steer={getattr(cmd, 'steering', 0.0):+.2f}",
        f"det={','.join(det_classes) if det_classes else '-'}",
        f"aruco={aruco_ids if aruco_ids else '-'}",
    ])
    return out
