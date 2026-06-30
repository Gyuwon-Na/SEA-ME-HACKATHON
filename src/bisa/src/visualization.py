"""Pure OpenCV overlay drawing for the PC-side D-Racer debug view.

No ROS dependencies so it can be unit-tested and reused. Every function takes a
BGR frame and draws in place (or on a copy supplied by the caller).
"""

from __future__ import annotations

import math

import cv2

# BGR colors.
COLOR_LANE = (0, 255, 0)        # detected lane lines (solid)
COLOR_LANE_RAW = (0, 160, 0)    # raw hough segments (faint)
COLOR_CENTER_LINE = (255, 255, 0)   # image vertical center line
COLOR_LANE_CENTER = (0, 165, 255)   # perceived lane center path
COLOR_STEER = (0, 0, 255)       # steering angle line
COLOR_ARUCO = (255, 0, 255)     # aruco marker
COLOR_ARUCO_TARGET = (0, 255, 255)  # target id marker
COLOR_ROI = (255, 128, 0)       # lane-detection ROI box
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
    """Draws raw hough segments, averaged lane lines, and the lane-center path.

    Perception runs in ROI-local coordinates, so every point is translated by
    ``roi_offset`` to land in the correct place on the full frame.
    """

    if not lane_viz:
        return
    ox, oy = lane_viz.get("roi_offset", (0, 0))
    for seg in lane_viz.get("hough_segments", []) or []:
        x1, y1, x2, y2 = seg
        cv2.line(frame, (x1 + ox, y1 + oy), (x2 + ox, y2 + oy), COLOR_LANE_RAW, 1)

    for key in ("hough_left", "hough_right"):
        line = lane_viz.get(key)
        if line:
            (x1, y1), (x2, y2) = line
            cv2.line(frame, (_ipt(x1) + ox, _ipt(y1) + oy), (_ipt(x2) + ox, _ipt(y2) + oy), COLOR_LANE, 3)

    # Perceived lane center as a path through far -> mid -> near centroids.
    pts = []
    for center_key, band_key in (("far_center", "far_band"),
                                 ("mid_center", "mid_band"),
                                 ("near_center", "near_band")):
        cx = lane_viz.get(center_key)
        band = lane_viz.get(band_key)
        if cx is not None and band is not None:
            y_mid = (band[0] + band[1]) // 2
            pts.append((_ipt(cx) + ox, y_mid + oy))
    for i, point in enumerate(pts):
        cv2.circle(frame, point, 5, COLOR_LANE_CENTER, -1)
        if i > 0:
            cv2.line(frame, pts[i - 1], point, COLOR_LANE_CENTER, 2)


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


def draw_overlay(frame, lane_viz, detections, markers, cmd, state, target_id=3):
    """Draws the full debug overlay on a copy of the frame and returns it."""

    out = frame.copy()
    draw_lane_roi(out, lane_viz)
    draw_lanes(out, lane_viz)
    draw_center_and_steering(out, float(getattr(cmd, "steering", 0.0)))
    draw_detections(out, detections)
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
