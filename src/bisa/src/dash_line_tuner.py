#!/usr/bin/env python3
"""On-vehicle yellow dashed-lane detector node (standalone, NOT in any launch yet).

Runs ON the car (all compute onboard): subscribes to the camera, detects the
YELLOW DASHED lane and the YELLOW SOLID cross-line, tells which SIDE the dash is
on (RIGHT = rotary ENTRY guide, LEFT = EXIT guide), and PUBLISHES the result so
the operator PC can watch/tune it exactly like the other bisa nodes — no cv2
window on the car, no remote compute.

Detection idea:
  * A LAB mask extracts yellow candidates (high b* channel; live-tunable).
  * A dash is a SHORT, ELONGATED blob. Each contour is kept only when its area,
    aspect ratio and length fall inside tunable bounds. The upper length/area
    bounds are what reject a SOLID line (one long blob) — so a solid line does
    NOT get reported as dashed.
  * Segments are split at a vertical divider (``side_split_x100``): those to the
    RIGHT count toward ENTRY, those to the LEFT toward EXIT. A side is "present"
    when it has >= min_segments dash segments.
  * The exit gate is a separate wide/thin/horizontal/centred/on-road yellow bar
    (the ``cross_*`` params); published as CROSS when present in the frame.

Run ON THE CAR (all thresholds are ROS params, so tuning needs no rebuild):
    ros2 run bisa dash_line_tuner
    # or: python3 src/bisa/src/dash_line_tuner.py

Publishes:
    /dash/debug/image/compressed   annotated overlay (view on the PC)
    /dash/debug/mask/compressed    yellow LAB mask (view on the PC)
    /dash/entry  /dash/exit        Bool: dash present on the RIGHT / LEFT
    /dash/cross                    Bool: yellow solid cross-line seen
    /dash/status                   String: one-line human-readable summary

Watch / tune it FROM THE PC (reuses existing tooling, no new code):
    ros2 run bisa viz_node --ros-args \
        -p debug_image_topic:=/dash/debug/image/compressed \
        -p lane_mask_topic:=/dash/debug/mask/compressed
    ros2 run rqt_reconfigure rqt_reconfigure     # slider tuning (or ros2 param set)
    ros2 topic echo /dash/status
"""

from __future__ import annotations

from dataclasses import dataclass, fields as dc_fields

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


# --------------------------------------------------------------------------- #
# Tunable parameters (declared as ROS params by the node, so they can be tuned
# live from the PC with rqt_reconfigure / ros2 param set — no colcon rebuild).
# --------------------------------------------------------------------------- #
@dataclass
class DashParams:
    """All live-tunable thresholds for the dashed-line detector."""

    # Region of interest (fractions of frame height) — lane markings live in the
    # lower part of the frame; the top is sky/background, excluded by default.
    roi_y0: float = 0.55
    roi_y1: float = 1.00

    # Road gate: lane markings sit ON the dark road, surrounded by it, while
    # background clutter (bright walls/furniture, colored objects) does not. We
    # build a dark-road mask (LAB L below road_l_max), dilate it, and keep only
    # dash segments whose centre lies on that dilated road region. This is what
    # rejects bright/colored background that the yellow mask alone catches.
    use_road_gate: int = 1      # 0/1 toggle
    road_l_max: int = 109        # LAB L (0..255); road is darker than this
    road_dilate: int = 3       # px; grows road so markings on it stay inside

    # Yellow LAB band. OpenCV 8-bit LAB: L 0..255 lightness, a 0..255 with 128
    # neutral (green<->red), b 0..255 with 128 neutral (blue<->yellow). Yellow is
    # the HIGH-b* region; b_min is the key discriminator. L_min drops the dark
    # road, a stays near neutral. Same colour space as the road mask, so it
    # reacts to the venue's light the same way the lane pipeline does.
    y_l_min: int = 100
    y_l_max: int = 255
    y_a_min: int = 110
    y_a_max: int = 150
    y_b_min: int = 150
    y_b_max: int = 255

    # Morphology (px). Open removes speckle; close bridges tiny gaps in one dash.
    open_k: int = 3
    close_k: int = 5

    # Per-segment shape gates. Areas/lengths are in ROI pixels.
    min_area: int = 15
    max_area: int = 4000
    min_aspect: int = 22       # (length/width)*10  -> 22 == 2.2:1 elongation
    min_len: int = 6
    max_len: int = 52         # upper bound rejects long SOLID line pieces
    max_width: int = 28        # dashes are THIN strips; rejects fat objects/boxes

    # A side is "dashed present" when it has >= this many qualifying segments.
    min_segments: int = 3

    # Left/right split for side reporting. A dash segment counts toward ENTRY
    # (rotary entry guide) when its centre is RIGHT of this vertical divider, and
    # toward EXIT (exit guide) when LEFT of it. Fraction of frame width *100
    # (50 == frame centre). Move it if the car is not centred on the road.
    side_split_x100: int = 50

    # Yellow SOLID cross-line (rotary exit gate). NOT a lane line: it lies
    # ACROSS the driving path (roughly horizontal in the image, wide + thin), as
    # opposed to the yellow guide line that runs ALONG travel. Detected from the
    # same yellow mask by shape: wide bounding box, limited thickness, high w/h.
    cross_min_len: int = 90      # min bbox width (px); the real gate is SHORT
    cross_max_thick: int = 24    # max bbox height (px); the real gate is thin
    cross_ratio_x10: int = 30    # min (width/height)*10 -> 30 == 3:1 horizontal
    # Only look for the gate in the near field (car drives OVER it): the bar's
    # centre row must be below this fraction of the frame. Rejects yellow high
    # up at walls / the along-travel guide line seen far ahead.
    cross_band_x100: int = 66    # near-field threshold, *100 (66 == 0.66*H)
    # Orientation: the gate is perpendicular to travel -> near-horizontal in the
    # image. Reject a yellow blob whose fitted line tilts more than this many
    # degrees from horizontal (kills diagonal guide-line pieces).
    cross_max_angle: int = 30    # deg from horizontal; 90 disables the filter
    # Isolation: a real gate has road ABOVE and below it; the guide line the car
    # follows keeps going (yellow continues upward). Reject the bar if the yellow
    # fraction in a band just above it exceeds this (the line continues = guide).
    cross_iso_x100: int = 30     # max yellow ratio above the bar; 100 disables
    # Centering: a real gate lies in front of the car and crosses the image
    # centre; the guide line the car peels off toward sits on one side. Reject a
    # bar whose horizontal midpoint is farther than this fraction of the width
    # from centre. Set 50 to disable (accept anywhere).
    cross_center_x100: int = 20  # max |midpoint-centre|/width; 50 disables
    # Road gate for the cross-line too: the real gate lies ON the road; yellow
    # clutter on furniture/objects does not. Require this fraction of the bar's
    # box to be road. 0 disables.
    cross_road_x100: int = 30    # min road fraction inside the bar; 0 disables


YELLOW_BGR = (0, 220, 255)   # RIGHT-side dash = rotary ENTRY guide
EXIT_BGR = (255, 200, 0)     # LEFT-side dash = EXIT guide (cyan)


# --------------------------------------------------------------------------- #
# Detection
# --------------------------------------------------------------------------- #
@dataclass
class Segment:
    """One dash candidate blob after shape filtering."""

    box: np.ndarray          # 4x2 int corner points (full-frame coords)
    center: tuple            # (x, y) full-frame
    length: float
    width: float
    aspect: float
    area: float


def _roi_bounds(h: int, p: DashParams) -> tuple[int, int]:
    y0 = int(np.clip(p.roi_y0, 0.0, 1.0) * h)
    y1 = int(np.clip(p.roi_y1, 0.0, 1.0) * h)
    if y1 <= y0:
        y1 = min(h, y0 + 1)
    return y0, y1


def yellow_mask(roi_bgr: np.ndarray, p: DashParams) -> np.ndarray:
    """Returns the yellow LAB mask over the ROI image.

    Yellow is the high-b* region of LAB. ``inRange`` over [L, a, b] keeps bright
    (L>=y_l_min), near-neutral-a, high-b pixels, which is the yellow paint under
    the venue lighting the same way the lane road mask sees it.
    """

    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    return cv2.inRange(
        lab,
        np.array([p.y_l_min, p.y_a_min, p.y_b_min], np.uint8),
        np.array([p.y_l_max, p.y_a_max, p.y_b_max], np.uint8),
    )


def road_gate_mask(roi_bgr: np.ndarray, p: DashParams) -> np.ndarray:
    """Dilated dark-road mask; a dash counts only if its centre is on it."""

    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    l_ch = lab[:, :, 0]
    road = cv2.inRange(l_ch, 0, int(p.road_l_max))
    d = max(0, int(p.road_dilate))
    if d > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * d + 1, 2 * d + 1))
        road = cv2.dilate(road, k)
    return road


def _clean(mask: np.ndarray, p: DashParams) -> np.ndarray:
    ok = max(1, int(p.open_k))
    ck = max(1, int(p.close_k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((ok, ok), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((ck, ck), np.uint8))
    return mask


def find_segments(mask: np.ndarray, y_offset: int, p: DashParams,
                  road: np.ndarray | None = None) -> list[Segment]:
    """Keeps only short, elongated blobs on the road — the shape of a dash.

    ``y_offset`` shifts ROI-local coordinates back onto the full frame so the
    overlay lines up with the original image. When ``road`` (a ROI-local road
    mask) is given, a segment is kept only if its centre lies on the road.
    """

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_aspect = p.min_aspect / 10.0
    out: list[Segment] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < p.min_area or area > p.max_area:
            continue
        (cx, cy), (w, h), _angle = cv2.minAreaRect(c)
        length = max(w, h)
        width = max(min(w, h), 1e-6)
        aspect = length / width
        if aspect < min_aspect:
            continue                      # not elongated enough -> not a dash
        if length < p.min_len or length > p.max_len:
            continue                      # too long -> solid line, not a dash
        if width > p.max_width:
            continue                      # too fat -> object/box, not a thin dash
        if road is not None:
            ry = int(np.clip(cy, 0, road.shape[0] - 1))
            rx = int(np.clip(cx, 0, road.shape[1] - 1))
            if road[ry, rx] == 0:
                continue                  # centre not on the road -> background
        box = cv2.boxPoints(((cx, cy), (w, h), _angle)).astype(np.int32)
        box[:, 1] += y_offset
        out.append(Segment(box, (int(cx), int(cy) + y_offset),
                           length, width, aspect, area))
    return out


def detect_cross_line(ymask: np.ndarray, y_offset: int, full_h: int, p: DashParams,
                      road: np.ndarray | None = None):
    """Finds a yellow SOLID line lying ACROSS the path (rotary exit gate).

    Returns the widest qualifying bar as a full-frame (x, y, w, h) bbox, else
    None. A gate is wide + thin + horizontal, near-field (the car drives over
    it), centred in front of the car, and ON the road — unlike the along-travel
    guide line (off to a side, continues upward) or yellow clutter on furniture
    (off the road, thicker). ``road`` is the ROI-local dilated road mask.
    """

    contours, _ = cv2.findContours(ymask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ratio = p.cross_ratio_x10 / 10.0
    band_y = p.cross_band_x100 / 100.0 * full_h
    iso_max = p.cross_iso_x100 / 100.0
    best = None
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if w < p.cross_min_len or h > p.cross_max_thick:
            continue
        if w / max(h, 1) < ratio:
            continue
        if (y_offset + y + h / 2.0) < band_y:
            continue                      # too high in frame -> not near-field
        # Centering: the gate crosses in front of the car (image centre); the
        # peel-off guide line sits to one side. Reject off-centre bars.
        full_w = ymask.shape[1]
        if abs((x + w / 2.0) - full_w / 2.0) > p.cross_center_x100 / 100.0 * full_w:
            continue
        # Orientation: reject blobs whose fitted line tilts too far from
        # horizontal (a gate is perpendicular to travel = near-horizontal).
        vx, vy, _, _ = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        angle = float(np.degrees(np.arctan2(abs(vy), abs(vx) + 1e-6)))  # 0=horiz
        if angle > p.cross_max_angle:
            continue
        # Isolation: if yellow continues in the band just above the bar, this is
        # the along-travel guide line (not an isolated crossing gate).
        iso_h = int(max(12, 1.5 * h))
        above = ymask[max(0, y - iso_h):y, x:x + w]
        if above.size and cv2.countNonZero(above) / float(above.size) > iso_max:
            continue
        # Road gate: the real gate sits on the road; furniture/object clutter
        # does not. Require enough road inside the bar's box.
        if road is not None and p.cross_road_x100 > 0:
            inside = road[y:y + h, x:x + w]
            if not inside.size or cv2.countNonZero(inside) / float(inside.size) < p.cross_road_x100 / 100.0:
                continue
        if best is None or w > best[2]:
            best = (x, y + y_offset, w, h)
    return best


@dataclass
class DetectResult:
    yellow: list[Segment]
    ymask: np.ndarray
    road: np.ndarray | None
    roi: tuple[int, int]
    full_w: int = 0                  # full frame width, for the L/R side split
    cross: tuple | None = None       # yellow solid cross-line bbox, or None

    def present(self, p: DashParams) -> bool:
        return len(self.yellow) >= p.min_segments

    def split_x(self, p: DashParams) -> int:
        """The vertical divider x (full-frame px) separating EXIT(L) / ENTRY(R)."""

        return int(np.clip(p.side_split_x100 / 100.0, 0.0, 1.0) * self.full_w)

    def sides(self, p: DashParams) -> tuple[list[Segment], list[Segment]]:
        """Splits dash segments into (left=EXIT, right=ENTRY) by centre x."""

        sx = self.split_x(p)
        left = [s for s in self.yellow if s.center[0] < sx]
        right = [s for s in self.yellow if s.center[0] >= sx]
        return left, right

    def present_left(self, p: DashParams) -> bool:
        return len(self.sides(p)[0]) >= p.min_segments

    def present_right(self, p: DashParams) -> bool:
        return len(self.sides(p)[1]) >= p.min_segments


def detect(frame_bgr: np.ndarray, p: DashParams) -> DetectResult:
    """Runs the yellow dashed-line + cross-gate detection on one BGR frame."""

    h, w = frame_bgr.shape[:2]
    y0, y1 = _roi_bounds(h, p)
    roi = frame_bgr[y0:y1]
    ymask = _clean(yellow_mask(roi, p), p)
    road = road_gate_mask(roi, p) if int(p.use_road_gate) else None
    yellow = find_segments(ymask, y0, p, road)
    cross = detect_cross_line(ymask, y0, h, p, road)
    return DetectResult(yellow, ymask, road, (y0, y1), w, cross)


# --------------------------------------------------------------------------- #
# Visualization (drawn on the car, published as JPEG for the PC to display)
# --------------------------------------------------------------------------- #
def draw_overlay(frame_bgr: np.ndarray, res: DetectResult, p: DashParams) -> np.ndarray:
    """Draws ROI, the L/R split, dash segments, the yellow cross-line, and a HUD."""

    view = frame_bgr.copy()
    y0, y1 = res.roi
    if res.road is not None:
        # Faint green tint over the accepted road region (the dash search area).
        tint = view[y0:y1].copy()
        tint[res.road > 0] = (0, 120, 0)
        view[y0:y1] = cv2.addWeighted(view[y0:y1], 0.75, tint, 0.25, 0)
    cv2.rectangle(view, (0, y0), (view.shape[1] - 1, y1 - 1), (80, 80, 80), 1)

    # Side split: LEFT segments (EXIT guide) in cyan, RIGHT (ENTRY guide) in
    # yellow, with the divider drawn so the split point is visible while tuning.
    left, right = res.sides(p)
    sx = res.split_x(p)
    cv2.line(view, (sx, y0), (sx, y1 - 1), (120, 120, 120), 1)
    for seg in left:
        cv2.drawContours(view, [seg.box], -1, EXIT_BGR, 2)
    for seg in right:
        cv2.drawContours(view, [seg.box], -1, YELLOW_BGR, 2)

    if res.cross is not None:
        x, y, w, h = res.cross
        cv2.rectangle(view, (x, y), (x + w, y + h), (0, 0, 255), 3)  # red = cross gate
        cv2.putText(view, "YELLOW SOLID (cross)", (x, max(14, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

    entry_on = len(right) >= p.min_segments   # dash on the RIGHT -> rotary ENTRY
    exit_on_side = len(left) >= p.min_segments  # dash on the LEFT -> EXIT guide
    _hud(view, [
        f"ENTRY (R): {len(right)} seg  ->  {'DETECTED' if entry_on else '-'}",
        f"EXIT  (L): {len(left)} seg  ->  {'DETECTED' if exit_on_side else '-'}",
        f"CROSS line: {'SEEN' if res.cross is not None else '-'}",
    ])
    return view


def mask_view(mask: np.ndarray, roi: tuple[int, int], full_shape, tint) -> np.ndarray:
    """Places an ROI mask back on a full-frame black canvas, tinted."""

    canvas = np.zeros(full_shape, np.uint8)
    y0, y1 = roi
    canvas[y0:y1][mask > 0] = tint
    return canvas


def _hud(frame: np.ndarray, lines: list[str]) -> None:
    y = 20
    for text in lines:
        cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y += 22


# --------------------------------------------------------------------------- #
# ROS 2 node (runs on the vehicle)
# --------------------------------------------------------------------------- #
class DashLineNode(Node):
    """Detects the yellow dashed lane on the live camera and publishes results.

    All ``DashParams`` fields are exposed as ROS parameters, so the operator PC
    tunes them live (rqt_reconfigure / ros2 param set) with no rebuild — the same
    workflow as the main bisa node. The annotated overlay + yellow mask are
    published as CompressedImage so an existing viewer (viz_node / rqt_image_view)
    shows them without the car opening any window.
    """

    def __init__(self):
        super().__init__("dash_line_tuner")
        self.p = DashParams()

        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("debug_image_topic", "/dash/debug/image/compressed")
        self.declare_parameter("mask_topic", "/dash/debug/mask/compressed")
        self.declare_parameter("publish_debug_image", True)
        self.image_topic = str(self.get_parameter("image_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        mask_topic = str(self.get_parameter("mask_topic").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        # Expose every DashParams field as a flat ROS parameter, typed from its
        # dataclass default (roi_y0/roi_y1 -> double, the rest -> integer).
        for f in dc_fields(DashParams):
            self.declare_parameter(f.name, getattr(self.p, f.name))
            setattr(self.p, f.name, self.get_parameter(f.name).value)
        self.add_on_set_parameters_callback(self._on_set_params)

        # Match the camera publisher: always take the newest frame, drop stale
        # ones instead of triggering RELIABLE retransmits over WiFi.
        img_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, self.image_topic, self._on_image, img_qos)
        self.debug_pub = self.create_publisher(CompressedImage, debug_topic, img_qos)
        self.mask_pub = self.create_publisher(CompressedImage, mask_topic, img_qos)
        self.entry_pub = self.create_publisher(Bool, "/dash/entry", 10)
        self.exit_pub = self.create_publisher(Bool, "/dash/exit", 10)
        self.cross_pub = self.create_publisher(Bool, "/dash/cross", 10)
        self.status_pub = self.create_publisher(String, "/dash/status", 10)

        self.get_logger().info(
            f"dash_line_tuner up: image={self.image_topic} -> "
            f"overlay={debug_topic}, mask={mask_topic}, "
            f"status=/dash/status,/dash/entry,/dash/exit,/dash/cross "
            f"(publish_debug_image={self.publish_debug})"
        )

    def _on_set_params(self, params) -> SetParametersResult:
        """Applies live DashParams edits from the PC; ignores non-param names."""

        for prm in params:
            if hasattr(self.p, prm.name):
                current = getattr(self.p, prm.name)
                try:
                    setattr(self.p, prm.name, type(current)(prm.value))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason=f"bad value for {prm.name}")
        return SetParametersResult(successful=True)

    def _publish_jpeg(self, pub, image_bgr, stamp) -> None:
        ok, jpg = cv2.imencode(".jpg", image_bgr)
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data = jpg.tobytes()
        pub.publish(msg)

    def _on_image(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        res = detect(frame, self.p)
        left, right = res.sides(self.p)
        entry = len(right) >= self.p.min_segments   # RIGHT dash -> rotary ENTRY
        exit_side = len(left) >= self.p.min_segments  # LEFT dash -> EXIT guide
        cross = res.cross is not None

        self.entry_pub.publish(Bool(data=bool(entry)))
        self.exit_pub.publish(Bool(data=bool(exit_side)))
        self.cross_pub.publish(Bool(data=bool(cross)))
        self.status_pub.publish(String(data=(
            f"ENTRY(R) {len(right)} {'OK' if entry else '-'}  |  "
            f"EXIT(L) {len(left)} {'OK' if exit_side else '-'}  |  "
            f"CROSS {'SEEN' if cross else '-'}")))

        if self.publish_debug:
            stamp = self.get_clock().now().to_msg()
            self._publish_jpeg(self.debug_pub, draw_overlay(frame, res, self.p), stamp)
            self._publish_jpeg(
                self.mask_pub, mask_view(res.ymask, res.roi, frame.shape, YELLOW_BGR), stamp)


def main(args=None) -> None:
    """Initializes rclpy and spins the dash-line detector node (run on the car)."""

    rclpy.init(args=args)
    node = DashLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
