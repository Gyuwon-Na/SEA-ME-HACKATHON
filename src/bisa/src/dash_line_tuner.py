#!/usr/bin/env python3
"""Standalone dashed-lane detection tuner (PC only, NOT part of driving.launch).

Purpose: verify — before wiring anything into the driving launch — that we can
robustly detect the YELLOW DASHED lane and the YELLOW SOLID cross-line (rotary
exit gate) from the camera feed. It plays a rosbag (the
``/camera/image/compressed`` topic) directly, so you only need this one command.

Detection idea:
  * An HSV mask extracts yellow candidates (live-tunable).
  * A dash is a SHORT, ELONGATED blob. Each contour is kept only when its area,
    aspect ratio and length fall inside tunable bounds. The upper length/area
    bounds are what reject a SOLID line (one long blob) — so a solid line does
    NOT get reported as dashed.
  * The dashed lane is "present" when it yields >= min_segments dash segments.
    The exit gate is a separate wide/thin/horizontal/centred/on-road bar (the
    ``cross_*`` params), counted with a min-lap time gate for the exit trigger.

Two modes:
  * GUI (default):  loops the bag with trackbars + 2 windows (overlay, yellow
                    mask). Keys: space=pause, n=step, r=restart,
                    s=dump YAML, q=quit.
  * --export DIR:   headless. Annotates every Nth frame to DIR and prints the
                    per-frame yellow segment counts. Lets you review
                    stills without a live GUI.

Run (GUI):
    python3 src/bisa/src/dash_line_tuner.py --bag lane_test_bag

Run (headless export):
    python3 src/bisa/src/dash_line_tuner.py --bag lane_test_bag \\
        --export /tmp/dash_out --stride 30
"""

from __future__ import annotations

import argparse
import glob
import os
import sqlite3
from dataclasses import dataclass, asdict

import cv2
import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    _HAVE_ROS = True
except Exception:  # pragma: no cover - allows running with raw-jpeg fallback
    _HAVE_ROS = False


# --------------------------------------------------------------------------- #
# Tunable parameters (kept local to this tool so it needs no shared-config
# edits / colcon rebuild while we are still validating the approach).
# --------------------------------------------------------------------------- #
@dataclass
class DashParams:
    """All live-tunable thresholds for the dashed-line detector."""

    # ---- TOP competition dial ------------------------------------------------
    # Minimum seconds that must pass after a counted gate crossing before the
    # NEXT one can count. One rotary lap physically takes at least this long, so
    # it stops guide-line false detections from inflating the crossing count and
    # declaring the lap "done" too early. Set it to the real lap time at the
    # venue. First slider so it is quick to change on the day.
    min_lap_sec: float = 15.0

    # Region of interest (fractions of frame height) — lane markings live in the
    # lower part of the frame; the top is sky/background, excluded by default.
    roi_y0: float = 0.55
    roi_y1: float = 1.00

    # Road gate: lane markings sit ON the dark road, surrounded by it, while
    # background clutter (bright walls/furniture, colored objects) does not. We
    # build a dark-road mask (LAB L below road_l_max), dilate it, and keep only
    # dash segments whose centre lies on that dilated road region. This is what
    # rejects bright/colored background that HSV alone catches.
    use_road_gate: int = 1      # 0/1 toggle
    road_l_max: int = 109        # LAB L (0..255); road is darker than this
    road_dilate: int = 3       # px; grows road so markings on it stay inside

    # Yellow HSV band (OpenCV H is 0..180).
    y_h_lo: int = 20
    y_h_hi: int = 50
    y_s_min: int = 32
    y_v_min: int = 140

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

    # A color is "dashed present" when it has >= this many qualifying segments.
    min_segments: int = 3

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
    # Exit logic: count each distinct crossing (debounced by cross_gap absent
    # frames). One full rotary lap re-passes the entry gate, so exit fires once
    # the gate has been seen >= exit_count times.
    cross_gap: int = 8           # frames the gate must be absent before re-count
    exit_count: int = 2          # crossings seen -> EXIT trigger (1 lap done)


# (trackbar label, DashParams field, scale)  scale 10 == value stored *10.
SLIDERS = [
    ("MIN LAP sec x10", "min_lap_sec", 10, 0, 600),   # top competition dial
    ("roi y0 x100", "roi_y0", 100, 0, 100),
    ("roi y1 x100", "roi_y1", 100, 0, 100),
    ("road gate 0/1", "use_road_gate", 1, 0, 1),
    ("road L max", "road_l_max", 1, 0, 255),
    ("road dilate", "road_dilate", 1, 0, 80),
    ("yellow H lo", "y_h_lo", 1, 0, 180),
    ("yellow H hi", "y_h_hi", 1, 0, 180),
    ("yellow S min", "y_s_min", 1, 0, 255),
    ("yellow V min", "y_v_min", 1, 0, 255),
    ("open k", "open_k", 1, 1, 15),
    ("close k", "close_k", 1, 1, 15),
    ("min area", "min_area", 1, 0, 2000),
    ("max area", "max_area", 1, 100, 20000),
    ("min aspect x10", "min_aspect", 1, 10, 100),
    ("min len", "min_len", 1, 1, 200),
    ("max len", "max_len", 1, 10, 640),
    ("max width", "max_width", 1, 3, 120),
    ("min segments", "min_segments", 1, 1, 8),
    ("cross min len", "cross_min_len", 1, 20, 640),
    ("cross max thick", "cross_max_thick", 1, 5, 200),
    ("cross ratio x10", "cross_ratio_x10", 1, 10, 100),
    ("cross band x100", "cross_band_x100", 1, 0, 100),
    ("cross max angle", "cross_max_angle", 1, 0, 90),
    ("cross iso x100", "cross_iso_x100", 1, 0, 100),
    ("cross center x100", "cross_center_x100", 1, 0, 50),
    ("cross road x100", "cross_road_x100", 1, 0, 100),
    ("cross gap frames", "cross_gap", 1, 1, 60),
    ("exit count", "exit_count", 1, 1, 6),
]

YELLOW_BGR = (0, 220, 255)


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
    """Returns the yellow HSV mask over the ROI image."""

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(
        hsv,
        np.array([p.y_h_lo, p.y_s_min, p.y_v_min], np.uint8),
        np.array([p.y_h_hi, 255, 255], np.uint8),
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
    cross: tuple | None = None       # yellow solid cross-line bbox, or None

    def present(self, p: DashParams) -> bool:
        return len(self.yellow) >= p.min_segments


def detect(frame_bgr: np.ndarray, p: DashParams) -> DetectResult:
    """Runs the yellow dashed-line + cross-gate detection on one BGR frame."""

    h = frame_bgr.shape[0]
    y0, y1 = _roi_bounds(h, p)
    roi = frame_bgr[y0:y1]
    ymask = _clean(yellow_mask(roi, p), p)
    road = road_gate_mask(roi, p) if int(p.use_road_gate) else None
    yellow = find_segments(ymask, y0, p, road)
    cross = detect_cross_line(ymask, y0, h, p, road)
    return DetectResult(yellow, ymask, road, (y0, y1), cross)


# Params that change the cross-line timeline (yellow mask shape + gate + gap).
# exit_count is a threshold applied afterwards, so it is NOT in the signature.
_CROSS_SIG_FIELDS = ("min_lap_sec", "roi_y0", "roi_y1", "y_h_lo", "y_h_hi",
                     "y_s_min", "y_v_min", "open_k", "close_k", "cross_min_len",
                     "cross_max_thick", "cross_ratio_x10", "cross_band_x100",
                     "cross_max_angle", "cross_iso_x100", "cross_center_x100",
                     "cross_road_x100", "use_road_gate", "road_l_max",
                     "road_dilate", "cross_gap")


def _cross_sig(p: DashParams) -> tuple:
    return tuple(round(float(getattr(p, f)), 4) for f in _CROSS_SIG_FIELDS)


def _cross_present(frame_bgr: np.ndarray, p: DashParams) -> bool:
    h = frame_bgr.shape[0]
    y0, y1 = _roi_bounds(h, p)
    roi = frame_bgr[y0:y1]
    ymask = _clean(yellow_mask(roi, p), p)
    road = road_gate_mask(roi, p) if int(p.use_road_gate) else None
    return detect_cross_line(ymask, y0, h, p, road) is not None


def cross_counts(buffers: list, timestamps: list, p: DashParams) -> list[int]:
    """Debounced cumulative crossing count at each frame index (time order).

    A crossing is counted when the gate reappears after >= ``cross_gap`` absent
    frames AND at least ``min_lap_sec`` seconds have passed since the previous
    counted crossing (the first crossing always counts). The time gate is what
    stops guide-line false detections from inflating the count between real laps.
    """

    gap = max(1, int(p.cross_gap))
    min_lap = float(p.min_lap_sec)
    counts, c, absent, last_t = [], 0, gap, None
    for buf, t in zip(buffers, timestamps):
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        seen = frame is not None and _cross_present(frame, p)
        if seen:
            if absent >= gap and (last_t is None or (t - last_t) >= min_lap):
                c += 1
                last_t = t
            absent = 0
        else:
            absent += 1
        counts.append(c)
    return counts


# --------------------------------------------------------------------------- #
# Visualization
# --------------------------------------------------------------------------- #
def draw_overlay(frame_bgr: np.ndarray, res: DetectResult, p: DashParams,
                 cross_count: int | None = None, exit_on: bool = False,
                 elapsed: float | None = None) -> np.ndarray:
    """Draws ROI, dash segments, the yellow cross-line, and a HUD.

    ``cross_count`` (debounced crossings seen so far), ``exit_on`` and
    ``elapsed`` (seconds since the first crossing) come from the temporal
    counter in the caller; when None those HUD lines are omitted.
    """

    view = frame_bgr.copy()
    y0, y1 = res.roi
    if res.road is not None:
        # Faint green tint over the accepted road region (the dash search area).
        tint = view[y0:y1].copy()
        tint[res.road > 0] = (0, 120, 0)
        view[y0:y1] = cv2.addWeighted(view[y0:y1], 0.75, tint, 0.25, 0)
    cv2.rectangle(view, (0, y0), (view.shape[1] - 1, y1 - 1), (80, 80, 80), 1)

    for seg in res.yellow:
        cv2.drawContours(view, [seg.box], -1, YELLOW_BGR, 2)

    if res.cross is not None:
        x, y, w, h = res.cross
        cv2.rectangle(view, (x, y), (x + w, y + h), (0, 0, 255), 3)  # red = cross gate
        cv2.putText(view, "YELLOW SOLID (cross)", (x, max(14, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

    y_on = res.present(p)
    lines = [
        f"YELLOW dash: {len(res.yellow)} seg  ->  {'DETECTED' if y_on else '-'}",
        f"CROSS line: {'SEEN' if res.cross is not None else '-'}",
    ]
    if cross_count is not None:
        lines.append(f"crossings: {cross_count}/{int(p.exit_count)}")
    if elapsed is not None:
        lines.append(f"lap time: +{elapsed:4.1f}s  (min {float(p.min_lap_sec):.1f}s)")
    _hud(view, lines)

    if exit_on:
        _banner(view, "EXIT TRIGGER  (1 lap done)")
    return view


def _banner(frame: np.ndarray, text: str) -> None:
    """Stamps a bold centered banner across the top (exit trigger cue)."""

    w = frame.shape[1]
    cv2.rectangle(frame, (0, 0), (w, 46), (0, 0, 180), -1)
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    cv2.putText(frame, text, ((w - tw) // 2, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)


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
# Bag reading
# --------------------------------------------------------------------------- #
def _resolve_db3(bag_path: str) -> str:
    if os.path.isdir(bag_path):
        hits = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
        if not hits:
            raise FileNotFoundError(f"no .db3 file under {bag_path}")
        return hits[0]
    return bag_path


def load_bag_records(bag_path: str, topic: str) -> tuple[list[np.ndarray], list[float]]:
    """Returns (compressed JPEG buffers, elapsed-seconds) for a topic, time order.

    Keeping compressed buffers (~119 MB) and decoding on demand avoids holding
    every decoded 640x480 BGR frame (~2 GB) in RAM. Timestamps are returned as
    seconds elapsed from the first message, for the lap-time exit gate.
    """

    db3 = _resolve_db3(bag_path)
    con = sqlite3.connect(db3)
    try:
        cur = con.cursor()
        row = cur.execute("SELECT id FROM topics WHERE name = ?", (topic,)).fetchone()
        if row is None:
            names = [r[0] for r in cur.execute("SELECT name FROM topics").fetchall()]
            raise ValueError(f"topic '{topic}' not in bag. available: {names}")
        rows = cur.execute(
            "SELECT data, timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (row[0],),
        ).fetchall()
    finally:
        con.close()
    if not rows:
        return [], []
    t0 = rows[0][1]
    bufs = [_jpeg_buffer(data) for data, _t in rows]
    ts = [(t - t0) / 1e9 for _data, t in rows]   # rosbag timestamps are ns
    return bufs, ts


def load_bag_buffers(bag_path: str, topic: str) -> list[np.ndarray]:
    """Compressed JPEG buffers only (timestamps discarded)."""

    return load_bag_records(bag_path, topic)[0]


def iter_bag_frames(bag_path: str, topic: str):
    """Yields decoded BGR frames from a rosbag2 sqlite3 file, in time order."""

    for buf in load_bag_buffers(bag_path, topic):
        yield cv2.imdecode(buf, cv2.IMREAD_COLOR)


def _jpeg_buffer(blob: bytes) -> np.ndarray:
    """Extracts the raw JPEG bytes from a serialized CompressedImage CDR blob."""

    if _HAVE_ROS:
        msg = deserialize_message(blob, CompressedImage)
        return np.frombuffer(msg.data, np.uint8)
    i = blob.find(b"\xff\xd8\xff")  # fallback: locate embedded JPEG SOI marker
    return np.frombuffer(blob[i:] if i >= 0 else blob, np.uint8)


# --------------------------------------------------------------------------- #
# Modes
# --------------------------------------------------------------------------- #
def dump_yaml(p: DashParams) -> None:
    print("\n# ---- dash_line_tuner values ----")
    for k, v in asdict(p).items():
        print(f"{k}: {v}")
    print("# --------------------------------\n")


def run_export(bag: str, topic: str, out_dir: str | None, video: str | None,
               stride: int, fps: float, p: DashParams) -> None:
    """Headless: annotate frames to stills (``out_dir``) and/or an mp4 (``video``).

    Every frame is annotated and written to the video; stills are written every
    ``stride``-th frame. Per-frame yellow segment counts are printed.
    """

    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    writer = None
    saved = total = 0
    y_hits = 0
    for idx, frame in enumerate(iter_bag_frames(bag, topic)):
        if frame is None:
            continue
        total += 1
        res = detect(frame, p)
        y_on = res.present(p)
        y_hits += int(y_on)
        view = draw_overlay(frame, res, p)
        if video:
            if writer is None:
                os.makedirs(os.path.dirname(os.path.abspath(video)), exist_ok=True)
                h, w = view.shape[:2]
                writer = cv2.VideoWriter(
                    video, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            writer.write(view)
        if out_dir and idx % stride == 0:
            path = os.path.join(out_dir, f"frame_{idx:05d}.jpg")
            cv2.imwrite(path, view)
            saved += 1
        if idx % stride == 0:
            cross = "[CROSS]" if res.cross is not None else ""
            print(f"frame {idx:5d}: yellow={len(res.yellow)} seg "
                  f"{'[YELLOW]' if y_on else '        '}  {cross}")
    if writer is not None:
        writer.release()
    print(f"\nprocessed {total} frames")
    if out_dir:
        print(f"saved {saved} annotated stills to {out_dir}")
    if video:
        print(f"saved annotated video to {video}")
    print(f"yellow dashed present in {y_hits}/{total} frames")


def run_gui(bag: str, topic: str, p: DashParams) -> None:
    """Interactive trackbar tuner looping the bag."""

    controls = "Dash Controls"
    win_overlay, win_y = "Dashes", "Yellow Mask"
    for name in (win_overlay, win_y):
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, 640, 360)
    cv2.namedWindow(controls, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(controls, 460, 640)
    cv2.moveWindow(win_overlay, 20, 20)
    cv2.moveWindow(win_y, 680, 20)
    cv2.moveWindow(controls, 20, 420)
    for label, fieldname, scale, lo, hi in SLIDERS:
        init = int(round(getattr(p, fieldname) * scale))
        cv2.createTrackbar(label, controls, max(lo, min(hi, init)), hi, lambda _v: None)
        if lo > 0:
            cv2.setTrackbarMin(label, controls, lo)

    def read_sliders() -> None:
        for label, fieldname, scale, _lo, _hi in SLIDERS:
            pos = cv2.getTrackbarPos(label, controls)
            setattr(p, fieldname, pos / scale if scale != 1 else pos)

    buffers = load_bag_buffers(bag, topic)
    if not buffers:
        print("no frames in bag")
        return
    print(f"loaded {len(buffers)} frames; space=pause n=step r=restart s=dump q=quit")

    i = 0
    paused = False
    while True:
        read_sliders()
        frame = cv2.imdecode(buffers[i], cv2.IMREAD_COLOR)
        if frame is None:
            i = (i + 1) % len(buffers)
            continue
        res = detect(frame, p)
        full = frame.shape
        cv2.imshow(win_overlay, draw_overlay(frame, res, p))
        cv2.imshow(win_y, mask_view(res.ymask, res.roi, full, YELLOW_BGR))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            dump_yaml(p)
        elif key == ord(" "):
            paused = not paused
        elif key == ord("r"):
            i = 0
        elif key == ord("n"):
            i = (i + 1) % len(buffers)
        if not paused and key != ord("n"):
            i = (i + 1) % len(buffers)
    cv2.destroyAllWindows()


def _label(img: np.ndarray, text: str) -> np.ndarray:
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def montage(frame: np.ndarray, res: DetectResult, p: DashParams,
            cross_count: int | None = None, exit_on: bool = False,
            elapsed: float | None = None) -> np.ndarray:
    """Overlay on top, the yellow mask below (one image)."""

    full = frame.shape
    overlay = draw_overlay(frame, res, p, cross_count, exit_on, elapsed)
    h, w = overlay.shape[:2]
    ymv = _label(mask_view(res.ymask, res.roi, full, YELLOW_BGR), "yellow mask")
    half = cv2.resize(ymv, (w, h // 2))
    return np.vstack([overlay, half])


# HTML is generated from the SLIDERS table so the two never drift apart.
def _web_page(n_frames: int, p: DashParams) -> bytes:
    rows = []
    for label, fieldname, scale, lo, hi in SLIDERS:
        val = int(round(getattr(p, fieldname) * scale))
        rows.append(
            f'<div class=row><label>{label}</label>'
            f'<input type=range id="{fieldname}" data-scale="{scale}" '
            f'min="{lo}" max="{hi}" value="{val}" oninput="upd()">'
            f'<span id="{fieldname}_v">{val}</span></div>')
    controls = "\n".join(rows)
    html = f"""<!doctype html><meta charset=utf-8><title>Dash Tuner</title>
<style>
 body{{margin:0;background:#1e1e1e;color:#ddd;font:13px system-ui;display:flex}}
 #left{{flex:0 0 340px;padding:10px;overflow:auto;height:100vh}}
 #right{{flex:1;display:flex;flex-direction:column;align-items:center;padding:10px}}
 .row{{display:flex;align-items:center;gap:6px;margin:2px 0}}
 .row label{{flex:0 0 120px}} .row input{{flex:1}} .row span{{flex:0 0 34px;text-align:right}}
 img{{max-width:100%;border:1px solid #444}} h3{{margin:6px 0}}
 #bar{{display:flex;gap:8px;align-items:center;width:100%;margin:6px 0}}
 button{{background:#333;color:#ddd;border:1px solid #555;padding:4px 10px;cursor:pointer}}
 pre{{white-space:pre-wrap;background:#111;padding:8px;font-size:12px}}
</style>
<div id=left>
 <h3>Dash params ({n_frames} frames)</h3>
 {controls}
 <button onclick="dump()">show YAML</button>
 <pre id=yaml></pre>
</div>
<div id=right>
 <div id=bar>
  <button onclick="play=!play;this.textContent=play?'pause':'play'">play</button>
  <input type=range id=fi min=0 max="{n_frames-1}" value=0 oninput="upd()" style=flex:1>
  <span id=filbl>0</span>
 </div>
 <div id=stat></div>
 <img id=view>
</div>
<script>
let play=false;
function params(){{
 let q=[]; document.querySelectorAll('#left input[type=range]').forEach(s=>{{
   q.push(s.id+'='+(s.value/(+s.dataset.scale)));
   document.getElementById(s.id+'_v').textContent=s.value;}});
 return q.join('&');
}}
async function upd(){{
 let i=document.getElementById('fi').value;
 document.getElementById('filbl').textContent=i;
 let r=await fetch('/frame?i='+i+'&'+params());
 document.getElementById('stat').textContent=r.headers.get('X-Stat')||'';
 let b=await r.blob(); document.getElementById('view').src=URL.createObjectURL(b);
}}
function dump(){{
 let o={{}}; document.querySelectorAll('#left input[type=range]').forEach(s=>{{}});
 let t=''; document.querySelectorAll('#left input[type=range]').forEach(s=>{{
   t+=s.id+': '+(s.value/(+s.dataset.scale))+'\\n';}});
 document.getElementById('yaml').textContent=t;
}}
setInterval(()=>{{ if(play){{ let f=document.getElementById('fi');
   f.value=(+f.value+1)%{n_frames}; upd(); }} }}, 120);
upd();
</script>"""
    return html.encode("utf-8")


def run_web(bag: str, topic: str, port: int, p: DashParams) -> None:
    """Serves a localhost tuner: browser is the GUI, OpenCV runs headless.

    Works when X/WSLg windows do not show up, because the visualization is an
    <img> in the browser (open http://localhost:PORT in the Windows browser).
    """

    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from urllib.parse import urlparse, parse_qs

    buffers, timestamps = load_bag_records(bag, topic)
    if not buffers:
        print("no frames in bag")
        return
    fields = {f: (scale) for _l, f, scale, _lo, _hi in SLIDERS}
    # Cross-line count timeline is expensive (scans all frames), so cache it
    # keyed by the params that affect it; recomputed only when those change.
    timeline_cache: dict = {}

    def counts_for(q: DashParams) -> list[int]:
        sig = _cross_sig(q)
        if sig not in timeline_cache:
            timeline_cache.clear()   # only keep the latest to bound memory
            print(f"  computing cross-line timeline ({len(buffers)} frames)...")
            timeline_cache[sig] = cross_counts(buffers, timestamps, q)
        return timeline_cache[sig]

    def elapsed_at(counts: list[int], i: int) -> float | None:
        """Seconds since the first counted crossing, for the HUD lap timer."""

        if counts[i] < 1:
            return None
        first = next((k for k in range(i + 1) if counts[k] >= 1), i)
        return timestamps[i] - timestamps[first]

    def params_from_query(qs: dict) -> DashParams:
        q = DashParams()
        for name, scale in fields.items():
            if name in qs:
                try:
                    setattr(q, name, float(qs[name][0]))
                except ValueError:
                    pass
        return q

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # silence per-request logging
            pass

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                body = _web_page(len(buffers), p)
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if u.path == "/frame":
                qs = parse_qs(u.query)
                i = int(float(qs.get("i", ["0"])[0])) % len(buffers)
                q = params_from_query(qs)
                frame = cv2.imdecode(buffers[i], cv2.IMREAD_COLOR)
                res = detect(frame, q)
                counts = counts_for(q)
                cc = counts[i]
                exit_on = cc >= int(q.exit_count)
                el = elapsed_at(counts, i)
                ok, jpg = cv2.imencode(".jpg", montage(frame, res, q, cc, exit_on, el))
                data = jpg.tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header(
                    "X-Stat",
                    f"frame {i}  |  CROSS "
                    f"{'SEEN' if res.cross is not None else '-'}  |  crossings "
                    f"{cc}/{int(q.exit_count)}  |  t=+{el:.1f}s" if el is not None
                    else f"frame {i}  |  CROSS "
                    f"{'SEEN' if res.cross is not None else '-'}  |  crossings "
                    f"{cc}/{int(q.exit_count)}")
                self.end_headers()
                self.wfile.write(data)
                return
            self.send_error(404)

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"\n  Dash web tuner running.  Open in your browser:")
    print(f"      http://localhost:{port}\n  (Ctrl+C to stop)\n")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", default="lane_test_bag", help="rosbag dir or .db3 file")
    ap.add_argument("--topic", default="/camera/image/compressed")
    ap.add_argument("--export", metavar="DIR", default=None,
                    help="headless: write annotated stills here (every --stride frame)")
    ap.add_argument("--video", metavar="PATH", default=None,
                    help="headless: write an annotated mp4 of the whole bag here")
    ap.add_argument("--stride", type=int, default=30,
                    help="still-export / print cadence (default every 30 frames)")
    ap.add_argument("--fps", type=float, default=30.0, help="output video fps")
    ap.add_argument("--web", nargs="?", type=int, const=8842, default=None,
                    metavar="PORT",
                    help="browser tuner on localhost:PORT (default 8842) — use "
                         "this when X/WSLg windows do not show")
    args = ap.parse_args()

    p = DashParams()
    if args.web is not None:
        run_web(args.bag, args.topic, args.web, p)
    elif args.export or args.video:
        run_export(args.bag, args.topic, args.export, args.video,
                   args.stride, args.fps, p)
    else:
        run_gui(args.bag, args.topic, p)


if __name__ == "__main__":
    main()
