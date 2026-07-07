"""Standalone traffic-light tuning tool (PC only, NOT part of driving.launch).

Purpose: verify red/green detection live and tune the values that affect it. It
runs the exact pipeline path — `preprocess_frame` -> `BestPthDetector` ->
`classify_light` (honoring ``traffic_light.classifier``) — so every value dialed
in here maps 1:1 onto ``dracer_params.yaml``.

One window:
  * "Detect" - the color-corrected frame the detector sees, with each YOLO light
               box drawn as a bbox colored by the classifier's verdict and a
               GREEN / RED (or "none") label.

Controls window "TL Controls": the color-correction chain (CLAHE / saturation /
brightness / ... — boosting these makes the lit lamp pop and detect better, and
the Detect window always shows the corrected frame so the effect is live), plus
the filters of the ACTIVE classifier — green/red HSV bounds for color/lit (lit
adds two brightness fields), or the LAB a-channel / L thresholds for lab — with
row_min_ratio and the YOLO conf/imgsz always shown. Keys: 'm' cycles the
classifier (color->lit->lab, rebuilding the sliders), 's' prints a copy-paste
YAML block, 'q' quits.

Run:  ros2 run bisa traffic_light_tuner
      ros2 run bisa traffic_light_tuner --ros-args -p video_path:=/path/to/clip.mp4
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from . import traffic_light
from .dracer_config import load_config, resolve_package_relative_path
from .object_detector import BestPthDetector

CONTROLS = "TL Controls"
WIN_DETECT = "Detect"

# (trackbar label, config path, kind, lo, hi)
#   kind 'x10'/'x100' store value*10 or *100; 'boff' stores value+50 (bipolar);
#   'imgsz32' stores imgsz/32; 'raw' stores the integer directly.
# The slider set depends on the classifier (cycle it with the 'm' key):
#   * color correction + row_min_ratio + conf + imgsz are always shown;
#   * color/lit modes add the green/red HSV bounds (lit adds two more);
#   * lab mode adds the LAB a-channel / L thresholds INSTEAD of HSV.
# Color correction runs before the detector so boosting saturation / CLAHE makes
# the lit lamp pop and detect better — the Detect window ALWAYS shows the
# corrected frame, so the effect is visible live.
CC_SLIDERS = [
    ("clahe_clip x10", "color_correction.clahe_clip", "x10", 0, 80),
    ("clahe_tile", "color_correction.clahe_tile", "raw", 1, 16),
    ("sat_boost x10", "color_correction.saturation_boost", "x10", 0, 30),
    ("brightness +50", "color_correction.brightness", "boff", 0, 100),
    ("contrast x10", "color_correction.contrast", "x10", 0, 30),
    ("saturation x10", "color_correction.saturation", "x10", 0, 30),
    ("gamma x10", "color_correction.gamma", "x10", 1, 30),
]
HSV_SLIDERS = [
    ("green H lo", "traffic_light.green_h_lo", "raw", 0, 180),
    ("green H hi", "traffic_light.green_h_hi", "raw", 0, 180),
    ("green S min", "traffic_light.green_s_min", "raw", 0, 255),
    ("green V min", "traffic_light.green_v_min", "raw", 0, 255),
    ("red H1 hi", "traffic_light.red_h1_hi", "raw", 0, 180),
    ("red H2 lo", "traffic_light.red_h2_lo", "raw", 0, 180),
    ("red S min", "traffic_light.red_s_min", "raw", 0, 255),
    ("red V min", "traffic_light.red_v_min", "raw", 0, 255),
]
LIT_SLIDERS = [
    ("row lit V min", "traffic_light.row_lit_v_min", "raw", 0, 255),
    ("row white S max", "traffic_light.row_white_s_max", "raw", 0, 255),
]
LAB_SLIDERS = [
    ("lab a red min", "traffic_light.lab_a_red_min", "raw", 128, 255),
    ("lab a green max", "traffic_light.lab_a_green_max", "raw", 0, 128),
    ("lab L min", "traffic_light.lab_l_min", "raw", 0, 255),
]
TAIL_SLIDERS = [
    ("row min ratio x100", "traffic_light.row_min_ratio", "x100", 0, 50),
    ("conf x100", "detector.conf", "x100", 1, 100),
    ("imgsz /32", "detector.imgsz", "imgsz32", 5, 30),
]
CLASSIFIER_CYCLE = ("color", "lit", "lab")


def sliders_for(mode: str) -> list:
    """Returns the trackbar set for a classifier: LAB gets LAB filters, HSV modes
    get the green/red bounds (lit adds two), plus the always-on CC/tail sliders."""

    sliders = list(CC_SLIDERS)
    if str(mode).lower() == "lab":
        sliders += LAB_SLIDERS
    else:
        sliders += HSV_SLIDERS
        if str(mode).lower() == "lit":
            sliders += LIT_SLIDERS
    return sliders + TAIL_SLIDERS


def default_config_path() -> str:
    """Finds the installed or source dracer_params.yaml for initial values."""

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("bisa")) / "config" / "dracer_params.yaml"
        if installed.exists():
            return str(installed)
    except Exception:
        pass
    for base in Path(__file__).resolve().parents:
        for candidate in (base / "config" / "dracer_params.yaml",
                          base / "src" / "bisa" / "config" / "dracer_params.yaml"):
            if candidate.exists():
                return str(candidate)
    return ""


def default_model_path() -> str:
    """Finds the installed or source best.pt checkpoint."""

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("bisa")) / "checkpoints" / "best.pt"
        if installed.exists():
            return str(installed)
    except Exception:
        pass
    for base in Path(__file__).resolve().parents:
        for candidate in (base / "checkpoints" / "best.pt",
                          base / "src" / "bisa" / "checkpoints" / "best.pt"):
            if candidate.exists():
                return str(candidate)
    return "checkpoints/best.pt"


class TlTunerNode(Node):
    """Reads camera frames, runs the light detector, and shows tuning views."""

    def __init__(self):
        """Loads config/model, builds the sliders, and opens the input source."""

        super().__init__("bisa_traffic_light_tuner")
        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("config_file", default_config_path())
        self.declare_parameter("model_path", default_model_path())
        self.declare_parameter("video_path", "")

        cfg_file = str(self.get_parameter("config_file").value)
        self.config = load_config(cfg_file)
        # Force per-frame inference with no rate limit / ROI gating for tuning.
        self.config.detector.inference_hz = 1000.0
        model_path = str(self.get_parameter("model_path").value)
        model_path = resolve_package_relative_path(__file__, model_path)
        self.detector = BestPthDetector(self.config, model_path, logger=self.get_logger())
        self.detector.load_model()

        # Active slider set follows the classifier; 'm' cycles it and rebuilds.
        self.sliders = sliders_for(self.config.traffic_light.classifier)
        self._build_windows()

        # Latest frame is stored by the input source and drawn by a steady GUI
        # timer. Decoupling the two means the windows keep repainting (waitKey
        # pumps every tick) and show a "waiting" placeholder even before any
        # camera frame arrives, instead of freezing blank until the first frame.
        self.latest_frame = None
        self.frame_count = 0

        video_path = str(self.get_parameter("video_path").value).strip()
        self.capture = None
        if video_path:
            self.capture = cv2.VideoCapture(int(video_path) if video_path.isdigit() else video_path)
            if not self.capture.isOpened():
                self.get_logger().error(f"Could not open video source: {video_path}")
            self.get_logger().info(f"TL tuner reading from video: {video_path}")
        else:
            topic = str(self.get_parameter("image_topic").value)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CompressedImage, topic, self._on_image, qos)
            self.get_logger().info(f"TL tuner subscribing to '{topic}' (focus a window, 's'=dump 'q'=quit)")

        # GUI runs on its own ~20 Hz timer, independent of frame arrival.
        self.create_timer(1.0 / 20.0, self._render)

    # ----- setup -------------------------------------------------------------

    def _build_windows(self) -> None:
        """Creates the image + control windows and the initial slider set."""

        cv2.namedWindow(WIN_DETECT, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_DETECT, 640, 480)
        # Spread the windows so WSLg does not stack them on top of each other
        # (a common "the window opened but I can't see it" cause).
        cv2.moveWindow(WIN_DETECT, 20, 20)
        self._build_controls()

    def _build_controls(self) -> None:
        """(Re)creates the controls window with the trackbars for ``self.sliders``."""

        cv2.namedWindow(CONTROLS, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CONTROLS, 460, 720)
        cv2.moveWindow(CONTROLS, 680, 20)
        for label, path, kind, lo, hi in self.sliders:
            init = self._encode(path, kind)
            init = max(lo, min(hi, init))
            cv2.createTrackbar(label, CONTROLS, init, hi, lambda _v: None)
            if lo > 0:
                cv2.setTrackbarMin(label, CONTROLS, lo)

    def _cycle_classifier(self) -> None:
        """Advances the classifier (color->lit->lab) and rebuilds the sliders."""

        cur = str(self.config.traffic_light.classifier).lower()
        idx = CLASSIFIER_CYCLE.index(cur) if cur in CLASSIFIER_CYCLE else 0
        self.config.traffic_light.classifier = CLASSIFIER_CYCLE[(idx + 1) % len(CLASSIFIER_CYCLE)]
        self.sliders = sliders_for(self.config.traffic_light.classifier)
        cv2.destroyWindow(CONTROLS)
        self._build_controls()
        self.get_logger().info(f"classifier -> {self.config.traffic_light.classifier}")

    def _cfg_get(self, path: str):
        """Reads a dotted config path; ``detector.conf`` returns the light conf."""

        if path == "detector.conf":
            return self.config.detector.conf.get("traffic_red", 0.4)
        obj = self.config
        for part in path.split("."):
            obj = getattr(obj, part)
        return obj

    def _encode(self, path: str, kind: str) -> int:
        """Converts a config value into its integer trackbar position."""

        val = self._cfg_get(path)
        if kind == "x10":
            return int(round(float(val) * 10))
        if kind == "x100":
            return int(round(float(val) * 100))
        if kind == "boff":
            return int(round(float(val))) + 50
        if kind == "imgsz32":
            return int(round(int(val) / 32))
        return int(bool(val)) if isinstance(val, bool) else int(val)

    # ----- per-frame ---------------------------------------------------------

    def _read_sliders(self) -> None:
        """Pulls every trackbar position back into the shared config object."""

        for label, path, kind, _lo, _hi in self.sliders:
            pos = cv2.getTrackbarPos(label, CONTROLS)
            if kind == "x10":
                value = pos / 10.0
            elif kind == "x100":
                value = pos / 100.0
            elif kind == "boff":
                value = pos - 50
            elif kind == "imgsz32":
                value = max(1, pos) * 32
            else:
                value = pos
            self._assign(path, value)

    def _assign(self, path: str, value) -> None:
        """Writes a value back to the config, matching the field's type."""

        if path == "detector.conf":
            # One slider tunes both light thresholds; signs keep their config value.
            self.config.detector.conf["traffic_green"] = float(value)
            self.config.detector.conf["traffic_red"] = float(value)
            return
        parts = path.split(".")
        obj = self.config
        for part in parts[:-1]:
            obj = getattr(obj, part)
        current = getattr(obj, parts[-1])
        setattr(obj, parts[-1], type(current)(value))

    def _detect_lights(self, frame_bgr: np.ndarray) -> list:
        """Runs the model and returns [(cls, conf, (x1,y1,x2,y2)), ...] for lights."""

        if self.detector.model is None:
            return []
        conf = min(self.config.detector.conf.get("traffic_green", 0.4),
                   self.config.detector.conf.get("traffic_red", 0.4))
        results = self.detector.model.predict(
            source=frame_bgr,
            imgsz=int(self.config.detector.imgsz),
            device=self.detector.device,
            conf=float(conf),
            verbose=False,
        )
        out = []
        if not results:
            return out
        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return out
        for box in boxes:
            cls_name = self.detector._class_name_from_index(int(box.cls[0].item()))
            if cls_name not in ("traffic_green", "traffic_red"):
                continue
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            out.append((cls_name, float(box.conf[0].item()), (x1, y1, x2, y2)))
        return out

    def _process(self, frame: np.ndarray) -> None:
        """Applies filters, detects, verifies, and refreshes the image windows.

        Key handling and waitKey live in :meth:`_render` so the GUI keeps
        pumping even when no frame is available.
        """

        self._read_sliders()
        # Always show the corrected frame so the color-correction sliders have a
        # visible effect; the pipeline applies the same chain (enabled by default).
        detect_input = traffic_light.apply_correction_chain(frame, self.config.color_correction)

        detect_view = detect_input.copy()
        n_on = 0
        for _cls, _conf, bbox in self._detect_lights(detect_input):
            x1, y1, x2, y2 = (int(v) for v in bbox)
            # The pipeline's classifier (config.classifier) decides the color;
            # only the box + GREEN/RED verdict is drawn.
            verdict, _scores = traffic_light.classify_light(detect_input, bbox, self.config)
            n_on += int(verdict is not None)
            color = self._verdict_color(verdict)
            cv2.rectangle(detect_view, (x1, y1), (x2, y2), color, 2 if verdict else 1)
            cv2.putText(detect_view, self._verdict_text(verdict),
                        (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

        self._hud(detect_view, [
            f"classifier={self.config.traffic_light.classifier}  lights={n_on}",
            "m=cycle classifier  s=dump params  q=quit",
        ])

        cv2.imshow(WIN_DETECT, detect_view)

    @staticmethod
    def _verdict_text(cls) -> str:
        """Maps a mission class to a short verdict label for the overlay."""

        return {"traffic_green": "GREEN", "traffic_red": "RED"}.get(cls, "none")

    @staticmethod
    def _verdict_color(cls) -> tuple:
        """BGR color for a verdict (green/red lamp, grey when undecided)."""

        return {"traffic_green": (0, 255, 0), "traffic_red": (0, 0, 255)}.get(cls, (160, 160, 160))

    @staticmethod
    def _hud(frame, lines) -> None:
        """Draws a small stacked HUD in the top-left corner."""

        y = 18
        for text in lines:
            cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y += 20

    def _dump_yaml(self) -> None:
        """Prints a copy-paste-ready YAML block of the tuned values."""

        cc, tl, det = self.config.color_correction, self.config.traffic_light, self.config.detector
        print("\n# ---- paste into dracer_params.yaml ----")
        print("color_correction:")
        print(f"  enabled: {str(cc.enabled).lower()}")
        for f in ("clahe_clip", "clahe_tile", "saturation_boost", "brightness",
                  "contrast", "saturation", "gamma"):
            print(f"  {f}: {getattr(cc, f)}")
        print("traffic_light:")
        print(f"  classifier: {tl.classifier}")
        fields = ["green_h_lo", "green_h_hi", "green_s_min", "green_v_min",
                  "red_h1_hi", "red_h2_lo", "red_s_min", "red_v_min", "row_min_ratio"]
        mode = str(tl.classifier).lower()
        if mode == "lit":
            fields += ["row_lit_v_min", "row_white_s_max"]
        elif mode == "lab":
            fields += ["lab_a_red_min", "lab_a_green_max", "lab_l_min"]
        for f in fields:
            print(f"  {f}: {getattr(tl, f)}")
        print("detector:")
        print(f"  imgsz: {det.imgsz}")
        print("  conf:")
        print(f"    traffic_green: {det.conf['traffic_green']}")
        print(f"    traffic_red: {det.conf['traffic_red']}")
        print("# ---------------------------------------\n")

    # ----- input callbacks ---------------------------------------------------

    def _on_image(self, msg: CompressedImage) -> None:
        """Stores the newest ROS frame; drawing happens in the GUI timer."""

        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.latest_frame = frame
            self.frame_count += 1

    def _render(self) -> None:
        """Steady GUI tick: draws the latest frame (or a placeholder) and pumps keys.

        Runs regardless of frame arrival so the windows never freeze blank. In
        video mode it also pulls the next frame from the capture here.
        """

        if self.capture is not None:
            ok, frame = self.capture.read()
            if ok:
                self.latest_frame = frame
                self.frame_count += 1
            else:
                self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)

        if self.latest_frame is None:
            placeholder = np.full((480, 640, 3), 40, dtype=np.uint8)
            self._hud(placeholder, [
                "Waiting for camera frames...",
                "check: ros2 topic hz /camera/image/compressed",
                "same ROS_DOMAIN_ID as the car?  q=quit",
            ])
            cv2.imshow(WIN_DETECT, placeholder)
        else:
            self._process(self.latest_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rclpy.shutdown()
        elif key == ord("s"):
            self._dump_yaml()
        elif key == ord("m"):
            self._cycle_classifier()


def main(args=None) -> None:
    """Initializes rclpy and spins the traffic-light tuner node."""

    rclpy.init(args=args)
    node = TlTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.capture is not None:
            node.capture.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
