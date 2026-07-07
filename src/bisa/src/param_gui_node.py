"""PC-side tkinter GUI to live-tune the bisa node's core parameters.

Sliders push values to the running bisa node via the SetParameters service, so
edits take effect immediately (the perception/control code reads the shared
config every frame). Run on the operator PC only.
"""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter

try:
    import yaml
except ModuleNotFoundError:
    yaml = None

# Curated tuning subset, grouped per algorithm (one Notebook tab per key).
# kind: 'f' float, 'i' int, 'b' bool (0/1).
# (label, dotted_name, kind, lo, hi)
SPEC = {
    "Lane": [
        ("canny low", "lane.hough_canny_low", "i", 0, 255),
        ("canny high", "lane.hough_canny_high", "i", 0, 255),
        ("hough thresh", "lane.hough_threshold", "i", 1, 200),
        ("hough min len", "lane.hough_min_line_length", "i", 1, 300),
        ("hough max gap", "lane.hough_max_line_gap", "i", 1, 400),
        ("slope min abs", "lane.hough_slope_min_abs", "f", 0.0, 2.0),
        ("min comp area", "lane.min_component_area_ratio", "f", 0.0, 0.1),
        ("fork area", "lane.fork_area_ratio", "f", 0.0, 0.2),
    ],
    "Steering (PP)": [
        ("lookahead (m)", "steering.lookahead_m", "f", 0.1, 2.0),
        ("wheelbase (m)", "steering.wheelbase_m", "f", 0.05, 0.5),
        ("lateral scale (m)", "steering.lateral_scale_m", "f", 0.05, 1.0),
        ("max steer (deg)", "steering.max_steer_deg", "f", 5.0, 60.0),
        ("pp gain", "steering.pp_gain", "f", 0.0, 3.0),
        ("curve blend", "steering.curve_blend", "f", 0.0, 3.0),
        ("straight limit", "steering.straight_limit", "f", 0.0, 1.0),
        ("s-curve limit", "steering.s_curve_limit", "f", 0.0, 1.0),
        ("rate limit", "steering.rate_limit_per_cmd", "f", 0.0, 0.5),
        ("steer sign", "steering.steer_sign", "i", -1, 1),
    ],
    "LAB Mask": [
        ("L min", "lane.lab_l_min", "i", 0, 255),
        ("L max", "lane.lab_l_max", "i", 0, 255),
        ("A min", "lane.lab_a_min", "i", 0, 255),
        ("A max", "lane.lab_a_max", "i", 0, 255),
        ("B min", "lane.lab_b_min", "i", 0, 255),
        ("B max", "lane.lab_b_max", "i", 0, 255),
        ("clahe clip", "lane.lab_clahe_clip", "f", 0.0, 8.0),
        ("clahe tile", "lane.lab_clahe_tile", "i", 1, 16),
        ("morph open", "lane.morph_open_kernel", "i", 1, 15),
        ("morph close", "lane.morph_close_kernel", "i", 1, 15),
    ],
    "Throttle": [
        ("min speed", "throttle.speed_min", "f", 0.0, 1.0),
        ("max speed", "throttle.speed_max", "f", 0.0, 1.0),
        ("launch cap", "throttle.launch_cap", "f", 0.0, 1.0),
        ("s-curve cap", "throttle.s_curve_cap", "f", 0.0, 1.0),
    ],
    "Color Correction": [
        ("enabled", "color_correction.enabled", "b", 0, 1),
        ("clahe clip", "color_correction.clahe_clip", "f", 0.0, 8.0),
        ("sat boost", "color_correction.saturation_boost", "f", 0.0, 3.0),
        ("brightness", "color_correction.brightness", "i", -50, 50),
        ("contrast", "color_correction.contrast", "f", 0.0, 3.0),
        ("saturation", "color_correction.saturation", "f", 0.0, 3.0),
        ("gamma", "color_correction.gamma", "f", 0.1, 3.0),
    ],
    "Traffic Light": [
        ("green H lo", "traffic_light.green_h_lo", "i", 0, 180),
        ("green H hi", "traffic_light.green_h_hi", "i", 0, 180),
        ("green S min", "traffic_light.green_s_min", "i", 0, 255),
        ("green V min", "traffic_light.green_v_min", "i", 0, 255),
        ("red H1 hi", "traffic_light.red_h1_hi", "i", 0, 180),
        ("red H2 lo", "traffic_light.red_h2_lo", "i", 0, 180),
        ("red S min", "traffic_light.red_s_min", "i", 0, 255),
        ("red V min", "traffic_light.red_v_min", "i", 0, 255),
        ("row min ratio", "traffic_light.row_min_ratio", "f", 0.0, 0.5),
        ("row lit V min (lit only)", "traffic_light.row_lit_v_min", "i", 0, 255),
        ("row white S max (lit only)", "traffic_light.row_white_s_max", "i", 0, 255),
        ("lab a red min (lab only)", "traffic_light.lab_a_red_min", "i", 128, 255),
        ("lab a green max (lab only)", "traffic_light.lab_a_green_max", "i", 0, 128),
        ("lab L min (lab only)", "traffic_light.lab_l_min", "i", 0, 255),
    ],
    "Detector conf": [
        ("imgsz (early detect)", "detector.imgsz", "i", 160, 960),
        ("green", "detector.conf.traffic_green", "f", 0.0, 1.0),
        ("red", "detector.conf.traffic_red", "f", 0.0, 1.0),
        ("sign left", "detector.conf.sign_left", "f", 0.0, 1.0),
        ("sign right", "detector.conf.sign_right", "f", 0.0, 1.0),
    ],
    "Lane ROI": [
        ("width (horiz)", "lane_roi.width", "i", 0, 640),
        ("height (vert)", "lane_roi.height", "i", 0, 480),
        ("x offset", "lane_roi.x_offset", "i", -1, 640),
        ("y offset", "lane_roi.y_offset", "i", -1, 480),
    ],
}


def get_nested(data, dotted, default=None):
    """Reads a dotted key path out of a nested dict, returning default if absent."""

    cur = data
    for part in dotted.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return default
    return cur


def default_config_path() -> str:
    """Finds the installed or source dracer_params.yaml for initial slider values."""

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


class ParamGuiNode(Node):
    """ROS node that owns the SetParameters client used by the tkinter GUI."""

    def __init__(self):
        """Creates the SetParameters client targeting the bisa node."""

        super().__init__("bisa_param_gui_node")
        self.declare_parameter("target_node", "bisa_autonomous_node")
        self.declare_parameter("config_file", default_config_path())
        self.target_node = str(self.get_parameter("target_node").value)
        self.config_file = str(self.get_parameter("config_file").value)
        service = f"/{self.target_node}/set_parameters"
        self.client = self.create_client(SetParameters, service)

    def load_yaml(self) -> dict:
        """Loads the params yaml for initial slider positions (best effort)."""

        if yaml is None or not self.config_file or not os.path.exists(self.config_file):
            return {}
        with open(self.config_file, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}

    def send(self, parameters) -> None:
        """Fires an async SetParameters request; results are pumped by spin_once."""

        if not self.client.service_is_ready():
            self.client.wait_for_service(timeout_sec=0.0)
            if not self.client.service_is_ready():
                self.get_logger().warning("bisa set_parameters service not ready yet")
                return
        request = SetParameters.Request()
        request.parameters = [p.to_parameter_msg() for p in parameters]
        self.client.call_async(request)


def build_param(name: str, kind: str, value) -> Parameter:
    """Builds an rclpy Parameter for one control's current value."""

    if kind == "b":
        return Parameter(name, Parameter.Type.BOOL, bool(round(value)))
    if kind == "i":
        return Parameter(name, Parameter.Type.INTEGER, int(round(value)))
    return Parameter(name, Parameter.Type.DOUBLE, float(value))


def main(args=None) -> None:
    """Launches the tkinter tuning GUI alongside an rclpy spin loop."""

    try:
        import tkinter as tk
        from tkinter import ttk
    except Exception as exc:  # pragma: no cover - GUI only on the operator PC.
        print(f"[param_gui] tkinter unavailable ({exc}); run this on a PC with a display.")
        return

    rclpy.init(args=args)
    node = ParamGuiNode()
    data = node.load_yaml()

    root = tk.Tk()
    root.title(f"D-Racer params -> {node.target_node}")

    def make_sender(name, kind):
        def _send(_event=None, n=name, k=kind):
            value = scales[n].get()
            node.send([build_param(n, k, value)])
        return _send

    # One Notebook tab per algorithm group so the active values are easy to scan.
    notebook = ttk.Notebook(root)
    notebook.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
    root.rowconfigure(0, weight=1)
    root.columnconfigure(0, weight=1)

    scales = {}
    for section, items in SPEC.items():
        tab = ttk.Frame(notebook, padding=8)
        notebook.add(tab, text=section)
        for row, (label, name, kind, lo, hi) in enumerate(items):
            ttk.Label(tab, text=label).grid(row=row, column=0, sticky="w", padx=4)
            if kind == "b":
                init = 1.0 if bool(get_nested(data, name, False)) else 0.0
                resolution = 1.0
            elif kind == "i":
                init = float(get_nested(data, name, lo))
                resolution = 1.0
            else:
                init = float(get_nested(data, name, lo))
                resolution = max((hi - lo) / 200.0, 0.001)
            scale = tk.Scale(tab, from_=lo, to=hi, orient="horizontal",
                             resolution=resolution, length=240)
            scale.set(init)
            scale.grid(row=row, column=1, padx=4, pady=1)
            scale.bind("<ButtonRelease-1>", make_sender(name, kind))
            scales[name] = scale

    status = ttk.Label(root, text=f"target: {node.target_node}  (drag a slider to apply)")
    status.grid(row=1, column=0, pady=4)

    def pump():
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(50, pump)

    def on_close():
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(50, pump)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
