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

# Curated tuning subset. kind: 'f' float, 'i' int, 'hsv_v' V of black_hsv_upper.
# (label, dotted_name, kind, lo, hi)
SPEC = {
    "Lane": [
        ("black V max", "lane.black_hsv_upper", "hsv_v", 0, 255),
        ("canny low", "lane.hough_canny_low", "i", 0, 255),
        ("canny high", "lane.hough_canny_high", "i", 0, 255),
        ("hough thresh", "lane.hough_threshold", "i", 1, 200),
        ("hough min len", "lane.hough_min_line_length", "i", 1, 300),
        ("hough max gap", "lane.hough_max_line_gap", "i", 1, 400),
        ("slope min abs", "lane.hough_slope_min_abs", "f", 0.0, 2.0),
        ("min comp area", "lane.min_component_area_ratio", "f", 0.0, 0.1),
        ("fork area", "lane.fork_area_ratio", "f", 0.0, 0.2),
        ("rotary area", "lane.rotary_area_ratio", "f", 0.0, 0.3),
    ],
    "Steering": [
        ("kp", "steering.kp", "f", 0.0, 4.0),
        ("kd", "steering.kd", "f", 0.0, 2.0),
        ("kcurv", "steering.kcurv", "f", 0.0, 2.0),
        ("straight limit", "steering.straight_limit", "f", 0.0, 1.0),
        ("s-curve limit", "steering.s_curve_limit", "f", 0.0, 1.0),
        ("rate limit", "steering.rate_limit_per_cmd", "f", 0.0, 0.5),
        ("steer sign", "steering.steer_sign", "i", -1, 1),
    ],
    "Throttle": [
        ("max", "throttle.max", "f", 0.0, 1.0),
        ("launch cap", "throttle.launch_cap", "f", 0.0, 1.0),
        ("s-curve cap", "throttle.s_curve_cap", "f", 0.0, 1.0),
    ],
    "Detector conf": [
        ("green", "detector.conf.traffic_green", "f", 0.0, 1.0),
        ("red", "detector.conf.traffic_red", "f", 0.0, 1.0),
        ("sign left", "detector.conf.sign_left", "f", 0.0, 1.0),
        ("sign right", "detector.conf.sign_right", "f", 0.0, 1.0),
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


def build_param(name: str, kind: str, value, hsv_base) -> Parameter:
    """Builds an rclpy Parameter for one control's current value."""

    if kind == "i":
        return Parameter(name, Parameter.Type.INTEGER, int(round(value)))
    if kind == "hsv_v":
        h, s = int(hsv_base[0]), int(hsv_base[1])
        return Parameter(name, Parameter.Type.INTEGER_ARRAY, [h, s, int(round(value))])
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
    hsv_base = get_nested(data, "lane.black_hsv_upper", [180, 90, 90])

    root = tk.Tk()
    root.title(f"D-Racer params -> {node.target_node}")

    def make_sender(name, kind):
        def _send(_event=None, n=name, k=kind):
            value = scales[n].get()
            node.send([build_param(n, k, value, hsv_base)])
        return _send

    scales = {}
    col = 0
    for section, items in SPEC.items():
        frame = ttk.LabelFrame(root, text=section)
        frame.grid(row=0, column=col, padx=6, pady=6, sticky="n")
        col += 1
        for row, (label, name, kind, lo, hi) in enumerate(items):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", padx=4)
            if kind == "hsv_v":
                init = float(hsv_base[2]) if len(hsv_base) > 2 else 90.0
                resolution = 1.0
            elif kind == "i":
                init = float(get_nested(data, name, lo))
                resolution = 1.0
            else:
                init = float(get_nested(data, name, lo))
                resolution = max((hi - lo) / 200.0, 0.001)
            scale = tk.Scale(frame, from_=lo, to=hi, orient="horizontal",
                             resolution=resolution, length=220)
            scale.set(init)
            scale.grid(row=row, column=1, padx=4, pady=1)
            scale.bind("<ButtonRelease-1>", make_sender(name, kind))
            scales[name] = scale

    status = ttk.Label(root, text=f"target: {node.target_node}  (drag a slider to apply)")
    status.grid(row=1, column=0, columnspan=max(col, 1), pady=4)

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
