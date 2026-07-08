"""PC-side tkinter power monitor for the D-Racer.

Subscribes to the INA219-derived power topics published by ``battery_node`` and
shows a live instrument panel:

* BATTERY   - measured voltage / current / power of the 2S 18650 pack.
* BOARD 5V  - the D3-G rail. Measured if a board-rail sensor publishes
              ``/power/board_voltage`` / ``/power/board_current_ma`` (Tier 2);
              otherwise the current is ESTIMATED from the idle baseline draw.
* MOTOR     - measured if ``/power/motor_power_w`` publishes (Tier 2); otherwise
              ESTIMATED as (battery power - idle baseline power).
* A voltage/throttle time plot with a brownout threshold line, so a motor-induced
  sag that browns out the board's 5V rail / USB camera is visible at a glance.

Only one INA219 exists by default (on the battery), so BOARD 5V and MOTOR are
estimates until per-rail sensors are wired; the panels switch to measured values
automatically when the optional topics appear. Run on the operator PC (needs a
display); it is a pure subscriber and never commands the vehicle.
"""

from __future__ import annotations

import signal
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:  # Control carries the throttle command used for the sag correlation plot.
    from control_msgs.msg import Control
except Exception:  # pragma: no cover - control_msgs always present on the PC.
    Control = None

try:  # Battery carries the voltage-derived charge percentage from battery_node.
    from battery_msgs.msg import Battery
except Exception:  # pragma: no cover - battery_msgs always present on the PC.
    Battery = None


# --- pure helpers (no ROS/tk, unit-testable) --------------------------------

def estimate_motor_w(battery_w, baseline_w):
    """Estimates motor power as battery draw above the idle baseline."""

    if battery_w is None or baseline_w is None:
        return None
    return max(0.0, battery_w - baseline_w)


def estimate_current_a(power_w, voltage_v):
    """Returns I=P/V in amperes, or None when inputs are missing/zero."""

    if power_w is None or voltage_v is None or voltage_v <= 0.0:
        return None
    return power_w / voltage_v


def build_plot_geometry(history, brownout_v, width, height, pad=6):
    """Maps (t, voltage, throttle) samples to canvas coordinates.

    Returns a dict with the voltage polyline points, throttle bar rects, the
    brownout threshold y (or None), and the voltage axis range. Returns None
    when there are too few samples to draw a line.
    """

    volts = [sample[1] for sample in history if sample[1] is not None]
    if len(volts) < 2:
        return None

    v_min = min(volts)
    v_max = max(volts)
    if brownout_v is not None:
        v_min = min(v_min, brownout_v)
        v_max = max(v_max, brownout_v)
    if v_max - v_min < 0.1:
        v_max += 0.05
        v_min -= 0.05

    count = len(history)

    def x_at(index):
        return pad + (index / (count - 1)) * (width - 2 * pad)

    def y_at(value):
        return pad + (1.0 - (value - v_min) / (v_max - v_min)) * (height - 2 * pad)

    line = [(x_at(i), y_at(s[1])) for i, s in enumerate(history) if s[1] is not None]

    bars = []
    bar_w = (width - 2 * pad) / count
    for i, sample in enumerate(history):
        magnitude = min(1.0, abs(sample[2] or 0.0))
        if magnitude < 0.02:
            continue
        bar_h = magnitude * (height - 2 * pad)
        bars.append((x_at(i), height - pad - bar_h, bar_w, bar_h))

    threshold_y = None
    if brownout_v is not None and v_min <= brownout_v <= v_max:
        threshold_y = y_at(brownout_v)

    return {"line": line, "bars": bars, "threshold_y": threshold_y, "v_min": v_min, "v_max": v_max}


# --- ROS node ---------------------------------------------------------------

class PowerMonitorNode(Node):
    """Collects power telemetry for the tkinter UI to render."""

    STALE_SEC = 3.0

    def __init__(self):
        """Declares topics/thresholds and subscribes to the power streams."""

        super().__init__("bisa_power_gui_node")
        self.declare_parameter("battery_voltage_topic", "/battery/voltage")
        self.declare_parameter("battery_current_topic", "/battery/current_ma")
        self.declare_parameter("battery_power_topic", "/battery/power_w")
        self.declare_parameter("control_topic", "/control")
        # Optional Tier-2 per-rail sensors; panels show measured values if present.
        self.declare_parameter("board_voltage_topic", "/power/board_voltage")
        self.declare_parameter("board_current_topic", "/power/board_current_ma")
        self.declare_parameter("motor_power_topic", "/power/motor_power_w")
        self.declare_parameter("brownout_voltage", 6.0)
        self.declare_parameter("board_nominal_voltage", 5.0)
        self.declare_parameter("board_current_limit_a", 5.0)
        self.declare_parameter("history_seconds", 30.0)

        self.brownout_voltage = float(self.get_parameter("brownout_voltage").value)
        self.board_nominal_voltage = float(self.get_parameter("board_nominal_voltage").value)
        self.board_current_limit_a = float(self.get_parameter("board_current_limit_a").value)
        self.history_seconds = float(self.get_parameter("history_seconds").value)

        self.battery_voltage = None
        self.battery_current_ma = None
        self.battery_power_w = None
        self.battery_t = None
        self.throttle = 0.0
        self.baseline_w = None
        # Empirical fact (i2c probe + drive test 2026-07-03): on this Waveshare
        # power board the INA219 shunt carries no load current (reads ~0 mA even
        # with the motor running) — only the bus VOLTAGE is real. Track the max
        # |I| seen; while it stays under the noise floor, show A/W as N/A instead
        # of a misleading "0". If real per-rail sensors are added later, current
        # rises above the floor and the panels light up automatically.
        self.max_abs_current_ma = 0.0
        self.current_sample_count = 0

        self.board_voltage = None
        self.board_current_ma = None
        self.board_v_t = None
        self.board_i_t = None
        self.motor_power_meas_w = None
        self.motor_w_t = None
        # Board-priority voltage guard telemetry from control_node (on the car).
        self.guard_scale = None
        self.guard_t = None
        # Voltage-derived charge percentage from battery_node (/battery_status).
        self.battery_percent = None
        self.battery_percent_t = None
        # On-car compute vitals from system_telemetry_node (CPU/temp/mem). A
        # stall in the onboard stack is almost always a saturated core or
        # thermal throttle, so surface them next to the power panels.
        self.cpu_percent = None
        self.cpu_percent_max = None
        self.cpu_temp_c = None
        self.mem_percent = None
        self.system_t = None

        self.history: deque = deque()

        self.create_subscription(
            Float32, str(self.get_parameter("battery_voltage_topic").value), self._on_battery_voltage, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("battery_current_topic").value), self._on_battery_current, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("battery_power_topic").value), self._on_battery_power, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("board_voltage_topic").value), self._on_board_voltage, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("board_current_topic").value), self._on_board_current, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("motor_power_topic").value), self._on_motor_power, 10
        )
        self.create_subscription(Float32, "/power/guard_scale", self._on_guard_scale, 10)
        self.create_subscription(Float32, "/system/cpu_percent", self._on_cpu_percent, 10)
        self.create_subscription(Float32, "/system/cpu_percent_max", self._on_cpu_percent_max, 10)
        self.create_subscription(Float32, "/system/cpu_temp_c", self._on_cpu_temp, 10)
        self.create_subscription(Float32, "/system/mem_percent", self._on_mem_percent, 10)
        if Battery is not None:
            self.create_subscription(Battery, "/battery_status", self._on_battery_status, 10)
        if Control is not None:
            self.create_subscription(
                Control, str(self.get_parameter("control_topic").value), self._on_control, 10
            )

    def _on_battery_voltage(self, msg):
        self.battery_voltage = float(msg.data)
        self.battery_t = time.monotonic()

    def _on_battery_current(self, msg):
        self.battery_current_ma = float(msg.data)
        self.current_sample_count += 1
        self.max_abs_current_ma = max(self.max_abs_current_ma, abs(self.battery_current_ma))

    def _on_battery_power(self, msg):
        # The watt message is the sampling trigger (battery_node sends V/I/W each
        # tick): refresh the idle baseline and push one history sample.
        self.battery_power_w = float(msg.data)
        now = time.monotonic()
        if abs(self.throttle) < 0.05:
            self.baseline_w = (
                self.battery_power_w if self.baseline_w is None
                else min(self.baseline_w, self.battery_power_w)
            )
        if self.battery_voltage is not None:
            # Plot the EFFECTIVE motor command: control_node applies the guard
            # scale after /control (forward only), so mirror that here or the
            # sag-correlation bars overstate the motor while the guard limits.
            scale = (
                self.guard_scale
                if (self._fresh(self.guard_t) and self.guard_scale is not None)
                else 1.0
            )
            effective_thr = self.throttle * scale if self.throttle > 0.0 else self.throttle
            self.history.append((now, self.battery_voltage, effective_thr))
            cutoff = now - self.history_seconds
            while self.history and self.history[0][0] < cutoff:
                self.history.popleft()

    def _on_control(self, msg):
        self.throttle = float(msg.throttle)

    def _on_board_voltage(self, msg):
        self.board_voltage = float(msg.data)
        self.board_v_t = time.monotonic()

    def _on_board_current(self, msg):
        self.board_current_ma = float(msg.data)
        self.board_i_t = time.monotonic()

    def _on_motor_power(self, msg):
        self.motor_power_meas_w = float(msg.data)
        self.motor_w_t = time.monotonic()

    def _on_guard_scale(self, msg):
        self.guard_scale = float(msg.data)
        self.guard_t = time.monotonic()

    def _on_battery_status(self, msg):
        self.battery_percent = float(msg.battery_status)
        self.battery_percent_t = time.monotonic()

    def _on_cpu_percent(self, msg):
        self.cpu_percent = float(msg.data)
        self.system_t = time.monotonic()

    def _on_cpu_percent_max(self, msg):
        self.cpu_percent_max = float(msg.data)
        self.system_t = time.monotonic()

    def _on_cpu_temp(self, msg):
        self.cpu_temp_c = float(msg.data)
        self.system_t = time.monotonic()

    def _on_mem_percent(self, msg):
        self.mem_percent = float(msg.data)
        self.system_t = time.monotonic()

    def system_fresh(self):
        return self._fresh(self.system_t)

    def _fresh(self, stamp):
        return stamp is not None and (time.monotonic() - stamp) < self.STALE_SEC

    def battery_fresh(self):
        return self._fresh(self.battery_t)

    def current_sensing_dead(self):
        """True when the shunt has clearly never carried load current.

        ~0 mA across many samples means the current path bypasses the shunt
        (the D-Racer Waveshare board wires it that way), so A/W would be
        misleading zeros rather than measurements.
        """

        return self.current_sample_count >= 25 and self.max_abs_current_ma < 5.0

    def motor_reading(self):
        """Returns (watts, is_measured). Falls back to the baseline estimate."""

        if self.motor_power_meas_w is not None and self._fresh(self.motor_w_t):
            return self.motor_power_meas_w, True
        return estimate_motor_w(self.battery_power_w, self.baseline_w), False

    def board_reading(self):
        """Returns (voltage, current_a, is_measured) for the 5V rail.

        Measured when a board-rail sensor publishes; otherwise the current is
        estimated from the idle baseline power referred to the 5V rail.
        """

        if self._fresh(self.board_v_t) or self._fresh(self.board_i_t):
            current_a = None if self.board_current_ma is None else self.board_current_ma / 1000.0
            return self.board_voltage, current_a, True
        est_a = estimate_current_a(self.baseline_w, self.board_nominal_voltage)
        return None, est_a, False


# --- tkinter UI -------------------------------------------------------------

COLORS = {
    "bg": "#1e1e1e",
    "panel": "#252526",
    "text": "#d4d4d4",
    "muted": "#9da1a6",
    "good": "#4ec9b0",
    "warn": "#dcdcaa",
    "bad": "#f48771",
    "motor": "#ce9178",
    "rail": "#569cd6",
}


def main(args=None):
    """Launches the tkinter power monitor alongside an rclpy spin loop."""

    try:
        import tkinter as tk
    except Exception as exc:  # pragma: no cover - GUI only on the operator PC.
        print(f"[power_gui] tkinter unavailable ({exc}); run this on a PC with a display.")
        return

    rclpy.init(args=args)
    node = PowerMonitorNode()

    root = tk.Tk()
    root.title("D-Racer Power Monitor")
    root.configure(bg=COLORS["bg"])

    def make_panel(parent, title):
        frame = tk.Frame(parent, bg=COLORS["panel"], bd=0, highlightthickness=1,
                         highlightbackground="#333")
        tk.Label(frame, text=title, bg=COLORS["panel"], fg=COLORS["muted"],
                 font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=10, pady=(8, 2))
        big = tk.Label(frame, text="--", bg=COLORS["panel"], fg=COLORS["text"],
                       font=("TkDefaultFont", 22, "bold"))
        big.pack(anchor="w", padx=10)
        sub = tk.Label(frame, text="", bg=COLORS["panel"], fg=COLORS["muted"],
                       justify="left", font=("TkDefaultFont", 10))
        sub.pack(anchor="w", padx=10, pady=(0, 8))
        return frame, big, sub

    panels = tk.Frame(root, bg=COLORS["bg"])
    panels.pack(fill="x", padx=10, pady=(10, 6))
    batt_frame, batt_big, batt_sub = make_panel(panels, "BATTERY (2S 18650)")
    board_frame, board_big, board_sub = make_panel(panels, "BOARD 5V RAIL")
    motor_frame, motor_big, motor_sub = make_panel(panels, "MOTOR")
    compute_frame, compute_big, compute_sub = make_panel(panels, "COMPUTE (CAR)")
    for i, frame in enumerate((batt_frame, board_frame, motor_frame, compute_frame)):
        frame.grid(row=0, column=i, sticky="nsew", padx=4)
        panels.columnconfigure(i, weight=1)

    status = tk.Label(root, text="WAITING FOR DATA", bg=COLORS["bg"], fg=COLORS["muted"],
                      font=("TkDefaultFont", 13, "bold"))
    status.pack(fill="x", padx=14, pady=2)

    # Board-priority guard state (control_node scales the motor down when the
    # pack sags so the D3-G's 5V rail keeps headroom).
    guard_label = tk.Label(root, text="", bg=COLORS["bg"], fg=COLORS["muted"],
                           font=("TkDefaultFont", 11, "bold"))
    guard_label.pack(fill="x", padx=14)

    canvas_w, canvas_h = 620, 150
    canvas = tk.Canvas(root, width=canvas_w, height=canvas_h, bg="#141414",
                       highlightthickness=1, highlightbackground="#333")
    canvas.pack(fill="x", padx=10, pady=6)

    footer = tk.Label(
        root,
        text=("전압 = 실측. 전류/전력: 이 보드 shunt엔 부하 전류가 안 흘러 N/A. "
              "레일별 INA219를 달아 /power/board_* /power/motor_*로 발행하면 실측 표시로 전환."),
        bg=COLORS["bg"], fg=COLORS["muted"], font=("TkDefaultFont", 9),
    )
    footer.pack(fill="x", padx=14, pady=(0, 8))

    def draw_plot():
        canvas.delete("all")
        geo = build_plot_geometry(list(node.history), node.brownout_voltage, canvas_w, canvas_h)
        if geo is None:
            canvas.create_text(canvas_w // 2, canvas_h // 2, text="Waiting for samples",
                               fill=COLORS["muted"])
            return
        for (bx, by, bw, bh) in geo["bars"]:
            canvas.create_rectangle(bx, by, bx + bw, by + bh, fill="#3a3320", width=0)
        if geo["threshold_y"] is not None:
            canvas.create_line(4, geo["threshold_y"], canvas_w - 4, geo["threshold_y"],
                               fill=COLORS["bad"], dash=(4, 3))
        flat = []
        for (px, py) in geo["line"]:
            flat.extend((px, py))
        if len(flat) >= 4:
            canvas.create_line(*flat, fill=COLORS["good"], width=2)
        canvas.create_text(6, 8, anchor="nw", fill=COLORS["muted"],
                           text=f"{geo['v_max']:.2f}V")
        canvas.create_text(6, canvas_h - 8, anchor="sw", fill=COLORS["muted"],
                           text=f"{geo['v_min']:.2f}V")

    def refresh():
        sensing_dead = node.current_sensing_dead()

        # BATTERY (voltage always measured; A/W only if the shunt carries current)
        pct_fresh = (node.battery_percent_t is not None
                     and (time.monotonic() - node.battery_percent_t) < node.STALE_SEC)
        if pct_fresh and node.battery_percent is not None:
            pct = node.battery_percent
            pct_color = (COLORS["good"] if pct > 50.0
                         else COLORS["warn"] if pct > 20.0 else COLORS["bad"])
            pct_line = f"잔량: {pct:.0f}% (전압 기반)"
        else:
            pct_color = None
            pct_line = "잔량: --%"

        if node.battery_fresh() and node.battery_voltage is not None:
            v = node.battery_voltage
            v_color = COLORS["bad"] if v <= node.brownout_voltage else (pct_color or COLORS["good"])
            batt_big.config(text=f"{v:.2f} V", fg=v_color)
            if sensing_dead:
                batt_sub.config(text=f"{pct_line}\nI/P: N/A — shunt 미배선")
            else:
                ma = "--" if node.battery_current_ma is None else f"{node.battery_current_ma:.0f} mA"
                w = "--" if node.battery_power_w is None else f"{node.battery_power_w:.2f} W"
                batt_sub.config(text=f"{pct_line}\nI: {ma}  P: {w}")
        else:
            batt_big.config(text="--", fg=COLORS["muted"])
            batt_sub.config(text="no data")

        # BOARD 5V (measured or estimated)
        board_v, board_a, board_measured = node.board_reading()
        if board_measured:
            vt = "-- V" if board_v is None else f"{board_v:.2f} V"
            board_big.config(text=vt, fg=COLORS["rail"])
            at = "--" if board_a is None else f"{board_a:.2f} A"
            near = "" if board_a is None else (
                "  ⚠ near 5A" if board_a >= node.board_current_limit_a * 0.9 else "")
            board_sub.config(text=f"I: {at} / {node.board_current_limit_a:.0f} A{near}\n(measured)")
        elif sensing_dead or board_a is None:
            board_big.config(text="N/A", fg=COLORS["muted"])
            board_sub.config(text="실측하려면 보드 5V 라인에\nINA219 추가 필요")
        else:
            board_big.config(text=f"~{board_a:.2f} A", fg=COLORS["warn"])
            board_sub.config(
                text=f"of {node.board_current_limit_a:.0f} A @ {node.board_nominal_voltage:.0f} V\n"
                     f"(추정 · 실측하려면 보드레일 센서)")

        # MOTOR (measured or estimated)
        motor_w, motor_measured = node.motor_reading()
        if motor_measured:
            motor_big.config(text=f"{motor_w:.1f} W", fg=COLORS["motor"])
            motor_a = estimate_current_a(motor_w, node.battery_voltage)
            at = "" if motor_a is None else f"I≈ {motor_a:.2f} A\n"
            motor_sub.config(text=f"{at}(measured)")
        elif sensing_dead or motor_w is None:
            motor_big.config(text="N/A", fg=COLORS["muted"])
            motor_sub.config(text="실측하려면 모터/ESC 라인에\nINA219 추가 필요")
        else:
            motor_big.config(text=f"{motor_w:.1f} W", fg=COLORS["motor"])
            motor_a = estimate_current_a(motor_w, node.battery_voltage)
            at = "" if motor_a is None else f"I≈ {motor_a:.2f} A\n"
            motor_sub.config(text=f"{at}(추정)")

        # COMPUTE (CAR) — CPU / temp / memory from system_telemetry_node. The
        # onboard stack runs CPU-only YOLO, so a stall shows up here first: a
        # single pegged core (cpu_max) stalls the control loop even while the
        # average looks calm, and thermal throttle drops throughput.
        if node.system_fresh() and node.cpu_percent is not None:
            cpu = node.cpu_percent
            temp = node.cpu_temp_c
            # Big number reflects the worst of CPU load and temperature so a
            # thermal problem is visible even when the CPU number looks fine.
            cpu_color = (COLORS["bad"] if cpu >= 92.0
                         else COLORS["warn"] if cpu >= 80.0 else COLORS["good"])
            temp_color = None
            if temp is not None:
                temp_color = (COLORS["bad"] if temp >= 80.0
                              else COLORS["warn"] if temp >= 70.0 else COLORS["good"])
            worst = cpu_color
            if temp_color == COLORS["bad"] or (temp_color == COLORS["warn"] and worst == COLORS["good"]):
                worst = temp_color
            compute_big.config(text=f"{cpu:.0f}%", fg=worst)
            core = "--" if node.cpu_percent_max is None else f"{node.cpu_percent_max:.0f}%"
            temp_str = "--" if temp is None else f"{temp:.0f}°C"
            temp_mark = " ⚠" if temp_color in (COLORS["warn"], COLORS["bad"]) else ""
            mem = "--" if node.mem_percent is None else f"{node.mem_percent:.0f}%"
            compute_sub.config(text=f"max core: {core}\ntemp: {temp_str}{temp_mark}  mem: {mem}")
        else:
            compute_big.config(text="--", fg=COLORS["muted"])
            compute_sub.config(text="no data — 차에서\nsystem_telemetry_node 실행?")

        # BOARD-PRIORITY GUARD line (from control_node on the car)
        guard_fresh = node.guard_t is not None and (time.monotonic() - node.guard_t) < node.STALE_SEC
        if guard_fresh and node.guard_scale is not None and node.guard_scale < 0.995:
            guard_label.config(
                text=f"⚡ 보드 우선 가드 작동 중 — 모터 {node.guard_scale * 100.0:.0f}%로 제한",
                fg=COLORS["warn"])
        elif guard_fresh:
            guard_label.config(text="보드 우선 가드: 대기 (모터 100%)", fg=COLORS["muted"])
        else:
            guard_label.config(text="", fg=COLORS["muted"])

        # STATUS banner
        if not node.battery_fresh() or node.battery_voltage is None:
            status.config(text="NO DATA — battery_node 실행 중인지 확인", fg=COLORS["muted"])
        elif node.battery_voltage <= node.brownout_voltage:
            status.config(text="⚠ BROWNOUT — 배터리 전압 붕괴", fg=COLORS["bad"])
        else:
            volts = [s[1] for s in node.history if s[1] is not None]
            droop = (max(volts) - min(volts)) if len(volts) >= 2 else 0.0
            if droop >= 0.8:
                status.config(text=f"⚠ VOLTAGE SAG — 부하 시 {droop:.2f}V 출렁임", fg=COLORS["warn"])
            else:
                status.config(text="SUPPLY OK", fg=COLORS["good"])

        draw_plot()

    # Ctrl+C: rclpy.init installs a SIGINT handler that only flips an internal
    # flag, but tkinter's mainloop never checks it, so Ctrl+C otherwise leaves
    # the window hanging. Flag a stop from the signal (the only reentrant-safe
    # thing to do — Tk calls aren't) and let the pump close the window from the
    # main thread.
    stop_requested = {"flag": False}

    def _request_stop(*_):
        stop_requested["flag"] = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    def pump():
        if stop_requested["flag"] or not rclpy.ok():
            root.destroy()
            return
        # spin_once executes at most ONE callback; subscriptions deliver ~35
        # msg/s while driving, so a single call per 100ms tick backlogs the
        # executor and the guard/battery labels lag by seconds. Drain a bounded
        # batch (20/tick = 200 cb/s budget) to stay ahead of the inflow.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.0)
        refresh()
        root.after(100, pump)

    def on_close():
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(100, pump)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
