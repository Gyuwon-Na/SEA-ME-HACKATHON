from collections import deque
from datetime import datetime, timezone
import threading
import time


class MonitorState:
    def __init__(
        self,
        stale_timeout_sec,
        image_source_width,
        image_source_height,
        brownout_voltage=6.0,
        power_history_len=150,
    ):
        self._lock = threading.Lock()
        self._stale_timeout_sec = stale_timeout_sec

        self._battery_status = None
        self._battery_updated_at = None
        self._battery_updated_monotonic = None

        self._image_frame = None
        self._image_width = image_source_width
        self._image_height = image_source_height
        self._image_updated_at = None
        self._image_updated_monotonic = None
        self._debug_frames = {
            'grayscale': None,
            'blur': None,
            'edge': None,
        }
        self._debug_widths = {
            'grayscale': image_source_width,
            'blur': image_source_width,
            'edge': image_source_width,
        }
        self._debug_heights = {
            'grayscale': image_source_height,
            'blur': image_source_height,
            'edge': image_source_height,
        }
        self._debug_updated_at = {
            'grayscale': None,
            'blur': None,
            'edge': None,
        }
        self._debug_updated_monotonic = {
            'grayscale': None,
            'blur': None,
            'edge': None,
        }

        self._throttle = None
        self._steering = None
        self._control_updated_at = None
        self._control_updated_monotonic = None

        self._is_recording = False
        self._recording_updated_at = None
        self._recording_updated_monotonic = None

        self._storage_used_percentage = None
        self._storage_used_bytes = None
        self._storage_total_bytes = None
        self._storage_updated_at = None
        self._storage_updated_monotonic = None

        # Power telemetry from the INA219 (voltage/current/watt) plus a rolling
        # window used to show voltage droop under load and a rough motor-vs-
        # baseline power split. brownout_voltage is the absolute sag alarm line;
        # set it to the pack (2S ~6.0V, 4S ~12.0V) via the monitor_node param.
        self._brownout_voltage = float(brownout_voltage)
        self._power_voltage = None
        self._power_current_ma = None
        self._power_watt = None
        self._power_updated_at = None
        self._power_updated_monotonic = None
        # Each sample: (voltage, current_ma, watt, throttle_at_sample).
        self._power_history = deque(maxlen=int(power_history_len))

    def _is_stale(self, updated_monotonic):
        if updated_monotonic is None:
            return True

        return (time.monotonic() - updated_monotonic) > self._stale_timeout_sec

    def _format_gb(self, size_bytes):
        if size_bytes is None:
            return '--'

        return f'{size_bytes / (1024 ** 3):.1f} GB'

    def update_battery(self, battery_status):
        clamped_value = max(0.0, min(100.0, float(battery_status)))

        with self._lock:
            self._battery_status = clamped_value
            self._battery_updated_at = datetime.now(timezone.utc)
            self._battery_updated_monotonic = time.monotonic()

    def update_image(self, frame_bytes, source_width, source_height):
        with self._lock:
            self._image_frame = frame_bytes
            self._image_width = int(source_width)
            self._image_height = int(source_height)
            self._image_updated_at = datetime.now(timezone.utc)
            self._image_updated_monotonic = time.monotonic()

    def update_control(self, throttle, steering):
        clamped_throttle = max(-1.0, min(1.0, float(throttle)))
        clamped_steering = max(-1.0, min(1.0, float(steering)))

        with self._lock:
            self._throttle = clamped_throttle
            self._steering = clamped_steering
            self._control_updated_at = datetime.now(timezone.utc)
            self._control_updated_monotonic = time.monotonic()

    def update_debug_image(self, image_key, frame_bytes, source_width, source_height):
        if image_key not in self._debug_frames:
            return

        with self._lock:
            self._debug_frames[image_key] = frame_bytes
            self._debug_widths[image_key] = int(source_width)
            self._debug_heights[image_key] = int(source_height)
            self._debug_updated_at[image_key] = datetime.now(timezone.utc)
            self._debug_updated_monotonic[image_key] = time.monotonic()

    def update_recording(self, is_recording):
        with self._lock:
            self._is_recording = bool(is_recording)
            self._recording_updated_at = datetime.now(timezone.utc)
            self._recording_updated_monotonic = time.monotonic()

    def update_storage(self, used_bytes, total_bytes):
        if total_bytes <= 0:
            return

        used_percentage = (float(used_bytes) / float(total_bytes)) * 100.0

        with self._lock:
            self._storage_used_percentage = max(0.0, min(100.0, used_percentage))
            self._storage_used_bytes = int(used_bytes)
            self._storage_total_bytes = int(total_bytes)
            self._storage_updated_at = datetime.now(timezone.utc)
            self._storage_updated_monotonic = time.monotonic()

    def update_power(self, voltage, current_ma, watt):
        with self._lock:
            self._power_voltage = None if voltage is None else float(voltage)
            self._power_current_ma = None if current_ma is None else float(current_ma)
            self._power_watt = None if watt is None else float(watt)
            self._power_updated_at = datetime.now(timezone.utc)
            self._power_updated_monotonic = time.monotonic()
            throttle = self._throttle if self._throttle is not None else 0.0
            if self._power_voltage is not None:
                self._power_history.append((
                    self._power_voltage,
                    self._power_current_ma,
                    self._power_watt,
                    float(throttle),
                ))

    def get_latest_frame(self):
        with self._lock:
            return self._image_frame

    def get_debug_frame(self, image_key):
        with self._lock:
            return self._debug_frames.get(image_key)

    def snapshot(self):
        with self._lock:
            battery_status = self._battery_status
            battery_updated_at = self._battery_updated_at
            battery_updated_monotonic = self._battery_updated_monotonic

            image_width = self._image_width
            image_height = self._image_height
            image_updated_at = self._image_updated_at
            image_updated_monotonic = self._image_updated_monotonic
            debug_widths = dict(self._debug_widths)
            debug_heights = dict(self._debug_heights)
            debug_updated_at = dict(self._debug_updated_at)
            debug_updated_monotonic = dict(self._debug_updated_monotonic)

            throttle = self._throttle
            steering = self._steering
            control_updated_at = self._control_updated_at
            control_updated_monotonic = self._control_updated_monotonic

            is_recording = self._is_recording
            recording_updated_at = self._recording_updated_at
            recording_updated_monotonic = self._recording_updated_monotonic

            storage_used_percentage = self._storage_used_percentage
            storage_used_bytes = self._storage_used_bytes
            storage_total_bytes = self._storage_total_bytes
            storage_updated_at = self._storage_updated_at
            storage_updated_monotonic = self._storage_updated_monotonic

            power_voltage = self._power_voltage
            power_current_ma = self._power_current_ma
            power_watt = self._power_watt
            power_updated_at = self._power_updated_at
            power_updated_monotonic = self._power_updated_monotonic
            brownout_voltage = self._brownout_voltage
            power_history = list(self._power_history)

        battery_has_data = battery_status is not None
        image_has_data = image_updated_at is not None
        control_has_data = throttle is not None and steering is not None
        storage_has_data = (
            storage_used_percentage is not None
            and storage_used_bytes is not None
            and storage_total_bytes is not None
        )

        power_has_data = power_voltage is not None
        volts = [sample[0] for sample in power_history if sample[0] is not None]
        v_min = min(volts) if volts else None
        v_max = max(volts) if volts else None
        droop = (v_max - v_min) if (v_min is not None and v_max is not None) else None
        # Baseline = the lightest power seen while roughly idle (|throttle|<0.05),
        # i.e. board + camera + servo draw. Motor draw is estimated as the excess
        # above that baseline. This is an ESTIMATE from one sensor, not a per-rail
        # measurement.
        idle_watts = [
            sample[2] for sample in power_history
            if sample[2] is not None and abs(sample[3]) < 0.05
        ]
        any_watts = [sample[2] for sample in power_history if sample[2] is not None]
        baseline_w = min(idle_watts) if idle_watts else (min(any_watts) if any_watts else None)
        motor_w = None
        if power_watt is not None and baseline_w is not None:
            motor_w = max(0.0, power_watt - baseline_w)
        sag_active = power_has_data and power_voltage <= brownout_voltage
        debug_image = {}
        for key in ('grayscale', 'blur', 'edge'):
            updated_at = debug_updated_at[key]
            debug_image[key] = {
                'has_data': updated_at is not None,
                'updated_at': None if updated_at is None else updated_at.isoformat(),
                'is_stale': self._is_stale(debug_updated_monotonic[key]),
                'resolution_display': f"{debug_widths[key]}x{debug_heights[key]}",
            }

        return {
            'battery': {
                'has_data': battery_has_data,
                'battery_status': None if battery_status is None else round(battery_status, 1),
                'battery_display': '--.-%' if battery_status is None else f'{battery_status:.1f}%',
                'updated_at': None if battery_updated_at is None else battery_updated_at.isoformat(),
                'is_stale': self._is_stale(battery_updated_monotonic),
            },
            'image': {
                'has_data': image_has_data,
                'updated_at': None if image_updated_at is None else image_updated_at.isoformat(),
                'is_stale': self._is_stale(image_updated_monotonic),
                'resolution_display': f'{image_width}x{image_height}',
            },
            'control': {
                'has_data': control_has_data,
                'updated_at': None if control_updated_at is None else control_updated_at.isoformat(),
                'is_stale': self._is_stale(control_updated_monotonic),
                'throttle': None if throttle is None else round(throttle, 2),
                'steering': None if steering is None else round(steering, 2),
            },
            'recording': {
                'has_data': recording_updated_at is not None,
                'updated_at': None if recording_updated_at is None else recording_updated_at.isoformat(),
                'is_stale': self._is_stale(recording_updated_monotonic),
                'is_recording': bool(is_recording),
            },
            'storage': {
                'has_data': storage_has_data,
                'updated_at': None if storage_updated_at is None else storage_updated_at.isoformat(),
                'is_stale': self._is_stale(storage_updated_monotonic),
                'used_percentage': (
                    None if storage_used_percentage is None else round(storage_used_percentage, 1)
                ),
                'used_display': (
                    '--.-%' if storage_used_percentage is None
                    else f'{storage_used_percentage:.1f}%'
                ),
                'used_space_display': self._format_gb(storage_used_bytes),
                'total_space_display': self._format_gb(storage_total_bytes),
            },
            'power': {
                'has_data': power_has_data,
                'is_stale': self._is_stale(power_updated_monotonic),
                'updated_at': None if power_updated_at is None else power_updated_at.isoformat(),
                'voltage': None if power_voltage is None else round(power_voltage, 3),
                'voltage_display': '--.- V' if power_voltage is None else f'{power_voltage:.2f} V',
                'current_ma': None if power_current_ma is None else round(power_current_ma, 1),
                'current_display': '--- mA' if power_current_ma is None else f'{power_current_ma:.0f} mA',
                'watt': None if power_watt is None else round(power_watt, 2),
                'watt_display': '--.- W' if power_watt is None else f'{power_watt:.2f} W',
                'v_min': None if v_min is None else round(v_min, 3),
                'v_max': None if v_max is None else round(v_max, 3),
                'droop': None if droop is None else round(droop, 3),
                'droop_display': '-- V' if droop is None else f'{droop:.2f} V',
                'baseline_w': None if baseline_w is None else round(baseline_w, 2),
                'motor_w': None if motor_w is None else round(motor_w, 2),
                'brownout_voltage': round(brownout_voltage, 2),
                'sag_active': bool(sag_active),
                'history': {
                    'v': [round(sample[0], 3) for sample in power_history],
                    'w': [None if sample[2] is None else round(sample[2], 2) for sample in power_history],
                    'thr': [round(sample[3], 2) for sample in power_history],
                },
            },
            'debug_image': debug_image,
        }
