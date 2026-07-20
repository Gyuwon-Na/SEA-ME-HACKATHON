import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml

from control_msgs.msg import Control
from joystick_msgs.msg import Joystick
from std_msgs.msg import Int8
from topst_utils.gamepads import ShanWanGamepad


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/HD-Racer-Kit/src/config/vehicle_config.yaml'


def get_default_data_acquisition_script_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'data_acquisition.sh'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/HD-Racer-Kit/src/data_acquisition.sh'


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        # ROS parameters
        self.declare_parameter('publish_topic', 'joystick')
        self.declare_parameter('nudge_topic', '/bisa/drive_nudge')
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('throttle_scale', 0.12)
        self.declare_parameter('throttle_deadzone', 0.05)
        self.declare_parameter('steering_deadzone', 0.05)
        self.declare_parameter('steering_axis', 'auto')
        self.declare_parameter('steering_trim', 0.0)
        self.declare_parameter('calibration_mode', False)
        self.declare_parameter('calibration_step', 0.1)
        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter(
            'data_acquisition_script',
            get_default_data_acquisition_script_path(),
        )
        self.declare_parameter('accel_ratio_step', 0.005)
        self.declare_parameter('accel_ratio_min', 0.12)
        self.declare_parameter('accel_ratio_max', 0.4)
        self.declare_parameter('debug_log_enable', True)
        self.declare_parameter('debug_log_hz', 5.0)
        # Start in AUTO; the A button latch-toggles AUTO<->MANUAL at runtime.
        self.declare_parameter('start_in_manual', False)

        publish_topic = str(self.get_parameter('publish_topic').value)
        nudge_topic = str(self.get_parameter('nudge_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be greater than 0')

        self.throttle_scale = float(self.get_parameter('throttle_scale').value)
        self.throttle_deadzone = float(self.get_parameter('throttle_deadzone').value)
        self.steering_deadzone = float(self.get_parameter('steering_deadzone').value)
        self.steering_axis = str(self.get_parameter('steering_axis').value)
        self.steering_trim = float(self.get_parameter('steering_trim').value)
        self.calibration_mode = bool(self.get_parameter('calibration_mode').value)
        self.calibration_step = float(self.get_parameter('calibration_step').value)
        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        self.data_acquisition_script = os.path.expanduser(
            str(self.get_parameter('data_acquisition_script').value)
        )
        self.accel_ratio_step = float(self.get_parameter('accel_ratio_step').value)
        self.accel_ratio_min = float(self.get_parameter('accel_ratio_min').value)
        self.accel_ratio_max = float(self.get_parameter('accel_ratio_max').value)
        self.debug_log_enable = bool(self.get_parameter('debug_log_enable').value)
        self.debug_log_hz = float(self.get_parameter('debug_log_hz').value)
        self.manual_mode = bool(self.get_parameter('start_in_manual').value)
        self.publish_hz = publish_hz

        if self.accel_ratio_min > self.accel_ratio_max:
            raise ValueError('accel_ratio_min must be less than or equal to accel_ratio_max')

        self.accel_ratio = self.clamp(
            self.throttle_scale,
            self.accel_ratio_min,
            self.accel_ratio_max,
        )

        self._prev_l1_pressed = False
        self._prev_r1_pressed = False
        self._prev_y_pressed = False
        self._prev_b_pressed = False
        self._prev_nudge_y_pressed = False
        self._prev_nudge_b_pressed = False
        self._prev_x_pressed = False
        self._prev_start_pressed = False
        self._prev_a_pressed = False
        self.e_stop_latched = False
        self.is_recording = False
        self.recording_process = None
        self._debug_left_y = 0.0
        self._debug_right_x = 0.0
        self._debug_right_y = 0.0
        self._debug_steering = 0.0
        self._debug_throttle = 0.0

        self.load_saved_calibration()

        self.joystick_pub = self.create_publisher(Joystick, publish_topic, 10)
        self.nudge_pub = self.create_publisher(Int8, nudge_topic, 10)
        self.gamepad = ShanWanGamepad()
        self.latest_input = None
        self.lock = threading.Lock()

        self.running = True
        self.reader_thread = threading.Thread(
            target=self.gamepad_read_loop,
            daemon=True,
        )
        self.reader_thread.start()

        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)
        if self.debug_log_enable and self.debug_log_hz > 0.0:
            self.debug_timer = self.create_timer(
                1.0 / self.debug_log_hz,
                self.debug_timer_callback,
            )

        self.get_logger().info(
            f'Joystick node started: topic={publish_topic}, nudge_topic={nudge_topic}, '
            f'publish_hz={self.publish_hz}, '
            f'throttle_scale={self.throttle_scale}, throttle_deadzone={self.throttle_deadzone}, '
            f'steering_deadzone={self.steering_deadzone}, steering_axis={self.steering_axis}, '
            f'steering_trim={self.steering_trim}, calibration_mode={self.calibration_mode}, '
            f'calibration_step={self.calibration_step}, '
            f'vehicle_config_file={self.vehicle_config_file}, '
            f'data_acquisition_script={self.data_acquisition_script}, '
            f'accel_ratio={self.accel_ratio}, accel_ratio_step={self.accel_ratio_step}, '
            f'accel_ratio_min={self.accel_ratio_min}, accel_ratio_max={self.accel_ratio_max}, '
            f'debug_log_enable={self.debug_log_enable}, debug_log_hz={self.debug_log_hz}'
        )

    @staticmethod
    def clamp(value, min_v=-1.0, max_v=1.0):
        return max(min(value, max_v), min_v)

    @staticmethod
    def deadzone(value, deadzone_value=0.05):
        return 0.0 if abs(value) < deadzone_value else value

    def load_saved_calibration(self):
        if not os.path.exists(self.vehicle_config_file):
            return

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as calibration_stream:
                calibration_data = yaml.safe_load(calibration_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return

        saved_trim = calibration_data.get('STEER_TRIM')
        if saved_trim is None:
            return

        self.steering_trim = float(saved_trim)
        self.set_parameters([
            Parameter('steering_trim', Parameter.Type.DOUBLE, self.steering_trim),
        ])

    def save_calibration(self):
        calibration_dir = os.path.dirname(self.vehicle_config_file)
        if calibration_dir:
            os.makedirs(calibration_dir, exist_ok=True)

        config_data = {}
        if os.path.exists(self.vehicle_config_file):
            try:
                with open(self.vehicle_config_file, 'r', encoding='utf-8') as calibration_stream:
                    config_data = yaml.safe_load(calibration_stream) or {}
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to merge existing vehicle config {self.vehicle_config_file}: {exc}'
                )

        config_data['STEER_TRIM'] = float(self.steering_trim)

        with open(self.vehicle_config_file, 'w', encoding='utf-8') as calibration_stream:
            yaml.safe_dump(config_data, calibration_stream, sort_keys=False)

    def update_steering_trim_from_buttons(self, data):
        if not self.calibration_mode:
            return

        y_pressed = bool(data.button_y)
        b_pressed = bool(data.button_b)
        trim_changed = False

        if y_pressed and not self._prev_y_pressed:
            self.steering_trim = self.clamp(self.steering_trim - self.calibration_step)
            trim_changed = True

        if b_pressed and not self._prev_b_pressed:
            self.steering_trim = self.clamp(self.steering_trim + self.calibration_step)
            trim_changed = True

        if trim_changed:
            self.set_parameters([
                Parameter('steering_trim', Parameter.Type.DOUBLE, self.steering_trim),
            ])
            try:
                self.save_calibration()
                self.get_logger().info(f'steering_trim updated to {self.steering_trim:.2f}')
            except Exception as exc:
                self.get_logger().error(f'Failed to save steering calibration: {exc}')

        self._prev_y_pressed = y_pressed
        self._prev_b_pressed = b_pressed

    def update_nudge_from_buttons(self, data):
        """Publish one autonomous nudge event per B/Y button press."""
        b_pressed = bool(data.button_b)
        y_pressed = bool(data.button_y)
        b_edge = b_pressed and not self._prev_nudge_b_pressed
        y_edge = y_pressed and not self._prev_nudge_y_pressed
        self._prev_nudge_b_pressed = b_pressed
        self._prev_nudge_y_pressed = y_pressed

        # B/Y retain their steering-trim behavior while explicit calibration is
        # active. In production calibration_mode is false, so B nudges LEFT and
        # Y nudges RIGHT without changing the latched AUTO/MANUAL mode.
        if (
            self.calibration_mode
            or self.manual_mode
            or self.e_stop_latched
            or bool(getattr(data, 'button_x', False))
        ):
            return
        if b_edge and y_edge:
            self.get_logger().warning('Ignoring simultaneous B/Y nudge buttons')
            return
        if b_edge:
            self.nudge_pub.publish(Int8(data=1))
            self.get_logger().info('Joystick nudge requested: LEFT (B)')
        elif y_edge:
            self.nudge_pub.publish(Int8(data=-1))
            self.get_logger().info('Joystick nudge requested: RIGHT (Y)')

    def update_accel_ratio_from_buttons(self, data):
        l1_pressed = bool(data.button_L1)
        r1_pressed = bool(data.button_R1)

        if l1_pressed and not self._prev_l1_pressed:
            self.accel_ratio = self.clamp(
                self.accel_ratio - self.accel_ratio_step,
                self.accel_ratio_min,
                self.accel_ratio_max,
            )
            self.get_logger().info(f'accel_ratio decreased to {self.accel_ratio:.3f}')

        if r1_pressed and not self._prev_r1_pressed:
            self.accel_ratio = self.clamp(
                self.accel_ratio + self.accel_ratio_step,
                self.accel_ratio_min,
                self.accel_ratio_max,
            )
            self.get_logger().info(f'accel_ratio increased to {self.accel_ratio:.3f}')

        self._prev_l1_pressed = l1_pressed
        self._prev_r1_pressed = r1_pressed

    def update_e_stop_from_buttons(self, data):
        x_pressed = bool(data.button_x)

        if x_pressed and not self._prev_x_pressed and not self.e_stop_latched:
            self.e_stop_latched = True
            self.get_logger().warning('E-STOP latched by joystick X button')

        self._prev_x_pressed = x_pressed

    def update_mode_from_buttons(self, data):
        # A button latch-toggles AUTO <-> MANUAL. control_node arbitrates on the
        # published manual_mode flag, so this is the single source of drive mode.
        a_pressed = bool(data.button_a)

        if a_pressed and not self._prev_a_pressed:
            self.manual_mode = not self.manual_mode
            self.get_logger().info(
                f'Drive mode -> {"MANUAL" if self.manual_mode else "AUTO"} (joystick A)'
            )

        self._prev_a_pressed = a_pressed

    def start_recording(self):
        if self.recording_process is not None and self.recording_process.poll() is None:
            self.is_recording = True
            return

        script_path = Path(self.data_acquisition_script)
        if not script_path.exists():
            self.is_recording = False
            self.recording_process = None
            self.get_logger().error(f'Data acquisition script not found: {script_path}')
            return

        try:
            self.recording_process = subprocess.Popen(
                [str(script_path)],
                cwd=str(script_path.parent),
                start_new_session=True,
            )
            self.is_recording = True
            self.get_logger().info(f'Recording started: {script_path}')
        except Exception as exc:
            self.is_recording = False
            self.recording_process = None
            self.get_logger().error(f'Failed to start recording: {exc}')

    def stop_recording(self):
        process = self.recording_process
        self.recording_process = None
        self.is_recording = False

        if process is None or process.poll() is not None:
            return

        try:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=5.0)
            self.get_logger().info('Recording stopped')
        except subprocess.TimeoutExpired:
            self.get_logger().warning('Recording did not stop in time. Sending SIGKILL.')
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
        except ProcessLookupError:
            pass
        except Exception as exc:
            self.get_logger().error(f'Failed to stop recording cleanly: {exc}')

    def sync_recording_state(self):
        if self.recording_process is None:
            self.is_recording = False
            return

        if self.recording_process.poll() is not None:
            self.recording_process = None
            self.is_recording = False
            self.get_logger().info('Recording process exited')

    def update_recording_from_buttons(self, data):
        start_pressed = bool(data.button_start)

        if start_pressed and not self._prev_start_pressed:
            self.sync_recording_state()
            if self.is_recording:
                self.stop_recording()
            else:
                self.start_recording()

        self._prev_start_pressed = start_pressed

    def read_steering_axis(self, data):
        right_x = self.clamp(data.analog_stick_right.x)
        right_y = self.clamp(data.analog_stick_right.y)

        if self.steering_axis == 'right_x':
            return right_x
        if self.steering_axis == 'right_y':
            return right_y
        return right_x if abs(right_x) >= abs(right_y) else right_y

    def gamepad_read_loop(self):
        while rclpy.ok() and self.running:
            try:
                data = self.gamepad.read_data()
                self.update_accel_ratio_from_buttons(data)
                self.update_steering_trim_from_buttons(data)
                self.update_nudge_from_buttons(data)
                self.update_e_stop_from_buttons(data)
                self.update_mode_from_buttons(data)
                self.update_recording_from_buttons(data)
                with self.lock:
                    self.latest_input = data
            except Exception as exc:
                self.get_logger().error(f'Gamepad read error: {exc}')
                time.sleep(0.1)

    def timer_callback(self):
        with self.lock:
            data = self.latest_input

        if data is None:
            return

        self.sync_recording_state()

        throttle_axis = self.deadzone(
            self.clamp(data.analog_stick_left.y),
            self.throttle_deadzone,
        )
        throttle = self.clamp(throttle_axis * self.accel_ratio)
        # 후진: ESC 후진 데드밴드(~0.4) 못넘으면 안움직여서 최소세기 floor 보정
        if throttle_axis < 0.0:
            throttle = -max(0.42, abs(throttle))

        steering = self.deadzone(
            self.read_steering_axis(data),
            self.steering_deadzone,
        )
        steering = self.clamp(steering + self.steering_trim)
        if self.e_stop_latched:
            throttle = 0.0

        self._debug_left_y = float(data.analog_stick_left.y)
        self._debug_right_x = float(data.analog_stick_right.x)
        self._debug_right_y = float(data.analog_stick_right.y)
        self._debug_steering = float(steering)
        self._debug_throttle = float(throttle)

        msg = Joystick()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'joystick'
        control_msg = Control()
        control_msg.header.stamp = msg.header.stamp
        control_msg.header.frame_id = 'joystick'
        control_msg.steering = float(steering)
        control_msg.throttle = float(throttle)
        msg.control_msg = control_msg
        msg.accel_ratio = float(self.accel_ratio)
        msg.e_stop_en = bool(self.e_stop_latched)
        msg.is_recording = bool(self.is_recording)
        msg.manual_mode = bool(self.manual_mode)
        self.joystick_pub.publish(msg)

    def debug_timer_callback(self):
        with self.lock:
            data = self.latest_input

        l1_state = int(bool(data.button_L1)) if data else 0
        r1_state = int(bool(data.button_R1)) if data else 0
        self.get_logger().info(
            f'[Joystick DBG] \n'
            f'left_y={self._debug_left_y:.2f} \n'
            f'right_x={self._debug_right_x:.2f} right_y={self._debug_right_y:.2f} \n'
            f'steering={self._debug_steering:.2f} throttle={self._debug_throttle:.2f} \n'
            f'accel_ratio={self.accel_ratio:.3f} \n'
            f'trim={self.steering_trim:.2f} \n'
            f'mode={"MANUAL" if self.manual_mode else "AUTO"} \n'
            f'e_stop={int(self.e_stop_latched)} \n'
            f'recording={int(self.is_recording)} \n'
            f'L1={l1_state} R1={r1_state}\n'
        )

    def destroy_node(self):
        self.running = False
        self.stop_recording()
        reader_thread = getattr(self, 'reader_thread', None)
        if reader_thread is not None and reader_thread.is_alive():
            reader_thread.join(timeout=0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
