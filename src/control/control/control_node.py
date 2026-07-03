import os
from pathlib import Path

import rclpy
from rclpy.node import Node
import yaml

from control_msgs.msg import Control
from joystick_msgs.msg import Joystick
from topst_utils.d3racer import D3Racer


def get_default_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return '/home/topst/D-Racer/src/config/vehicle_config.yaml'


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # ROS parameters
        self.declare_parameter('i2c_bus', 3)
        self.declare_parameter('pca9685_addr', 0x40)
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('throttle_channel', 1)
        self.declare_parameter('vehicle_config_file', get_default_vehicle_config_path())
        self.declare_parameter('use_joystick_control', False)
        self.declare_parameter('joystick_topic', 'joystick')
        self.declare_parameter('control_topic', '/control')
        self.declare_parameter('command_hz', 10.0)
        # Failsafe: if the active source goes silent longer than this, cut
        # throttle (hold steering). Guards against WiFi/PC/joystick dropouts.
        self.declare_parameter('command_timeout_sec', 0.5)

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        pca9685_addr = int(self.get_parameter('pca9685_addr').value)
        steering_channel = int(self.get_parameter('steering_channel').value)
        throttle_channel = int(self.get_parameter('throttle_channel').value)
        self.vehicle_config_file = os.path.expanduser(
            str(self.get_parameter('vehicle_config_file').value)
        )
        self.use_joystick_control = bool(self.get_parameter('use_joystick_control').value)
        joystick_topic = str(self.get_parameter('joystick_topic').value)
        control_topic = str(self.get_parameter('control_topic').value)
        command_hz = float(self.get_parameter('command_hz').value)
        if command_hz <= 0.0:
            raise ValueError('command_hz must be greater than 0')
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        self.command_hz = command_hz
        self.steer_trim = self.load_steer_trim()

        self.d3_racer = D3Racer(
            i2c_bus=i2c_bus,
            pca9685_addr=pca9685_addr,
            steering_channel=steering_channel,
            throttle_channel=throttle_channel,
        )

        self.get_logger().info(
            'd3_racer configured:\n'
            f'  i2c_bus={i2c_bus}\n'
            f'  pca9685_addr=0x{pca9685_addr:02X}\n'
            f'  steering_channel={steering_channel}\n'
            f'  throttle_channel={throttle_channel}\n'
            f'  steer_trim={self.steer_trim}\n'
            f'  use_joystick_control={self.use_joystick_control}\n'
            f'  joystick_topic={joystick_topic}\n'
            f'  control_topic={control_topic}\n'
            f'  command_hz={self.command_hz}\n'
            f'  vehicle_config_file={self.vehicle_config_file}'
        )

        # Drive mode is chosen at runtime by the joystick (manual_mode flag).
        # use_joystick_control only seeds the initial mode for backward compat.
        self.manual_mode = self.use_joystick_control
        self.e_stop_active = False
        self.last_steering = self.steer_trim

        # Latest command from each source, kept independently so switching modes
        # is instant and one source never clobbers the other.
        self.auto_steering = self.steer_trim
        self.auto_throttle = 0.0
        self.last_auto_time = None
        self.manual_steering = self.steer_trim
        self.manual_throttle = 0.0
        self.last_joystick_time = None

        # Control inputs
        self.create_subscription(
            Joystick,
            joystick_topic,
            self.joystick_callback,
            10,
        )
        self.create_subscription(
            Control,
            control_topic,
            self.control_callback,
            10,
        )

        # Command output loop
        self.timer = self.create_timer(1.0 / self.command_hz, self.timer_callback)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        if self.e_stop_active:
            self.apply_actuation(self.last_steering, 0.0)
            return

        # Arbitrate: MANUAL uses the joystick, AUTO uses /control.
        if self.manual_mode:
            steering, throttle, last_time = (
                self.manual_steering, self.manual_throttle, self.last_joystick_time
            )
        else:
            steering, throttle, last_time = (
                self.auto_steering, self.auto_throttle, self.last_auto_time
            )

        # Failsafe: if the active source is stale (never arrived or dropped out),
        # cut throttle but keep steering so the car coasts straight to a stop.
        if last_time is None or (self.now_sec() - last_time) > self.command_timeout_sec:
            throttle = 0.0

        self.last_steering = steering
        self.apply_actuation(steering, throttle)

    def apply_actuation(self, steering, throttle):
        self.d3_racer.set_steering_percent(float(steering))
        self.d3_racer.set_throttle_percent(float(throttle))

    def joystick_callback(self, msg: Joystick):
        if bool(msg.e_stop_en):
            self.engage_e_stop()
            return

        if self.e_stop_active:
            return

        # The joystick owns the drive mode; record its command either way so a
        # switch to MANUAL takes effect on the very next tick.
        previous_mode = self.manual_mode
        self.manual_mode = bool(msg.manual_mode)
        if self.manual_mode != previous_mode:
            self.get_logger().info(
                f'Drive mode -> {"MANUAL" if self.manual_mode else "AUTO"}'
            )
        self.manual_steering = float(msg.control_msg.steering)
        self.manual_throttle = float(msg.control_msg.throttle)
        self.last_joystick_time = self.now_sec()

    def control_callback(self, msg: Control):
        if self.e_stop_active:
            return

        # Always record the autonomous command (even while in MANUAL) so a switch
        # back to AUTO is instant. Mode selection happens in timer_callback.
        self.auto_steering = float(msg.steering)
        self.auto_throttle = float(msg.throttle)
        self.last_auto_time = self.now_sec()

    def engage_e_stop(self):
        if self.e_stop_active:
            return

        self.e_stop_active = True
        self.apply_actuation(self.last_steering, 0.0)
        self.get_logger().warning('E-STOP engaged. Ignoring incoming throttle commands.')

    def load_steer_trim(self):
        if not os.path.exists(self.vehicle_config_file):
            return 0.0

        try:
            with open(self.vehicle_config_file, 'r', encoding='utf-8') as config_stream:
                config_data = yaml.safe_load(config_stream) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read vehicle config file {self.vehicle_config_file}: {exc}'
            )
            return 0.0

        return float(config_data.get('STEER_TRIM', 0.0))

    def destroy_node(self):
        try:
            if hasattr(self, 'd3_racer') and self.d3_racer is not None:
                self.apply_actuation(self.steer_trim, 0.0)
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
