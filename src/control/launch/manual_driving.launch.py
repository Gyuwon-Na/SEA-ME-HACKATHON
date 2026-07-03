# Standalone CAR-SIDE manual driving / steer-trim calibration: it starts its own
# control_node (starts in MANUAL) plus a car-side joystick_node.
#
# WARNING: do NOT run this alongside vehicle.launch.py or driving.launch.py.
# Those already start a control_node; two control_nodes both drive the same
# PCA9685 over I2C and fight each other. For normal auto/manual driving use
# vehicle.launch (car) + driving.launch (PC) and toggle mode with the joystick
# A button instead of launching this.
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def get_vehicle_config_path():
    for base_path in Path(__file__).resolve().parents:
        candidate = base_path / 'src' / 'config' / 'vehicle_config.yaml'
        if candidate.exists():
            return str(candidate)
    return str(Path('/home/topst/D-Racer/src/config/vehicle_config.yaml'))


def generate_launch_description():
    vehicle_config_path = get_vehicle_config_path()

    return LaunchDescription([
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[
                {
                    'use_joystick_control': True,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
        Node(
            package='joystick',
            executable='joystick_node',
            name='joystick_node',
            output='screen',
            parameters=[
                {
                    'calibration_mode': True,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
    ])
