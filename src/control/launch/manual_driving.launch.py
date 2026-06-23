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
