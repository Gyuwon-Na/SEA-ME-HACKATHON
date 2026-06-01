from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    allow_reverse = LaunchConfiguration('allow_reverse')

    return LaunchDescription([
        DeclareLaunchArgument(
            'allow_reverse',
            default_value='true',
            description='Allow joystick throttle to go negative for reverse driving.',
        ),
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
                    'allow_reverse': allow_reverse,
                    'vehicle_config_file': vehicle_config_path,
                },
            ],
        ),
    ])
