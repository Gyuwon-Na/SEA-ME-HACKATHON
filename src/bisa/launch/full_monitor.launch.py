from launch import LaunchDescription
from launch_ros.actions import Node
import os


def get_vehicle_config_path():
    candidates = [
        os.path.expanduser('~/D-Racer-Kit/src/config/vehicle_config.yaml'),
        os.path.expanduser('~/D-Racer/src/config/vehicle_config.yaml'),
    ]

    for path in candidates:
        if os.path.exists(path):
            return path

    return os.path.expanduser('~/D-Racer-Kit/src/config/vehicle_config.yaml')


def generate_launch_description():
    vehicle_config_path = get_vehicle_config_path()

    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[
            {
                'vehicle_config_file': vehicle_config_path,
                'publish_topic': '/camera/image/compressed',
                'publish_hz': 30.0,
                'debug_log': False,
            }
        ],
    )

    battery_node = Node(
        package='battery',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[
            {
                'publish_topic': 'battery_status',
                'publish_hz': 10.0,
                'debug_log': False,
            }
        ],
    )

    monitor_node = Node(
        package='monitor',
        executable='monitor_node',
        name='monitor_node',
        output='screen',
        parameters=[
            {
                'vehicle_config_file': vehicle_config_path,
                'battery_topic': 'battery_status',
                'image_topic': '/camera/image/compressed',
                'debug_image': False,
                'web_host': '0.0.0.0',
                'web_port': 5000,
                'debug_log': False,
            }
        ],
    )

    return LaunchDescription([
        camera_node,
        battery_node,
        monitor_node,
    ])
