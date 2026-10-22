from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(__file__).resolve().parent.parent / 'config' / 'mission_skidpad.yaml'
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(default_config),
        description='Path to a mission node parameter YAML (skidpad or braking_test).',
    )
    return LaunchDescription([
        config_arg,
        Node(
            package='autokart_mission',
            executable='mission_node',
            name='autokart_mission',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
