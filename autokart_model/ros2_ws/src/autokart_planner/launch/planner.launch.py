from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(__file__).resolve().parent.parent / 'config' / 'planner_builtin.yaml'
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(default_config),
        description='Path to a planner node parameter YAML (builtin or reused).',
    )
    return LaunchDescription([
        config_arg,
        Node(
            package='autokart_planner',
            executable='planner_node',
            name='autokart_planner',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
