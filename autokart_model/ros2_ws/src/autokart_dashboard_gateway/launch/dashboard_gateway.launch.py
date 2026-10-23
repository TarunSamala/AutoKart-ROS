from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(__file__).resolve().parent.parent / 'config' / 'dashboard_gateway_synthetic.yaml'
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(default_config),
        description='Path to an autokart_dashboard_gateway parameter YAML (synthetic or live).',
    )
    return LaunchDescription([
        config_arg,
        Node(
            package='autokart_dashboard_gateway',
            executable='dashboard_gateway_node',
            name='autokart_dashboard_gateway',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
