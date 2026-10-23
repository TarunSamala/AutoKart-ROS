from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(__file__).resolve().parent.parent / 'config' / 'synthetic.yaml'
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(default_config),
        description='Path to a perception_adapter parameter YAML (synthetic or topics_live).',
    )
    return LaunchDescription([
        config_arg,
        Node(
            package='autokart_perception_adapter',
            executable='perception_adapter_node',
            name='autokart_perception_adapter',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
