from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(__file__).resolve().parent.parent / 'config' / 'imu_synthetic.yaml'
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(default_config),
        description='Path to an autokart_imu parameter YAML (synthetic or live).',
    )
    return LaunchDescription([
        config_arg,
        Node(
            package='autokart_imu',
            executable='imu_node',
            name='autokart_imu',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
