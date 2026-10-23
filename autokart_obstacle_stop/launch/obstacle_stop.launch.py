from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = Path(__file__).resolve().parent.parent / "config" / "obstacle_stop.yaml"
    config_arg = DeclareLaunchArgument(
        "config_file", default_value=str(default_config),
        description="Path to the obstacle-stop parameter file",
    )
    return LaunchDescription([
        config_arg,
        Node(
            package="autokart_obstacle_stop",
            executable="obstacle_stop_node",
            name="autokart_obstacle_stop",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])

