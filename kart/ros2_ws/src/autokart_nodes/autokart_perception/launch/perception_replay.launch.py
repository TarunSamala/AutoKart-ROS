"""Disarmed recorded-frame inspection; replay_dir must be provided explicitly."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("replay_dir", default_value=""),
        Node(package="autokart_perception", executable="perception_node",
             name="autokart_perception", output="screen",
             parameters=[{"mode": "replay", "replay_dir": LaunchConfiguration("replay_dir")}]),
    ])
