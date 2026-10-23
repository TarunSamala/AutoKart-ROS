import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("autokart_safety_bridge"),
        "config",
        "front_safety_envelope_monitor.yaml",
    )
    return LaunchDescription([
        Node(
            package="autokart_safety_bridge",
            executable="front_safety_envelope_monitor",
            name="front_safety_envelope_monitor",
            parameters=[config],
            output="screen",
        ),
    ])
