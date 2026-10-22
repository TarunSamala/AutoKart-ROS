"""One command starts the whole stack (vehicle stays DISARMED until the operator arms it on the dashboard).

    ros2 launch autokart_bringup autokart.launch.py
Options: params:=/path/to/autokart.yaml  data_mode:=LIVE|REPLAY|SIMULATED 
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("autokart_bringup"), "config", "autokart.yaml")
    params = LaunchConfiguration("params")
    nodes = [
        Node(package="autokart_vehicle_bridge", executable="autokart_vehicle_bridge_node", name="autokart_vehicle_bridge",
             parameters=[params, {"data_mode": LaunchConfiguration("data_mode")}], output="screen"),
        Node(package="autokart_perception", executable="autokart_perception_node", name="autokart_perception",
             parameters=[params], output="screen"),
        Node(package="autokart_mapping", executable="autokart_mapping_node", name="autokart_mapping",
             parameters=[params], output="screen"),
        Node(package="autokart_planner", executable="autokart_planner_node", name="autokart_planner",
             parameters=[params], output="screen"),
        Node(package="autokart_controller", executable="autokart_controller_node", name="autokart_controller",
             parameters=[params], output="screen"),
        Node(package="autokart_safety", executable="autokart_safety_node", name="autokart_safety",
             parameters=[params], output="screen"),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("data_mode", default_value="LIVE"),
        *nodes,
    ])
