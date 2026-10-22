"""Disarmed RC-03-06 diagnostics. Odometry remains an external prerequisite.

The front safety envelope monitor (autokart_safety_bridge) is DIAGNOSTIC_ONLY:
no VCU/brake/throttle publisher or client exists in this stack. See
AUTOKART_SAFETY_ENVELOPE.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    safety_config = os.path.join(
        get_package_share_directory("autokart_safety_bridge"),
        "config", "front_safety_envelope_monitor.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("perception_mode", default_value="synthetic"),
        DeclareLaunchArgument("replay_dir", default_value=""),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("model_manifest", default_value=""),
        DeclareLaunchArgument("camera_library", default_value=""),
        DeclareLaunchArgument("camera_configuration", default_value=""),
        DeclareLaunchArgument("source", default_value="SIMULATED"),
        DeclareLaunchArgument("gateway_source", default_value="SIMULATION"),
        DeclareLaunchArgument("live_odom_verified", default_value="false"),
        DeclareLaunchArgument("mission", default_value="SKIDPAD"),
        DeclareLaunchArgument("brake_min_mps2", default_value="0.0"),
        DeclareLaunchArgument("brake_provenance", default_value="UNKNOWN"),
        DeclareLaunchArgument("launch_gateway", default_value="false"),
        DeclareLaunchArgument("gateway_port", default_value="8765"),
        DeclareLaunchArgument("perception_period_s", default_value="0.02"),
        DeclareLaunchArgument("dashboard_static_root", default_value=""),
        DeclareLaunchArgument("calibration_artifact", default_value=""),
        DeclareLaunchArgument("wheelbase_m", default_value="1.0"),
        DeclareLaunchArgument("lookahead_m", default_value="1.5"),
        DeclareLaunchArgument("controller_speed_limit_mps", default_value="5.0"),
        DeclareLaunchArgument("launch_safety_monitor", default_value="true"),
        # Provenance the safety monitor stamps onto every /safety/envelope message
        # (never defaults to LIVE -- see SafetyEnvelope.msg / RC-06 addendum).
        DeclareLaunchArgument("safety_data_source", default_value="UNVERIFIED"),
        Node(package="autokart_perception", executable="perception_node",
             parameters=[{"mode": LaunchConfiguration("perception_mode"),
                          "replay_dir": LaunchConfiguration("replay_dir"),
                          "model_path": LaunchConfiguration("model_path"),
                          "model_manifest": LaunchConfiguration("model_manifest"),
                          "camera_library": LaunchConfiguration("camera_library"),
                          "camera_configuration": LaunchConfiguration("camera_configuration"),
                          "period_s": LaunchConfiguration("perception_period_s"),
                          "calibration_artifact": LaunchConfiguration("calibration_artifact")}], output="screen"),
        Node(package="autokart_mapping", executable="mapping_node",
             parameters=[{"source_mode": LaunchConfiguration("source"),
                          "live_odom_verified": LaunchConfiguration("live_odom_verified")}], output="screen"),
        Node(package="autokart_planner", executable="planner_node",
             parameters=[{"source": LaunchConfiguration("source"),
                          "mission": LaunchConfiguration("mission"),
                          "brake_min_mps2": LaunchConfiguration("brake_min_mps2"),
                          "brake_provenance": LaunchConfiguration("brake_provenance")}], output="screen"),
        Node(package="autokart_controller", executable="controller_node",
             parameters=[{"source": LaunchConfiguration("source"),
                          "wheelbase_m": LaunchConfiguration("wheelbase_m"),
                          "lookahead_m": LaunchConfiguration("lookahead_m"),
                          "maximum_speed_mps": LaunchConfiguration("controller_speed_limit_mps")}], output="screen"),
        Node(package="autokart_safety_bridge", executable="front_safety_envelope_monitor",
             name="front_safety_envelope_monitor",
             condition=IfCondition(LaunchConfiguration("launch_safety_monitor")),
             parameters=[safety_config, {"data_source": LaunchConfiguration("safety_data_source")}],
             output="screen"),
        Node(package="autokart_dashboard_gateway", executable="dashboard_gateway",
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration("launch_gateway"), "' == 'true' and '",
                                                     LaunchConfiguration("dashboard_static_root"), "' == ''"])),
             arguments=["--source", LaunchConfiguration("gateway_source"), "--port", LaunchConfiguration("gateway_port"),
                        "--scenario", LaunchConfiguration("mission")], output="screen"),
        Node(package="autokart_dashboard_gateway", executable="dashboard_gateway",
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration("launch_gateway"), "' == 'true' and '",
                                                     LaunchConfiguration("dashboard_static_root"), "' != ''"])),
             arguments=["--source", LaunchConfiguration("gateway_source"), "--port", LaunchConfiguration("gateway_port"),
                        "--scenario", LaunchConfiguration("mission"),
                        "--static-root", LaunchConfiguration("dashboard_static_root")], output="screen"),
    ])
