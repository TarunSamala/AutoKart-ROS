"""Phase-1 ROS graph. The supervisor YAML ships with all verified gates false."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


ROOT = Path(os.environ["AUTOKART_ROOT"]).resolve()
ROS_SRC = ROOT / "kart" / "ros2_ws" / "src"
COMMISSIONING = ROS_SRC / "autokart_bringup/config/commissioning.yaml"
PERCEPTION = ROOT / "autonomy_stack_phase1" / "phase_01_perception"


def generate_launch_description():
    return LaunchDescription([
        Node(package="autokart_vehicle_bridge", executable="vehicle_bridge", name="autokart_vehicle_bridge",
             output="screen"),
        Node(package="autokart_vehicle_bridge", executable="run_logger", name="autokart_run_logger",
             parameters=[{"output_dir": str(ROOT / "results/runs")}], output="screen"),
        Node(package="autokart_localization", executable="localization_node", name="autokart_localization",
             parameters=[str(COMMISSIONING)],
             output="screen"),
        Node(package="autokart_perception", executable="perception_node", name="autokart_perception",
             parameters=[{"mode": "live", "model_path": str(ROOT / "models/yolo26s/best.onnx"),
                          "model_manifest": str(ROOT / "models/yolo26s/best.json"),
                          "camera_library": str(PERCEPTION / "native/build-x86_64-linux-gnu/libhp60c_bridge.so"),
                          "camera_configuration": str(ROOT / "autonomy_stack_phase1/vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json")}],
             output="screen"),
        Node(package="autokart_mapping", executable="mapping_node", name="autokart_mapping",
             parameters=[{"source_mode": "LIVE", "live_odom_verified": False}], output="screen"),
        Node(package="autokart_planner", executable="planner_node", name="autokart_planner",
             parameters=[str(COMMISSIONING), {"mission": "BRAKING_TEST", "source": "LIVE", "brake_min_mps2": 0.0,
                          "brake_margin_m": 1.0}], output="screen"),
        Node(package="autokart_controller", executable="controller_node", name="autokart_controller",
             parameters=[{"source": "LIVE"}], output="screen"),
        Node(package="autokart_vcu_safety", executable="vcu_safety_node", name="autokart_vcu_safety",
             parameters=[str(ROS_SRC / "autokart_vcu_safety/config/vcu_safety.yaml")], output="screen"),
        Node(package="autokart_dashboard_gateway", executable="dashboard_gateway", name="autokart_dashboard_gateway",
             arguments=["--source", "REAL_KART", "--host", "127.0.0.1", "--port", "8765"],
             output="screen"),
    ])
