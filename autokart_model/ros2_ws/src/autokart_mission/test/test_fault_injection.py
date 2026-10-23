"""Fault-injection tests: a missing dependency or malformed input must produce an INVALID status and
must never crash the node or silently publish a fabricated mission decision as valid.
"""
import json
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter  # noqa: E402
from autokart_msgs.msg import MissionDebug  # noqa: E402

from autokart_mission.node import MissionNode, ros_time  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"
VEHICLE_GEOMETRY_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_vehicle_geometry"
PLANNER_PATH = REPO_ROOT / "kart" / "ros2_ws" / "src" / "autokart_nodes" / "autokart_planner"
ALL_PRESENT = ALGORITHM_PATH.is_dir() and VEHICLE_GEOMETRY_PATH.is_dir() and PLANNER_PATH.is_dir()


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_for(predicate, timeout_s=5.0, step_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step_s)
    return False


def test_missing_dependency_keeps_reporting_fault_not_just_once(ros_context, tmp_path):
    missing = str(tmp_path / "does_not_exist")
    node = MissionNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, missing),
        Parameter('vehicle_geometry_package_path', Parameter.Type.STRING, missing),
        Parameter('planner_package_path', Parameter.Type.STRING, missing),
    ])
    assert node.algorithm is None
    assert node.startup_error is not None and "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error

    received = []
    sub_node = rclpy.create_node("test_subscriber_mission_fault")
    sub_node.create_subscription(
        MissionDebug, "/autokart_mission/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=3.0), \
            "expected repeated fault status, not a single message followed by silence"
        for message in received:
            payload = json.loads(message.payload_json)
            assert payload["valid"] is False
            assert "DEPENDENCY_PATH_NOT_FOUND" in payload["reason_codes"][0]
            assert payload["vehicle_authorized"] is False
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


@pytest.mark.skipif(not ALL_PRESENT, reason="full kart/autonomy_stack_phase1 checkout not present")
def test_malformed_mapping_status_is_rejected_not_fabricated(ros_context):
    node = MissionNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, str(ALGORITHM_PATH)),
        Parameter('vehicle_geometry_package_path', Parameter.Type.STRING, str(VEHICLE_GEOMETRY_PATH)),
        Parameter('planner_package_path', Parameter.Type.STRING, str(PLANNER_PATH)),
    ])
    assert node.algorithm is not None, node.startup_error

    received = {"status": None}
    sub_node = rclpy.create_node("test_subscriber_mission_malformed")
    sub_node.create_subscription(
        MissionDebug, "/autokart_mission/status", lambda m: received.__setitem__("status", m), 10)
    mapping_pub = sub_node.create_publisher(MissionDebug, "/autokart_mapping/status", 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        stamp = node.get_clock().now().nanoseconds
        message = MissionDebug()
        message.measurement_time = ros_time(stamp)
        message.frame_id = "odom"
        message.stage = "MAPPING"
        message.payload_json = "{not valid json"
        mapping_pub.publish(message)
        assert _wait_for(lambda: received["status"] is not None, timeout_s=5.0)
        payload = json.loads(received["status"].payload_json)
        assert payload["valid"] is False
        assert "MAPPING_STATUS_MALFORMED" in payload["reason_codes"]
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
