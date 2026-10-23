"""Fault-injection tests: a missing dependency or disqualified input must produce an INVALID status
and must never crash the node or silently publish a stale/fabricated map as valid.
"""
import json
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from autokart_msgs.msg import Cone, ConeArray, MissionDebug  # noqa: E402

from autokart_mapping.node import MappingNode, ros_time  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"


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
    missing_path = str(tmp_path / "does_not_exist")
    node = MappingNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, missing_path),
    ])
    assert node.algorithm is None
    assert node.startup_error is not None and "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error

    received = []
    sub_node = rclpy.create_node("test_subscriber_mapping_fault")
    sub_node.create_subscription(
        MissionDebug, "/autokart_mapping/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    import threading
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=3.0), \
            "expected repeated fault status, not a single message followed by silence"
        for message in received:
            payload = json.loads(message.payload_json)
            assert payload["valid"] is False
            assert "DEPENDENCY_PATH_NOT_FOUND" in payload["reason_codes"][0]
            assert payload["tracked_count"] == 0
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


@pytest.mark.skipif(not REAL_ALGORITHM_PATH.is_dir(), reason="autonomy_stack_phase1 checkout not present")
def test_unqualified_cone_frame_is_rejected_not_fabricated(ros_context):
    node = MappingNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, str(REAL_ALGORITHM_PATH)),
    ])
    received = {"status": None}
    sub_node = rclpy.create_node("test_subscriber_mapping_frame_fault")
    sub_node.create_subscription(
        MissionDebug, "/autokart_mapping/status", lambda m: received.__setitem__("status", m), 10)
    odom_pub = sub_node.create_publisher(Odometry, "/autokart/odom", qos_profile_sensor_data)
    cones_pub = sub_node.create_publisher(ConeArray, "/autokart_mapping/cones_in", qos_profile_sensor_data)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    import threading
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        stamp = 1_000_000_000
        assert _wait_for(lambda: sub_node.count_subscribers("/autokart/odom") >= 1, timeout_s=3.0)
        odom = Odometry()
        odom.header.stamp = ros_time(stamp)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        time.sleep(0.1)
        array = ConeArray()
        array.measurement_time = ros_time(stamp)
        array.frame_id = "camera_optical"  # unqualified: not base_link
        item = Cone()
        item.id = 0
        item.position.x, item.position.y = 2.0, 1.5
        item.semantic_class = 1
        item.confidence = 0.9
        item.position_std_m = 0.05
        array.cones.append(item)
        cones_pub.publish(array)
        assert _wait_for(lambda: received["status"] is not None, timeout_s=5.0)
        payload = json.loads(received["status"].payload_json)
        assert payload["valid"] is False
        assert "UNQUALIFIED_CONE_FRAME" in payload["reason_codes"]
        assert payload["tracked_count"] == 0
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
