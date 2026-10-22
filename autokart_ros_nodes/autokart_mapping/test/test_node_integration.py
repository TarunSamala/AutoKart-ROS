"""Integration test: real rclpy node, real pub/sub, real reused algorithm.

Requires a sourced ROS 2 Humble environment (rclpy, nav_msgs, autokart_msgs) and the
autonomy_stack_phase1 checkout (the external algorithm_package_path dependency; see DEPENDENCIES.md).
Run via: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
"""
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from autokart_msgs.msg import Cone, ConeArray, MissionDebug  # noqa: E402

from autokart_mapping.node import MappingNode, ros_time  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"

pytestmark = pytest.mark.skipif(
    not REAL_ALGORITHM_PATH.is_dir(),
    reason="autonomy_stack_phase1 checkout not present in this environment")


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


def _odom(stamp, x=1.0, y=2.0):
    message = Odometry()
    message.header.stamp = ros_time(stamp)
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = message.pose.covariance[7] = 0.0025
    return message


def _cone_array(stamp):
    array = ConeArray()
    array.measurement_time = ros_time(stamp)
    array.frame_id = "base_link"
    for index, (x, y, klass) in enumerate(((2, 1.5, 1), (2, -1.5, 2), (5, 1.5, 1),
                                           (5, -1.5, 2), (8, 1.5, 1), (8, -1.5, 2))):
        item = Cone()
        item.id = index
        item.position.x, item.position.y = float(x), float(y)
        item.semantic_class = klass
        item.confidence = 0.95
        item.position_std_m = 0.05
        array.cones.append(item)
    return array


def test_mapping_node_publishes_observed_map_and_status(ros_context):
    node = MappingNode(parameter_overrides=[
        Parameter('algorithm_package_path', Parameter.Type.STRING, str(REAL_ALGORITHM_PATH)),
    ])
    assert node.algorithm is not None, node.startup_error

    received = {"map": None, "status": None}
    sub_node = rclpy.create_node("test_subscriber")
    sub_node.create_subscription(
        ConeArray, "/autokart_mapping/cones", lambda m: received.__setitem__("map", m),
        qos_profile_sensor_data)
    sub_node.create_subscription(
        MissionDebug, "/autokart_mapping/status", lambda m: received.__setitem__("status", m), 10)
    odom_pub = sub_node.create_publisher(Odometry, "/autokart/odom", qos_profile_sensor_data)
    cones_pub = sub_node.create_publisher(ConeArray, "/autokart_mapping/cones_in", qos_profile_sensor_data)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        stamp = 1_000_000_000
        assert _wait_for(lambda: sub_node.count_subscribers("/autokart/odom") >= 1, timeout_s=3.0)
        odom_pub.publish(_odom(stamp, x=0.0, y=0.0))
        time.sleep(0.1)
        cones_pub.publish(_cone_array(stamp))
        assert _wait_for(lambda: received["status"] is not None, timeout_s=5.0)
        import json
        payload = json.loads(received["status"].payload_json)
        assert payload["valid"] is True
        assert payload["tracked_count"] == 6
        assert payload["geometry"]["skidpad_topology"] is not None
        assert payload["vehicle_authorized"] is False
        assert received["map"] is not None
        assert len(received["map"].cones) == 6
        assert received["map"].frame_id == "odom"
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
