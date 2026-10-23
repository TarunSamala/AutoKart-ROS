"""Integration test: real rclpy node, synthetic backend, standalone timer mode
(no publisher/camera node required)."""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from autokart_msgs.msg import ConeArray, SensorDeliveryMetadata  # noqa: E402
from autokart_perception_adapter.node import PerceptionAdapterNode  # noqa: E402


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


def test_synthetic_standalone_publishes_cones_and_status(ros_context):
    from rclpy.parameter import Parameter

    node = PerceptionAdapterNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'synthetic'),
        Parameter('input_mode', Parameter.Type.STRING, 'standalone_timer'),
        Parameter('standalone_period_s', Parameter.Type.DOUBLE, 0.05),
    ])

    received = {"cones": None, "status": None}
    sub_node = rclpy.create_node("test_subscriber_perception")
    sub_node.create_subscription(
        ConeArray, "/perception/cones", lambda m: received.__setitem__("cones", m), qos_profile_sensor_data)
    sub_node.create_subscription(SensorDeliveryMetadata, "/perception/status",
                                  lambda m: received.__setitem__("status", m), 10)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: received["cones"] is not None and received["status"] is not None)
        assert received["status"].quality == "OK"
        assert received["status"].sensor_id == "autokart_perception_adapter"
        assert len(received["cones"].cones) == 4
        labels = {c.semantic_class for c in received["cones"].cones}
        assert labels == {1, 2}  # blue, yellow per LABELS map
        assert received["cones"].ground_truth is True  # synthetic source is ground truth, not a LIVE claim
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
