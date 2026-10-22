"""Integration test: real rclpy node, real pub/sub, synthetic backend.

Requires a sourced ROS 2 Humble environment (rclpy, sensor_msgs, autokart_msgs).
Run via: colcon test --packages-select autokart_hp60c
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image, CameraInfo  # noqa: E402
from autokart_msgs.msg import SensorDeliveryMetadata  # noqa: E402

from autokart_hp60c.node import HP60CNode  # noqa: E402


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


def test_hp60c_node_publishes_rgb_depth_and_status(ros_context):
    import rclpy as _rclpy
    from rclpy.parameter import Parameter

    node = HP60CNode()
    node.set_parameters([Parameter('backend', Parameter.Type.STRING, 'synthetic')])

    received = {"rgb": None, "depth": None, "rgb_info": None, "status": None}
    sub_node = _rclpy.create_node("test_subscriber")
    sub_node.create_subscription(
        Image, "/hp60c/rgb/image_raw", lambda m: received.__setitem__("rgb", m), qos_profile_sensor_data)
    sub_node.create_subscription(
        Image, "/hp60c/depth/image_raw", lambda m: received.__setitem__("depth", m), qos_profile_sensor_data)
    sub_node.create_subscription(
        CameraInfo, "/hp60c/rgb/camera_info", lambda m: received.__setitem__("rgb_info", m), qos_profile_sensor_data)
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/hp60c/status", lambda m: received.__setitem__("status", m), 10)

    executor = _rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: received["rgb"] is not None and received["status"] is not None, timeout_s=5.0)
        assert received["rgb"].encoding == "rgb8"
        assert received["rgb"].height == 480 and received["rgb"].width == 640
        assert received["depth"].encoding == "16UC1"
        assert received["rgb_info"].k[0] > 0  # fx
        assert received["status"].sensor_id == "hp60c"
        assert received["status"].quality == "OK"
        assert received["status"].fault_state == "NONE"
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
