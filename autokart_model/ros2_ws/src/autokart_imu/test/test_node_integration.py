"""Integration test: real rclpy node, real pub/sub, synthetic backend.

Requires a sourced ROS 2 Humble environment (rclpy, sensor_msgs, autokart_msgs).
Run via: colcon test --packages-select autokart_imu
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from autokart_msgs.msg import SensorDeliveryMetadata  # noqa: E402

from autokart_imu.node import ImuNode  # noqa: E402


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_for(predicate, timeout_s=5.0, step_s=0.02):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step_s)
    return False


def test_imu_node_publishes_data_and_status_with_orientation_sentinel(ros_context):
    import rclpy as _rclpy
    from rclpy.parameter import Parameter

    node = ImuNode(parameter_overrides=[
        # Deliberately non-zero (same reasoning as autokart_hp60c's own node
        # integration test, which does not override its backend's period to
        # 0.0 either): a tight, un-throttled capture loop can starve the
        # MultiThreadedExecutor thread that is supposed to dispatch this same
        # process's subscription callbacks, losing messages. 0.01s is fast
        # enough for a 5s test timeout, slow enough to yield regularly.
        Parameter('synthetic_period_s', Parameter.Type.DOUBLE, 0.01),
        Parameter('calibration_samples', Parameter.Type.INTEGER, 5),
    ])

    received = {"imu": None, "status": None}
    sub_node = _rclpy.create_node("test_subscriber_imu")
    sub_node.create_subscription(
        Imu, "/imu/data_raw", lambda m: received.__setitem__("imu", m), qos_profile_sensor_data)
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/imu/status", lambda m: received.__setitem__("status", m), 10)

    executor = _rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: received["imu"] is not None, timeout_s=5.0), \
            "expected at least one Imu message once the calibration window completes"
        imu = received["imu"]
        assert imu.header.frame_id == "imu_link"
        # Orientation is not estimated by this node: the sensor_msgs/Imu sentinel
        # for "orientation unavailable" must be set, never a fabricated heading.
        assert imu.orientation_covariance[0] == -1.0
        # Gravity must appear on the Z accelerometer axis (stationary synthetic backend).
        assert 9.0 < imu.linear_acceleration.z < 10.6
        assert received["status"].quality == "OK"
        assert received["status"].sensor_id == "mpu6500"
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_imu_node_reports_calibrating_before_first_data(ros_context):
    import rclpy as _rclpy
    from rclpy.parameter import Parameter

    node = ImuNode(parameter_overrides=[
        Parameter('synthetic_period_s', Parameter.Type.DOUBLE, 0.0),
        Parameter('calibration_samples', Parameter.Type.INTEGER, 1_000_000),  # never completes in this test
    ])
    received = {"status": None, "imu_count": 0}
    sub_node = _rclpy.create_node("test_subscriber_calibrating")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/imu/status", lambda m: received.__setitem__("status", m), 10)
    sub_node.create_subscription(
        Imu, "/imu/data_raw", lambda m: received.__setitem__("imu_count", received["imu_count"] + 1),
        qos_profile_sensor_data)

    executor = _rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "CALIBRATING")
        time.sleep(0.2)
        assert received["imu_count"] == 0, "must not publish Imu data before the bias calibration window completes"
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
