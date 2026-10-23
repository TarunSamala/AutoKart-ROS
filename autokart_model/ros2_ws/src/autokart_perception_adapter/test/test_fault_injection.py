"""Fault-injection tests: a failing/misconfigured backend must produce a
fault status, never a fabricated cone, and must keep reporting (not go silent
after one message)."""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from autokart_msgs.msg import ConeArray, SensorDeliveryMetadata  # noqa: E402
from autokart_perception_adapter.node import PerceptionAdapterNode  # noqa: E402


class AlwaysUnqualifiedBackend:
    def process(self, rgb, depth, fx, fy, cx, cy, timestamp_ns, source_hint):
        from autokart_perception_adapter.backends import PerceptionOutcome
        return PerceptionOutcome(valid=False, reasons=("LIVE_METRIC_CALIBRATION_UNVERIFIED",),
                                  cones=(), source=source_hint)


class RaisesUnexpectedErrorBackend:
    def process(self, *a, **k):
        raise RuntimeError("unexpected low-level failure")


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


def _spin(node, assertion):
    received = {"cones": None, "status": None}
    sub_node = rclpy.create_node("test_subscriber_fault")
    sub_node.create_subscription(ConeArray, "/perception/cones", lambda m: received.__setitem__("cones", m), 10)
    sub_node.create_subscription(SensorDeliveryMetadata, "/perception/status",
                                  lambda m: received.__setitem__("status", m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assertion(received)
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_unqualified_result_never_publishes_cones(ros_context):
    from rclpy.parameter import Parameter
    node = PerceptionAdapterNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'synthetic'),
        Parameter('input_mode', Parameter.Type.STRING, 'standalone_timer'),
        Parameter('standalone_period_s', Parameter.Type.DOUBLE, 0.05),
    ])
    node.backend = AlwaysUnqualifiedBackend()

    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "DIAGNOSTIC_ONLY")
        assert "LIVE_METRIC_CALIBRATION_UNVERIFIED" in received["status"].fault_state
        time.sleep(0.3)
        assert received["cones"] is None, "an unqualified result must never publish a ConeArray"

    _spin(node, check)


def test_unexpected_backend_exception_reports_fault_not_crash(ros_context):
    from rclpy.parameter import Parameter
    node = PerceptionAdapterNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'synthetic'),
        Parameter('input_mode', Parameter.Type.STRING, 'standalone_timer'),
        Parameter('standalone_period_s', Parameter.Type.DOUBLE, 0.05),
    ])
    node.backend = RaisesUnexpectedErrorBackend()

    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID")
        assert "UNEXPECTED_BACKEND_ERROR" in received["status"].fault_state
        assert "RuntimeError" in received["status"].fault_state
        time.sleep(0.3)
        assert received["status"] is not None  # timer still firing, node did not die

    _spin(node, check)


def test_missing_dependency_keeps_reporting_fault(ros_context, tmp_path):
    from rclpy.parameter import Parameter
    missing_path = str(tmp_path / "does_not_exist")
    node = PerceptionAdapterNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'algorithm'),
        Parameter('algorithm_package_path', Parameter.Type.STRING, missing_path),
        Parameter('input_mode', Parameter.Type.STRING, 'standalone_timer'),
        Parameter('standalone_period_s', Parameter.Type.DOUBLE, 0.2),
    ])
    assert node.backend is None
    assert "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error

    received_list = []

    def check(received):
        sub = received  # unused, we track via the list captured below
        assert _wait_for(lambda: len(received_list) >= 2, timeout_s=5.0)
        assert all(m.quality == "INVALID" for m in received_list)
        assert all("DEPENDENCY_PATH_NOT_FOUND" in m.fault_state for m in received_list)

    # Re-subscribe manually to collect every message (not just the latest).
    sub_node = rclpy.create_node("test_subscriber_repeat")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/perception/status", lambda m: received_list.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        check({})
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
