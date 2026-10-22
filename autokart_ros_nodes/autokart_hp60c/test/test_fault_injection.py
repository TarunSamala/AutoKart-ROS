"""Fault-injection tests: a failing backend must produce a fault status and
must never crash the node or silently publish stale/zero data as valid.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from autokart_msgs.msg import SensorDeliveryMetadata  # noqa: E402
from autokart_hp60c.backends import CameraBackendError  # noqa: E402
from autokart_hp60c.node import HP60CNode  # noqa: E402


class AlwaysFailsBackend:
    def __init__(self, *a, **k):
        self.started = False

    def start(self):
        self.started = True

    def read(self):
        raise CameraBackendError("CAMERA_DISCONNECTED")

    def close(self):
        self.started = False


class RaisesUnexpectedErrorBackend:
    """Simulates a backend bug that raises something other than
    CameraBackendError. The node must still report a fault and keep the
    capture thread alive, not die silently."""

    def __init__(self, *a, **k):
        self._calls = 0

    def start(self):
        pass

    def read(self):
        self._calls += 1
        raise RuntimeError("unexpected low-level failure")

    def close(self):
        pass


class FailsOnceThenRecoversBackend:
    def __init__(self, *a, **k):
        self._calls = 0

    def start(self):
        pass

    def read(self):
        self._calls += 1
        if self._calls <= 2:
            raise CameraBackendError("STALE_FRAME")
        from autokart_hp60c.backends import SyntheticBackend
        if not hasattr(self, "_fallback"):
            self._fallback = SyntheticBackend(period_s=0.0)
            self._fallback.start()
        return self._fallback.read()

    def close(self):
        pass


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


def _spin_with_backend(ros_context, backend, assertion):
    node = HP60CNode(start_capture_thread=False)
    node.backend = backend  # inject before capture starts: deterministic, no race with the default backend
    node.startup_error = None
    node.start_capture()

    received = {"status": None}
    sub_node = rclpy.create_node("test_subscriber_fault")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/hp60c/status", lambda m: received.__setitem__("status", m), 10)

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


def test_persistent_camera_disconnect_publishes_invalid_fault(ros_context):
    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID")
        assert received["status"].fault_state == "CAMERA_DISCONNECTED"
        assert received["status"].payload_present is False

    _spin_with_backend(ros_context, AlwaysFailsBackend(), check)


def test_unexpected_exception_reports_fault_and_keeps_running(ros_context):
    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID")
        assert "UNEXPECTED_BACKEND_ERROR" in received["status"].fault_state
        assert "RuntimeError" in received["status"].fault_state
        # thread must still be alive and still publishing, not dead after one exception
        time.sleep(0.3)
        assert received["status"] is not None  # still receiving, loop did not die

    _spin_with_backend(ros_context, RaisesUnexpectedErrorBackend(), check)


def test_missing_backend_keeps_reporting_fault_not_just_once(ros_context, tmp_path):
    from rclpy.parameter import Parameter
    missing_path = str(tmp_path / "does_not_exist")
    node = HP60CNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'hp60c'),
        Parameter('algorithm_package_path', Parameter.Type.STRING, missing_path),
    ])
    received = []
    sub_node = rclpy.create_node("test_subscriber_repeat_fault")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/hp60c/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=5.0), \
            "expected repeated fault status, not a single message followed by silence"
        assert all(m.quality == "INVALID" for m in received)
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_recovers_to_ok_after_transient_fault(ros_context):
    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID",
                          timeout_s=5.0), "expected the first two STALE_FRAME reads to surface as INVALID"
        assert received["status"].fault_state == "STALE_FRAME"
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "OK",
                          timeout_s=5.0), "expected recovery to OK once the backend starts returning frames"

    _spin_with_backend(ros_context, FailsOnceThenRecoversBackend(), check)


def test_node_with_missing_dependency_reports_fault_not_crash(ros_context, tmp_path):
    """backend=hp60c with a non-existent algorithm_package_path must fail closed
    at real node startup (constructor), not raise, and must keep reporting the
    fault over /hp60c/status rather than silently staying quiet."""
    from rclpy.parameter import Parameter

    missing_path = str(tmp_path / "does_not_exist")
    node = HP60CNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'hp60c'),
        Parameter('algorithm_package_path', Parameter.Type.STRING, missing_path),
    ])
    assert node.backend is None
    assert node.startup_error is not None and "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error

    received = {"status": None}
    sub_node = rclpy.create_node("test_subscriber_dep")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/hp60c/status", lambda m: received.__setitem__("status", m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: received["status"] is not None)
        assert received["status"].quality == "INVALID"
        assert "DEPENDENCY_PATH_NOT_FOUND" in received["status"].fault_state
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
