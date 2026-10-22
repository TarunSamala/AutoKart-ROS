"""Fault-injection tests: a failing/misbehaving backend must produce a fault
status and must never crash the node, publish a fabricated sample, or go
silent.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from autokart_msgs.msg import SensorDeliveryMetadata  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from autokart_imu.backends import ImuBackendError, RawSample  # noqa: E402
from autokart_imu.node import ImuNode  # noqa: E402


class AlwaysFailsBackend:
    def __init__(self, *a, **k):
        pass

    def start(self):
        pass

    def read(self):
        raise ImuBackendError("I2C_READ_FAILED")

    def close(self):
        pass


class RaisesUnexpectedErrorBackend:
    def __init__(self, *a, **k):
        pass

    def start(self):
        pass

    def read(self):
        raise RuntimeError("unexpected low-level failure")

    def close(self):
        pass


class NonMonotonicThenGoodBackend:
    """First sample ok, second sample's timestamp goes backwards, then
    resumes forward -- the node must drop exactly the bad one and recover."""

    def __init__(self, *a, **k):
        self._seq = 0
        self._t = 1_000_000_000

    def start(self):
        pass

    def read(self):
        # A small sleep, same as a real sensor's sample period: an
        # un-throttled fake backend can starve the ROS executor thread that
        # must dispatch this same test's subscription callbacks (discovered
        # via this exact test; see SENIOR_REVIEWER_REPORT.md).
        time.sleep(0.01)
        self._seq += 1
        if self._seq == 2:
            t = self._t - 500_000_000  # goes backwards
        else:
            self._t += 5_000_000
            t = self._t
        return RawSample(timestamp_ns=t, sequence=self._seq, gyro_radps=(0.0, 0.0, 0.01),
                          accel_mps2=(0.0, 0.0, 9.81), saturated=False, source="FAKE")

    def close(self):
        pass


class SaturatedThenGoodBackend:
    def __init__(self, *a, **k):
        self._seq = 0
        self._t = 0

    def start(self):
        pass

    def read(self):
        time.sleep(0.01)
        self._seq += 1
        self._t += 5_000_000
        saturated = self._seq <= 2
        return RawSample(timestamp_ns=self._t, sequence=self._seq, gyro_radps=(0.0, 0.0, 0.01),
                          accel_mps2=(0.0, 0.0, 9.81), saturated=saturated, source="FAKE")

    def close(self):
        pass


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


def _spin_with_backend(backend, assertion, extra_subs=None):
    node = ImuNode(start_capture_thread=False)
    node.backend = backend
    node.startup_error = None
    node.start_capture()

    # "status_history" records every message (not just the latest) so a test
    # can reliably check "did fault X ever occur", independent of how fast the
    # fake backend races ahead of the subscriber callback between polls.
    received = {"status": None, "status_history": [], "imu_count": 0}
    sub_node = rclpy.create_node("test_subscriber_imu_fault")

    def _on_status(m):
        received["status"] = m
        received["status_history"].append(m)

    sub_node.create_subscription(SensorDeliveryMetadata, "/imu/status", _on_status, 10)
    sub_node.create_subscription(
        Imu, "/imu/data_raw", lambda m: received.__setitem__("imu_count", received["imu_count"] + 1),
        qos_profile_sensor_data)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assertion(received)
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)  # let any in-flight callback finish before destroy
        node.destroy_node()
        sub_node.destroy_node()


def test_persistent_read_failure_publishes_invalid_fault(ros_context):
    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID")
        assert received["status"].fault_state == "I2C_READ_FAILED"
        assert received["status"].payload_present is False

    _spin_with_backend(AlwaysFailsBackend(), check)


def test_unexpected_exception_reports_fault_and_keeps_running(ros_context):
    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID")
        assert "UNEXPECTED_BACKEND_ERROR" in received["status"].fault_state
        assert "RuntimeError" in received["status"].fault_state
        time.sleep(0.2)
        assert received["status"] is not None  # thread kept publishing, did not die

    _spin_with_backend(RaisesUnexpectedErrorBackend(), check)


def test_nonmonotonic_timestamp_is_dropped_and_node_recovers(ros_context):
    def check(received):
        assert _wait_for(lambda: any(s.fault_state == "NON_MONOTONIC_TIMESTAMP"
                                      for s in received["status_history"]), timeout_s=5.0)
        # Node must keep running and eventually return to OK once good samples resume.
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "OK",
                          timeout_s=5.0)

    _spin_with_backend(NonMonotonicThenGoodBackend(), check)


def test_saturated_sample_is_dropped_never_published_as_data(ros_context):
    def check(received):
        assert _wait_for(lambda: any(s.fault_state == "SATURATED_SAMPLE"
                                      for s in received["status_history"]), timeout_s=5.0)
        # Every status published while a sample was saturated must say so
        # (quality=INVALID, payload_present=False) -- never OK with no
        # corresponding real sample, i.e. a saturated reading is never
        # silently waved through as valid Imu data.
        for s in received["status_history"]:
            if s.fault_state == "SATURATED_SAMPLE":
                assert s.quality == "INVALID" and s.payload_present is False
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "OK",
                          timeout_s=5.0)

    _spin_with_backend(SaturatedThenGoodBackend(), check)


def test_missing_backend_keeps_reporting_fault_not_just_once(ros_context, tmp_path):
    """backend=mpu6500 with no smbus2 installed (true in this environment) must
    fail closed at construction and keep reporting, not go silent after one
    status message."""
    from rclpy.parameter import Parameter
    node = ImuNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'mpu6500'),
    ])
    assert node.backend is None
    assert node.startup_error is not None and "DEPENDENCY_UNAVAILABLE" in node.startup_error

    received = []
    sub_node = rclpy.create_node("test_subscriber_imu_dep")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/imu/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=5.0), \
            "expected repeated fault status, not a single message followed by silence"
        assert all(m.quality == "INVALID" for m in received)
        assert all("DEPENDENCY_UNAVAILABLE" in m.fault_state for m in received)
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)  # let any in-flight callback finish before destroy
        node.destroy_node()
        sub_node.destroy_node()


def test_sigint_shutdown_exits_cleanly_without_rclerror():
    """Regression: the real entry point, run as its own process and stopped
    with SIGINT (Ctrl-C), must exit 0 without an RCLError traceback. Before
    the fix, main() called rclpy.shutdown() after Humble's SIGINT handler had
    already shut the context down. Does NOT use the ros_context fixture --
    the child process owns its own rclpy context."""
    import os
    import signal
    import subprocess
    import sys
    from pathlib import Path
    pkg_root = Path(__file__).resolve().parent.parent
    env = dict(os.environ)
    env["PYTHONPATH"] = str(pkg_root) + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.Popen(
        [sys.executable, "-c", "from autokart_imu.node import main; main()"],
        cwd=str(pkg_root), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        time.sleep(2.5)
        assert proc.poll() is None, "node exited before SIGINT: " + proc.stderr.read()
        proc.send_signal(signal.SIGINT)
        out, err = proc.communicate(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
    assert "RCLError" not in err, err
    assert "Traceback" not in err, err
    assert proc.returncode == 0, (proc.returncode, err)
