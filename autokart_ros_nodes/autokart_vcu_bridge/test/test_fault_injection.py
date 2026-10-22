"""Fault-injection tests: a failing/misconfigured backend, or a missing
core.py dependency, must produce a fault status and must never crash the
node, silently send a fabricated throttle value, or go silent.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from autokart_msgs.msg import SensorDeliveryMetadata, VcuState  # noqa: E402
from autokart_vcu_bridge.backends import VcuBackendError  # noqa: E402
from autokart_vcu_bridge.node import VcuBridgeNode  # noqa: E402


class AlwaysFailsPollBackend:
    def __init__(self, *a, **k):
        pass

    def start(self):
        pass

    def send_command(self, seq, throttle_mv, brake_pct):
        pass

    def send_request(self, seq, kind):
        pass

    def poll(self):
        raise VcuBackendError("SERIAL_READ_FAILED")

    def close(self):
        pass


class RaisesUnexpectedErrorBackend:
    def __init__(self, *a, **k):
        pass

    def start(self):
        pass

    def send_command(self, seq, throttle_mv, brake_pct):
        pass

    def send_request(self, seq, kind):
        pass

    def poll(self):
        raise RuntimeError("unexpected low-level failure")

    def close(self):
        pass


class RecordingBackend:
    def __init__(self, fail_requests=False):
        self.requests = []
        self.closed = False
        self.fail_requests = fail_requests

    def start(self):
        pass

    def send_command(self, seq, throttle_mv, brake_pct):
        pass

    def send_request(self, seq, kind):
        if self.fail_requests:
            raise VcuBackendError("SERIAL_WRITE_FAILED")
        self.requests.append(kind)

    def poll(self):
        return []

    def close(self):
        self.closed = True


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


def _spin_with_backend(backend, assertion, tick_hz=20.0):
    from rclpy.parameter import Parameter
    node = VcuBridgeNode(start_timer=False, parameter_overrides=[
        Parameter('tick_hz', Parameter.Type.DOUBLE, tick_hz),
    ])
    node.backend = backend
    node.startup_error = None
    node.start_timer()

    received = {"status": None, "state": None}
    sub_node = rclpy.create_node("test_subscriber_vcu_fault")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/vcu_bridge/status", lambda m: received.__setitem__("status", m), 10)
    sub_node.create_subscription(VcuState, "/vcu/state", lambda m: received.__setitem__("state", m), 10)

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


def test_persistent_poll_failure_publishes_invalid_fault_and_safe_vcu_state(ros_context):
    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID")
        assert received["status"].fault_state == "SERIAL_READ_FAILED"
        assert received["status"].payload_present is False
        # No fresh telemetry was ever obtained: VcuState must say so, never a
        # fabricated ARMED/SAFE guess.
        assert _wait_for(lambda: received["state"] is not None and received["state"].state == "UNKNOWN")
        assert received["state"].limiting_reason == "NO_VCU_TELEMETRY"

    _spin_with_backend(AlwaysFailsPollBackend(), check)


def test_unexpected_exception_reports_fault_and_keeps_running(ros_context):
    def check(received):
        assert _wait_for(lambda: received["status"] is not None and received["status"].quality == "INVALID")
        assert "UNEXPECTED_BACKEND_ERROR" in received["status"].fault_state
        assert "RuntimeError" in received["status"].fault_state
        time.sleep(0.2)
        assert received["status"] is not None  # timer kept ticking and publishing, did not die

    _spin_with_backend(RaisesUnexpectedErrorBackend(), check)


def test_missing_backend_keeps_reporting_fault_not_just_once(ros_context, tmp_path):
    """backend=serial with a non-existent algorithm_package_path must fail
    closed at real node startup (constructor), not raise, and must keep
    reporting the fault over /vcu_bridge/status rather than going silent."""
    from rclpy.parameter import Parameter
    missing_path = str(tmp_path / "does_not_exist")
    node = VcuBridgeNode(parameter_overrides=[
        Parameter('backend', Parameter.Type.STRING, 'serial'),
        Parameter('algorithm_package_path', Parameter.Type.STRING, missing_path),
        Parameter('tick_hz', Parameter.Type.DOUBLE, 20.0),
    ])
    assert node.backend is None
    assert node.startup_error is not None and "DEPENDENCY_PATH_NOT_FOUND" in node.startup_error

    received = []
    sub_node = rclpy.create_node("test_subscriber_vcu_dep")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/vcu_bridge/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=5.0), \
            "expected repeated fault status, not a single message followed by silence"
        assert all(m.quality == "INVALID" for m in received)
        assert all("DEPENDENCY_PATH_NOT_FOUND" in m.fault_state for m in received)
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)  # let any in-flight callback finish before destroy
        node.destroy_node()
        sub_node.destroy_node()


def test_missing_core_dependency_falls_back_to_safe_zero_output_and_reports_it(ros_context):
    """With the default (empty) algorithm_package_path, core.py cannot be
    imported: the node must still run (synthetic backend works fine), but
    must report CORE_LOGIC_UNAVAILABLE on every status message -- never
    silently behave as if the real control law were active."""
    node = VcuBridgeNode(start_timer=False)
    assert node._core is None
    assert node._core_error == "CORE_LOGIC_PATH_NOT_CONFIGURED"
    node.start_timer()

    received = []
    sub_node = rclpy.create_node("test_subscriber_vcu_core_missing")
    sub_node.create_subscription(
        SensorDeliveryMetadata, "/vcu_bridge/status", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=5.0)
        assert all(m.quality == "INVALID" for m in received)
        assert all("CORE_LOGIC_UNAVAILABLE" in m.fault_state for m in received)
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)  # let any in-flight callback finish before destroy
        node.destroy_node()
        sub_node.destroy_node()


def test_shutdown_sends_best_effort_disarm(ros_context):
    """Regression for the senior-review finding: destroying the node must try
    to DIS the VCU before closing the port (the reference
    autokart_vehicle_bridge node does this in its main())."""
    node = VcuBridgeNode(start_timer=False)
    backend = RecordingBackend()
    node.backend = backend
    node.destroy_node()
    assert backend.requests == ["DIS"]
    assert backend.closed


def test_shutdown_disarm_failure_does_not_raise_and_still_closes(ros_context):
    node = VcuBridgeNode(start_timer=False)
    backend = RecordingBackend(fail_requests=True)
    node.backend = backend
    node.destroy_node()  # must not raise
    assert backend.closed


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
        [sys.executable, "-c", "from autokart_vcu_bridge.node import main; main()"],
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
