"""Fault-injection tests: psutil missing must degrade gracefully (never
crash node construction or the publish loop); an invalid resource_backend
name is a configuration error and must fail loudly at construction instead
(a typo should be visible to the operator, not silently swallowed)."""
import builtins
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_system_monitor.resource_backends import ResourceBackendError  # noqa: E402
from autokart_system_monitor.node import SystemMonitorNode  # noqa: E402


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


def test_node_still_publishes_when_psutil_is_unavailable(ros_context, monkeypatch):
    real_import = builtins.__import__

    def fake_import(name, *args, **kwargs):
        if name == "psutil":
            raise ImportError("simulated: psutil not installed")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)

    from rclpy.parameter import Parameter
    node = SystemMonitorNode(parameter_overrides=[
        Parameter('heartbeat_topics', Parameter.Type.STRING_ARRAY, ['/test/unused_topic']),
        Parameter('resource_backend', Parameter.Type.STRING, 'psutil'),
        Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 10.0),
    ])
    received = []
    sub_node = rclpy.create_node("test_sub_no_psutil")
    sub_node.create_subscription(DiagnosticArray, "/system/health", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 1, timeout_s=3.0)
        resources = [s for s in received[-1].status if s.name.endswith("resources")][0]
        values = {kv.key: kv.value for kv in resources.values}
        assert values["cpu_percent"] == "null" and values["ram_percent"] == "null"
        assert "PSUTIL_IMPORT_FAILED" in resources.message
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_invalid_resource_backend_name_fails_loudly_at_construction(ros_context):
    """A typo in config (not a transient missing-dependency condition) must
    be visible immediately, not silently treated as 'unavailable'."""
    from rclpy.parameter import Parameter
    with pytest.raises(ResourceBackendError):
        SystemMonitorNode(parameter_overrides=[
            Parameter('resource_backend', Parameter.Type.STRING, 'not-a-real-backend'),
        ])
