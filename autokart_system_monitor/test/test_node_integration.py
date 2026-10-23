"""Integration test: real rclpy node, real dynamic topic discovery, real
diagnostic_msgs/DiagnosticArray publication. Requires a sourced ROS 2 Humble
environment (rclpy, diagnostic_msgs, std_msgs, rosidl_runtime_py).
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from autokart_system_monitor.node import SystemMonitorNode  # noqa: E402
from autokart_system_monitor.monitor_core import OK, STALE  # noqa: E402


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


def _levels_by_name(msg: DiagnosticArray) -> dict:
    return {s.name: s.level[0] if isinstance(s.level, (bytes, bytearray)) else s.level for s in msg.status}


def test_monitor_works_standalone_with_no_dependencies_running(ros_context):
    """Nothing publishes on either configured heartbeat topic at all -- the
    node must still build, launch, and report (never-seen/STALE), not
    error or hang."""
    from rclpy.parameter import Parameter
    node = SystemMonitorNode(parameter_overrides=[
        Parameter('heartbeat_topics', Parameter.Type.STRING_ARRAY, ['/test/never_published_a', '/test/never_published_b']),
        Parameter('resource_backend', Parameter.Type.STRING, 'none'),
        Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 10.0),
        Parameter('topic_discovery_period_s', Parameter.Type.DOUBLE, 0.1),
    ])
    received = []
    sub_node = rclpy.create_node("test_sub_health")
    sub_node.create_subscription(DiagnosticArray, "/system/health", lambda m: received.append(m), 10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert _wait_for(lambda: len(received) >= 2, timeout_s=3.0)
        levels = _levels_by_name(received[-1])
        assert all(level == STALE for name, level in levels.items() if name != "autokart_system_monitor: resources")
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()


def test_a_dependency_that_starts_then_stops_publishing_is_tracked_correctly(ros_context):
    """The exact scenario the mission called out: a dependency node that
    stops publishing must show up as stale, not silently look fine, and
    one that was never running must not block the other key from being
    tracked independently."""
    from rclpy.parameter import Parameter
    node = SystemMonitorNode(parameter_overrides=[
        Parameter('heartbeat_topics', Parameter.Type.STRING_ARRAY, ['/test/dep_a', '/test/dep_b']),
        Parameter('resource_backend', Parameter.Type.STRING, 'none'),
        Parameter('heartbeat_timeout_s', Parameter.Type.DOUBLE, 0.3),
        Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 10.0),
        Parameter('topic_discovery_period_s', Parameter.Type.DOUBLE, 0.1),
    ])
    received = []
    sub_node = rclpy.create_node("test_sub_health2")
    sub_node.create_subscription(DiagnosticArray, "/system/health", lambda m: received.append(m), 10)
    pub_node = rclpy.create_node("test_dep_a_publisher")
    dep_a_pub = pub_node.create_publisher(String, "/test/dep_a", 10)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    executor.add_node(pub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        stop_publishing = threading.Event()

        def publish_loop():
            while not stop_publishing.is_set():
                dep_a_pub.publish(String(data="heartbeat"))
                time.sleep(0.05)

        publisher_thread = threading.Thread(target=publish_loop, daemon=True)
        publisher_thread.start()

        def dep_a_is_ok():
            if not received:
                return False
            levels = _levels_by_name(received[-1])
            return levels.get("autokart_system_monitor: /test/dep_a") == OK

        assert _wait_for(dep_a_is_ok, timeout_s=3.0), "dep_a should become OK once it starts publishing"
        levels = _levels_by_name(received[-1])
        assert levels.get("autokart_system_monitor: /test/dep_b") == STALE, "dep_b never published -- must stay stale"

        # Now stop dep_a and confirm it goes back to STALE, not frozen OK.
        stop_publishing.set()
        publisher_thread.join(timeout=1.0)

        def dep_a_is_stale_again():
            if not received:
                return False
            levels = _levels_by_name(received[-1])
            return levels.get("autokart_system_monitor: /test/dep_a") == STALE

        assert _wait_for(dep_a_is_stale_again, timeout_s=3.0), \
            "dep_a stopped publishing -- must show stale again, not silently look fine"
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
        pub_node.destroy_node()


def test_best_effort_publisher_is_detected_not_falsely_reported_stale(ros_context):
    """Regression test for a real defect found in review: heartbeat
    subscriptions were created with default RELIABLE QoS. A publisher using
    BEST_EFFORT (sensor-data QoS -- e.g. /hp60c/rgb/image_raw) is
    QoS-incompatible with a reliable subscriber, so its messages were never
    delivered and a perfectly healthy dependency was reported STALE
    forever. Pre-fix this test fails (dep stays STALE)."""
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    node = SystemMonitorNode(parameter_overrides=[
        Parameter('heartbeat_topics', Parameter.Type.STRING_ARRAY, ['/test/best_effort_dep']),
        Parameter('resource_backend', Parameter.Type.STRING, 'none'),
        Parameter('heartbeat_timeout_s', Parameter.Type.DOUBLE, 0.5),
        Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 10.0),
        Parameter('topic_discovery_period_s', Parameter.Type.DOUBLE, 0.1),
    ])
    received = []
    sub_node = rclpy.create_node("test_sub_best_effort")
    sub_node.create_subscription(DiagnosticArray, "/system/health", lambda m: received.append(m), 10)
    pub_node = rclpy.create_node("test_best_effort_publisher")
    pub = pub_node.create_publisher(String, "/test/best_effort_dep", qos_profile_sensor_data)
    executor = rclpy.executors.MultiThreadedExecutor()
    for n in (node, sub_node, pub_node):
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    stop = threading.Event()

    def publish_loop():
        while not stop.is_set():
            pub.publish(String(data="hb"))
            time.sleep(0.05)

    publisher_thread = threading.Thread(target=publish_loop, daemon=True)
    publisher_thread.start()
    try:
        def ok():
            return bool(received) and _levels_by_name(received[-1]).get(
                "autokart_system_monitor: /test/best_effort_dep") == OK
        assert _wait_for(ok, timeout_s=4.0), \
            "healthy best-effort publisher must be seen, not reported STALE due to QoS mismatch"
    finally:
        stop.set()
        publisher_thread.join(timeout=1.0)
        executor.shutdown()
        for n in (node, sub_node, pub_node):
            n.destroy_node()


def test_startup_status_latches_even_after_dependency_goes_stale(ros_context):
    from rclpy.parameter import Parameter
    node = SystemMonitorNode(parameter_overrides=[
        Parameter('heartbeat_topics', Parameter.Type.STRING_ARRAY, ['/test/dep_c']),
        Parameter('resource_backend', Parameter.Type.STRING, 'none'),
        Parameter('heartbeat_timeout_s', Parameter.Type.DOUBLE, 0.2),
        Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 10.0),
        Parameter('topic_discovery_period_s', Parameter.Type.DOUBLE, 0.1),
    ])
    received = []
    sub_node = rclpy.create_node("test_sub_startup")
    sub_node.create_subscription(DiagnosticArray, "/system/startup_status", lambda m: received.append(m), 10)
    pub_node = rclpy.create_node("test_dep_c_publisher")
    dep_c_pub = pub_node.create_publisher(String, "/test/dep_c", 10)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)
    executor.add_node(pub_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        for _ in range(5):
            dep_c_pub.publish(String(data="heartbeat"))
            time.sleep(0.05)

        def latched_ok():
            if not received:
                return False
            levels = _levels_by_name(received[-1])
            return levels.get("autokart_system_monitor: /test/dep_c") == OK

        assert _wait_for(latched_ok, timeout_s=3.0)
        # Now stop publishing and wait past the timeout -- /system/health
        # would go stale, but /system/startup_status must stay latched OK.
        time.sleep(0.6)
        levels = _levels_by_name(received[-1])
        assert levels.get("autokart_system_monitor: /test/dep_c") == OK, \
            "startup_status must never un-latch once a dependency has been seen at least once"
    finally:
        executor.shutdown()
        node.destroy_node()
        sub_node.destroy_node()
        pub_node.destroy_node()
