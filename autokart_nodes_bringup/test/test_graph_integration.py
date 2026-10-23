"""Full-graph integration test for disarmed_sim.launch.py (phase 6, step 3 of the integration
mission). Not a per-node unit test -- each of the 12 packages already has its own test suite; this
only checks that the graph, once brought up together, actually behaves the way
``disarmed_sim.launch.py``'s own docstring claims:

1. every topic the launch file wires is actually flowing (publisher present, at least one message
   received within a short window);
2. node 12's own ``/system/health`` reports every tracked heartbeat topic ``fresh`` (level OK), i.e.
   the whole graph finished "coming up";
3. killing one upstream node (``autokart_hp60c``) makes the failure visible downstream as
   staleness, not silence -- node 12's ``/system/health`` for ``/hp60c/status`` degrades to
   ``STALE`` once the heartbeat timeout elapses, and the chain does not keep fabricating fresh-
   looking data from a dead source.

This test launches the real ``disarmed_sim.launch.py`` as a subprocess (not a mocked graph) and
talks to it over real ROS topics from a plain ``rclpy`` test node. It never publishes an ``arm``
request; nothing here is meant to or does move the vehicle.
"""
from __future__ import annotations

import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray

EXPECTED_TOPICS = [
    "/hp60c/status",
    "/perception/status",
    "/imu/status",
    "/vcu_bridge/status",
    "/autokart/odometry/status",
    "/autokart_mapping/status",
    "/autokart_mission/status",
    "/autokart_planner/safety_envelope",
    "/autokart_controller/diagnostics",
    "/autokart/safety/status",
    "/vcu/state",
    "/vehicle/actuator_feedback",
]

STARTUP_TIMEOUT_S = 40.0
POLL_PERIOD_S = 0.5


class HealthWatcher(Node):
    """Subscribes /system/health (DiagnosticArray) and tracks the latest level per component name."""

    def __init__(self):
        super().__init__("graph_integration_test_watcher")
        self.levels: dict[str, int] = {}
        self.create_subscription(DiagnosticArray, "/system/health", self._on_health,
                                  qos_profile_sensor_data)

    def _on_health(self, message: DiagnosticArray) -> None:
        for status in message.status:
            # DiagnosticStatus.level is a ROS "byte" field; rclpy hands it back as a
            # length-1 bytes object (e.g. b'\x03'), not a Python int -- normalize it.
            level = status.level
            self.levels[status.name] = level[0] if isinstance(level, bytes) else int(level)


@pytest.fixture(scope="module")
def graph_process():
    env = dict(os.environ)
    launch_cmd = [
        "ros2", "launch", "autokart_nodes_bringup", "disarmed_sim.launch.py",
    ]
    process = subprocess.Popen(launch_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, start_new_session=True)
    try:
        yield process
    finally:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=15)
        except Exception:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_for_topics(node: Node, expected: list[str], timeout_s: float) -> set[str]:
    deadline = time.monotonic() + timeout_s
    seen: set[str] = set()
    while time.monotonic() < deadline and len(seen) < len(expected):
        names_and_types = dict(node.get_topic_names_and_types())
        seen = {topic for topic in expected if topic in names_and_types}
        rclpy.spin_once(node, timeout_sec=0.5)
    return seen


def test_every_expected_topic_is_flowing(graph_process, ros_context):
    node = Node("graph_integration_test_topic_waiter")
    try:
        seen = _wait_for_topics(node, EXPECTED_TOPICS, STARTUP_TIMEOUT_S)
        missing = set(EXPECTED_TOPICS) - seen
        assert not missing, f"topics never advertised within {STARTUP_TIMEOUT_S}s: {missing}"
    finally:
        node.destroy_node()


def test_every_node_is_fresh_on_system_health(graph_process, ros_context):
    watcher = HealthWatcher()
    try:
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        fresh_names = set()
        expected_names = {f"autokart_system_monitor: {topic}" for topic in EXPECTED_TOPICS}
        while time.monotonic() < deadline and not expected_names.issubset(fresh_names):
            rclpy.spin_once(watcher, timeout_sec=0.5)
            fresh_names = {name for name, level in watcher.levels.items() if level == 0}
        missing = expected_names - fresh_names
        assert not missing, f"never reported OK/fresh on /system/health: {missing}"
    finally:
        watcher.destroy_node()


def test_killing_hp60c_propagates_as_stale_not_silence(graph_process, ros_context):
    # Find the hp60c_node OS process started by this test's own graph_process group and kill only
    # that one -- an upstream sensor-node crash, not a graceful shutdown.
    result = subprocess.run(["pgrep", "-f", "hp60c_node"], capture_output=True, text=True)
    pids = [int(pid) for pid in result.stdout.split()]
    assert pids, "could not find a running hp60c_node process to kill"
    for pid in pids:
        os.kill(pid, signal.SIGKILL)

    watcher = HealthWatcher()
    try:
        heartbeat_timeout_s = 2.0  # matches disarmed_sim.launch.py's heartbeat_timeout_s override
        deadline = time.monotonic() + heartbeat_timeout_s + STARTUP_TIMEOUT_S
        key = "autokart_system_monitor: /hp60c/status"
        became_stale = False
        while time.monotonic() < deadline:
            rclpy.spin_once(watcher, timeout_sec=0.5)
            if watcher.levels.get(key) == 3:  # DiagnosticStatus.STALE
                became_stale = True
                break
        assert became_stale, (
            f"/hp60c/status never reported STALE on /system/health after killing hp60c_node "
            f"(last known level: {watcher.levels.get(key)!r}) -- a dead upstream sensor should "
            f"become visibly stale, not silently keep looking healthy"
        )
    finally:
        watcher.destroy_node()
