"""autokart_system_monitor: health and startup-evidence reporting node.

This node REPORTS; it has NO safety authority. It cannot gate, stop, arm, or
authorize anything -- contrast with ``autokart_safety`` (node 10), which IS
the vehicle's arming authority. A WARN/ERROR/STALE reading here is
diagnostic information for a human or a dashboard, never an automatic
vehicle action.

Tracks, standalone (degrades gracefully if nothing else is running):
- ROS node heartbeat/liveness, by dynamically subscribing to each configured
  topic once its message type becomes visible in the ROS graph (a
  dependency that is not running yet, or never runs, simply never gets
  subscribed and reports as "never seen" -- never an error, never a crash).
- CPU/RAM via a pluggable resource backend (``resource_backends.py``);
  ``psutil`` is optional, never hard-required.

Publishes standard ``diagnostic_msgs/DiagnosticArray`` on three topics:
``/diagnostics`` (live, standard ROS diagnostics convention),
``/system/health`` (same live snapshot, this node's own namespace),
``/system/startup_status`` (a one-way "has every tracked dependency been
seen at least once since this node started" latch -- see
``monitor_core.StartupLatch``).
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .monitor_core import ComponentStatus, HealthReport, StartupLatch, evaluate_health
from .resource_backends import build_resource_backend

OWN_OUTPUT_TOPICS = frozenset({"/diagnostics", "/system/health", "/system/startup_status"})


class SystemMonitorNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("autokart_system_monitor", **node_kwargs)
        self.declare_parameter("heartbeat_topics", ["/hp60c/status", "/autokart/safety/status"])
        self.declare_parameter("heartbeat_timeout_s", 2.0)
        self.declare_parameter("resource_backend", "psutil")
        self.declare_parameter("cpu_warn_pct", 90.0)
        self.declare_parameter("ram_warn_pct", 90.0)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("topic_discovery_period_s", 2.0)

        requested_topics = [str(t) for t in self.get_parameter("heartbeat_topics").value]
        # Never treat this node's OWN outputs as a dependency heartbeat: it
        # would receive its own messages and report that entry permanently
        # "fresh" -- self-generated, false liveness evidence.
        self.heartbeat_topics = [t for t in requested_topics if t not in OWN_OUTPUT_TOPICS]
        self.rejected_heartbeat_topics = [t for t in requested_topics if t in OWN_OUTPUT_TOPICS]
        for topic in self.rejected_heartbeat_topics:
            self.get_logger().warning(
                f"system_monitor: ignoring heartbeat topic {topic}: it is one of this node's own outputs")
        self.heartbeat_timeout_s = float(self.get_parameter("heartbeat_timeout_s").value)
        self.cpu_warn_pct = float(self.get_parameter("cpu_warn_pct").value)
        self.ram_warn_pct = float(self.get_parameter("ram_warn_pct").value)
        self.resource_backend = build_resource_backend(str(self.get_parameter("resource_backend").value))

        self.last_seen: dict[str, float] = {}
        self._subscribed: set[str] = set()
        self.startup_latch = StartupLatch(self.heartbeat_topics)

        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.health_pub = self.create_publisher(DiagnosticArray, "/system/health", 10)
        self.startup_pub = self.create_publisher(DiagnosticArray, "/system/startup_status", 10)

        self._try_subscribe_all()
        discovery_period = float(self.get_parameter("topic_discovery_period_s").value)
        self.create_timer(discovery_period, self._try_subscribe_all)
        publish_period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(publish_period, self._tick)

    def _try_subscribe_all(self) -> None:
        """Best-effort, idempotent: resolve each configured heartbeat
        topic's message type from the live ROS graph and subscribe once.
        A topic whose publisher is not up yet simply isn't in the graph
        yet -- retried on the next timer tick, never an error."""
        try:
            graph_topics = dict(self.get_topic_names_and_types())
        except Exception as exc:  # noqa: BLE001 - graph introspection must never crash this node
            self.get_logger().warning(f"system_monitor: topic discovery failed: {exc}")
            return
        for topic in self.heartbeat_topics:
            if topic in self._subscribed:
                continue
            types = graph_topics.get(topic)
            if not types:
                continue
            try:
                from rosidl_runtime_py.utilities import get_message
                msg_type = get_message(types[0])
            except (ImportError, ValueError, AttributeError, ModuleNotFoundError) as exc:
                self.get_logger().warning(f"system_monitor: could not resolve type for {topic} ({types[0]}): {exc}")
                continue
            # BEST_EFFORT + VOLATILE (sensor-data QoS): a best-effort
            # subscriber is QoS-compatible with BOTH reliable and best-effort
            # publishers, so this monitor never falsely reports a healthy
            # dependency STALE just because that dependency publishes with
            # sensor-data QoS. Using the default (RELIABLE) here was a real
            # defect found in review -- see
            # test_best_effort_publisher_is_detected_not_falsely_reported_stale.
            # Liveness only needs "a message arrived", so best-effort's
            # possible drops are harmless at heartbeat rates.
            self.create_subscription(msg_type, topic, self._make_heartbeat_handler(topic),
                                     qos_profile_sensor_data)
            self._subscribed.add(topic)
            self.get_logger().info(f"system_monitor: subscribed to {topic} ({types[0]})")

    def _make_heartbeat_handler(self, topic: str):
        def handler(_msg) -> None:
            self.last_seen[topic] = time.monotonic()
        return handler

    def _age(self, topic: str):
        seen = self.last_seen.get(topic)
        return None if seen is None else time.monotonic() - seen

    def _tick(self) -> None:
        ages = {topic: self._age(topic) for topic in self.heartbeat_topics}
        timeouts = {topic: self.heartbeat_timeout_s for topic in self.heartbeat_topics}
        sample = self.resource_backend.sample()
        report = evaluate_health(ages, timeouts, sample, self.cpu_warn_pct, self.ram_warn_pct)
        self.startup_latch.observe(ages)

        self._publish(self.diagnostics_pub, report)
        self._publish(self.health_pub, report)
        self._publish(self.startup_pub, self.startup_latch.status())

    def _publish(self, publisher, report: HealthReport) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for component in report.components:
            msg.status.append(self._to_diagnostic_status(component))
        publisher.publish(msg)

    @staticmethod
    def _to_diagnostic_status(component: ComponentStatus) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.level = bytes([component.level])
        status.name = f"autokart_system_monitor: {component.name}"
        status.message = component.message
        status.hardware_id = "autokart_system_monitor"
        status.values = [KeyValue(key=k, value=v) for k, v in sorted(component.values.items())]
        return status


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
