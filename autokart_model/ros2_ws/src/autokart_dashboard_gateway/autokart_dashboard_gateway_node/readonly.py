"""Read-only adapters for the existing Mission Control, preserving source claims."""
from __future__ import annotations
import json
import math
import threading
import time

def safe_value(value):
    if isinstance(value, dict):
        return {str(k): safe_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, bytes, bytearray)):
        return [safe_value(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, "tolist"):
        return safe_value(value.tolist())
    return value


class PublishedState:
    """Bounded latest-message cache. Arrival age never asserts sensor validity."""
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = {}

    def update(self, topic, payload):
        with self._lock:
            payload = safe_value(payload)
            if len(json.dumps(payload, default=str)) > 262144:
                payload = {"valid": False, "source": "UNVERIFIED", "reason": "PAYLOAD_SIZE_LIMIT"}
            self._latest[topic] = (time.monotonic(), payload)

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            return {"read_only": True, "vehicle_authorized": False,
                    "output_inhibited": True,
                    "topics": {key: {"arrival_age_s": max(0.0, now - value[0]),
                                      "published": value[1]}
                               for key, value in self._latest.items()}}


class OdometryCompanion:
    """Only source-matched stamped observations may enter the legacy pose view."""
    def __init__(self, source, sink, now_ns):
        self.expected = "LIVE" if source == "REAL_KART" else "SIMULATED"
        self.sink, self.now_ns = sink, now_ns
        self.messages, self.statuses = {}, {}

    def on_pose(self, message):
        stamp = int(message.header.stamp.sec)*1_000_000_000 + int(message.header.stamp.nanosec)
        self.messages[stamp] = message
        self._try(stamp)
        for old in sorted(self.messages)[:-16]:
            self.messages.pop(old, None)

    def on_status(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict) or type(payload.get("sample_time_ns")) is not int:
                return
            stamp = payload["sample_time_ns"]
            self.statuses[stamp] = payload
            self._try(stamp)
            for old in sorted(self.statuses)[:-16]:
                self.statuses.pop(old, None)
        except (ValueError, TypeError):
            return

    def _try(self, stamp):
        if stamp not in self.messages or stamp not in self.statuses:
            return
        message, status = self.messages.pop(stamp), self.statuses.pop(stamp)
        source = {"LIVE_PHYSICAL":"LIVE", "SYNTHETIC":"SIMULATED"}.get(status.get("source"), status.get("source"))
        age = self.now_ns() - stamp
        if (status.get("schema_version") == 1 and status.get("valid") is True and
                source == self.expected and 0 <= age <= 200_000_000):
            self.sink(message)


def make_readonly_node(gateway, store, camera=None):
    """Reuse upstream subscribers/converters and remove its request publishers."""
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from diagnostic_msgs.msg import DiagnosticArray
    from autokart_msgs.msg import MissionDebug, ConeArray, SensorDeliveryMetadata
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict

    node = gateway.make_ros_node(store, fsds=False, camera=camera)
    for attribute in ("request_pub", "demo_drive_pub"):
        publisher = getattr(node, attribute, None)
        if publisher is not None:
            node.destroy_publisher(publisher)
            setattr(node, attribute, None)
    # No HTTP/WebSocket handler receives these sinks either.
    node.send_request = lambda *args, **kwargs: False
    node.send_demo_drive_message = lambda *args, **kwargs: False
    node.send_demo_drive_ping = lambda: None
    # The legacy factory also listens to unpaired vehicle/autokart odom.
    # Remove them so every rendered pose passes the companion gate.
    for subscription in list(node.subscriptions):
        if subscription.topic_name in {"/vehicle/odometry", "/autokart/odom", "/fsds/testing_only/odom"}:
            node.destroy_subscription(subscription)
    state = PublishedState()
    odometry = OdometryCompanion(store.source, node._odometry,
                                lambda: node.get_clock().now().nanoseconds)

    def capture(topic):
        def handler(msg):
            if hasattr(msg, "payload_json"):
                try:
                    payload = json.loads(msg.payload_json, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                except (ValueError, TypeError):
                    payload = {"valid": False, "source": "UNVERIFIED", "reason": "MALFORMED_JSON"}
                state.update(topic, {"measurement_time": message_to_ordereddict(msg.measurement_time),
                                     "frame_id": msg.frame_id, "payload": payload})
            elif isinstance(msg, String):
                try:
                    payload = json.loads(msg.data, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                except (ValueError, TypeError):
                    payload = {"valid": False, "source": "UNVERIFIED", "reason": "MALFORMED_JSON"}
                state.update(topic, payload)
            else:
                state.update(topic, message_to_ordereddict(msg))
        return handler

    node.create_subscription(Odometry, "/odom", odometry.on_pose, qos_profile_sensor_data)
    node.create_subscription(String, "/vehicle/odometry_status", odometry.on_status, 10)
    node.create_subscription(Odometry, "/odom", capture("/odom"), qos_profile_sensor_data)
    node.create_subscription(ConeArray, "/mapping/cones", node._tracked_cones, qos_profile_sensor_data)
    for topic in (
                  "/perception/diagnostics", "/hp60c/diagnostics", "/mapping/status", "/mission/state", "/mission/constraints",
                  "/planning/status", "/control/status", "/control/request",
                  "/safety/status", "/vehicle/authorized_command"):
        node.create_subscription(MissionDebug, topic, capture(topic), 10)
    for topic in ("/hp60c/status", "/perception/status", "/imu/status", "/vehicle/status"):
        node.create_subscription(SensorDeliveryMetadata, topic, capture(topic), 10)
    for topic in ("/vehicle/telemetry", "/vehicle/stop_evidence", "/vehicle/odometry_status", "/planning/speed_profile"):
        node.create_subscription(String, topic, capture(topic), 10)
    for topic in ("/system/health", "/system/startup_status", "/diagnostics"):
        node.create_subscription(DiagnosticArray, topic, capture(topic), 10)
    if camera is not None:
        node.create_subscription(Image, "/hp60c/rgb/image_raw", node._rgb, qos_profile_sensor_data)
        node.create_subscription(Image, "/hp60c/depth/image_raw", node._depth, qos_profile_sensor_data)
    return node, state


def attach_state_endpoint(app, state):
    from aiohttp import web
    async def snapshot(_request):
        return web.json_response(state.snapshot())
    app.router.add_get("/integration/api/state", snapshot)
