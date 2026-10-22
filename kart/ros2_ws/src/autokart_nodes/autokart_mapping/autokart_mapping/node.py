"""Pose-aware observed cone map; all outputs are diagnostic and disarmed."""

from __future__ import annotations

import json
from math import atan2, cos, hypot, isfinite, pi, sin, sqrt
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from autokart_msgs.msg import Cone as RosCone, ConeArray, MissionDebug

from autokart_perception.application import CoreRuntime
from autokart_perception.shared_core import PoseEstimate, fit_skidpad_topology
from autokart_perception.types import Cone


LABELS = {1: "blue", 2: "yellow", 3: "orange", 4: "large_orange"}
CLASSES = {value: key for key, value in LABELS.items()}


def ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ros_time(value: int):
    from builtin_interfaces.msg import Time
    stamp = Time()
    stamp.sec, stamp.nanosec = divmod(int(value), 1_000_000_000)
    return stamp


def pose_from_odom(message: Odometry) -> PoseEstimate:
    q = message.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    covariance = message.pose.covariance
    uncertainty = sqrt(max(0.0, float(covariance[0]), float(covariance[7])))
    speed = message.twist.twist.linear
    return PoseEstimate(float(message.pose.pose.position.x),
                        float(message.pose.pose.position.y), atan2(siny, cosy),
                        sqrt(speed.x * speed.x + speed.y * speed.y),
                        ns(message.header.stamp), uncertainty,
                        frame_id=message.header.frame_id,
                        valid=message.child_frame_id == "base_link")


def observed_exit_centerline(tracked: list[dict], topology: dict) -> list[tuple[float, float]]:
    """Pair observed exit cones in odom; leave the route absent if ambiguous."""
    if not topology.get("valid"):
        return []
    left, right = topology["left"], topology["right"]
    ex, ey = topology["separation_axis_xy"]
    tx, ty = -ey, ex
    origin = ((left["center_x_m"] + right["center_x_m"]) / 2,
              (left["center_y_m"] + right["center_y_m"]) / 2)
    sides = {"blue": [], "yellow": []}
    for cone in tracked:
        label = cone.get("label")
        if label not in sides:
            continue
        x, y = float(cone["x_m"]), float(cone["y_m"])
        lateral = (x - origin[0]) * ex + (y - origin[1]) * ey
        forward = (x - origin[0]) * tx + (y - origin[1]) * ty
        if forward >= 4.5 and (lateral < 0 if label == "blue" else lateral > 0) and abs(lateral) <= 3.0:
            sides[label].append((forward, lateral, x, y))
    pairs = []
    used_yellow = set()
    for blue in sorted(sides["blue"]):
        matches = [(abs(blue[0] - yellow[0]) + abs(yellow[1] - blue[1] - 3.0), index, yellow)
                   for index, yellow in enumerate(sides["yellow"])
                   if index not in used_yellow and abs(blue[0] - yellow[0]) <= 0.8
                   and 2.5 <= yellow[1] - blue[1] <= 4.5]
        matches.sort()
        if not matches or (len(matches) > 1 and matches[1][0] - matches[0][0] < 0.25):
            continue
        _, index, yellow = matches[0]
        used_yellow.add(index)
        pairs.append(((blue[0] + yellow[0]) / 2,
                      ((blue[2] + yellow[2]) / 2, (blue[3] + yellow[3]) / 2)))
    pairs.sort()
    if len(pairs) < 3 or pairs[-1][0] - pairs[0][0] < 5.0:
        return []
    if any(not 0.8 <= b[0] - a[0] <= 4.0 for a, b in zip(pairs, pairs[1:])):
        return []
    return [point for _, point in pairs]


class MappingEngine:
    def __init__(self):
        self.core = CoreRuntime().algorithms
        self.last_stamp = -1
        self.last_pose_stamp = -1
        self.last_pose = None

    def clear(self):
        self.core = CoreRuntime().algorithms
        self.last_stamp = -1
        self.last_pose_stamp = -1
        self.last_pose = None

    def update(self, message: ConeArray, status: dict, pose: PoseEstimate) -> dict:
        stamp = ns(message.measurement_time)
        reasons = []
        if message.frame_id != "base_link" or message.ground_truth:
            reasons.append("UNQUALIFIED_CONE_FRAME")
        if status.get("sample_time_ns") != stamp or not status.get("valid"):
            reasons.append("PERCEPTION_STATUS_MISMATCH")
        if status.get("source") not in {"LIVE", "REPLAY", "SIMULATED", "SYNTHETIC"}:
            reasons.append("SOURCE_UNQUALIFIED")
        if stamp <= self.last_stamp or stamp <= 0:
            reasons.append("NONMONOTONE_CONES")
        if pose.timestamp_ns <= self.last_pose_stamp:
            reasons.append("NONMONOTONE_ODOM")
        if self.last_pose is not None and pose.timestamp_ns > self.last_pose_stamp:
            delta_s = (pose.timestamp_ns - self.last_pose_stamp) / 1e9
            displacement = hypot(pose.x_m - self.last_pose.x_m, pose.y_m - self.last_pose.y_m)
            yaw_delta = abs(atan2(sin(pose.yaw_rad - self.last_pose.yaw_rad),
                                  cos(pose.yaw_rad - self.last_pose.yaw_rad)))
            if displacement > max(1.0, max(pose.speed_mps, self.last_pose.speed_mps) * delta_s + 0.5) or yaw_delta > pi / 2:
                reasons.append("ODOM_DISCONTINUITY")
        if not pose.fresh(stamp, 150_000_000):
            reasons.append("INVALID_OR_STALE_ODOM")
        if not message.cones:
            reasons.append("NO_OBSERVED_CONES")
        if reasons:
            self.clear()
            return {"valid": False, "reason_codes": reasons, "tracked_cones": [],
                    "geometry": {}, "source": status.get("source", "UNKNOWN"), "sample_time_ns": stamp}
        observations = []
        seen = set()
        for item in message.cones:
            label = LABELS.get(item.semantic_class)
            xy = (float(item.position.x), float(item.position.y))
            sigma = float(item.position_std_m)
            if (label is None or item.id in seen or
                    not all(isfinite(value) for value in (*xy, sigma, item.confidence)) or
                    sigma < 0 or sigma > 0.5 or not 0 <= item.confidence <= 1):
                self.clear()
                return {"valid": False, "reason_codes": ["INVALID_CONE"], "tracked_cones": [],
                        "geometry": {}, "source": status["source"], "sample_time_ns": stamp}
            seen.add(item.id)
            observations.append(Cone(int(item.id), label, float(item.confidence), xy,
                                     sigma, sqrt(xy[0] ** 2 + xy[1] ** 2), (0, 0, 0, 0), stamp))
        raw = self.core.evaluate(observations, pose, "STRAIGHT_TRACK", now_ns=stamp,
                                 source=status["source"])
        self.last_stamp, self.last_pose_stamp = stamp, pose.timestamp_ns
        self.last_pose = pose
        tracked = raw["tracked_cones"]
        geometry = raw["geometry"]
        def odom_xy(x, y):
            return (pose.x_m + cos(pose.yaw_rad) * x - sin(pose.yaw_rad) * y,
                    pose.y_m + sin(pose.yaw_rad) * x + cos(pose.yaw_rad) * y)
        geometry["observation_cones_odom"] = [
            {"id": cone.id, "label": cone.label, "confidence": cone.confidence,
             "x_m": odom_xy(*cone.xy)[0], "y_m": odom_xy(*cone.xy)[1]}
            for cone in observations]
        geometry["centerline_odom"] = [odom_xy(*point) for point in geometry.get("centerline", [])]
        geometry["skidpad_topology"] = fit_skidpad_topology(tracked)
        geometry["exit_centerline_odom"] = observed_exit_centerline(tracked, geometry["skidpad_topology"])
        return {"valid": True, "reason_codes": [], "tracked_cones": tracked,
                "geometry": geometry, "source": status["source"],
                "sample_time_ns": stamp, "pose_time_ns": pose.timestamp_ns,
                "algorithm_version": raw["algorithm_version"],
                "configuration_sha256": raw["configuration_sha256"]}


class MappingNode(Node):
    def __init__(self):
        super().__init__("autokart_mapping")
        self.declare_parameter("source_mode", "SIMULATED")
        self.declare_parameter("live_odom_verified", False)
        self.source_mode = str(self.get_parameter("source_mode").value)
        self.live_odom_verified = bool(self.get_parameter("live_odom_verified").value)
        if self.source_mode not in {"SIMULATED", "REPLAY", "LIVE"}:
            raise ValueError("unsupported mapping source_mode")
        self.engine = MappingEngine()
        self.status_by_stamp = {}
        self.pending_cones = {}
        self.poses = {}
        self.cycle = 0
        self.last_receive = None
        self.map_pub = self.create_publisher(ConeArray, "/autokart/mapping/cones", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(MissionDebug, "/autokart/mapping/status", 10)
        self.dashboard_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
        self.create_subscription(MissionDebug, "/autokart/perception/status", self.on_status, 10)
        self.create_subscription(Odometry, "/autokart/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(ConeArray, "/perception/cones", self.on_cones, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)

    def on_status(self, message):
        try:
            payload = json.loads(message.payload_json)
            if payload.get("schema_version") == 1 and payload.get("sample_time_ns") == ns(message.measurement_time):
                self.status_by_stamp[ns(message.measurement_time)] = payload
                if len(self.status_by_stamp) > 8:
                    del self.status_by_stamp[min(self.status_by_stamp)]
                if not payload.get("valid"):
                    self.engine.clear()
                    self.publish({"valid": False, "source": payload.get("source", "UNKNOWN"),
                                  "sample_time_ns": ns(message.measurement_time),
                                  "reason_codes": ["PERCEPTION_INVALID", *payload.get("reason_codes", [])],
                                  "tracked_cones": [], "geometry": {}})
                else:
                    self._maybe_process(ns(message.measurement_time))
        except (ValueError, TypeError):
            self.engine.clear()

    def on_odom(self, message):
        try:
            pose = pose_from_odom(message)
            self.poses[pose.timestamp_ns] = pose
            if len(self.poses) > 16:
                del self.poses[min(self.poses)]
        except (ValueError, OverflowError):
            self.poses.clear()
            self.engine.clear()
        for stamp in sorted(self.pending_cones):
            self._maybe_process(stamp)

    def on_cones(self, message):
        stamp = ns(message.measurement_time)
        self.pending_cones[stamp] = (message, time.monotonic())
        self._maybe_process(stamp)

    def _maybe_process(self, stamp):
        pending = self.pending_cones.get(stamp)
        status = self.status_by_stamp.get(stamp)
        pose_stamps = [value for value in self.poses if 0 <= stamp - value <= 150_000_000]
        if pending is None or status is None or not pose_stamps:
            return
        pose = self.poses[max(pose_stamps)]
        message, _ = self.pending_cones.pop(stamp)
        self.status_by_stamp.pop(stamp, None)
        if status.get("source") != self.source_mode:
            reason = "SOURCE_MODE_MISMATCH"
        elif self.source_mode == "LIVE" and not self.live_odom_verified:
            reason = "LIVE_ODOM_UNVERIFIED"
        elif self.source_mode == "LIVE" and pose.position_std_m <= 0:
            reason = "LIVE_ODOM_COVARIANCE_UNQUALIFIED"
        else:
            reason = None
        if reason is not None:
            self.engine.clear()
            result = {"valid": False, "source": status.get("source", "UNKNOWN"),
                      "sample_time_ns": stamp, "reason_codes": [reason],
                      "tracked_cones": [], "geometry": {}}
        else:
            result = self.engine.update(message, status, pose)
        self.last_receive = time.monotonic()
        self.publish(result)

    def on_timer(self):
        expired = [stamp for stamp, (_, received) in self.pending_cones.items()
                   if time.monotonic() - received > 0.3]
        for stamp in expired:
            self.pending_cones.pop(stamp, None)
            self.status_by_stamp.pop(stamp, None)
            self.engine.clear()
            self.publish({"valid": False, "source": "UNKNOWN", "sample_time_ns": stamp,
                          "reason_codes": ["STATUS_OR_ODOM_TIMEOUT"],
                          "tracked_cones": [], "geometry": {}})
        if self.last_receive is not None and time.monotonic() - self.last_receive > 0.5:
            self.last_receive = None
            self.engine.clear()
            self.publish({"valid": False, "source": "UNKNOWN",
                          "sample_time_ns": self.get_clock().now().nanoseconds,
                          "reason_codes": ["MAP_INPUT_TIMEOUT"],
                          "tracked_cones": [], "geometry": {}})

    def publish(self, result):
        self.cycle += 1
        stamp = result["sample_time_ns"]
        map_message = ConeArray()
        map_message.measurement_time = ros_time(stamp)
        map_message.delivery_time = ros_time(self.get_clock().now().nanoseconds)
        map_message.frame_id = "odom"
        map_message.ground_truth = False
        if result["valid"]:
            for item in result["tracked_cones"]:
                cone = RosCone()
                cone.id = int(item["id"])
                cone.semantic_class = CLASSES[item["label"]]
                cone.position.x = float(item["x_m"])
                cone.position.y = float(item["y_m"])
                cone.confidence = float(item["confidence"])
                cone.position_std_m = float(item["sigma_m"])
                map_message.cones.append(cone)
        self.map_pub.publish(map_message)
        payload = {"schema_version": 1, "source": result["source"],
                   "processed_at_monotonic_ns": time.monotonic_ns(),
                   "valid": result["valid"], "reason_codes": result["reason_codes"],
                   "frame_id": "odom", "sample_time_ns": stamp,
                   "expires_ns": stamp + 150_000_000, "sequence": self.cycle,
                   "tracked_count": len(map_message.cones),
                   "geometry": result["geometry"], "vehicle_authorized": False,
                   "published_to_vcu": False}
        status_message = MissionDebug()
        status_message.measurement_time = ros_time(stamp)
        status_message.cycle = self.cycle
        status_message.frame_id = "odom"
        status_message.stage = "MAPPING"
        status_message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(status_message)
        status_message.stage = "mapping"
        self.dashboard_pub.publish(status_message)


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
