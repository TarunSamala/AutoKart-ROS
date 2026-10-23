"""autokart_mission: standalone mission/braking-FSM node (node 07 of the 12-node architecture).

Wraps the existing, reused ``autokart_planner.mission.PlannerEngine`` (skidpad gate-crossing/
winding-number progress FSM, braking stop-distance physics with dead-reckoning) as its own
standalone node, separate from the planner (node 08, not yet built in this library). Outputs mission
*constraints* (allowable branch, lap index, speed cap/mode) via ``autokart_msgs/MissionDebug`` --
never a path, a ``MotionCommand``, or any other motor command. See CONTRACT.md for the honest
accounting of what the reused FSM does and does not implement against the master architecture's
named-state vocabulary.
"""
from __future__ import annotations

import json
import time
from math import atan2, sin, sqrt, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from autokart_msgs.msg import ActuatorFeedback, Cone, ConeArray, MissionDebug

from .algorithm import MissionAlgorithm, MissionAlgorithmError
from .stop_verification import StopVerifier
from .braking_fsm import BrakingFSM


def ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ros_time(value: int):
    from builtin_interfaces.msg import Time
    stamp = Time()
    stamp.sec, stamp.nanosec = divmod(int(value), 1_000_000_000)
    return stamp


def pose_from_odom(message: Odometry, pose_estimate_cls):
    """Same quaternion-to-yaw/covariance-to-sigma conversion as autokart_mapping's own
    ``pose_from_odom`` and the existing, reused ``kart/ros2_ws`` adapters -- duplicated here (not
    imported from the sibling package) since each ROS-adapter package in this library is standalone
    and independently exportable; it is five lines of generic math, not the reused algorithm."""
    q = message.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    covariance = message.pose.covariance
    uncertainty = sqrt(max(0.0, float(covariance[0]), float(covariance[7])))
    speed = message.twist.twist.linear
    return pose_estimate_cls(float(message.pose.pose.position.x),
                             float(message.pose.pose.position.y), atan2(siny, cosy),
                             sqrt(speed.x * speed.x + speed.y * speed.y),
                             ns(message.header.stamp), uncertainty,
                             frame_id=message.header.frame_id,
                             valid=message.child_frame_id == "base_link")


def mission_constraints(mission: str, result: dict) -> dict:
    """Projects the reused PlannerEngine result into the "mission constraints" vocabulary the master
    architecture asks node 07 to output (allowable branch, lap index, speed cap/mode). This is a
    direct relabeling of already-computed, already-tested PlannerEngine fields -- not a new safety
    decision, and not a renaming of PlannerEngine's own internal state names (see CONTRACT.md for why
    those are reported, not silently remapped)."""
    state = result.get("mission_state", {})
    if mission == "SKIDPAD":
        branch = state.get("state", "UNKNOWN")
        lap_index = int(state.get("right_laps", 0)) + int(state.get("left_laps", 0))
    else:
        branch = state.get("state", "UNKNOWN")
        lap_index = 0
    speed_cap = float(result.get("target_speed_mps", 0.0)) if result.get("valid") else 0.0
    return {"allowable_branch": branch, "lap_index": lap_index,
            "speed_cap_mps": speed_cap, "mode": result.get("decision", "HOLD")}


class MissionNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("autokart_mission", **node_kwargs)
        self.declare_parameter("algorithm_package_path", "")
        self.declare_parameter("vehicle_geometry_package_path", "")
        self.declare_parameter("planner_package_path", "")
        self.declare_parameter("mission", "SKIDPAD")
        self.declare_parameter("source", "SIMULATED")
        self.declare_parameter("brake_min_mps2", 0.0)
        self.declare_parameter("brake_delay_s", 0.0)
        self.declare_parameter("t_latency", -1.0)
        self.declare_parameter("brake_provenance", "UNKNOWN")
        self.declare_parameter("brake_margin_m", 1.0)
        self.declare_parameter("speed_limit_mps", 1.0)
        self.declare_parameter("camera_to_bumper_m", -1.0)   # -1.0 sentinel -> use GEOMETRY default
        self.declare_parameter("stop_gap_from_bumper_m", 1.0)
        self.declare_parameter("lateral_margin_m", -1.0)      # -1.0 sentinel -> use GEOMETRY default
        self.declare_parameter("footprint_width_m", -1.0)     # -1.0 sentinel -> use GEOMETRY default
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("mapping_status_topic", "/autokart_mapping/status")
        self.declare_parameter("feedback_topic", "/vehicle/actuator_feedback")
        self.declare_parameter("obstacles_topic", "/perception/obstacles")
        self.declare_parameter("status_topic", "/autokart_mission/status")
        self.declare_parameter("status_max_age_s", 0.3)
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("stop_hold_s", 1.0)
        self.declare_parameter("stop_evidence_max_age_s", 0.15)
        self.declare_parameter("stop_evidence_topic", "/vehicle/stop_evidence")

        self.mission = str(self.get_parameter("mission").value)
        self.source = str(self.get_parameter("source").value)
        self.status_max_age_s = float(self.get_parameter("status_max_age_s").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)

        self.stop_verifier = StopVerifier(self.source,
            hold_s=float(self.get_parameter("stop_hold_s").value),
            max_age_s=float(self.get_parameter("stop_evidence_max_age_s").value))
        self.braking_fsm = BrakingFSM()
        self.stop_evidence = None
        self.last_result = None
        self.mission_fault_latched = False
        self.algorithm = None
        self.engine = None
        self.startup_error = None
        try:
            self.algorithm = MissionAlgorithm(
                str(self.get_parameter("algorithm_package_path").value),
                str(self.get_parameter("vehicle_geometry_package_path").value),
                str(self.get_parameter("planner_package_path").value))
            self._build_engine()
        except MissionAlgorithmError as exc:
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_mission startup failed: {self.startup_error}")

        self.poses = {}
        self.odom_statuses = {}
        self.source_mode = self.source
        self.feedback = None
        self.statuses = {}
        self.obstacles = {}
        self.obstacle_statuses = {}
        self.cycle = 0
        self.last_update = None

        self.status_pub = self.create_publisher(
            MissionDebug, str(self.get_parameter("status_topic").value), 10)
        self.canonical_pub = self.create_publisher(MissionDebug, "/mission/state", 10)
        self.create_subscription(String, str(self.get_parameter("stop_evidence_topic").value),
                                 self.on_stop_evidence, 10)
        self.create_service(Trigger, "/mission/start_preview", self.on_start)
        self.create_service(Trigger, "/mission/reset", self.on_reset)
        self.create_service(Trigger, "/mission/interrupt", self.on_interrupt)
        self.create_subscription(String, "/vehicle/odometry_status", self.on_odom_status, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(MissionDebug, str(self.get_parameter("mapping_status_topic").value),
                                 self.on_mapping_status, 10)
        self.create_subscription(ActuatorFeedback, str(self.get_parameter("feedback_topic").value),
                                 self.on_feedback, 10)
        self.create_subscription(MissionDebug, "/perception/obstacle_status_base", self.on_obstacle_status, 10)
        self.create_subscription(ConeArray, str(self.get_parameter("obstacles_topic").value),
                                 self.on_obstacles, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)

    def _build_engine(self):
        brake = None
        minimum = float(self.get_parameter("brake_min_mps2").value)
        if minimum > 0:
            latency = float(self.get_parameter("t_latency").value)
            if latency < 0:
                latency = float(self.get_parameter("brake_delay_s").value)
            if self.source != "LIVE" or latency > 0:
                brake = self.algorithm.BrakeCapability(
                    minimum, latency, str(self.get_parameter("brake_provenance").value),
                    margin_m=float(self.get_parameter("brake_margin_m").value))
        geometry = self.algorithm.GEOMETRY

        def resolved(param_name, geometry_value):
            value = float(self.get_parameter(param_name).value)
            return geometry_value if value < 0 else value

        footprint_width_m = resolved("footprint_width_m",
                                     geometry.bumper_width_m + 2.0 * geometry.lateral_margin_m)
        camera_to_bumper_m = resolved("camera_to_bumper_m", geometry.camera_to_front_bumper_horizontal_m)
        lateral_margin_m = resolved("lateral_margin_m", geometry.lateral_margin_m)
        self.engine = self.algorithm.PlannerEngine(
            self.mission, self.source, brake,
            speed_limit_mps=float(self.get_parameter("speed_limit_mps").value),
            footprint_width_m=footprint_width_m,
            camera_to_bumper_m=camera_to_bumper_m,
            stop_gap_from_bumper_m=float(self.get_parameter("stop_gap_from_bumper_m").value),
            lateral_margin_m=lateral_margin_m)

    def on_odom(self, message):
        if self.algorithm is None:
            return
        pose = pose_from_odom(message, self.algorithm.PoseEstimate)
        self.poses[pose.timestamp_ns] = pose
        if len(self.poses) > 256:
            del self.poses[min(self.poses)]
        for stamp in sorted(self.statuses):
            self._maybe_process(stamp)

    def on_stop_evidence(self, message):
        try:
            self.stop_evidence = json.loads(message.data)
        except (ValueError, TypeError):
            self.stop_evidence = None

    def on_start(self, request, response):
        response.success = self.mission == "BRAKING_TEST" and self.braking_fsm.request_start()
        response.message = "PREVIEW_START_ACCEPTED_OUTPUTS_INHIBITED" if response.success else "MISSION_NOT_READY"
        return response

    def on_reset(self, request, response):
        verification = self.stop_verifier.update(self.stop_evidence, self.get_clock().now().nanoseconds)
        disarmed = isinstance(self.stop_evidence, dict) and self.stop_evidence.get("disarmed") is True
        response.success = self.braking_fsm.reset(disarmed=disarmed, verification=verification)
        if response.success:
            self.mission_fault_latched = False
        if response.success and self.engine is not None:
            self.engine.reset()
        response.message = "RESET" if response.success else "RESET_REQUIRES_DISARMED_AND_VERIFIED_HOLD"
        return response

    def on_interrupt(self, request, response):
        self.mission_fault_latched = True
        self.braking_fsm.update(corridor_valid=False, ready=False, stop_trigger=True,
                                verification=self.stop_verifier.result, interrupted=True)
        response.success, response.message = True, "STOP_LATCHED"
        return response

    def on_feedback(self, message):
        self.feedback = message

    def on_mapping_status(self, message):
        if self.algorithm is None:
            return
        try:
            payload = json.loads(message.payload_json)
        except (ValueError, TypeError):
            self._reject_and_publish("MAPPING_STATUS_MALFORMED", ns(message.measurement_time))
            return
        stamp = ns(message.measurement_time)
        if not isinstance(payload, dict) or payload.get("schema_version") != 1 or payload.get("sample_time_ns") != stamp:
            self._reject_and_publish("MAPPING_STATUS_INVALID", stamp)
            return
        now_ns = self.get_clock().now().nanoseconds
        if (not 0 <= now_ns - stamp <= 250_000_000 or
                type(payload.get("expires_ns")) is not int or payload["expires_ns"] <= now_ns):
            self._reject_and_publish("MAPPING_STATUS_STALE", stamp)
            return
        self.statuses[stamp] = (payload, time.monotonic())
        for old in sorted(self.statuses)[:-16]:
            del self.statuses[old]
        self._maybe_process(stamp)

    def on_obstacle_status(self, message):
        try:
            payload=json.loads(message.payload_json)
            if not isinstance(payload,dict):
                raise ValueError()
            stamp=ns(message.measurement_time)
            if payload.get("schema_version") != 1 or payload.get("sample_time_ns") != stamp:
                raise ValueError()
            self.obstacle_statuses[stamp]=payload
            for old in sorted(self.obstacle_statuses)[:-8]:
                del self.obstacle_statuses[old]
            self._maybe_process(stamp)
        except (ValueError,TypeError):
            self.obstacle_statuses.clear()

    def on_obstacles(self, message: ConeArray):
        stamp = ns(message.measurement_time)
        if message.frame_id != "base_link" or (message.ground_truth and self.source != "SIMULATED"):
            return
        self.obstacles[stamp] = [
            {"x_m": cone.position.x, "y_m": cone.position.y,
             "radius_m": max(0.0, cone.position_std_m)}
            for cone in message.cones if cone.semantic_class == 0
        ]
        if len(self.obstacles) > 8:
            del self.obstacles[min(self.obstacles)]
        self._maybe_process(stamp)

    def _reject_and_publish(self, reason: str, stamp: int):
        if self.engine is not None:
            self.publish(self.engine.reject(reason, stamp))
        else:
            self.publish({"valid": False, "reason_codes": [reason], "sample_time_ns": stamp,
                          "decision": "HOLD", "target_speed_mps": 0.0,
                          "mission_state": {"state": "UNKNOWN"}, "stop_point_m": None})

    def on_odom_status(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict) or payload.get("schema_version") != 1 or type(payload.get("sample_time_ns")) is not int:
                raise ValueError()
            self.odom_statuses[payload["sample_time_ns"]] = payload
            for old in sorted(self.odom_statuses)[:-32]:
                del self.odom_statuses[old]
            for stamp in sorted(self.statuses):
                self._maybe_process(stamp)
        except (ValueError, TypeError):
            self.odom_statuses.clear()

    def _pose_qualified(self, pose):
        status = self.odom_statuses.get(pose.timestamp_ns, {})
        source = {"SYNTHETIC":"SIMULATED", "LIVE_PHYSICAL":"LIVE"}.get(status.get("source"),status.get("source"))
        now = self.get_clock().now().nanoseconds
        return (status.get("valid") is True and status.get("continuity_lost") is False and
                source == self.source_mode and type(status.get("expires_ns")) is int and
                status["expires_ns"] > now and 0 <= now-pose.timestamp_ns <= 250_000_000 and
                (source != "LIVE" or status.get("physical_qualified") is True))

    def pose_at_capture(self, stamp):
        if stamp in self.poses:
            return self.poses[stamp] if self._pose_qualified(self.poses[stamp]) else None
        before = [x for x in self.poses if 0 < stamp-x <= 150_000_000]
        after = [x for x in self.poses if 0 < x-stamp <= 150_000_000]
        if not before or not after:
            return None
        a,b = self.poses[max(before)],self.poses[min(after)]
        if not self._pose_qualified(a) or not self._pose_qualified(b) or not a.valid or not b.valid or a.frame_id != b.frame_id:
            return None
        t=(stamp-a.timestamp_ns)/(b.timestamp_ns-a.timestamp_ns)
        yaw_delta=(b.yaw_rad-a.yaw_rad+pi)%(2*pi)-pi
        if abs(yaw_delta)>pi/2:
            return None
        return self.algorithm.PoseEstimate(a.x_m+(b.x_m-a.x_m)*t,a.y_m+(b.y_m-a.y_m)*t,
            a.yaw_rad+yaw_delta*t,a.speed_mps+(b.speed_mps-a.speed_mps)*t,stamp,
            max(a.position_std_m,b.position_std_m),frame_id=a.frame_id,valid=True)

    def _maybe_process(self, stamp):
        pair = self.statuses.get(stamp)
        pose = self.pose_at_capture(stamp)
        if pair is None or pose is None:
            return
        payload, _ = pair
        now_ns=self.get_clock().now().nanoseconds
        if type(payload.get("expires_ns")) is not int or payload["expires_ns"] <= now_ns or not 0 <= now_ns-stamp <= 250_000_000:
            self.statuses.pop(stamp,None)
            self._reject_and_publish("MAPPING_STATUS_STALE",stamp)
            return
        if self.mission == "BRAKING_TEST":
            coverage=self.obstacle_statuses.get(stamp,{})
            if stamp not in self.obstacles or not coverage:
                return
            source={"SYNTHETIC":"SIMULATED","LIVE_PHYSICAL":"LIVE"}.get(coverage.get("source"),coverage.get("source"))
            if (coverage.get("valid") is not True or coverage.get("coverage_complete") is not True or
                    coverage.get("frame_id") != "base_link" or source != self.source or
                    type(coverage.get("expires_ns")) is not int or coverage["expires_ns"] <= now_ns):
                self.statuses.pop(stamp,None)
                self._reject_and_publish("OBSTACLE_COVERAGE_UNAVAILABLE",stamp)
                return
            self.obstacle_statuses.pop(stamp,None)
        self.statuses.pop(stamp, None)
        feedback_valid = self.stop_verifier.update(
            self.stop_evidence, self.get_clock().now().nanoseconds)["verified"]
        result = self.engine.evaluate(payload, pose, self.obstacles.pop(stamp, []),
                                      stop_feedback_valid=feedback_valid)
        self.last_update = time.monotonic()
        self.publish(result)

    def on_timer(self):
        if (self.braking_fsm.stop_latched or self.mission_fault_latched) and self.last_result is not None:
            self.publish(dict(self.last_result))
        if self.algorithm is None:
            self._reject_and_publish(self.startup_error or "BACKEND_UNAVAILABLE",
                                     self.get_clock().now().nanoseconds)
            return
        expired = [stamp for stamp, (_, received) in self.statuses.items()
                   if time.monotonic() - received > self.status_max_age_s]
        for stamp in expired:
            self.statuses.pop(stamp, None)
            self._reject_and_publish("MAP_PAIR_OR_OBSTACLE_TIMEOUT", stamp)
        if self.last_update is not None and time.monotonic() - self.last_update > self.input_timeout_s:
            self.last_update = None
            self._reject_and_publish("MISSION_INPUT_TIMEOUT", self.get_clock().now().nanoseconds)

    def publish(self, result: dict):
        result = dict(result)
        self.last_result = dict(result)
        verification = self.stop_verifier.update(self.stop_evidence, self.get_clock().now().nanoseconds)
        if self.mission == "BRAKING_TEST":
            valid = bool(result.get("valid"))
            was_latched = self.braking_fsm.stop_latched
            snapshot = self.braking_fsm.update(
                corridor_valid=valid, ready=valid,
                stop_trigger=result.get("decision") in {"STOP", "STOP_CANDIDATE"},
                verification=verification,
                emergency=isinstance(self.stop_evidence, dict) and self.stop_evidence.get("estop_active") is True)
            if snapshot["stop_latched"] and not was_latched:
                self.stop_verifier.restart_hold()
                verification = self.stop_verifier.result
                snapshot["stop_verification"] = verification
            result["mission_state"] = snapshot
            if snapshot["state"] != "RUNNING":
                result["valid"] = False
                result["target_speed_mps"] = 0.0
                result["decision"] = "STOP" if snapshot["stop_latched"] else "HOLD"
                result["reason_codes"] = list(dict.fromkeys(
                    result.get("reason_codes", []) + snapshot["reason_codes"] +
                    ([] if snapshot["state"] == "STOP_VERIFIED" else [snapshot["state"]])))
        if isinstance(self.stop_evidence, dict) and self.stop_evidence.get("estop_active") is True:
            self.mission_fault_latched = True
        if self.mission_fault_latched:
            result.update(valid=False, target_speed_mps=0.0, decision="STOP", path_odom=[])
            result["mission_state"] = {**result.get("mission_state", {}), "state":"FAULT", "stop_latched":True}
            result["reason_codes"] = list(dict.fromkeys(result.get("reason_codes", []) + ["MISSION_INTERRUPTED_OR_ESTOP"]))
        self.cycle += 1
        stamp = result["sample_time_ns"]
        payload = {"schema_version": 1, "source": self.source,
                   "processed_at_monotonic_ns": time.monotonic_ns(),
                   "mission": self.mission,
                   "valid": bool(result["valid"]), "reason_codes": result["reason_codes"],
                   "frame_id": "odom", "sample_time_ns": stamp,
                   "expires_ns": stamp + 100_000_000 if result["valid"] else stamp,
                   "sequence": self.cycle, "decision": result["decision"],
                   "target_speed_mps": result["target_speed_mps"],
                   "mission_state": result["mission_state"],
                   "mission_constraints": mission_constraints(self.mission, result),
                   "stop_point_m": result.get("stop_point_m"),
                   "path_odom": result.get("path_odom", []) if result["valid"] else [],
                   "stop_verification": verification,
                   "stopping_distance_m": result.get("stopping_distance_m"),
                   "vehicle_authorized": False, "published_to_vcu": False}
        message = MissionDebug()
        message.measurement_time = ros_time(max(0, stamp))
        message.cycle = self.cycle
        message.frame_id = "odom"
        message.stage = "MISSION"
        message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(message)
        self.canonical_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
