"""autokart_planner: standalone ROS 2 node for path-candidate curvature/speed-profile/collision/
stop-envelope evaluation. Never actuates anything -- outputs are candidates/requests only,
consumed by node 10 (safety, built by another agent in this session), which is the actual
authority. See CONTRACT.md.
"""
from __future__ import annotations

import json
import time
from math import hypot

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from autokart_msgs.msg import ConeArray, SafetyEnvelope

from .algorithms import build_algorithm_backend, AlgorithmBackendError
from .planner_core import PlannerCore
from .geometry_params import default_geometry, load_geometry, VehicleGeometryError

PATH_TIMER_PERIOD_S = 0.1


def ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ros_time(stamp_ns: int):
    from builtin_interfaces.msg import Time
    stamp_ns = max(0, int(stamp_ns))
    return Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


class PlannerNode(Node):
    def __init__(self, **node_kwargs):
        # **node_kwargs (e.g. parameter_overrides=[...]) is threaded through to rclpy.Node so
        # tests can construct this node with specific parameters deterministically, the same
        # pattern autokart_hp60c's HP60CNode uses for its fault-injection tests.
        super().__init__("autokart_planner", **node_kwargs)
        self.declare_parameter("algorithm_backend", "builtin")
        self.declare_parameter("algorithm_package_path", "")
        self.declare_parameter("vehicle_geometry_yaml_path", "")
        self.declare_parameter("footprint_width_m", 0.0)  # 0.0 => use geometry-derived default
        # UNVERIFIED nominal defaults -- see SENIOR_REVIEWER_REPORT.md. This repository has NOT
        # measured peak lateral grip or a guaranteed minimum braking deceleration for this kart
        # (see PROJECT_INDEX.md: "stopping performance remain unknown or unverified"). Leaving
        # a_brake_min_mps2 at 0.0 makes the node fail closed (BRAKE_CAPABILITY_UNQUALIFIED) until
        # an operator explicitly configures a measured value -- it is not a guessed nonzero number.
        self.declare_parameter("a_lat_max_mps2", 2.0)  # UNVERIFIED nominal, see CONTRACT.md
        self.declare_parameter("a_brake_min_mps2", 0.0)  # 0.0 = unconfigured -> fails closed
        self.declare_parameter("t_total_latency_s", 0.0)  # 0.0 = unconfigured
        self.declare_parameter("d_margin_m", 1.0)  # commissioning default, see CONTRACT.md
        self.declare_parameter("speed_limit_mps", 1.0)
        self.declare_parameter("curvature_eps_m_inv", 1e-3)
        self.declare_parameter("path_max_age_s", 0.5)
        self.declare_parameter("odom_max_age_s", 0.15)
        self.declare_parameter("odom_topic", "/autokart/odom")
        self.declare_parameter("reference_path_topic", "/autokart_planner/reference_path")
        self.declare_parameter("obstacles_topic", "/autokart_planner/obstacles")
        self.declare_parameter("source", "SIMULATED")

        self.source = str(self.get_parameter("source").value)
        self.odom_max_age_ns = int(float(self.get_parameter("odom_max_age_s").value) * 1e9)

        geometry_yaml_path = str(self.get_parameter("vehicle_geometry_yaml_path").value)
        self.startup_error = None
        try:
            geometry = load_geometry(geometry_yaml_path) if geometry_yaml_path else default_geometry()
        except VehicleGeometryError as exc:
            geometry = None
            self.startup_error = f"VEHICLE_GEOMETRY_ERROR:{exc}"
            self.get_logger().error(f"autokart_planner startup failed: {self.startup_error}")

        configured_footprint = float(self.get_parameter("footprint_width_m").value)
        footprint_width_m = configured_footprint if configured_footprint > 0 else (
            geometry.footprint_width_m if geometry is not None else 0.0)

        backend_name = str(self.get_parameter("algorithm_backend").value)
        self.engine = None
        if self.startup_error is None:
            try:
                algorithm = build_algorithm_backend(
                    backend_name,
                    algorithm_package_path=str(self.get_parameter("algorithm_package_path").value))
                self.engine = PlannerCore(
                    algorithm=algorithm,
                    a_lat_max_mps2=float(self.get_parameter("a_lat_max_mps2").value),
                    a_brake_min_mps2=float(self.get_parameter("a_brake_min_mps2").value),
                    t_total_latency_s=float(self.get_parameter("t_total_latency_s").value),
                    d_margin_m=float(self.get_parameter("d_margin_m").value),
                    footprint_width_m=footprint_width_m,
                    speed_limit_mps=float(self.get_parameter("speed_limit_mps").value),
                    curvature_eps_m_inv=float(self.get_parameter("curvature_eps_m_inv").value),
                    max_path_age_ns=int(float(self.get_parameter("path_max_age_s").value) * 1e9),
                )
            except AlgorithmBackendError as exc:
                self.startup_error = str(exc)
                self.get_logger().error(f"autokart_planner startup failed: {self.startup_error}")

        self.pose = None  # (x, y, speed_mps, received_wall_s)
        self.obstacles: list = []
        self.cycle = 0
        # Initialized to "now", not None: if no reference path ever arrives after startup, the
        # path_max_age_s timeout in on_timer must still fire and keep reporting
        # PATH_MISSING_OR_STALE -- a node that only starts its staleness clock on the FIRST
        # received path would stay completely silent (no envelope published at all) for as long as
        # upstream never sends anything, which is indistinguishable from "not started yet" to a
        # downstream consumer and is exactly the kind of silent-until-first-input gap this
        # library's fault-injection convention (autokart_hp60c's _capture_loop precedent) exists to
        # catch. Found by this review; see test_fault_injection.py.
        self.last_path_wall_s = time.monotonic()

        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Path, str(self.get_parameter("reference_path_topic").value),
                                 self.on_path, 10)
        self.create_subscription(ConeArray, str(self.get_parameter("obstacles_topic").value),
                                 self.on_obstacles, qos_profile_sensor_data)

        self.path_pub = self.create_publisher(Path, "/autokart_planner/path", 10)
        self.envelope_pub = self.create_publisher(SafetyEnvelope, "/autokart_planner/safety_envelope", 10)
        # std_msgs/String with a documented JSON payload, not autokart_msgs/MissionDebug: see
        # DEPENDENCIES.md "Environment finding" -- MissionDebug.msg exists in the canonical source
        # tree (kart/ros2_ws/src/autokart_msgs/msg/MissionDebug.msg) but is verifiably NOT built
        # into the kart/ros2_ws install this library's own README says to source (a stale
        # install/source drift discovered during this node's development, not introduced by it).
        # std_msgs/String is the most standard message available for a JSON side-channel; see
        # CONTRACT.md for the payload schema and the migration note if MissionDebug is ever rebuilt.
        self.debug_pub = self.create_publisher(String, "/autokart_planner/speed_profile", 10)

        self.create_timer(PATH_TIMER_PERIOD_S, self.on_timer)

    def on_odom(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        speed = hypot(linear.x, linear.y)
        self.pose = (float(message.pose.pose.position.x), float(message.pose.pose.position.y),
                     float(speed), time.monotonic())

    def on_obstacles(self, message: ConeArray) -> None:
        if message.frame_id != "odom" or message.ground_truth:
            # Obstacles must already be expressed in the same frame as the reference path (see
            # CONTRACT.md "Frames"); this node does no frame transform. A mismatched frame is
            # logged and the update is dropped rather than silently misinterpreted.
            self.get_logger().warning(
                f"autokart_planner: ignoring obstacles with frame_id={message.frame_id!r} "
                f"ground_truth={message.ground_truth} (expected frame_id='odom', ground_truth=False)")
            return
        self.obstacles = [
            {"x_m": cone.position.x, "y_m": cone.position.y,
             "radius_m": max(0.0, cone.position_std_m)}
            for cone in message.cones
        ]

    def on_path(self, message: Path) -> None:
        self.last_path_wall_s = time.monotonic()
        stamp_ns = ns(message.header.stamp)
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self._process(points, stamp_ns)

    def on_timer(self) -> None:
        if self.engine is None:
            # Keep reporting the startup fault for as long as the node runs, rather than
            # publishing one status and then going silent forever (see autokart_hp60c's
            # _capture_loop precedent for the same convention and its reviewer-found bug).
            self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
            return
        now = time.monotonic()
        # Deliberately does NOT reset last_path_wall_s to None after firing once: as long as no
        # new path arrives, every timer tick re-publishes PATH_MISSING_OR_STALE, so a downstream
        # consumer never sees a single stale report followed by silence.
        if self.last_path_wall_s is not None and now - self.last_path_wall_s > \
                float(self.get_parameter("path_max_age_s").value):
            self.publish(self.engine.reject("PATH_MISSING_OR_STALE", self.get_clock().now().nanoseconds))

    def _process(self, points: list, stamp_ns: int) -> None:
        if self.engine is None:
            self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.pose is None or time.monotonic() - self.pose[3] > self.odom_max_age_ns / 1e9:
            result = self.engine.reject("ODOM_MISSING_OR_STALE", stamp_ns)
        else:
            pose_x, pose_y, pose_speed, _ = self.pose
            result = self.engine.evaluate(
                path_points=points, path_stamp_ns=stamp_ns, now_ns=now_ns,
                pose_x=pose_x, pose_y=pose_y, pose_speed_mps=pose_speed, pose_fresh=True,
                obstacles=self.obstacles,
            )
        self.publish(result)

    def _publish_fault(self, reason: str) -> None:
        self.cycle += 1
        stamp_ns = self.get_clock().now().nanoseconds
        result = {
            "valid": False, "reason_codes": [reason], "decision": "HOLD",
            "path_odom": [], "speed_profile": [], "curvature_profile": [],
            "sample_time_ns": stamp_ns, "sequence": self.cycle,
            "maximum_speed_mps": 0.0, "clear_distance_m": 0.0,
            "stopping_distance_m": None, "limiting_reason": reason,
        }
        self.publish(result)

    def publish(self, result: dict) -> None:
        self.cycle += 1
        stamp_ns = max(0, result["sample_time_ns"])
        path = Path()
        path.header.stamp = ros_time(stamp_ns)
        path.header.frame_id = "odom"
        for x, y in result["path_odom"]:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        envelope = SafetyEnvelope()
        envelope.timestamp = ros_time(stamp_ns)
        envelope.frame_id = "odom"
        envelope.maximum_speed_mps = float(result["maximum_speed_mps"])
        envelope.maximum_acceleration_mps2 = 0.0  # not computed by this node; see CONTRACT.md
        guaranteed_decel = self.engine.a_brake_min_mps2 if self.engine is not None else 0.0
        envelope.guaranteed_deceleration_mps2 = float(guaranteed_decel)
        envelope.clear_distance_m = float(result["clear_distance_m"])
        stopping = result.get("stopping_distance_m")
        envelope.stopping_distance_m = float(stopping) if stopping is not None else 0.0
        lateral_limit = self.engine.a_lat_max_mps2 if self.engine is not None else 0.0
        envelope.lateral_limit_mps2 = float(lateral_limit)
        envelope.limiting_reason = str(result["limiting_reason"])
        envelope.valid = bool(result["valid"])
        # autokart_msgs/SafetyEnvelope.msg (source tree, kart/ros2_ws/src/autokart_msgs) declares a
        # `data_source` field, but the already-built install this node runs against in this dev
        # environment (kart/ros2_ws/install, sourced per this library's README) was generated from
        # an older version of that .msg file and does NOT have it -- a stale-install/source drift
        # discovered during this node's own integration testing, not introduced by this change (see
        # DEPENDENCIES.md "Environment finding"). Guarded so this node works against either: the
        # provenance is always carried in the speed-profile JSON payload's "source" field regardless.
        if hasattr(envelope, "data_source"):
            envelope.data_source = self.source
        self.envelope_pub.publish(envelope)

        payload = {
            "schema_version": 1, "source": self.source,
            "processed_at_monotonic_ns": time.monotonic_ns(),
            "valid": bool(result["valid"]), "reason_codes": result["reason_codes"],
            "decision": result["decision"], "sample_time_ns": stamp_ns, "sequence": result["sequence"],
            "frame_id": "odom", "stage": "PLANNING",
            "speed_profile_mps": [float(v) for v in result["speed_profile"]],
            "curvature_profile_m_inv": [float(k) for k in result["curvature_profile"]],
            "vehicle_authorized": False, "published_to_vcu": False,
        }
        debug = String()
        debug.data = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.debug_pub.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
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
