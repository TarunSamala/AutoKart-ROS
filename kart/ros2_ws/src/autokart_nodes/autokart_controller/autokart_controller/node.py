"""ROS transport for a disarmed Pure Pursuit command candidate.

Publishes ``autokart_msgs/MotionCommand`` only to a candidate topic. No
VCU/actuator transport subscribes to that topic and this node is not part of
``full_stack.launch.py``; it carries no vehicle authority.
"""

from __future__ import annotations

import json
import time
from math import atan2, cos, sin, sqrt

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from autokart_msgs.msg import ActuatorFeedback, MissionDebug, MotionCommand as RosMotionCommand
from kart_sim.types import PlanarKartState
from nav_msgs.msg import Odometry, Path

from autokart_mapping.node import ns, ros_time
from .engine import ControllerEngine

STATUS_PAIR_TIMEOUT_S = 0.3
INPUT_TIMEOUT_S = 0.5


def planar_state_from_odom(message: Odometry) -> tuple[PlanarKartState, bool]:
    q = message.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    linear = message.twist.twist.linear
    state = PlanarKartState(
        time_s=ns(message.header.stamp) / 1e9,
        x_m=float(message.pose.pose.position.x),
        y_m=float(message.pose.pose.position.y),
        yaw_rad=atan2(siny, cosy),
        u_mps=sqrt(linear.x * linear.x + linear.y * linear.y),
        yaw_rate_radps=float(message.twist.twist.angular.z),
    )
    valid = message.child_frame_id == "base_link" and message.header.frame_id == "odom"
    return state, valid


class ControllerNode(Node):
    def __init__(self):
        super().__init__("autokart_controller")
        self.declare_parameter("source", "SIMULATED")
        self.declare_parameter("wheelbase_m", 1.0)
        self.declare_parameter("lookahead_m", 1.5)
        self.declare_parameter("maximum_steering_rad", 0.6)
        self.declare_parameter("maximum_steering_rate_radps", 3.0)
        self.declare_parameter("maximum_speed_mps", 5.0)
        self.declare_parameter("acceleration_limit_mps2", 3.0)
        self.declare_parameter("deceleration_limit_mps2", 5.0)
        self.engine = ControllerEngine(
            source=str(self.get_parameter("source").value),
            wheelbase_m=float(self.get_parameter("wheelbase_m").value),
            lookahead_m=float(self.get_parameter("lookahead_m").value),
            maximum_steering_rad=float(self.get_parameter("maximum_steering_rad").value),
            maximum_steering_rate_radps=float(self.get_parameter("maximum_steering_rate_radps").value),
            maximum_speed_mps=float(self.get_parameter("maximum_speed_mps").value),
            acceleration_limit_mps2=float(self.get_parameter("acceleration_limit_mps2").value),
            deceleration_limit_mps2=float(self.get_parameter("deceleration_limit_mps2").value),
        )
        self.paths: dict[int, list[tuple[float, float]]] = {}
        self.statuses: dict[int, tuple[dict, float]] = {}
        self.pose: PlanarKartState | None = None
        self.pose_valid = False
        self.pose_time_ns: int | None = None
        self.feedback = None
        self.cycle = 0
        self.last_compute_wall_s: float | None = None
        self.last_update = None

        self.create_subscription(Path, "/autokart/planner/path_candidate", self.on_path, qos_profile_sensor_data)
        self.create_subscription(MissionDebug, "/autokart/planner/status", self.on_status, 10)
        self.create_subscription(Odometry, "/autokart/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(ActuatorFeedback, "/vehicle/actuator_feedback", self.on_feedback, 10)
        self.command_pub = self.create_publisher(RosMotionCommand, "/autokart/controller/motion_command_candidate", 10)
        self.status_pub = self.create_publisher(MissionDebug, "/autokart/controller/status", 10)
        self.dashboard_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
        self.create_timer(0.1, self.on_timer)

    def on_path(self, message: Path) -> None:
        stamp = ns(message.header.stamp)
        self.paths[stamp] = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        if len(self.paths) > 8:
            del self.paths[min(self.paths)]
        self._maybe_process(stamp)

    def on_status(self, message: MissionDebug) -> None:
        try:
            payload = json.loads(message.payload_json)
        except (ValueError, TypeError):
            self.publish(self.engine.reject("PLANNER_STATUS_MALFORMED", ns(message.measurement_time)))
            return
        stamp = ns(message.measurement_time)
        if payload.get("schema_version") != 1 or payload.get("sample_time_ns") != stamp:
            self.publish(self.engine.reject("PLANNER_STATUS_SCHEMA_INVALID", stamp))
            return
        self.statuses[stamp] = (payload, time.monotonic())
        if len(self.statuses) > 8:
            del self.statuses[min(self.statuses)]
        self._maybe_process(stamp)

    def on_odom(self, message: Odometry) -> None:
        state, valid = planar_state_from_odom(message)
        self.pose = state
        self.pose_valid = valid
        self.pose_time_ns = ns(message.header.stamp)

    def on_feedback(self, message: ActuatorFeedback) -> None:
        self.feedback = message

    def _maybe_process(self, stamp: int) -> None:
        path = self.paths.get(stamp)
        pair = self.statuses.get(stamp)
        if path is None or pair is None:
            return
        payload, _ = pair
        self.paths.pop(stamp, None)
        self.statuses.pop(stamp, None)
        now_ns = self.pose_time_ns if self.pose_time_ns is not None else stamp
        wall_now = time.monotonic()
        dt_s = 0.1 if self.last_compute_wall_s is None else max(1e-3, wall_now - self.last_compute_wall_s)
        self.last_compute_wall_s = wall_now
        brake_stuck = self.feedback is not None and not self.feedback.valid
        result = self.engine.evaluate(
            path_points=path,
            path_stamp_ns=stamp,
            planner_status=payload,
            pose=self.pose,
            pose_valid=self.pose_valid,
            pose_time_ns=self.pose_time_ns,
            now_ns=now_ns,
            dt_s=dt_s,
            brake_stuck=brake_stuck,
        )
        self.last_update = wall_now
        self.publish(result)

    def on_timer(self) -> None:
        now = time.monotonic()
        expired_status = [stamp for stamp, (_, received) in self.statuses.items() if now - received > STATUS_PAIR_TIMEOUT_S]
        for stamp in expired_status:
            self.statuses.pop(stamp, None)
            self.paths.pop(stamp, None)
            self.publish(self.engine.reject("CONTROLLER_INPUT_PAIR_TIMEOUT", stamp))
        if self.last_update is not None and now - self.last_update > INPUT_TIMEOUT_S:
            self.last_update = None
            self.publish(self.engine.reject("CONTROLLER_INPUT_TIMEOUT", self.get_clock().now().nanoseconds))

    def publish(self, result: dict) -> None:
        self.cycle += 1
        stamp = max(0, result["sample_time_ns"])
        command = RosMotionCommand()
        command.timestamp = ros_time(stamp)
        command.expiry = ros_time(stamp)
        command.source = f"autokart_controller/{self.engine.source}"
        command.sequence = result["sequence"]
        command.mode = "speed_steering"
        command.target_speed_mps = result["target_speed_mps"]
        command.steering_angle_rad = result["steering_angle_rad"]
        command.curvature_1pm = 0.0
        command.use_curvature = False
        command.acceleration_limit_mps2 = result["acceleration_limit_mps2"]
        command.deceleration_limit_mps2 = result["deceleration_limit_mps2"]
        command.valid = result["valid"]
        command.integrity = result["integrity"] & 0xFFFFFFFF
        self.command_pub.publish(command)

        payload = {
            "schema_version": 1,
            "processed_at_monotonic_ns": time.monotonic_ns(),
            "source": self.engine.source,
            "valid": bool(result["valid"]),
            "reason_codes": result["reason_codes"],
            "frame_id": "odom",
            "sample_time_ns": stamp,
            "sequence": self.cycle,
            "target_speed_mps": result["target_speed_mps"],
            "steering_angle_rad": result["steering_angle_rad"],
            "mission_state": result.get("mission_state"),
            "decision": result.get("decision"),
            "vehicle_authorized": False,
            "published_to_vcu": False,
        }
        message = MissionDebug()
        message.measurement_time = ros_time(stamp)
        message.cycle = self.cycle
        message.frame_id = "odom"
        message.stage = "CONTROL"
        message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(message)
        message.stage = "control"
        self.dashboard_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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
