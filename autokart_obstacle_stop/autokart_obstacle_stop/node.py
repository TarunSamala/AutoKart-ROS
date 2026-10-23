"""ROS 2 command guard: qualified forward cone detection forces speed to zero."""

from __future__ import annotations

import json
import time

import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from autokart_msgs.msg import ConeArray, MotionCommand

from .core import ConeSample, GuardConfig, ObstacleStopGuard


def ros_time(value_ns: int) -> RosTime:
    stamp = RosTime()
    stamp.sec, stamp.nanosec = divmod(max(0, int(value_ns)), 1_000_000_000)
    return stamp


class ObstacleStopNode(Node):
    """Filters a controller candidate before the existing VCU safety gate."""

    def __init__(self, *, start_timer: bool = True, **node_kwargs):
        super().__init__("autokart_obstacle_stop", **node_kwargs)
        declare = self.declare_parameter
        declare("cones_topic", "/perception/cones")
        declare("candidate_topic", "/autokart_controller/motion_command_candidate")
        declare("output_topic", "/autokart/obstacle_stop/motion_command_candidate")
        declare("status_topic", "/autokart/obstacle_stop/status")
        declare("minimum_x_m", 0.10)
        declare("stop_distance_m", 3.0)
        declare("corridor_half_width_m", 0.75)
        declare("minimum_confidence", 0.50)
        declare("perception_timeout_s", 0.50)
        declare("command_timeout_s", 0.25)
        declare("stop_hold_s", 1.0)
        declare("clear_frames_required", 3)
        declare("allow_ground_truth", False)
        declare("publish_hz", 20.0)
        declare("command_validity_s", 0.10)

        config = GuardConfig(
            minimum_x_m=float(self.get_parameter("minimum_x_m").value),
            stop_distance_m=float(self.get_parameter("stop_distance_m").value),
            corridor_half_width_m=float(self.get_parameter("corridor_half_width_m").value),
            minimum_confidence=float(self.get_parameter("minimum_confidence").value),
            perception_timeout_s=float(self.get_parameter("perception_timeout_s").value),
            stop_hold_s=float(self.get_parameter("stop_hold_s").value),
            clear_frames_required=int(self.get_parameter("clear_frames_required").value),
            allow_ground_truth=bool(self.get_parameter("allow_ground_truth").value),
        )
        self.guard = ObstacleStopGuard(config)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.validity_ns = int(float(self.get_parameter("command_validity_s").value) * 1e9)
        self.output_sequence = 0
        self.candidate = None
        self.candidate_wall_s = None

        self.command_pub = self.create_publisher(
            MotionCommand, str(self.get_parameter("output_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            ConeArray, str(self.get_parameter("cones_topic").value), self.on_cones,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            MotionCommand, str(self.get_parameter("candidate_topic").value), self.on_candidate, 10
        )

        self.timer = None
        if start_timer:
            publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
            self.timer = self.create_timer(1.0 / publish_hz, self.on_timer)

    def on_cones(self, message: ConeArray) -> None:
        samples = [
            ConeSample(
                cone_id=int(cone.id),
                x_m=float(cone.position.x),
                y_m=float(cone.position.y),
                confidence=float(cone.confidence),
                position_std_m=float(cone.position_std_m),
            )
            for cone in message.cones
        ]
        self.guard.ingest_frame(
            time.monotonic(), frame_id=str(message.frame_id),
            ground_truth=bool(message.ground_truth), cones=samples,
        )

    def on_candidate(self, message: MotionCommand) -> None:
        self.candidate = message
        self.candidate_wall_s = time.monotonic()

    @staticmethod
    def _copy_command(source: MotionCommand) -> MotionCommand:
        output = MotionCommand()
        output.timestamp = source.timestamp
        output.expiry = source.expiry
        output.source = source.source
        output.sequence = source.sequence
        output.mode = source.mode
        output.target_speed_mps = source.target_speed_mps
        output.steering_angle_rad = source.steering_angle_rad
        output.curvature_1pm = source.curvature_1pm
        output.use_curvature = source.use_curvature
        output.acceleration_limit_mps2 = source.acceleration_limit_mps2
        output.deceleration_limit_mps2 = source.deceleration_limit_mps2
        output.valid = source.valid
        output.integrity = source.integrity
        return output

    def _stop_command(self, now_ns: int, *, valid: bool, source_command=None) -> MotionCommand:
        output = (
            self._copy_command(source_command)
            if source_command is not None
            else MotionCommand()
        )
        output.timestamp = ros_time(now_ns)
        output.expiry = ros_time(now_ns + self.validity_ns if valid else now_ns)
        output.source = "autokart_obstacle_stop"
        self.output_sequence = (self.output_sequence + 1) & 0xFFFFFFFF
        output.sequence = self.output_sequence
        output.mode = "speed_steering"
        output.target_speed_mps = 0.0
        output.steering_angle_rad = 0.0
        output.curvature_1pm = 0.0
        output.use_curvature = False
        output.acceleration_limit_mps2 = 0.0
        output.deceleration_limit_mps2 = 0.0
        output.valid = bool(valid)
        output.integrity = 0
        return output

    def on_timer(self) -> None:
        now_wall = time.monotonic()
        now_ns = self.get_clock().now().nanoseconds
        decision = self.guard.decision(now_wall)
        command_age = None if self.candidate_wall_s is None else now_wall - self.candidate_wall_s
        command_fresh = bool(
            self.candidate is not None and command_age is not None
            and command_age <= self.command_timeout_s
        )

        if not command_fresh:
            output = self._stop_command(now_ns, valid=False)
            action = "STOP_INVALID"
            reason = "COMMAND_MISSING_OR_STALE"
        elif decision.stop_required:
            # A qualified obstacle is an intentional valid zero-speed request. A stale or
            # unqualified perception source is fail-closed and therefore invalid.
            qualified_obstacle = decision.reason in {
                "OBSTACLE_IN_FORWARD_CORRIDOR", "OBSTACLE_STOP_HOLD"
            }
            output = self._stop_command(
                now_ns,
                valid=bool(qualified_obstacle and self.candidate.valid),
                source_command=self.candidate,
            )
            action = "STOP"
            reason = decision.reason
        else:
            output = self._copy_command(self.candidate)
            self.output_sequence = (self.output_sequence + 1) & 0xFFFFFFFF
            output.timestamp = ros_time(now_ns)
            output.expiry = ros_time(now_ns + self.validity_ns)
            output.source = "autokart_obstacle_stop/" + str(self.candidate.source)
            output.sequence = self.output_sequence
            output.integrity = 0
            action = "PASS"
            reason = decision.reason

        self.command_pub.publish(output)
        status = String()
        status.data = json.dumps({
            "schema_version": 1,
            "action": action,
            "reason": reason,
            "stop_required": bool(decision.stop_required),
            "nearest_clearance_m": decision.nearest_clearance_m,
            "cone_id": decision.cone_id,
            "perception_age_s": decision.perception_age_s,
            "command_age_s": command_age,
            "output_valid": bool(output.valid),
            "output_speed_mps": float(output.target_speed_mps),
            "vehicle_authorized": False,
            "published_to_vcu": False,
        }, sort_keys=True, allow_nan=False)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
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
