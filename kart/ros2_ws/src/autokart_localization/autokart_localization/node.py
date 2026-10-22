"""ROS transport for wheel-encoder odometry, sourced from the real VCU's axle encoder + observed steering.

Fills the previously-empty autokart_localization package shell. Publishes nav_msgs/Odometry on /autokart/odom
(consumed by autokart_mapping, autokart_planner, autokart_controller -- see docs/NODE_CONTRACTS.md) and on
/vehicle/odometry (consumed by autokart_dashboard_gateway in its default REAL_KART source mode). A paired
MissionDebug status companion (stage LOCALIZATION / localization) carries the status field
(ENCODER_STEER_MODEL / ENCODER_ONLY_NO_YAW / INVALID) that nav_msgs/Odometry has no room for, following the
same paired-status convention the perception/mapping/planner/controller packages already use.

If the axle encoder feedback is missing, stale, or itself reports invalid, this node stops publishing
/autokart/odom and /vehicle/odometry entirely (rather than republishing a frozen, increasingly wrong pose) and
publishes an explicit invalid status instead -- downstream freshness checks then halt on their own.
"""
from __future__ import annotations

import json
import math
import time


def main(args=None) -> None:
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from autokart_msgs.msg import ActuatorFeedback, MissionDebug
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit(f"NOT_EXECUTED_EXTERNAL_DEPENDENCY: {exc}") from exc

    from autokart_mapping.node import ns, ros_time
    from .odometry import OdometryConfig, STATUS_INVALID, WheelOdometry

    FEEDBACK_TIMEOUT_S = 0.3

    class LocalizationNode(Node):
        def __init__(self) -> None:
            super().__init__("autokart_localization")
            p = self.declare_parameter
            self.wheelbase_m = float(p("wheelbase_m", 1.0).value)
            self.counts_per_m = float(p("counts_per_m", 0.0).value)
            self.encoder_forward_sign = int(p("encoder_forward_sign", 1).value)
            self.position_variance_m2 = float(p("position_variance_m2", 0.25).value)
            self.yaw_variance_rad2 = float(p("yaw_variance_rad2", 0.25).value)
            self.odom = WheelOdometry(OdometryConfig(wheelbase_m=self.wheelbase_m,
                                                     counts_per_m=self.counts_per_m,
                                                     encoder_forward_sign=self.encoder_forward_sign))
            self.last_feedback_stamp_ns: int | None = None
            self.last_feedback_wall: float | None = None
            self.last_count_wall: float | None = None
            self.feedback_valid = False
            self.last_count_sample: tuple[int, int, int] | None = None
            self.latest_steering_rad: float | None = None
            self.cycle = 0

            self.odom_pub = self.create_publisher(Odometry, "/autokart/odom", qos_profile_sensor_data)
            self.vehicle_odom_pub = self.create_publisher(Odometry, "/vehicle/odometry", qos_profile_sensor_data)
            self.status_pub = self.create_publisher(MissionDebug, "/autokart/localization/status", 10)
            self.dashboard_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
            self.create_subscription(ActuatorFeedback, "/vehicle/actuator_feedback", self.on_feedback,
                                     qos_profile_sensor_data)
            self.create_subscription(String, "/autokart/vcu/axle_counts", self.on_counts, 10)
            self.create_timer(0.3, self.on_timer)  # catches a feedback stream that stopped arriving entirely

        def on_feedback(self, message: ActuatorFeedback) -> None:
            stamp_ns = ns(message.measurement_time)
            now_wall = time.monotonic()
            dt_s = 0.05 if self.last_feedback_stamp_ns is None else max(
                1e-3, (stamp_ns - self.last_feedback_stamp_ns) / 1e9)
            self.last_feedback_stamp_ns = stamp_ns
            self.last_feedback_wall = now_wall
            self.feedback_valid = bool(message.valid)

            if not message.valid:
                self.odom.status = STATUS_INVALID
                self.publish_status(stamp_ns, False, ["FEEDBACK_INVALID"])
                return

            self.latest_steering_rad = message.road_wheel_angle_rad
            if self.counts_per_m > 0:
                return  # calibrated mode publishes only from fresh raw counts

            ok = self.odom.update(dt_s, message.motor_speed_radps, message.road_wheel_angle_rad)
            if not ok:
                self.publish_status(stamp_ns, False, ["TIMING_OR_SPEED_INVALID"])
                return

            self.publish_odom(stamp_ns)
            self.publish_status(stamp_ns, True, ["CPR_X_TYRE_RADIUS_FALLBACK_VERIFY"])

        def on_counts(self, message: String) -> None:
            if self.counts_per_m <= 0:
                return
            try:
                sample = json.loads(message.data)
                stamp_ns, counts, fwseq = (int(sample[key]) for key in ("stamp_ns", "counts", "fwseq"))
            except (ValueError, TypeError, KeyError):
                self.odom.status = STATUS_INVALID
                self.publish_status(self.get_clock().now().nanoseconds, False, ["AXLE_COUNT_MALFORMED"])
                return
            previous = self.last_count_sample
            self.last_count_sample = (stamp_ns, counts, fwseq)
            self.last_count_wall = time.monotonic()
            if (not self.feedback_valid or self.last_feedback_wall is None or
                    self.last_count_wall - self.last_feedback_wall > FEEDBACK_TIMEOUT_S):
                self.odom.status = STATUS_INVALID
                self.publish_status(stamp_ns, False, ["FEEDBACK_INVALID_OR_STALE"])
                return
            if previous is None:
                return  # first count anchors position, it cannot measure travel
            prior_ns, prior_counts, prior_seq = previous
            dt_s = (stamp_ns - prior_ns) / 1e9
            delta = counts - prior_counts
            if fwseq <= prior_seq or not self.odom.update_counts(dt_s, delta, self.latest_steering_rad):
                self.odom.status = STATUS_INVALID
                self.publish_status(stamp_ns, False, ["AXLE_COUNT_SEQUENCE_OR_TIMING_INVALID"])
                return
            self.publish_odom(stamp_ns)
            self.publish_status(stamp_ns, True, [])

        def on_timer(self) -> None:
            stale = (self.last_feedback_wall is None or
                    time.monotonic() - self.last_feedback_wall > FEEDBACK_TIMEOUT_S or
                    (self.counts_per_m > 0 and (self.last_count_wall is None or
                     time.monotonic() - self.last_count_wall > FEEDBACK_TIMEOUT_S)))
            if stale:
                self.odom.status = STATUS_INVALID
                self.publish_status(self.get_clock().now().nanoseconds, False, ["FEEDBACK_STALE_OR_ABSENT"])

        def publish_odom(self, stamp_ns: int) -> None:
            message = Odometry()
            message.header.stamp = ros_time(stamp_ns)
            message.header.frame_id = "odom"
            message.child_frame_id = "base_link"
            message.pose.pose.position.x = self.odom.x_m
            message.pose.pose.position.y = self.odom.y_m
            message.pose.pose.orientation.z = float(math.sin(self.odom.yaw_rad / 2))
            message.pose.pose.orientation.w = float(math.cos(self.odom.yaw_rad / 2))
            # Conservative, NOT measured, placeholder covariance -- see position_variance_m2/yaw_variance_rad2.
            cov = [0.0] * 36
            cov[0] = cov[7] = self.position_variance_m2
            cov[35] = self.yaw_variance_rad2
            message.pose.covariance = cov
            message.twist.twist.linear.x = self.odom.speed_mps
            message.twist.twist.angular.z = self.odom.yaw_rate_radps
            self.odom_pub.publish(message)
            self.vehicle_odom_pub.publish(message)

        def publish_status(self, stamp_ns: int, valid: bool, reason_codes: list[str]) -> None:
            self.cycle += 1
            payload = {
                "schema_version": 1, "source": "LIVE", "valid": bool(valid),
                "reason_codes": reason_codes, "frame_id": "odom", "sample_time_ns": max(0, stamp_ns),
                "sequence": self.cycle, "status": self.odom.status,
                "wheelbase_m": self.wheelbase_m,
                "counts_per_m": self.counts_per_m if self.counts_per_m > 0 else None,
                "encoder_forward_sign": self.encoder_forward_sign,
                "distance_source": "CALIBRATED_COUNTS" if self.counts_per_m > 0 else "CPR_X_TYRE_RADIUS_FALLBACK_VERIFY",
                "speed_mps": self.odom.speed_mps if valid else None,
                "vehicle_authorized": False,
            }
            message = MissionDebug()
            message.measurement_time = ros_time(max(0, stamp_ns))
            message.cycle = self.cycle
            message.frame_id = "odom"
            message.stage = "LOCALIZATION"
            message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
            self.status_pub.publish(message)
            message.stage = "localization"
            self.dashboard_pub.publish(message)

    rclpy.init(args=args)
    node = LocalizationNode()
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
