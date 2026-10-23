"""autokart_odometry: wheel-encoder + observed-steering odometry (standalone library node).

Wraps the existing, already-tested ``autokart_localization.odometry.WheelOdometry``
(bicycle model; imported lazily from ``algorithm_package_path``, never
reimplemented) and publishes ``nav_msgs/Odometry`` on ``/odom`` plus the single
``odom -> base_link`` TF. This node is the ONLY publisher of that TF in this
package. **Running it alongside any other odom->base_link writer -- in
kart/ros2_ws that is the Gazebo follower plugin; the production
autokart_localization node publishes Odometry but no TF -- creates a
duplicate-TF-writer hazard the operator must avoid** (set publish_tf:=false or
run only one). This node does not and cannot detect or resolve that
system-level conflict (see CONTRACT.md).

NO IMU FUSION: yaw is a bicycle-model estimate from observed steering, never a
measurement, and the autokart_imu node's gyro is not consumed here. When
steering is unavailable, yaw is frozen (``ENCODER_ONLY_NO_YAW``), never
guessed. When speed/timing is invalid, no odometry and no TF are published at
all (never a frozen, increasingly-wrong pose).

Input source (``source`` parameter): ``synthetic`` (deterministic, no
hardware) or ``actuator_feedback`` (node 04's ``/vehicle/actuator_feedback``).
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from autokart_msgs.msg import ActuatorFeedback, SensorDeliveryMetadata

from .backends import OdometryBackendError, OdomSample, SyntheticSampleSource, load_wheel_odometry

UNOBSERVED_VARIANCE = 1.0e6  # planar odometry: z/roll/pitch (and lateral/vertical twist) are not observed


def _ns(t) -> int:
    return int(t.sec) * 1_000_000_000 + int(t.nanosec)


class OdometryNode(Node):
    def __init__(self, start_timers: bool = True, **node_kwargs):
        super().__init__("autokart_odometry", **node_kwargs)
        p = self.declare_parameter
        p("source", "synthetic")
        p("algorithm_package_path", "")
        p("wheelbase_m", 0.0)          # MEASURE: 0.0 = not configured -> fails closed (no guessed default)
        p("counts_per_m", 0.0)         # 0 = speed mode (measured speed); >0 = calibrated-counts mode
        p("encoder_forward_sign", 1)
        p("position_variance_m2", 0.25)      # placeholder, NOT measured
        p("yaw_variance_rad2", 0.25)         # placeholder, NOT measured
        p("speed_variance_m2ps2", 0.25)      # placeholder, NOT measured
        p("yaw_rate_variance_rad2ps2", 0.25)  # placeholder, NOT measured
        p("odom_frame_id", "odom")
        p("base_frame_id", "base_link")
        p("publish_tf", True)
        p("input_timeout_s", 0.3)
        p("watchdog_hz", 5.0)
        p("feedback_topic", "/vehicle/actuator_feedback")
        p("synthetic_rate_hz", 50.0)
        p("synthetic_speed_mps", 1.0)
        p("synthetic_steering_rad", 0.0)
        p("synthetic_steering_invalid_after", 0)
        p("synthetic_speed_invalid_after", 0)
        p("synthetic_gap_after", 0)
        p("synthetic_nonmonotonic_after", 0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.source_name = str(g("source"))
        self.wheelbase_m = float(g("wheelbase_m"))
        self.counts_per_m = float(g("counts_per_m"))
        self.encoder_forward_sign = int(g("encoder_forward_sign"))
        self.position_variance = float(g("position_variance_m2"))
        self.yaw_variance = float(g("yaw_variance_rad2"))
        self.speed_variance = float(g("speed_variance_m2ps2"))
        self.yaw_rate_variance = float(g("yaw_rate_variance_rad2ps2"))
        self.odom_frame_id = str(g("odom_frame_id"))
        self.base_frame_id = str(g("base_frame_id"))
        self.publish_tf = bool(g("publish_tf"))
        self.input_timeout_s = float(g("input_timeout_s"))

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/odom/status", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.sequence = 0
        self.cumulative_captured = 0
        self.cumulative_delivered = 0
        self.cumulative_dropped = 0
        self._last_fault = None
        self.prev: OdomSample | None = None
        self.last_input_wall = None
        self.odom = None
        self.algo = None
        self.startup_error = None
        self.synthetic = None
        self._timers = []

        try:
            self.algo = load_wheel_odometry(str(g("algorithm_package_path")))
            # docs/NODE_CONTRACTS.md requires finite pose/twist covariance; a zero/negative/NaN variance
            # would claim certainty (or nonsense) to every downstream consumer. Found in senior review:
            # previously published as configured, unchecked.
            for name, value in (("position_variance_m2", self.position_variance),
                                ("yaw_variance_rad2", self.yaw_variance),
                                ("speed_variance_m2ps2", self.speed_variance),
                                ("yaw_rate_variance_rad2ps2", self.yaw_rate_variance)):
                if not (math.isfinite(value) and value > 0.0):
                    raise OdometryBackendError(f"INVALID_CONFIG:{name} must be finite and > 0, got {value}")
            if self.source_name not in ("synthetic", "actuator_feedback"):
                raise OdometryBackendError(f"UNKNOWN_SOURCE:{self.source_name}")
            if self.counts_per_m > 0 and self.source_name == "actuator_feedback":
                # ActuatorFeedback carries no axle counts (see autokart_vcu_bridge's documented gap):
                # refuse rather than silently switch to speed mode.
                raise OdometryBackendError("COUNTS_MODE_UNSUPPORTED:ActuatorFeedback carries no axle counts")
            try:
                self.odom = self.algo.WheelOdometry(self.algo.OdometryConfig(
                    wheelbase_m=self.wheelbase_m, counts_per_m=self.counts_per_m,
                    encoder_forward_sign=self.encoder_forward_sign))
            except ValueError as exc:  # the reused algorithm's own config validation
                raise OdometryBackendError(f"INVALID_CONFIG:{exc}") from exc
            if self.source_name == "synthetic":
                self.synthetic = SyntheticSampleSource(
                    start_ns=self.get_clock().now().nanoseconds,
                    rate_hz=float(g("synthetic_rate_hz")), speed_mps=float(g("synthetic_speed_mps")),
                    steering_rad=float(g("synthetic_steering_rad")), counts_per_m=self.counts_per_m,
                    steering_invalid_after=int(g("synthetic_steering_invalid_after")),
                    speed_invalid_after=int(g("synthetic_speed_invalid_after")),
                    gap_after=int(g("synthetic_gap_after")),
                    nonmonotonic_after=int(g("synthetic_nonmonotonic_after")))
            else:
                self.create_subscription(ActuatorFeedback, str(g("feedback_topic")), self._on_feedback, 10)
        except OdometryBackendError as exc:
            self.odom = None
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_odometry startup failed: {self.startup_error}")

        if start_timers:
            self.start_timers()

    def start_timers(self):
        """Split out from __init__ so tests can construct the node, adjust it,
        then start deterministically (same pattern as the other nodes)."""
        hz = max(0.5, float(self.get_parameter("watchdog_hz").value))
        self._timers.append(self.create_timer(1.0 / hz, self._watchdog))
        if self.synthetic is not None:
            rate = float(self.get_parameter("synthetic_rate_hz").value)
            self._timers.append(self.create_timer(1.0 / rate, self._on_synthetic_tick))

    # --- inputs -----------------------------------------------------------
    def _on_synthetic_tick(self) -> None:
        self.ingest(self.synthetic.next_sample())

    def _on_feedback(self, message: ActuatorFeedback) -> None:
        # motor_speed_radps carries the rear-axle LINEAR speed in m/s (documented field reuse by the
        # VCU bridge, see docs/VCU_PROTOCOL.md) -- it is not a motor-shaft angular velocity.
        self.ingest(OdomSample(
            stamp_ns=_ns(message.measurement_time), speed_mps=float(message.motor_speed_radps),
            axle_counts=None, steering_rad=float(message.road_wheel_angle_rad),
            valid=bool(message.valid), source="ACTUATOR_FEEDBACK"))

    # --- core ---------------------------------------------------------------
    def _reject(self, reason: str) -> None:
        """Invalidate, re-anchor on the next sample, publish no odometry/TF."""
        self.odom.status = self.algo.STATUS_INVALID
        self.prev = None
        self.cumulative_dropped += 1
        self._publish_fault(reason)

    def ingest(self, sample: OdomSample) -> None:
        if self.odom is None:
            return  # startup fault: the watchdog keeps reporting it
        self.last_input_wall = time.monotonic()
        self.cumulative_captured += 1

        if not sample.valid:
            self._reject("INPUT_INVALID")
            return
        if self.prev is None:
            self.prev = sample  # first sample after (re)start only anchors; it cannot measure travel
            self._publish_status("INVALID", "AWAITING_SECOND_SAMPLE", False)
            return
        dt_s = (sample.stamp_ns - self.prev.stamp_ns) / 1e9
        if dt_s <= 0.0:
            self._reject("NON_MONOTONIC_TIMESTAMP")
            return

        if self.counts_per_m > 0:
            if sample.axle_counts is None or self.prev.axle_counts is None:
                self._reject("AXLE_COUNTS_UNAVAILABLE")
                return
            ok = self.odom.update_counts(dt_s, int(sample.axle_counts - self.prev.axle_counts),
                                         sample.steering_rad)
        else:
            ok = self.odom.update(dt_s, sample.speed_mps, sample.steering_rad)
        if not ok:
            self._reject("TIMING_OR_SPEED_INVALID")
            return
        self.prev = sample

        self._publish_odometry()
        self.sequence += 1
        self.cumulative_delivered += 1
        if self.odom.status == self.algo.STATUS_ENCODER_STEER_MODEL:
            self._publish_status("OK", "NONE", True)
            self._last_fault = None
        else:  # STATUS_ENCODER_ONLY_NO_YAW
            self._publish_status("DEGRADED", "STEERING_UNAVAILABLE_YAW_FROZEN", True)

    def _watchdog(self) -> None:
        if self.odom is None:
            self._publish_fault(self.startup_error or "ALGORITHM_UNAVAILABLE")
            return
        now = time.monotonic()
        if self.last_input_wall is None or now - self.last_input_wall > self.input_timeout_s:
            self.odom.status = self.algo.STATUS_INVALID
            self.prev = None
            self._publish_fault("INPUT_STALE_OR_ABSENT")

    # --- outputs ------------------------------------------------------------
    def _publish_odometry(self) -> None:
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.odom.yaw_rad / 2.0), math.cos(self.odom.yaw_rad / 2.0)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = float(self.odom.x_m)
        msg.pose.pose.position.y = float(self.odom.y_m)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)
        pose_cov = [0.0] * 36
        pose_cov[0] = pose_cov[7] = self.position_variance
        pose_cov[14] = pose_cov[21] = pose_cov[28] = UNOBSERVED_VARIANCE
        pose_cov[35] = self.yaw_variance
        msg.pose.covariance = pose_cov
        msg.twist.twist.linear.x = float(self.odom.speed_mps)
        msg.twist.twist.angular.z = float(self.odom.yaw_rate_radps)
        twist_cov = [0.0] * 36
        twist_cov[0] = self.speed_variance
        twist_cov[7] = twist_cov[14] = twist_cov[21] = twist_cov[28] = UNOBSERVED_VARIANCE
        twist_cov[35] = self.yaw_rate_variance
        msg.twist.covariance = twist_cov
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = float(self.odom.x_m)
            t.transform.translation.y = float(self.odom.y_m)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(t)

    def _publish_fault(self, reason: str) -> None:
        if reason != self._last_fault:
            self.get_logger().warning(f"autokart_odometry fault: {reason}")
        self._last_fault = reason
        self._publish_status("INVALID", reason, False)

    def _publish_status(self, quality: str, fault_state: str, payload_present: bool) -> None:
        msg = SensorDeliveryMetadata()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.sensor_id = "wheel_odometry"
        msg.sequence = self.sequence
        # wheelbase/counts_per_m/encoder sign have not been physically verified (no lifted-axle or
        # push test in this environment): never claim otherwise.
        msg.calibration_version = "UNVERIFIED"
        msg.quality = quality
        msg.fault_state = fault_state
        msg.payload_present = payload_present
        msg.cumulative_captured = self.cumulative_captured
        msg.cumulative_delivered = self.cumulative_delivered
        msg.cumulative_dropped = self.cumulative_dropped
        self.status_pub.publish(msg)

    def destroy_node(self):
        for t in self._timers:
            t.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
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
