"""AutoKart front safety envelope monitor -- DIAGNOSTIC MODE ONLY.

Publishes a CLEAR/BRAKING/CRITICAL/UNKNOWN classification of the front
safety envelope to /safety/envelope. This node holds no publisher or
client for any VCU, brake, throttle or drive-inhibit topic/service, and
must not gain one: vehicle_authority stays OFF. It refuses to start if a
launch/parameter override tries to change that invariant, and rejects any
later runtime attempt to change vehicle_authority or safety_monitor_mode
away from their diagnostic-only values (RC-06 audit finding A).

Every published message also carries data_source, naming where the obstacle/
odometry input actually came from (LIVE/SIMULATED/REPLAY/UNVERIFIED). This
node cannot itself verify that claim -- it only relays what the operator or
launch file configured -- but it never defaults to LIVE: an unconfigured or
invalid value is published as UNVERIFIED, never silently presented as a
real physical camera (RC-06 audit finding B).
"""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from autokart_msgs.msg import ConeArray, SafetyEnvelope
from autokart_vehicle_geometry.geometry import GEOMETRY
from nav_msgs.msg import Odometry

from .envelope import EnvelopeParameters, classify

DIAGNOSTIC_ONLY_PARAMETERS = {"vehicle_authority": "OFF", "safety_monitor_mode": "DIAGNOSTIC_ONLY"}
VALID_DATA_SOURCES = {"LIVE", "SIMULATED", "REPLAY"}


class FrontSafetyEnvelopeMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__("front_safety_envelope_monitor", **kwargs)

        # Registered before any declare_parameter call so even the initial
        # (launch-file-provided) values for these two are validated, not
        # just later ros2-param-set attempts.
        self.add_on_set_parameters_callback(self._reject_unsafe_parameter_changes)

        self.declare_parameter("vehicle_authority", "OFF")
        self.declare_parameter("safety_monitor_mode", "DIAGNOSTIC_ONLY")
        if str(self.get_parameter("vehicle_authority").value) != "OFF":
            raise RuntimeError(
                "front_safety_envelope_monitor refuses to start: vehicle_authority must be OFF")
        if str(self.get_parameter("safety_monitor_mode").value) != "DIAGNOSTIC_ONLY":
            raise RuntimeError(
                "front_safety_envelope_monitor refuses to start: only safety_monitor_mode=DIAGNOSTIC_ONLY is implemented")

        self.declare_parameter("data_source", "UNVERIFIED")
        self.data_source = str(self.get_parameter("data_source").value)
        if self.data_source not in VALID_DATA_SOURCES:
            self.get_logger().warning(
                f"data_source={self.data_source!r} is not one of {sorted(VALID_DATA_SOURCES)}; "
                "publishing UNVERIFIED rather than guessing")
            self.data_source = "UNVERIFIED"

        self.declare_parameter("odometry_topic", "/vehicle/odometry")
        self.declare_parameter("obstacle_topic", "/perception/obstacles")
        self.declare_parameter("publish_topic", "/safety/envelope")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_odometry_age_s", 0.5)
        self.declare_parameter("max_obstacle_age_s", 0.5)

        self.declare_parameter("minimum_bumper_clearance_m", 1.0)
        self.declare_parameter("bumper_setback_m", GEOMETRY.camera_to_front_bumper_horizontal_m)
        self.declare_parameter("corridor_half_width_m", GEOMETRY.half_bumper_width_m)
        self.declare_parameter("response_latency_s", 0.25)
        self.declare_parameter("guaranteed_deceleration_mps2", 2.0)
        self.declare_parameter("distance_margin_m", 0.3)
        self.declare_parameter("calibration_verified", False)

        self.params = EnvelopeParameters(
            minimum_bumper_clearance_m=float(self.get_parameter("minimum_bumper_clearance_m").value),
            bumper_setback_m=float(self.get_parameter("bumper_setback_m").value),
            corridor_half_width_m=float(self.get_parameter("corridor_half_width_m").value),
            response_latency_s=float(self.get_parameter("response_latency_s").value),
            guaranteed_deceleration_mps2=float(self.get_parameter("guaranteed_deceleration_mps2").value),
            distance_margin_m=float(self.get_parameter("distance_margin_m").value),
        )
        self.calibration_verified = bool(self.get_parameter("calibration_verified").value)
        self.max_odometry_age_s = float(self.get_parameter("max_odometry_age_s").value)
        self.max_obstacle_age_s = float(self.get_parameter("max_obstacle_age_s").value)

        self._speed_mps = None
        self._odometry_stamp = None
        self._obstacle_positions_xy: list[tuple[float, float]] = []
        self._obstacle_stamp = None

        self.create_subscription(
            Odometry, str(self.get_parameter("odometry_topic").value), self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(
            ConeArray, str(self.get_parameter("obstacle_topic").value), self._on_obstacles, qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            SafetyEnvelope, str(self.get_parameter("publish_topic").value), 10)

        period_s = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period_s, self._publish)

        self.get_logger().info(
            "front_safety_envelope_monitor started: DIAGNOSTIC_ONLY, vehicle_authority=OFF, "
            f"data_source={self.data_source}, calibration_verified={self.calibration_verified}")

    def _reject_unsafe_parameter_changes(self, params) -> SetParametersResult:
        for param in params:
            required = DIAGNOSTIC_ONLY_PARAMETERS.get(param.name)
            if required is not None and param.value != required:
                return SetParametersResult(
                    successful=False,
                    reason=(f"{param.name} is a diagnostic-only, non-negotiable constant; "
                            f"it cannot be changed from {required!r} at runtime"))
        return SetParametersResult(successful=True)

    def _on_odometry(self, message: Odometry) -> None:
        self._speed_mps = float(message.twist.twist.linear.x)
        self._odometry_stamp = self.get_clock().now()

    def _on_obstacles(self, message: ConeArray) -> None:
        self._obstacle_positions_xy = [
            (float(cone.position.x), float(cone.position.y)) for cone in message.cones
        ]
        self._obstacle_stamp = self.get_clock().now()

    def _age_s(self, stamp) -> float:
        if stamp is None:
            return float("inf")
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _publish(self) -> None:
        odometry_fresh = self._age_s(self._odometry_stamp) <= self.max_odometry_age_s
        obstacle_fresh = self._age_s(self._obstacle_stamp) <= self.max_obstacle_age_s

        result = classify(
            speed_mps=self._speed_mps,
            odometry_fresh=odometry_fresh,
            obstacle_feed_fresh=obstacle_fresh,
            obstacle_positions_xy=self._obstacle_positions_xy,
            params=self.params,
        )

        message = SafetyEnvelope()
        message.timestamp = self.get_clock().now().to_msg()
        message.frame_id = "base_link"
        message.guaranteed_deceleration_mps2 = self.params.guaranteed_deceleration_mps2
        message.clear_distance_m = result.clearance_m if result.clearance_m is not None else float("nan")
        message.stopping_distance_m = result.trigger_distance_m if result.trigger_distance_m is not None else float("nan")
        reason = result.reason if self.calibration_verified else f"{result.reason}:CALIBRATION_UNVERIFIED"
        message.limiting_reason = f"{result.state}:{reason}"
        message.valid = result.valid
        message.data_source = self.data_source
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FrontSafetyEnvelopeMonitor()
    except (RuntimeError, rclpy.exceptions.InvalidParameterValueException) as exc:
        # The latter is raised by _reject_unsafe_parameter_changes itself when a launch
        # file tries to override vehicle_authority/safety_monitor_mode at declare time.
        rclpy.logging.get_logger("front_safety_envelope_monitor").fatal(str(exc))
        rclpy.shutdown()
        raise SystemExit(1) from exc
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
