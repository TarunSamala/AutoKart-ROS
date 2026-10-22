"""autokart_imu: MPU-6500 IMU driver node.

Publishes ``sensor_msgs/Imu`` on ``/imu/data_raw`` and a delivery-status
message (``autokart_msgs/SensorDeliveryMetadata``, reused rather than
inventing a new type -- same pattern as ``autokart_hp60c``) on ``/imu/status``.

There is no IMU anywhere else in this repository to defer to: this is new
code (see backends.py's module docstring). It follows the master
architecture doc's explicit instruction: "if yaw is not directly observed,
set IMU orientation covariance sentinel -1 ... rather than publish a
fictitious absolute heading" -- this node NEVER integrates gyro into an
absolute heading itself (that remains odometry's/a future fusion node's job);
it only applies a stationary bias correction and publishes the corrected
angular rate and linear acceleration.
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from autokart_msgs.msg import SensorDeliveryMetadata

from .backends import build_backend, ImuBackendError, RawSample


def _diag_covariance(variance: float) -> list:
    cov = [0.0] * 9
    cov[0] = cov[4] = cov[8] = variance
    return cov


def sample_to_imu_msg(sample: RawSample, bias: tuple, frame_id: str, stamp,
                       gyro_variance: float, accel_variance: float) -> Imu:
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    # Orientation is not measured by this sensor/node (no magnetometer fusion,
    # no absolute-heading estimate computed here): per sensor_msgs/Imu's own
    # documented convention, element 0 of orientation_covariance = -1 means
    # "orientation data is unavailable, ignore orientation and the covariance
    # matrix"; the quaternion fields are left as an all-zero (non-unit, i.e.
    # deliberately invalid) placeholder rather than a plausible-looking
    # identity rotation.
    msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0.0
    msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    gx, gy, gz = sample.gyro_radps
    bx, by, bz = bias
    msg.angular_velocity.x = gx - bx
    msg.angular_velocity.y = gy - by
    msg.angular_velocity.z = gz - bz
    msg.angular_velocity_covariance = _diag_covariance(gyro_variance)

    ax, ay, az = sample.accel_mps2
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az
    msg.linear_acceleration_covariance = _diag_covariance(accel_variance)
    return msg


class ImuNode(Node):
    def __init__(self, start_capture_thread: bool = True, **node_kwargs):
        super().__init__("autokart_imu", **node_kwargs)
        self.declare_parameter("backend", "synthetic")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("i2c_bus_num", 1)
        self.declare_parameter("i2c_address", 0x68)
        self.declare_parameter("gyro_fs_dps", 500)
        self.declare_parameter("accel_fs_g", 4)
        self.declare_parameter("calibration_samples", 100)
        # Placeholder covariances: NOT independently characterized against a
        # real sensor in this session (no hardware). Documented in CONTRACT.md,
        # not silently presented as a measured noise spec.
        self.declare_parameter("gyro_variance_rad2ps2", 0.0004)
        self.declare_parameter("accel_variance_m2ps4", 0.01)
        # Synthetic-only test/demo hooks (ignored by the real backend).
        self.declare_parameter("synthetic_period_s", 0.005)
        self.declare_parameter("synthetic_rotation_radps_z", 0.0)
        self.declare_parameter("synthetic_rotation_after_sample", 0)
        self.declare_parameter("synthetic_saturate_after", 0)
        self.declare_parameter("synthetic_nonmonotonic_after", 0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.calibration_samples = max(1, int(self.get_parameter("calibration_samples").value))
        self.gyro_variance = float(self.get_parameter("gyro_variance_rad2ps2").value)
        self.accel_variance = float(self.get_parameter("accel_variance_m2ps4").value)

        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/imu/status", 10)

        self.sequence = 0
        self.cumulative_captured = 0
        self.cumulative_delivered = 0
        self.cumulative_dropped = 0
        self._last_fault = None
        self._stop = threading.Event()

        self.bias = None
        self._calib_count = 0
        self._calib_sum = (0.0, 0.0, 0.0)
        self.last_timestamp_ns = None

        backend_name = str(self.get_parameter("backend").value)
        saturate_after = int(self.get_parameter("synthetic_saturate_after").value) or None
        nonmonotonic_after = int(self.get_parameter("synthetic_nonmonotonic_after").value) or None
        try:
            self.backend = build_backend(
                backend_name,
                i2c_bus_num=int(self.get_parameter("i2c_bus_num").value),
                i2c_address=int(self.get_parameter("i2c_address").value),
                gyro_fs_dps=int(self.get_parameter("gyro_fs_dps").value),
                accel_fs_g=int(self.get_parameter("accel_fs_g").value),
                period_s=float(self.get_parameter("synthetic_period_s").value),
                rotation_radps_z=float(self.get_parameter("synthetic_rotation_radps_z").value),
                rotation_after_sample=int(self.get_parameter("synthetic_rotation_after_sample").value),
                saturate_after=saturate_after,
                nonmonotonic_after=nonmonotonic_after,
            )
            self.backend.start()
            self.startup_error = None
        except ImuBackendError as exc:
            self.backend = None
            self.startup_error = str(exc)
            self.get_logger().error(f"autokart_imu startup failed: {self.startup_error}")

        self._thread = None
        if start_capture_thread:
            self.start_capture()

    def start_capture(self):
        """Split out from __init__, same reasoning as HP60CNode: lets tests
        construct the node, swap in a fault-injecting backend, then start
        capture deterministically."""
        self._thread = threading.Thread(target=self._capture_loop, name="imu_capture", daemon=True)
        self._thread.start()

    def _capture_loop(self):
        if self.backend is None:
            while not self._stop.is_set():
                self._publish_fault(self.startup_error or "BACKEND_UNAVAILABLE")
                self._stop.wait(1.0)
            return
        while not self._stop.is_set():
            try:
                sample = self.backend.read()
            except ImuBackendError as exc:
                self.cumulative_dropped += 1
                self._publish_fault(str(exc))
                self._stop.wait(0.05)
                continue
            except Exception as exc:  # noqa: BLE001 - deliberate: an unexpected fault must surface as a
                # reported fault, never kill this thread silently while /imu/status looks frozen-but-fine.
                self.cumulative_dropped += 1
                self._publish_fault(f"UNEXPECTED_BACKEND_ERROR:{type(exc).__name__}:{exc}")
                self._stop.wait(0.05)
                continue

            if self.last_timestamp_ns is not None and sample.timestamp_ns <= self.last_timestamp_ns:
                # Do NOT advance last_timestamp_ns here: a single bad sample
                # must not permanently desynchronize the monotonicity check.
                self.cumulative_dropped += 1
                self._publish_fault("NON_MONOTONIC_TIMESTAMP")
                # Same bounded backoff as every other fault path below: a
                # backend that persistently emits bad timestamps/saturated
                # samples must not hot-spin this thread with no yield point,
                # which would starve the executor thread publishing/serving
                # this very status topic (found via this node's own
                # fault-injection tests, see SENIOR_REVIEWER_REPORT.md).
                self._stop.wait(0.01)
                continue
            self.last_timestamp_ns = sample.timestamp_ns

            if sample.saturated:
                self.cumulative_dropped += 1
                self._publish_fault("SATURATED_SAMPLE")
                self._stop.wait(0.01)
                continue

            if self.bias is None:
                self._calib_count += 1
                self._calib_sum = tuple(a + b for a, b in zip(self._calib_sum, sample.gyro_radps))
                self.cumulative_captured += 1
                self._publish_status(quality="CALIBRATING", fault_state="NONE", payload_present=False)
                if self._calib_count >= self.calibration_samples:
                    self.bias = tuple(s / self._calib_count for s in self._calib_sum)
                    self.get_logger().info(
                        f"autokart_imu stationary bias estimated over {self._calib_count} samples: "
                        f"{self.bias} rad/s (assumes the sensor was stationary at startup -- not "
                        f"independently verified in this environment, see CONTRACT.md)")
                continue

            stamp = self.get_clock().now().to_msg()
            msg = sample_to_imu_msg(sample, self.bias, self.frame_id, stamp,
                                     self.gyro_variance, self.accel_variance)
            if not self._safe_publish(self.imu_pub, msg):
                continue
            self.sequence += 1
            self.cumulative_captured += 1
            self.cumulative_delivered += 1  # synchronous publish, no queue: delivered == captured, same as hp60c
            self._publish_status(quality="OK", fault_state="NONE", payload_present=True)
            self._last_fault = None

    def _safe_publish(self, publisher, msg) -> bool:
        """Publish from the capture thread without letting a publish failure
        kill the thread with an uncaught traceback. If the rclpy context is
        already gone (SIGINT: Humble's handler shuts the context down BEFORE
        destroy_node() sets self._stop), stop the loop quietly instead. Found
        by test_sigint_shutdown_exits_cleanly_without_rclerror -- see
        SENIOR_REVIEWER_REPORT.md."""
        try:
            publisher.publish(msg)
            return True
        except Exception as exc:  # noqa: BLE001
            if not self.context.ok():
                self._stop.set()
                return False
            self.get_logger().error(f"autokart_imu publish failed: {type(exc).__name__}: {exc}")
            return False

    def _publish_fault(self, reason: str):
        if reason != self._last_fault:
            self.get_logger().warning(f"autokart_imu fault: {reason}")
        self._last_fault = reason
        self._publish_status(quality="INVALID", fault_state=reason, payload_present=False)

    def _publish_status(self, quality: str, fault_state: str, payload_present: bool):
        msg = SensorDeliveryMetadata()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.sensor_id = "mpu6500"
        msg.sequence = self.sequence
        msg.quality = quality
        msg.fault_state = fault_state
        msg.payload_present = payload_present
        msg.cumulative_captured = self.cumulative_captured
        msg.cumulative_delivered = self.cumulative_delivered
        msg.cumulative_dropped = self.cumulative_dropped
        self._safe_publish(self.status_pub, msg)

    def destroy_node(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.backend is not None:
            try:
                self.backend.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Humble's default SIGINT handler shuts the context down itself and
        # spin() then raises ExternalShutdownException; calling
        # rclpy.shutdown() again unguarded raised RCLError on every Ctrl-C
        # (found during this node's live clean-workspace validation run).
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
