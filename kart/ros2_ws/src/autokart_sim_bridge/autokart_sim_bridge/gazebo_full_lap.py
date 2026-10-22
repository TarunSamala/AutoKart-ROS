"""Native ROS 2 recorder/publisher for the Gazebo follower full-lap demo."""

from __future__ import annotations

import argparse
import cProfile
import json
import math
import multiprocessing
import os
import queue
import struct
import threading
from collections.abc import Sized
from collections import defaultdict
from dataclasses import asdict
from hashlib import sha256
from pathlib import Path
from time import monotonic, sleep


def _seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _stamp(seconds: float, target: object) -> None:
    target.sec = int(math.floor(max(0.0, seconds)))
    target.nanosec = int(round((seconds - target.sec) * 1e9))
    if target.nanosec >= 1_000_000_000:
        target.sec += 1
        target.nanosec -= 1_000_000_000


def _downsample_ppm(message: object, destination: Path) -> None:
    if not message.data or message.width <= 0 or message.height <= 0:
        return
    channels = 3 if message.encoding.lower() in {"rgb8", "bgr8"} else 1
    target_width = min(160, int(message.width))
    target_height = min(90, int(message.height))
    x_step = max(1, int(message.width) // target_width)
    y_step = max(1, int(message.height) // target_height)
    pixels = bytearray()
    actual_width = 0
    actual_height = 0
    for y in range(0, int(message.height), y_step):
        if actual_height >= target_height:
            break
        row_width = 0
        for x in range(0, int(message.width), x_step):
            if row_width >= target_width:
                break
            offset = y * int(message.step) + x * channels
            if channels == 3:
                values = bytes(message.data[offset:offset + 3])
                if message.encoding.lower() == "bgr8":
                    values = values[::-1]
                pixels.extend(values)
            else:
                value = message.data[offset]
                pixels.extend((value, value, value))
            row_width += 1
        actual_width = row_width
        actual_height += 1
    destination.write_bytes(
        f"P6\n{actual_width} {actual_height}\n255\n".encode("ascii") + bytes(pixels)
    )


def _isolated_core_worker(
    scenario_path: str,
    output_dir: str,
    follower_hz: float,
    realtime_synchronized: bool,
    snapshot_queue: object,
    result_queue: object,
) -> None:
    """Run authority/VCU/plant in a spawned process with one-way snapshots."""
    affinity = os.environ.get("AUTOKART_CORE_CPU_LIST", "").strip()
    if affinity and hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(
                0, {int(cpu) for cpu in affinity.split(",") if cpu.strip()}
            )
        except (OSError, ValueError) as error:
            raise RuntimeError(
                f"failed to apply authoritative-core CPU affinity: {affinity}"
            ) from error
    next_snapshot_s = 0.0
    period_s = 1.0 / follower_hz

    def publish(snapshot: object) -> None:
        nonlocal next_snapshot_s
        if snapshot.simulation_time_s + 1e-12 < next_snapshot_s:
            return
        snapshot_queue.put(snapshot)
        next_snapshot_s += period_s

    try:
        from kart_sim.gazebo_full_lap import execute_full_lap
        profile_path = os.environ.get("AUTOKART_ISOLATED_CORE_PROFILE")
        arguments = (
            scenario_path,
            output_dir,
        )
        keywords = {
            "follower_hz": follower_hz,
            "follower_enabled": False,
            "realtime": realtime_synchronized,
            "snapshot_sink": publish,
        }
        if profile_path:
            profiler = cProfile.Profile()
            result = profiler.runcall(execute_full_lap, *arguments, **keywords)
            profiler.dump_stats(profile_path)
        else:
            result = execute_full_lap(*arguments, **keywords)
        result_queue.put(("ok", result))
    except BaseException as error:
        result_queue.put(("error", f"{type(error).__name__}: {error}"))
    finally:
        snapshot_queue.put(None)


def _run_isolated_core(
    scenario_path: Path,
    output_dir: Path,
    follower_hz: float,
    realtime_synchronized: bool,
    recorder: object,
) -> object:
    context = multiprocessing.get_context("spawn")
    snapshot_queue = context.Queue(maxsize=1)
    result_queue = context.Queue(maxsize=1)
    process = context.Process(
        target=_isolated_core_worker,
        args=(
            str(scenario_path),
            str(output_dir),
            follower_hz,
            realtime_synchronized,
            snapshot_queue,
            result_queue,
        ),
        name="autokart_authoritative_core",
    )
    process.start()
    try:
        while True:
            try:
                snapshot = snapshot_queue.get(timeout=0.5)
            except queue.Empty:
                if not process.is_alive():
                    break
                continue
            if snapshot is None:
                break
            recorder.publish_snapshot(snapshot)
        process.join(timeout=10.0)
        if process.is_alive():
            process.terminate()
            process.join(timeout=5.0)
            raise RuntimeError("isolated authoritative core did not terminate")
        try:
            status, payload = result_queue.get(timeout=2.0)
        except queue.Empty as error:
            raise RuntimeError(
                "isolated authoritative core returned no result"
            ) from error
        if status != "ok":
            raise RuntimeError(f"isolated authoritative core failed: {payload}")
        return payload
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=5.0)
        snapshot_queue.close()
        result_queue.close()


class NativeFollowerRecorder:
    def __init__(
        self,
        scenario_path: Path,
        output_dir: Path,
        follower_hz: float,
        preset_path: Path | None = None,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from autokart_msgs.msg import (
            AuthoritativeVehicleSnapshot,
            GazeboFollowerState,
            SensorDeliveryMetadata,
        )
        from diagnostic_msgs.msg import DiagnosticArray
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, Imu, JointState, LaserScan

        class RecorderNode(Node):
            pass

        self.rclpy = rclpy
        self.AuthoritativeVehicleSnapshot = AuthoritativeVehicleSnapshot
        self.SensorDeliveryMetadata = SensorDeliveryMetadata
        self.ImuMessage = Imu
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        preset = {}
        if preset_path is not None:
            import yaml
            preset = yaml.safe_load(preset_path.read_text(encoding="utf-8"))
        ros_configuration = preset.get("ros", {})
        self.executor_threads = int(
            ros_configuration.get("executor_threads", 4)
        )
        sensor_queue_depth = int(
            ros_configuration.get("sensor_queue_depth", 10)
        )
        metadata_queue_depth = int(
            ros_configuration.get("metadata_queue_depth", 8192)
        )
        if min(
            self.executor_threads, sensor_queue_depth, metadata_queue_depth
        ) <= 0:
            raise RuntimeError("ROS executor and queue settings must be positive")
        self.node = RecorderNode("autokart_gazebo_full_lap_recorder")
        reliable = QoSProfile(
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=sensor_queue_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        imu_qos = QoSProfile(
            depth=4096,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        metadata_qos = QoSProfile(
            depth=metadata_queue_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.node.create_publisher(
            AuthoritativeVehicleSnapshot,
            "/autokart/follower/authoritative_snapshot",
            reliable,
        )
        self.metadata_publisher = self.node.create_publisher(
            SensorDeliveryMetadata, "/sensors/delivery_metadata", metadata_qos
        )
        self.imu_publisher = self.node.create_publisher(Imu, "/sensors/imu", imu_qos)
        self.imu_metadata_publisher = self.node.create_publisher(
            SensorDeliveryMetadata,
            "/sensors/imu/delivery_metadata",
            metadata_qos,
        )
        self.node.create_subscription(
            GazeboFollowerState, "/autokart/follower/state", self._on_follower, reliable
        )
        self.node.create_subscription(
            Image, "/sensors/camera/image", self._on_rgb, sensor_qos
        )
        self.node.create_subscription(
            Image, "/sensors/depth/image", self._on_depth, sensor_qos
        )
        self.node.create_subscription(
            LaserScan, "/sensors/lidar/scan", self._on_lidar, sensor_qos
        )
        metadata_subscription = self.node.create_subscription(
            SensorDeliveryMetadata,
            "/sensors/delivery_metadata",
            self._on_metadata,
            metadata_qos,
        )
        self.clock_subscription = self.node.create_subscription(
            Clock, "/clock", self._on_clock, reliable
        )
        self.joint_subscription = self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, sensor_qos
        )
        self.node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, reliable
        )
        self.follower_period_s = 1.0 / follower_hz
        synchronization = preset.get("synchronization", {})
        recorder = preset.get("recorder", {})
        self.process_isolation = bool(
            recorder.get("process_isolation", False)
        )
        self.realtime_synchronized = (
            preset.get("scheduler_mode") == "realtime_synchronized"
        )
        imu_configuration = preset.get("sensors", {}).get("imu", {})
        self.imu_source = str(imu_configuration.get("source", "internal"))
        self.imu_rate_hz = float(imu_configuration.get("rate_hz", 200.0))
        if self.imu_source not in {"internal", "gazebo"}:
            raise RuntimeError("IMU source must be internal or gazebo")
        if self.imu_rate_hz <= 0.0:
            raise RuntimeError("IMU rate must be positive")
        if self.imu_source == "gazebo":
            self.node.create_subscription(
                Imu, "/sensors/imu", self._on_imu, imu_qos
            )
        # Retain a strong reference for rclpy implementations that do not keep
        # subscriptions alive solely through the node.
        self._metadata_subscription = metadata_subscription
        self.maximum_follower_lag_s = float(
            synchronization.get("maximum_follower_lag_s", 0.030)
        )
        self.acknowledgement_timeout_s = float(
            synchronization.get("acknowledgement_timeout_s", 2.0)
        )
        self.maximum_payload_hashes_per_sensor = int(
            recorder.get("maximum_payload_hashes_per_sensor", 1_000_000)
        )
        self.follower_evidence_stride = int(
            recorder.get("follower_evidence_stride", 1)
        )
        if self.maximum_follower_lag_s <= 0.0:
            raise RuntimeError("maximum follower lag must be positive")
        if self.acknowledgement_timeout_s <= 0.0:
            raise RuntimeError("follower acknowledgement timeout must be positive")
        if (
            self.maximum_payload_hashes_per_sensor <= 0
            or self.follower_evidence_stride <= 0
        ):
            raise RuntimeError("payload evidence limit must be positive")
        self.next_publish_s = 0.0
        self.sent_snapshots: dict[int, object] = {}
        self.follower_records: list[dict] = []
        self.joint_records: list[dict] = []
        self.sensor_metadata: list[dict] = []
        self.diagnostics: list[dict] = []
        self.clock_offsets: list[float] = []
        self.clock_topic_count = 0
        self.last_clock_timestamp_s: float | None = None
        self.repeated_clock_timestamps = 0
        self.last_follower_gazebo_time_s: float | None = None
        self.joint_topic_sample_count = 0
        self.sensor_counts: dict[str, int] = defaultdict(int)
        self.empty_payloads: dict[str, int] = defaultdict(int)
        self.sensor_hashes: dict[str, list[str]] = defaultdict(list)
        self.maximum_position_error_m = 0.0
        self.maximum_orientation_error_rad = 0.0
        self.maximum_steering_error_rad = 0.0
        self.maximum_wheel_error_rad = 0.0
        self.follower_messages_received = 0
        self.maximum_applied_updates = 0
        self.timestamp_violations = 0
        self.last_sensor_stamp: dict[str, float] = {}
        self.latest_snapshot: object | None = None
        self._previous_imu_snapshot: object | None = None
        self._next_imu_capture_s = 0.0
        self._imu_sequence = 0
        self._follower_was_connected = False
        self._missing_follower_checks = 0
        self._lock = threading.Lock()
        self._acknowledgement = threading.Condition(self._lock)
        self._last_follower_time_s = -1.0
        self.maximum_acknowledgement_lag_s = 0.0
        self.acknowledgement_waits = 0
        if self.executor_threads == 1:
            self.executor = rclpy.executors.SingleThreadedExecutor()
        else:
            self.executor = rclpy.executors.MultiThreadedExecutor(
                num_threads=self.executor_threads
            )
        self.executor.add_node(self.node)
        profile_path = os.environ.get("AUTOKART_ROS_PROFILE")
        def spin_executor() -> None:
            if not profile_path:
                self.executor.spin()
                return
            profiler = cProfile.Profile()
            profiler.enable()
            try:
                self.executor.spin()
            finally:
                profiler.disable()
                profiler.dump_stats(profile_path)
        self.spin_thread = threading.Thread(target=spin_executor, daemon=True)
        self.spin_thread.start()
        self.scenario_path = scenario_path

    def wait_for_follower(self, timeout_s: float = 20.0) -> None:
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                self._follower_was_connected = True
                return
            sleep(0.05)
        raise RuntimeError("Gazebo follower snapshot subscriber did not appear")

    def publish_snapshot(self, snapshot: object) -> None:
        self.latest_snapshot = snapshot
        self._publish_internal_imu(snapshot)
        if snapshot.simulation_time_s + 1e-12 < self.next_publish_s:
            return
        if self._follower_was_connected:
            if self.publisher.get_subscription_count() == 0:
                self._missing_follower_checks += 1
                if self._missing_follower_checks >= 5:
                    raise RuntimeError(
                        "Gazebo process/follower subscription lost; native run invalid"
                    )
            else:
                self._missing_follower_checks = 0
        message = self.AuthoritativeVehicleSnapshot()
        _stamp(snapshot.simulation_time_s, message.header.stamp)
        message.header.frame_id = "map"
        message.simulation_time_s = snapshot.simulation_time_s
        message.sequence = snapshot.sequence
        message.position_x_m = snapshot.position_x_m
        message.position_y_m = snapshot.position_y_m
        message.position_z_m = snapshot.position_z_m
        message.roll_rad = snapshot.roll_rad
        message.pitch_rad = snapshot.pitch_rad
        message.yaw_rad = snapshot.yaw_rad
        message.linear_velocity_body_mps = list(
            snapshot.linear_velocity_body_mps
        )
        message.angular_velocity_body_radps = list(
            snapshot.angular_velocity_body_radps
        )
        message.front_left_steering_rad = snapshot.front_left_steering_rad
        message.front_right_steering_rad = snapshot.front_right_steering_rad
        message.front_left_wheel_rotation_rad = \
            snapshot.front_left_wheel_rotation_rad
        message.front_right_wheel_rotation_rad = \
            snapshot.front_right_wheel_rotation_rad
        message.rear_left_wheel_rotation_rad = \
            snapshot.rear_left_wheel_rotation_rad
        message.rear_right_wheel_rotation_rad = \
            snapshot.rear_right_wheel_rotation_rad
        message.front_left_wheel_speed_radps = \
            snapshot.front_left_wheel_speed_radps
        message.front_right_wheel_speed_radps = \
            snapshot.front_right_wheel_speed_radps
        message.rear_left_wheel_speed_radps = \
            snapshot.rear_left_wheel_speed_radps
        message.rear_right_wheel_speed_radps = \
            snapshot.rear_right_wheel_speed_radps
        message.rear_axle_rotation_rad = snapshot.rear_axle_rotation_rad
        message.rear_axle_speed_radps = snapshot.rear_axle_speed_radps
        message.valid = snapshot.valid
        message.reset_epoch = snapshot.reset_epoch
        message.configuration_hash = snapshot.configuration_hash
        self.sent_snapshots[int(snapshot.sequence)] = snapshot
        self.publisher.publish(message)
        self.next_publish_s += self.follower_period_s
        # Bound the one-way bridge queue.  This backpressure is outside the
        # physics/VCU chain and makes aggregate RTF represent the complete
        # rendered follower rather than an ahead-running producer.
        required_time_s = snapshot.simulation_time_s - self.maximum_follower_lag_s
        if required_time_s > 0.0:
            deadline = monotonic() + self.acknowledgement_timeout_s
            with self._acknowledgement:
                while self._last_follower_time_s + 1e-12 < required_time_s:
                    remaining = deadline - monotonic()
                    if remaining <= 0.0:
                        raise RuntimeError(
                            "Gazebo follower acknowledgement exceeded bounded timeout"
                        )
                    self.acknowledgement_waits += 1
                    self._acknowledgement.wait(timeout=min(0.010, remaining))
                self.maximum_acknowledgement_lag_s = max(
                    self.maximum_acknowledgement_lag_s,
                    snapshot.simulation_time_s - self._last_follower_time_s,
                )

    def _publish_internal_imu(self, snapshot: object) -> None:
        from kart_sim.gazebo_follower import quaternion_from_rpy
        from kart_sim.sensors import ImuSensor

        if not hasattr(self, "_imu_model"):
            self._imu_model = ImuSensor(
                "imu", "imu_link", self.imu_rate_hz,
                noise_std=0.002, bias_mps2=0.001, bias_radps=0.0002,
                processing_s=0.001, transport_s=0.002, seed=20260729,
            )
        previous = self._previous_imu_snapshot
        dt = (
            snapshot.simulation_time_s - previous.simulation_time_s
            if previous is not None else 0.0
        )
        ax = (
            (snapshot.linear_velocity_body_mps[0] - previous.linear_velocity_body_mps[0]) / dt
            if previous is not None and dt > 0.0 else 0.0
        )
        ay = (
            (snapshot.linear_velocity_body_mps[1] - previous.linear_velocity_body_mps[1]) / dt
            if previous is not None and dt > 0.0 else 0.0
        )
        self._previous_imu_snapshot = snapshot
        due = self._imu_model.update(snapshot.simulation_time_s, {
            "ax_mps2": ax,
            "ay_mps2": ay,
            "yaw_rate_radps": snapshot.angular_velocity_body_radps[2],
        })
        if not due:
            return
        orientation = quaternion_from_rpy(
            snapshot.roll_rad, snapshot.pitch_rad, snapshot.yaw_rad
        )
        for sample in due:
            message = self.ImuMessage()
            _stamp(sample.header.measurement_timestamp_s, message.header.stamp)
            message.header.frame_id = "imu_link"
            message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = orientation
            gyro = tuple(map(float, sample.payload["gyro_radps"]))
            acceleration = tuple(map(float, sample.payload["accel_mps2"]))
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gyro
            message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = acceleration
            self.imu_publisher.publish(message)
            payload = struct.pack(
                "<10d",
                message.orientation.x, message.orientation.y,
                message.orientation.z, message.orientation.w,
                message.angular_velocity.x, message.angular_velocity.y,
                message.angular_velocity.z, message.linear_acceleration.x,
                message.linear_acceleration.y, message.linear_acceleration.z,
            )
            self._record_payload("imu", message, payload)
            metadata = self.SensorDeliveryMetadata()
            metadata.header = message.header
            metadata.sensor_id = "imu"
            metadata.sequence = int(sample.header.sequence)
            metadata.reset_epoch = int(snapshot.reset_epoch)
            _stamp(sample.header.measurement_timestamp_s, metadata.capture_time)
            _stamp(
                sample.header.measurement_timestamp_s + 0.001,
                metadata.processing_completion_time,
            )
            _stamp(sample.header.delivery_timestamp_s, metadata.delivery_time)
            metadata.calibration_version = "SIM-IMU-1"
            metadata.quality = sample.header.quality
            metadata.fault_state = (
                sample.header.fault_flags[0] if sample.header.fault_flags else "NONE"
            )
            metadata.payload_present = True
            metadata.payload_size = 10 * 8
            metadata.encoding = "ROS_IMU_FLOAT64"
            metadata.width = 10
            metadata.height = 1
            metadata.cumulative_captured = int(sample.header.sequence)
            metadata.cumulative_delivered = int(sample.header.sequence)
            metadata.cumulative_dropped = 0
            metadata.warmup_rejections = 0
            metadata.intentional_rate_decimations = 0
            metadata.queue_overflow_drops = 0
            metadata.processing_deadline_misses = 0
            metadata.transport_drops = 0
            metadata.stale_rejections = 0
            metadata.consecutive_drops = 0
            metadata.maximum_consecutive_drops = 0
            metadata.queue_depth = 0
            metadata.maximum_queue_depth = 1
            self.sensor_metadata.append(self._metadata_row(metadata))
            self.imu_metadata_publisher.publish(metadata)

    def _on_follower(self, message: object) -> None:
        simulation_time_s = float(message.simulation_time_s)
        gazebo_time_s = float(message.gazebo_simulation_time_s)
        self.follower_messages_received += 1
        self.maximum_applied_updates = max(
            self.maximum_applied_updates, int(message.applied_updates)
        )
        self.clock_offsets.append(float(message.clock_offset_s))
        if (
            self.last_follower_gazebo_time_s is not None
            and gazebo_time_s < self.last_follower_gazebo_time_s
        ):
            self.timestamp_violations += 1
        self.last_follower_gazebo_time_s = gazebo_time_s
        with self._acknowledgement:
            self._last_follower_time_s = max(
                self._last_follower_time_s, simulation_time_s
            )
            self._acknowledgement.notify_all()
        if (
            (self.follower_messages_received - 1)
            % self.follower_evidence_stride != 0
        ):
            return
        snapshot = self.sent_snapshots.get(int(message.sequence))
        if snapshot is None:
            return
        dx = message.pose.position.x - snapshot.position_x_m
        dy = message.pose.position.y - snapshot.position_y_m
        dz = message.pose.position.z - snapshot.position_z_m
        position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
        from kart_sim.gazebo_follower import quaternion_from_rpy
        expected = quaternion_from_rpy(snapshot.roll_rad, snapshot.pitch_rad, snapshot.yaw_rad)
        observed = (
            message.pose.orientation.x, message.pose.orientation.y,
            message.pose.orientation.z, message.pose.orientation.w,
        )
        dot = min(1.0, abs(sum(a * b for a, b in zip(expected, observed))))
        orientation_error = 2.0 * math.acos(dot)
        positions = dict(zip(message.joint_names, message.joint_positions_rad))
        steering_error = max(
            abs(positions["front_left_steering_joint"] - snapshot.front_left_steering_rad),
            abs(positions["front_right_steering_joint"] - snapshot.front_right_steering_rad),
        )
        wheel_error = max(
            abs(positions["front_left_wheel_joint"] + snapshot.front_left_wheel_rotation_rad),
            abs(positions["front_right_wheel_joint"] + snapshot.front_right_wheel_rotation_rad),
            abs(positions["rear_solid_axle_joint"] + snapshot.rear_axle_rotation_rad),
        )
        self.maximum_position_error_m = max(self.maximum_position_error_m, position_error)
        self.maximum_orientation_error_rad = max(self.maximum_orientation_error_rad, orientation_error)
        self.maximum_steering_error_rad = max(self.maximum_steering_error_rad, steering_error)
        self.maximum_wheel_error_rad = max(self.maximum_wheel_error_rad, wheel_error)
        self.follower_records.append({
            "simulation_time_s": float(message.simulation_time_s),
            "gazebo_simulation_time_s": float(message.gazebo_simulation_time_s),
            "sequence": int(message.sequence),
            "reset_epoch": int(message.reset_epoch),
            "x_m": float(message.pose.position.x),
            "y_m": float(message.pose.position.y),
            "z_m": float(message.pose.position.z),
            "quaternion_xyzw": observed,
            "position_error_m": position_error,
            "orientation_error_rad": orientation_error,
            "clock_offset_s": float(message.clock_offset_s),
        })
        self.joint_records.append({
            "simulation_time_s": float(message.simulation_time_s),
            "names": list(message.joint_names),
            "positions_rad": positions,
            "velocities_radps": dict(
                zip(message.joint_names, message.joint_velocities_radps)
            ),
            "evidence_source": "GazeboFollowerState",
        })

    def _on_joint_state(self, message: object) -> None:
        self.joint_topic_sample_count += 1
        if self.joint_topic_sample_count >= 10 and self.joint_subscription is not None:
            subscription = self.joint_subscription
            self.joint_subscription = None
            self.node.destroy_subscription(subscription)

    def _record_payload(
        self, sensor_id: str, message: object, payload: Sized
    ) -> None:
        self.sensor_counts[sensor_id] += 1
        if len(payload) == 0:
            self.empty_payloads[sensor_id] += 1
            return
        stamp = _seconds(message.header.stamp)
        previous = self.last_sensor_stamp.get(sensor_id)
        if previous is not None and stamp <= previous:
            self.timestamp_violations += 1
        self.last_sensor_stamp[sensor_id] = stamp
        if len(self.sensor_hashes[sensor_id]) < self.maximum_payload_hashes_per_sensor:
            digest = sha256(bytes(payload)).hexdigest()
            if (
                not self.sensor_hashes[sensor_id]
                or digest != self.sensor_hashes[sensor_id][-1]
            ):
                self.sensor_hashes[sensor_id].append(digest)

    def _on_rgb(self, message: object) -> None:
        self._record_payload("rgb_camera", message, message.data)
        count = self.sensor_counts["rgb_camera"]
        if count in {1, 30, 120, 240}:
            _downsample_ppm(
                message, self.output_dir / f"camera_sample/frame_{count:04d}.ppm"
            )
        if count % 30 == 0:
            _downsample_ppm(message, self.output_dir / "camera_sample/latest.ppm")

    def _on_depth(self, message: object) -> None:
        self._record_payload("depth_camera", message, message.data)
        count = self.sensor_counts["depth_camera"]
        if count in {1, 30, 120, 240} and message.encoding == "32FC1":
            payload = bytes(message.data)
            values = [
                value[0] for value in struct.iter_unpack("<f", payload[: min(len(payload), 4096)])
            ]
            finite = [value for value in values if math.isfinite(value)]
            sample = {
                "capture_time_s": _seconds(message.header.stamp),
                "width": int(message.width),
                "height": int(message.height),
                "encoding": message.encoding,
                "payload_sha256": sha256(payload).hexdigest(),
                "finite_values_in_prefix": len(finite),
                "minimum_finite_m": min(finite) if finite else None,
                "maximum_finite_m": max(finite) if finite else None,
            }
            (self.output_dir / f"depth_sample/frame_{count:04d}.json").write_text(
                json.dumps(sample, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )

    def _on_lidar(self, message: object) -> None:
        self._record_payload("lidar", message, message.ranges)
        count = self.sensor_counts["lidar"]
        if count in {1, 20}:
            lines = [
                "index,range_m",
                *[
                    f"{index},{float(value):.9g}"
                    for index, value in enumerate(message.ranges)
                ],
            ]
            (self.output_dir / f"lidar_sample/scan_{count:04d}.csv").write_text(
                "\n".join(lines) + "\n", encoding="utf-8"
            )

    def _on_imu(self, message: object) -> None:
        payload = struct.pack(
            "<10d",
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w,
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z, message.linear_acceleration.x,
            message.linear_acceleration.y, message.linear_acceleration.z,
        )
        self._record_payload("imu", message, payload)

    def _metadata_row(self, message: object) -> dict:
        capture = _seconds(message.capture_time)
        delivery = _seconds(message.delivery_time)
        if abs(_seconds(message.header.stamp) - capture) > 1e-9:
            self.timestamp_violations += 1
        return {
            "sensor_id": message.sensor_id,
            "frame_id": message.header.frame_id,
            "sequence": int(message.sequence),
            "reset_epoch": int(message.reset_epoch),
            "capture_time_s": capture,
            "processing_completion_time_s": _seconds(message.processing_completion_time),
            "delivery_time_s": delivery,
            "calibration_version": message.calibration_version,
            "quality": message.quality,
            "fault_state": message.fault_state,
            "payload_present": bool(message.payload_present),
            "payload_size": int(message.payload_size),
            "encoding": message.encoding,
            "width": int(message.width),
            "height": int(message.height),
            "cumulative_captured": int(message.cumulative_captured),
            "cumulative_delivered": int(message.cumulative_delivered),
            "cumulative_dropped": int(message.cumulative_dropped),
            "warmup_rejections": int(message.warmup_rejections),
            "intentional_rate_decimations":
                int(message.intentional_rate_decimations),
            "queue_overflow_drops": int(message.queue_overflow_drops),
            "processing_deadline_misses":
                int(message.processing_deadline_misses),
            "transport_drops": int(message.transport_drops),
            "stale_rejections": int(message.stale_rejections),
            "consecutive_drops": int(message.consecutive_drops),
            "maximum_consecutive_drops":
                int(message.maximum_consecutive_drops),
            "queue_depth": int(message.queue_depth),
            "maximum_queue_depth": int(message.maximum_queue_depth),
        }

    def _on_metadata(self, message: object) -> None:
        self.sensor_metadata.append(self._metadata_row(message))

    def _on_clock(self, message: object) -> None:
        clock_time = _seconds(message.clock)
        if (
            self.last_clock_timestamp_s is not None
            and clock_time < self.last_clock_timestamp_s
        ):
            self.timestamp_violations += 1
        if (
            self.last_clock_timestamp_s is not None
            and clock_time == self.last_clock_timestamp_s
        ):
            self.repeated_clock_timestamps += 1
        self.last_clock_timestamp_s = clock_time
        self.clock_topic_count += 1
        if self.clock_topic_count >= 10 and self.clock_subscription is not None:
            subscription = self.clock_subscription
            self.clock_subscription = None
            self.node.destroy_subscription(subscription)

    def _on_diagnostics(self, message: object) -> None:
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            level = (
                status.level[0]
                if isinstance(status.level, (bytes, bytearray))
                else int(status.level)
            )
            self.diagnostics.append({
                "timestamp_s": _seconds(message.header.stamp),
                "level": level,
                "name": status.name,
                "message": status.message,
                "values": values,
            })

    def finalize(self, core_result: object) -> dict:
        sleep(0.5)
        self.executor.shutdown(timeout_sec=5.0)
        self.spin_thread.join(timeout=5.0)
        self.node.destroy_node()
        sensor_summary = {}
        for sensor_id in ("rgb_camera", "depth_camera", "lidar", "imu"):
            metadata = [row for row in self.sensor_metadata if row["sensor_id"] == sensor_id]
            delivered = self.sensor_counts[sensor_id]
            delivered = max(
                delivered,
                max((row["cumulative_delivered"] for row in metadata), default=0),
            )
            captured = max(
                delivered,
                max((row["cumulative_captured"] for row in metadata), default=0),
            )
            sensor_summary[sensor_id] = {
                "captured": captured,
                "delivered": delivered,
                "dropped": max(
                    (row["cumulative_dropped"] for row in metadata), default=0
                ),
                "drops_by_reason": {
                    field: max((row[field] for row in metadata), default=0)
                    for field in (
                        "warmup_rejections",
                        "intentional_rate_decimations",
                        "queue_overflow_drops",
                        "processing_deadline_misses",
                        "transport_drops",
                        "stale_rejections",
                    )
                },
                "maximum_consecutive_drops": max(
                    (
                        row["maximum_consecutive_drops"]
                        for row in metadata
                    ),
                    default=0,
                ),
                "observed_by_recorder": self.sensor_counts[sensor_id],
                "recorder_observation_loss": max(
                    0, delivered - self.sensor_counts[sensor_id]
                ),
                "empty_payloads": self.empty_payloads[sensor_id],
                "unique_payload_hashes": len(set(self.sensor_hashes[sensor_id])),
                "maximum_queue_depth": max(
                    (row["maximum_queue_depth"] for row in metadata), default=0
                ),
                "maximum_latency_s": max(
                    (row["delivery_time_s"] - row["capture_time_s"] for row in metadata),
                    default=0.0,
                ),
                "first_capture_time_s": min(
                    (row["capture_time_s"] for row in metadata), default=0.0
                ),
                "last_capture_time_s": max(
                    (row["capture_time_s"] for row in metadata), default=0.0
                ),
            }
        native = {
            "follower_updates": self.maximum_applied_updates,
            "follower_state_messages": self.follower_messages_received,
            "follower_evidence_samples": len(self.follower_records),
            "follower_evidence_stride": self.follower_evidence_stride,
            "joint_updates": len(self.joint_records),
            "maximum_position_error_m": self.maximum_position_error_m,
            "maximum_orientation_error_rad": self.maximum_orientation_error_rad,
            "maximum_steering_error_rad": self.maximum_steering_error_rad,
            "maximum_wheel_rotation_error_rad": self.maximum_wheel_error_rad,
            "maximum_clock_offset_s": max(map(abs, self.clock_offsets), default=0.0),
            "clock_topic_messages": self.clock_topic_count,
            "repeated_clock_timestamps": self.repeated_clock_timestamps,
            "clock_continuous_validation_source":
                "GazeboFollowerState.gazebo_simulation_time_s",
            "joint_topic_startup_samples": self.joint_topic_sample_count,
            "joint_continuous_validation_source": "GazeboFollowerState",
            "timestamp_violations": self.timestamp_violations,
            "sensors": sensor_summary,
            "critical_diagnostics": sum(
                row["level"] >= 2 for row in self.diagnostics
            ),
            "double_physics_diagnostics": sum(
                row["values"].get("double_physics_detected") == "true"
                for row in self.diagnostics
            ),
            "maximum_acknowledgement_lag_s":
                self.maximum_acknowledgement_lag_s,
            "acknowledgement_waits": self.acknowledgement_waits,
            "payload_hash_limit_per_sensor":
                self.maximum_payload_hashes_per_sensor,
            "ros_executor_threads": self.executor_threads,
            "authoritative_core_process_isolated": self.process_isolation,
        }
        timing_path = self.output_dir / "timing_report.json"
        if timing_path.exists():
            native["timing"] = json.loads(
                timing_path.read_text(encoding="utf-8")
            )
        for path, rows in (
            (self.output_dir / "gazebo_pose_log.jsonl", self.follower_records),
            (self.output_dir / "joint_state_log.jsonl", self.joint_records),
            (self.output_dir / "sensor_metadata.jsonl", self.sensor_metadata),
            (self.output_dir / "diagnostics/native.jsonl", self.diagnostics),
        ):
            path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
                encoding="utf-8",
            )
        (self.output_dir / "native_results.json").write_text(
            json.dumps(native, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        summary_path = self.output_dir / "summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["native_gazebo"] = native
        summary["rendered_sensor_evidence"] = (
            "native Gazebo RGB, depth and LiDAR payloads validated"
        )
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        from kart_sim.gazebo_full_lap import generate_acceptance_plots
        generate_acceptance_plots(self.output_dir)
        manifest_path = self.output_dir / "run_manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["native_gazebo_executed"] = True
        manifest["files"] = {}
        for path in sorted(self.output_dir.rglob("*")):
            if path.is_file() and path != manifest_path:
                manifest["files"][str(path.relative_to(self.output_dir))] = {
                    "sha256": sha256(path.read_bytes()).hexdigest(),
                    "bytes": path.stat().st_size,
                }
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return native


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        type=Path,
        default=Path("configs/scenarios/gazebo_follower_full_lap.yaml"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/gazebo_follower/full_lap"),
    )
    parser.add_argument("--follower-hz", type=float, default=100.0)
    parser.add_argument("--preset", type=Path)
    arguments = parser.parse_args()
    import rclpy
    rclpy.init()
    recorder = NativeFollowerRecorder(
        arguments.scenario,
        arguments.output_dir,
        arguments.follower_hz,
        arguments.preset,
    )
    try:
        recorder.wait_for_follower()
        from kart_sim.gazebo_full_lap import execute_full_lap
        core_profile_path = os.environ.get("AUTOKART_CORE_PROFILE")
        if recorder.process_isolation and not core_profile_path:
            core_result = _run_isolated_core(
                arguments.scenario,
                arguments.output_dir,
                arguments.follower_hz,
                recorder.realtime_synchronized,
                recorder,
            )
        elif core_profile_path:
            profiler = cProfile.Profile()
            core_result = profiler.runcall(
                execute_full_lap,
                arguments.scenario,
                arguments.output_dir,
                follower_hz=arguments.follower_hz,
                follower_enabled=False,
                realtime=True,
                snapshot_sink=recorder.publish_snapshot,
            )
            profiler.dump_stats(core_profile_path)
        else:
            core_result = execute_full_lap(
                arguments.scenario,
                arguments.output_dir,
                follower_hz=arguments.follower_hz,
                follower_enabled=False,
                realtime=True,
                snapshot_sink=recorder.publish_snapshot,
            )
        native = recorder.finalize(core_result)
        result = {"core": asdict(core_result), "native": native}
        print(json.dumps(result, indent=2, sort_keys=True))
        if not core_result.completed or not core_result.controlled_stop_complete:
            raise SystemExit(2)
        required = native["sensors"]
        sensor_tail_allowance = {
            "rgb_camera": 0.25,
            "depth_camera": 0.25,
            "lidar": 0.25,
            "imu": 0.25,
        }
        if arguments.preset is not None:
            import yaml
            preset = yaml.safe_load(
                arguments.preset.read_text(encoding="utf-8")
            )
            for sensor_name in sensor_tail_allowance:
                rate = float(
                    preset.get("sensors", {}).get(sensor_name, {})
                    .get("rate_hz", 0.0)
                )
                if rate > 0.0:
                    sensor_tail_allowance[sensor_name] = max(
                        0.25, 1.0 / rate + 0.05
                    )
        if any(
            required[name]["delivered"] <= 0 or required[name]["empty_payloads"] > 0
            for name in ("rgb_camera", "depth_camera", "lidar", "imu")
        ):
            raise SystemExit(3)
        if (
            native["follower_updates"] < 0.95 * core_result.finish_time_s
            * arguments.follower_hz
            or native["follower_state_messages"] <= 0
            or native["maximum_clock_offset_s"] > 0.002
            or native["timestamp_violations"] > 0
            or native["double_physics_diagnostics"] > 0
            or any(
                required[name]["last_capture_time_s"]
                < core_result.finish_time_s - sensor_tail_allowance[name]
                for name in ("rgb_camera", "depth_camera", "lidar", "imu")
            )
        ):
            raise SystemExit(4)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
