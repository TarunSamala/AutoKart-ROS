"""Native pause, resume, reset and disconnect probe for the Gazebo follower."""

from __future__ import annotations

import argparse
import json
import threading
from pathlib import Path
from time import monotonic, sleep


def _stamp(seconds: float, target: object) -> None:
    target.sec = int(seconds)
    target.nanosec = int(round((seconds - target.sec) * 1_000_000_000))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/gazebo_follower/synchronization/sync_probe.json"),
    )
    arguments = parser.parse_args()

    import rclpy
    from autokart_msgs.msg import (
        AuthoritativeVehicleSnapshot,
        GazeboFollowerState,
        SensorDeliveryMetadata,
    )
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    reliable = QoSProfile(
        depth=128,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    rclpy.init()
    node = Node("autokart_gazebo_sync_probe")
    publisher = node.create_publisher(
        AuthoritativeVehicleSnapshot,
        "/autokart/follower/authoritative_snapshot",
        reliable,
    )
    lock = threading.Lock()
    states: list[GazeboFollowerState] = []
    clocks: list[float] = []
    metadata: list[tuple[str, int, int]] = []

    def on_state(message: GazeboFollowerState) -> None:
        with lock:
            states.append(message)

    def on_clock(message: Clock) -> None:
        with lock:
            clocks.append(message.clock.sec + message.clock.nanosec * 1e-9)

    def on_metadata(message: SensorDeliveryMetadata) -> None:
        with lock:
            metadata.append((message.sensor_id, message.reset_epoch, message.sequence))

    node.create_subscription(
        GazeboFollowerState, "/autokart/follower/state", on_state, reliable
    )
    node.create_subscription(Clock, "/clock", on_clock, reliable)
    node.create_subscription(
        SensorDeliveryMetadata, "/sensors/delivery_metadata", on_metadata,
        QoSProfile(
            depth=1024,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        ),
    )
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def wait_until(predicate: object, timeout_s: float, description: str) -> None:
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            with lock:
                if predicate():
                    return
            sleep(0.02)
        raise RuntimeError(f"timeout waiting for {description}")

    def publish(sequence: int, epoch: int, simulation_time_s: float) -> None:
        message = AuthoritativeVehicleSnapshot()
        _stamp(simulation_time_s, message.header.stamp)
        message.header.frame_id = "map"
        message.simulation_time_s = simulation_time_s
        message.sequence = sequence
        message.position_x_m = 10.0
        message.position_y_m = 0.0
        message.position_z_m = 0.0
        message.roll_rad = 0.0
        message.pitch_rad = 0.0
        message.yaw_rad = 0.0
        message.linear_velocity_body_mps = [0.0, 0.0, 0.0]
        message.angular_velocity_body_radps = [0.0, 0.0, 0.0]
        message.front_left_steering_rad = 0.0
        message.front_right_steering_rad = 0.0
        message.front_left_wheel_rotation_rad = 0.0
        message.front_right_wheel_rotation_rad = 0.0
        message.rear_left_wheel_rotation_rad = 0.0
        message.rear_right_wheel_rotation_rad = 0.0
        message.front_left_wheel_speed_radps = 0.0
        message.front_right_wheel_speed_radps = 0.0
        message.rear_left_wheel_speed_radps = 0.0
        message.rear_right_wheel_speed_radps = 0.0
        message.rear_axle_rotation_rad = 0.0
        message.rear_axle_speed_radps = 0.0
        message.valid = True
        message.reset_epoch = epoch
        message.configuration_hash = "a" * 64
        publisher.publish(message)

    try:
        wait_until(
            lambda: publisher.get_subscription_count() > 0,
            10.0, "follower snapshot subscription",
        )
        publish(1, 0, 0.001)
        wait_until(
            lambda: states and states[-1].sequence == 1,
            5.0, "first follower state",
        )
        publish(2, 0, 0.011)
        wait_until(
            lambda: states and states[-1].sequence == 2 and not states[-1].follower_frozen,
            5.0, "second follower state",
        )
        sleep(0.1)
        with lock:
            pause_clock = clocks[-1]
            pause_metadata = len(metadata)
        wait_until(
            lambda: states and states[-1].sequence == 2
            and states[-1].follower_frozen and states[-1].follower_valid,
            2.0, "pause watchdog",
        )
        with lock:
            paused_clock_unchanged = clocks[-1] == pause_clock
            paused_sensors_stopped = len(metadata) == pause_metadata

        publish(3, 0, 0.021)
        wait_until(
            lambda: states and states[-1].sequence == 3
            and not states[-1].follower_frozen and states[-1].follower_valid,
            5.0, "resume",
        )
        with lock:
            resumed_without_jump = abs(states[-1].pose.position.x - 10.0) < 1e-12

        publish(1, 1, 0.001)
        wait_until(
            lambda: states and states[-1].reset_epoch == 1
            and states[-1].sequence == 1 and states[-1].follower_valid,
            5.0, "reset epoch",
        )
        with lock:
            reset_clock = clocks[-1]
        reset_rewound_explicitly = abs(reset_clock - 0.001) <= 0.001

        node.destroy_publisher(publisher)
        wait_until(
            lambda: states and states[-1].follower_frozen
            and not states[-1].follower_valid,
            3.0, "disconnect watchdog",
        )

        publisher = node.create_publisher(
            AuthoritativeVehicleSnapshot,
            "/autokart/follower/authoritative_snapshot",
            reliable,
        )
        wait_until(
            lambda: publisher.get_subscription_count() > 0,
            5.0, "reconnected follower snapshot subscription",
        )
        publish(2, 1, 0.011)
        wait_until(
            lambda: states and states[-1].sequence == 2
            and states[-1].follower_valid and not states[-1].follower_frozen,
            5.0, "reconnected follower",
        )
        with lock:
            before_delay_clock = clocks[-1]
        publish(3, 1, 0.051)
        wait_until(
            lambda: states and states[-1].sequence == 2
            and states[-1].follower_frozen and not states[-1].follower_valid
            and states[-1].state_age_s >= 0.039,
            5.0, "bounded bridge-delay rejection",
        )
        sleep(0.1)
        with lock:
            delayed_clock_frozen = clocks[-1] == before_delay_clock
            delayed_state_age_s = states[-1].state_age_s

        report = {
            "SY1_pause": {
                "follower_frozen": True,
                "clock_unchanged": paused_clock_unchanged,
                "sensor_delivery_stopped": paused_sensors_stopped,
            },
            "SY2_resume": {
                "follower_valid": True,
                "no_pose_jump": resumed_without_jump,
            },
            "SY3_reset": {
                "new_reset_epoch": 1,
                "explicit_clock_rewind": reset_rewound_explicitly,
                "autonomous_restart_source": "not_present_in_read_only_follower",
            },
            "SY4_bridge_delay": {
                "injected_snapshot_interval_s": 0.040,
                "reported_state_age_s": delayed_state_age_s,
                "follower_frozen": True,
                "follower_valid": False,
                "clock_unchanged": delayed_clock_frozen,
            },
            "SY5_disconnect": {
                "follower_frozen": True,
                "follower_valid": False,
            },
            "passed": all((
                paused_clock_unchanged,
                paused_sensors_stopped,
                resumed_without_jump,
                reset_rewound_explicitly,
                delayed_clock_frozen,
                delayed_state_age_s >= 0.039,
            )),
        }
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        if not report["passed"]:
            raise SystemExit(2)
    finally:
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
