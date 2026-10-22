"""Finite native ROS 2 graph smoke test for the generated AutoKart message."""

from __future__ import annotations

import rclpy
from autokart_msgs.msg import MotionCommand


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("autokart_native_smoke")
    received: list[MotionCommand] = []
    publisher = node.create_publisher(MotionCommand, "/control/motion_command", 1)
    subscription = node.create_subscription(MotionCommand, "/control/motion_command", received.append, 1)
    message = MotionCommand()
    message.source = "native_smoke"
    message.sequence = 1
    message.mode = "speed_steering"
    message.target_speed_mps = 0.0
    message.acceleration_limit_mps2 = 1.0
    message.deceleration_limit_mps2 = 1.0
    message.valid = True
    publisher.publish(message)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.02)
        if received:
            break
    node.destroy_subscription(subscription)
    node.destroy_publisher(publisher)
    node.destroy_node()
    rclpy.shutdown()
    if not received:
        print("native_ros2_message_round_trip: FAILED")
        return 1
    print("native_ros2_message_round_trip: PASS")
    print("command_path: /control/motion_command -> VCU bridge (no actuator topic)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
