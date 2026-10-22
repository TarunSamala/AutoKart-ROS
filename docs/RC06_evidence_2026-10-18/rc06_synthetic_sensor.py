#!/usr/bin/env python3
"""RC-06 integration harness: stands in for the physical HP60C + perception
stack, which is not attached to this sandbox. Publishes real ROS 2 messages
on the real topic names so the real front_safety_envelope_monitor and
dashboard_gateway nodes can be exercised end to end over real DDS and a real
WebSocket connection. This script is test infrastructure only, not part of
the delivered packages.

Phases (each ~5s), looping:
  CLEAR    -> obstacle far ahead, corridor otherwise empty
  BRAKING  -> obstacle inside the speed-dependent braking envelope
  CRITICAL -> obstacle inside the 1.0 m minimum clearance
  STALE    -> this node stops publishing odometry/obstacles/images entirely,
              to exercise the UNKNOWN/stale-data fail-safe path
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from autokart_msgs.msg import Cone, ConeArray, MissionDebug, VcuState
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

PHASES = ["CLEAR", "BRAKING", "CRITICAL", "STALE"]
PHASE_S = 5.0
BUMPER_SETBACK_M = 1.0668
SPEED_MPS = 2.0


def ros_time(node: Node) -> RosTime:
    return node.get_clock().now().to_msg()


class SyntheticSensor(Node):
    def __init__(self):
        super().__init__("rc06_synthetic_sensor")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pub = self.create_publisher(Odometry, "/vehicle/odometry", qos)
        self.obstacle_pub = self.create_publisher(ConeArray, "/perception/obstacles", qos)
        self.vcu_pub = self.create_publisher(VcuState, "/vcu/state", 10)
        self.rgb_pub = self.create_publisher(Image, "/autokart/perception/rgb", qos)
        self.depth_pub = self.create_publisher(Image, "/autokart/perception/depth", qos)
        self.debug_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
        self.start = time.monotonic()
        self.last_phase = None
        self.image_sequence = 0
        self.create_timer(1.0 / 20.0, self.tick_fast)
        self.create_timer(1.0 / 10.0, self.tick_images)

    def phase(self) -> str:
        elapsed = time.monotonic() - self.start
        index = int(elapsed // PHASE_S) % len(PHASES)
        return PHASES[index]

    def tick_fast(self):
        phase = self.phase()
        if phase != self.last_phase:
            self.get_logger().info(f"RC06_PHASE_TRANSITION -> {phase} @ {time.monotonic() - self.start:.1f}s")
            self.last_phase = phase
        if phase == "STALE":
            return  # Deliberately stop publishing to exercise UNKNOWN/stale paths.

        odom = Odometry()
        odom.header.stamp = ros_time(self)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = SPEED_MPS
        self.odom_pub.publish(odom)

        obstacles = ConeArray()
        obstacles.measurement_time = ros_time(self)
        obstacles.delivery_time = ros_time(self)
        obstacles.frame_id = "base_link"
        cone = Cone()
        cone.id = 1
        if phase == "CLEAR":
            cone.position.x = BUMPER_SETBACK_M + 10.0
        elif phase == "BRAKING":
            cone.position.x = BUMPER_SETBACK_M + 2.0  # trigger at 2m/s is 2.8m
        elif phase == "CRITICAL":
            cone.position.x = BUMPER_SETBACK_M + 0.4
        cone.position.y = 0.0
        obstacles.cones = [cone]
        self.obstacle_pub.publish(obstacles)

        vcu = VcuState()
        vcu.timestamp = ros_time(self)
        vcu.state = "ARMED"
        vcu.motor_enable = False
        vcu.active_faults = []
        self.vcu_pub.publish(vcu)

    def tick_images(self):
        phase = self.phase()
        if phase == "STALE":
            return
        width, height = 640, 480
        color = {"CLEAR": (40, 180, 80), "BRAKING": (40, 200, 230), "CRITICAL": (40, 40, 220)}.get(phase, (120, 120, 120))
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
        rgb[:, :] = color
        depth = np.full((height, width), 2000, dtype=np.uint16)
        stamp = ros_time(self)
        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "camera_color_optical"
        rgb_msg.width, rgb_msg.height = width, height
        rgb_msg.encoding = "bgr8"
        rgb_msg.step = width * 3
        rgb_msg.data = rgb.tobytes()
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_color_optical"
        depth_msg.width, depth_msg.height = width, height
        depth_msg.encoding = "mono16"
        depth_msg.step = width * 2
        depth_msg.data = depth.tobytes()
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)

        self.image_sequence += 1
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        debug = MissionDebug()
        debug.measurement_time = stamp
        debug.frame_id = "camera_color_optical"
        debug.stage = "perception"
        debug.payload_json = json.dumps({
            "sample_time_ns": stamp_ns, "source": "LIVE", "valid": True,
            "sequence": self.image_sequence, "reason_codes": [],
            "calibration_status": "MOUNT_CONFIGURED_UNVERIFIED",
            "model_sha256": "rc06-synthetic-harness", "detections": [], "cone_details": [],
        })
        self.debug_pub.publish(debug)


def main():
    rclpy.init()
    node = SyntheticSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
