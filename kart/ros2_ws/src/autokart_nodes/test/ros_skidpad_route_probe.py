"""Qualified simulated-cone fixture through real ROS mapping and planner nodes."""

from __future__ import annotations

import json
import os
from math import cos, pi, sin
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from autokart_msgs.msg import ActuatorFeedback, Cone, ConeArray, MissionDebug


def ros_time(ns):
    from builtin_interfaces.msg import Time
    item = Time()
    item.sec, item.nanosec = divmod(ns, 1_000_000_000)
    return item


def course_cones():
    items = []
    for cx, blue, yellow in ((-9.125, 7.625, 10.625), (9.125, 10.625, 7.625)):
        for klass, radius in ((1, blue), (2, yellow)):
            for index in range(24):
                angle = 2 * pi * index / 24
                items.append((len(items), klass, cx + radius * cos(angle), radius * sin(angle)))
    for y in (5.0, 8.0, 11.0):
        for klass, x in ((1, -1.5), (2, 1.5)):
            items.append((len(items), klass, x, y))
    return items


class Probe(Node):
    def __init__(self):
        super().__init__("skidpad_route_probe")
        self.odom_pub = self.create_publisher(Odometry, "/autokart/odom", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(MissionDebug, "/autokart/perception/status", 10)
        self.cones_pub = self.create_publisher(ConeArray, "/perception/cones", qos_profile_sensor_data)
        self.feedback_pub = self.create_publisher(ActuatorFeedback, "/vehicle/actuator_feedback", 10)
        self.responses = {}
        self.valid_paths = 0
        self.create_subscription(MissionDebug, "/autokart/planner/status", self.on_planner, 10)
        self.cones = course_cones()

    def on_planner(self, message):
        payload = json.loads(message.payload_json)
        self.responses[payload["sample_time_ns"]] = payload
        if payload["valid"]:
            self.valid_paths += 1

    def send(self, stamp, cx, angle, speed=1.0, feedback=False):
        x, y = cx + 9.125 * cos(angle), 9.125 * sin(angle)
        heading = angle - pi / 2 if cx > 0 else angle + pi / 2
        self.send_pose(stamp, x, y, heading, speed, feedback)

    def send_pose(self, stamp, x, y, heading, speed=1.0, feedback=False):
        c, s = cos(heading), sin(heading)
        odom = Odometry()
        odom.header.stamp = ros_time(stamp)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = sin(heading / 2)
        odom.pose.pose.orientation.w = cos(heading / 2)
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.0025
        odom.twist.twist.linear.x = speed
        if feedback:
            item = ActuatorFeedback()
            item.measurement_time = ros_time(stamp)
            item.valid = True
            for _ in range(3):
                self.feedback_pub.publish(item)
                time.sleep(0.02)
        status = MissionDebug()
        status.measurement_time = ros_time(stamp)
        status.cycle = stamp
        status.frame_id = "base_link"
        status.stage = "PERCEPTION"
        status.payload_json = json.dumps({"schema_version": 1, "source": "SIMULATED",
                                          "valid": True, "frame_id": "base_link",
                                          "sample_time_ns": stamp, "expires_ns": stamp + 150_000_000,
                                          "sequence": stamp, "vehicle_authorized": False})
        array = ConeArray()
        array.measurement_time = array.delivery_time = ros_time(stamp)
        array.frame_id = "base_link"
        for cone_id, klass, world_x, world_y in self.cones:
            dx, dy = world_x - x, world_y - y
            item = Cone()
            item.id = cone_id
            item.semantic_class = klass
            item.position.x = float(c * dx + s * dy)
            item.position.y = float(-s * dx + c * dy)
            item.confidence = 0.95
            item.position_std_m = 0.05
            array.cones.append(item)
        self.odom_pub.publish(odom)
        self.status_pub.publish(status)
        self.cones_pub.publish(array)

    def wait(self, stamp):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and stamp not in self.responses:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.responses.pop(stamp, None)


def main():
    log_path = Path(sys.argv[1])
    with log_path.open("w") as log:
        processes = [subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
                     for command in (
                         ["ros2", "run", "autokart_mapping", "mapping_node"],
                         ["ros2", "run", "autokart_planner", "planner_node", "--ros-args",
                          "-p", "brake_min_mps2:=3.0", "-p", "brake_provenance:=SIMULATED"])]
        rclpy.init()
        probe = Probe()
        try:
            ready_by = time.monotonic() + 8
            while time.monotonic() < ready_by:
                if (probe.count_subscribers("/autokart/odom") >= 2 and
                        probe.count_subscribers("/perception/cones") >= 1):
                    break
                rclpy.spin_once(probe, timeout_sec=0.1)
            else:
                print("ROS_SUBSCRIBERS_NOT_READY")
                return 1
            stamp = 1_000_000_000
            states = []
            for index in range(81):
                probe.send(stamp, 9.125, pi - index * 2 * pi / 40)
                result = probe.wait(stamp)
                if result is None:
                    print(f"PLANNER_TIMEOUT_RIGHT_{index}")
                    return 1
                states.append(result["mission_state"]["state"])
                stamp += 1_500_000_000
            for index in range(1, 83):
                probe.send(stamp, -9.125, index * 2 * pi / 40)
                result = probe.wait(stamp)
                if result is None:
                    print(f"PLANNER_TIMEOUT_LEFT_{index}")
                    return 1
                states.append(result["mission_state"]["state"])
                stamp += 1_500_000_000
            exit_decisions = []
            for y, speed, feedback in ((3.5, 1.0, False), (5.0, 1.0, False),
                                       (8.0, 1.0, False), (10.4, 0.5, False),
                                       (11.0, 0.0, False), (11.0, 0.0, True)):
                probe.send_pose(stamp, 0.0, y, pi / 2, speed=speed, feedback=feedback)
                result = probe.wait(stamp)
                if result is None:
                    print(f"PLANNER_TIMEOUT_EXIT_{y}")
                    return 1
                states.append(result["mission_state"]["state"])
                exit_decisions.append(result["decision"])
                stamp += 3_000_000_000
            summary = {"source": "SIMULATED", "samples": len(states),
                       "right_laps": result["mission_state"]["right_laps"],
                       "left_laps": result["mission_state"]["left_laps"],
                       "final_state": result["mission_state"]["state"],
                       "valid_path_samples": probe.valid_paths,
                       "transition_seen": "TRANSITION" in states,
                       "stop_pending_seen": "STOP_PENDING" in states,
                       "exit_path_seen": "EXIT_PATH_CANDIDATE" in exit_decisions,
                       "exit_decisions": exit_decisions,
                       "vehicle_authorized": False}
            print(json.dumps(summary, sort_keys=True))
            return 0 if (summary["right_laps"] == 2 and summary["left_laps"] == 2 and
                         summary["final_state"] == "STOP" and summary["transition_seen"] and
                         summary["stop_pending_seen"] and summary["exit_path_seen"] and
                         summary["valid_path_samples"] > 0) else 1
        finally:
            probe.destroy_node()
            rclpy.shutdown()
            for process in processes:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            for process in processes:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()


if __name__ == "__main__":
    sys.exit(main())
