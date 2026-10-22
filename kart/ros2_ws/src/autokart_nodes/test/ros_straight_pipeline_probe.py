"""Read-only ROS integration probe with explicitly simulated odometry."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import urlopen

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from autokart_msgs.msg import Cone, ConeArray, MissionDebug


class Probe(Node):
    def __init__(self, mission: str):
        super().__init__("rc03_rc05_probe")
        self.mission = mission
        self.perception_valid = 0
        self.mapping_valid = 0
        self.planner_hold = 0
        self.planner_valid = 0
        self.reasons = []
        self.latest_odom = None
        self.odom_pub = self.create_publisher(Odometry, "/autokart/odom", qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(ConeArray, "/autokart/planner/obstacles", qos_profile_sensor_data)
        self.create_subscription(MissionDebug, "/autokart/perception/status", self.on_perception, 10)
        self.create_subscription(MissionDebug, "/autokart/mapping/status", self.on_mapping, 10)
        self.create_subscription(MissionDebug, "/autokart/planner/status", self.on_planner, 10)

    def on_perception(self, message):
        payload = json.loads(message.payload_json)
        if not payload.get("valid") or payload.get("source") != "SIMULATED":
            return
        self.perception_valid += 1
        odom = Odometry()
        odom.header.stamp = message.measurement_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0] = 0.0025
        odom.pose.covariance[7] = 0.0025
        if self.mission == "BRAKING_TEST":
            obstacles = ConeArray()
            obstacles.measurement_time = message.measurement_time
            obstacles.frame_id = "base_link"
            item = Cone()
            item.id = 999
            item.semantic_class = 0
            item.position.x = 4.0
            item.position.y = 0.0
            item.position_std_m = 0.3
            item.confidence = 0.95
            obstacles.cones.append(item)
            self.obstacle_pub.publish(obstacles)
        self.latest_odom = odom
        self.odom_pub.publish(odom)

    def on_mapping(self, message):
        payload = json.loads(message.payload_json)
        if payload.get("valid") and payload.get("source") == "SIMULATED":
            if payload.get("tracked_count", 0) >= 6:
                self.mapping_valid += 1

    def on_planner(self, message):
        payload = json.loads(message.payload_json)
        if payload.get("valid"):
            self.planner_valid += 1
        if not payload.get("valid") and payload.get("source") == "SIMULATED":
            self.planner_hold += 1
            self.reasons.extend(payload.get("reason_codes", []))
            if self.latest_odom is not None:
                self.odom_pub.publish(self.latest_odom)


def gateway_frame_valid(frame: dict, target_reason: str) -> bool:
    stages = frame.get("diagnostics") or {}
    perception = stages.get("perception") or {}
    mapping = stages.get("mapping") or {}
    planning = stages.get("planning") or {}
    return (perception.get("source") == "SIMULATED" and
            mapping.get("valid") is True and planning.get("valid") is False and
            target_reason in planning.get("reason_codes", []) and
            frame["vehicle"]["target_speed_mps"] is None)


def main() -> int:
    log_path = Path(sys.argv[1])
    mission = sys.argv[2] if len(sys.argv) > 2 else "SKIDPAD"
    if mission not in {"SKIDPAD", "BRAKING_TEST"}:
        raise ValueError("unsupported probe mission")
    target_reason = "OBSERVED_LOOPS_UNAVAILABLE" if mission == "SKIDPAD" else "CORRIDOR_BLOCKED"
    with log_path.open("w") as launch_log:
        launch = subprocess.Popen(
            ["ros2", "launch", "autokart_bringup", "rc03_rc05_diagnostics.launch.py",
             "brake_min_mps2:=3.0", "brake_provenance:=SIMULATED",
             f"mission:={mission}",
             "launch_gateway:=true", "gateway_port:=18765"],
            stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        probe = Probe(mission)
        try:
            deadline = time.monotonic() + 15
            gateway_valid = False
            camera_valid = False
            status_valid = False
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
                if probe.perception_valid and probe.mapping_valid and probe.planner_hold:
                    try:
                        with urlopen("http://127.0.0.1:18765/mission/api/replay", timeout=0.5) as response:
                            frames = json.load(response)["frames"]
                        gateway_valid = any(gateway_frame_valid(frame, target_reason) for frame in frames)
                        with urlopen("http://127.0.0.1:18765/mission/api/status", timeout=0.5) as response:
                            live_status = json.load(response)
                        status_valid = (live_status["source"] == "SIMULATION" and
                                        live_status["vehicle_authorized"] is False and
                                        live_status["stages"]["perception"]["payload"]["source"] == "SIMULATED")
                        with urlopen("http://127.0.0.1:18765/mission/api/camera", timeout=0.5) as response:
                            camera = json.load(response)
                        if camera.get("available") and camera.get("source") == "SIMULATED":
                            frame_id = camera["frame_id"]
                            with urlopen(f"http://127.0.0.1:18765/mission/api/camera/{frame_id}/rgb", timeout=0.5) as response:
                                rgb = response.read()
                            with urlopen(f"http://127.0.0.1:18765/mission/api/camera/{frame_id}/depth", timeout=0.5) as response:
                                depth = response.read()
                            camera_valid = (rgb.startswith(b"\xff\xd8") and depth.startswith(b"\xff\xd8")
                                            and camera["depth_qualified"] is False)
                    except (OSError, ValueError, TypeError, AttributeError):
                        gateway_valid = False
                    if gateway_valid and status_valid and camera_valid and target_reason in probe.reasons:
                        break
            result = {"perception_valid_samples": probe.perception_valid,
                      "mapping_valid_samples": probe.mapping_valid,
                      "planner_hold_samples": probe.planner_hold,
                      "planner_valid_samples": probe.planner_valid,
                      "planner_reasons": sorted(set(probe.reasons)),
                      "mission": mission,
                      "gateway_diagnostics_valid": gateway_valid,
                      "gateway_status_valid": status_valid,
                      "paired_camera_jpeg_valid": camera_valid,
                      "source": "SIMULATED", "vehicle_authorized": False}
            print(json.dumps(result, sort_keys=True))
            return 0 if (probe.perception_valid > 0 and probe.mapping_valid > 0 and
                         probe.planner_hold > 0 and
                         target_reason in probe.reasons and
                         probe.planner_valid == 0 and gateway_valid and status_valid and camera_valid) else 1
        finally:
            probe.destroy_node()
            rclpy.shutdown()
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(launch.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                launch.wait()


if __name__ == "__main__":
    sys.exit(main())
