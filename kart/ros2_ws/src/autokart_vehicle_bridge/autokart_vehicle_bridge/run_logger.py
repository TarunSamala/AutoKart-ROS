"""Read-only CSV recorder. Starts one file per launch; never commands the VCU."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
import time


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
    from autokart_msgs.msg import ActuatorFeedback, MissionDebug, VcuState
    from .run_log import RunCsvWriter

    class RunLogger(Node):
        def __init__(self):
            super().__init__("autokart_run_logger")
            folder = Path(str(self.declare_parameter("output_dir", "results/runs").value)).resolve()
            folder.mkdir(parents=True, exist_ok=True)
            self.path = folder / f"run_{datetime.now(timezone.utc):%Y%m%dT%H%M%S_%fZ}.csv"
            self.file = self.path.open("x", newline="", encoding="utf-8")
            self.writer = RunCsvWriter(self.file)
            self.speed = None
            self.dac_mv = None
            self.counts = None
            self.state = "UNKNOWN"
            self.create_subscription(ActuatorFeedback, "/vehicle/actuator_feedback", self.on_feedback, 10)
            self.create_subscription(String, "/autokart/vcu/axle_counts", self.on_counts, 10)
            self.create_subscription(VcuState, "/vcu/state", self.on_state, 10)
            self.create_subscription(MissionDebug, "/autokart/planner/status", self.on_plan, 10)
            self.get_logger().info(f"Run CSV: {self.path}")

        def on_feedback(self, message):
            self.speed = float(message.motor_speed_radps) if message.valid else None

        def on_counts(self, message):
            try:
                payload = json.loads(message.data)
                self.counts = int(payload["counts"])
                self.dac_mv = int(payload["dac_mv"])
                self.writer.write({"time": time.time(), "speed": self.speed,
                    "dac_mv": self.dac_mv, "state": self.state, "axle_counts": self.counts,
                    "decision": "VCU_TEL"})
            except (ValueError, TypeError, KeyError):
                self.counts = self.dac_mv = None

        def on_state(self, message):
            self.state = str(message.state)

        def on_plan(self, message):
            try:
                payload = json.loads(message.payload_json)
            except (ValueError, TypeError):
                return
            self.writer.write({"time": time.time(),
                "cone_forward_distance": payload.get("cone_forward_distance_m"),
                "bumper_to_cone": payload.get("bumper_to_cone_m"),
                "speed": self.speed, "dac_mv": self.dac_mv,
                "decision": payload.get("decision"), "state": self.state,
                "axle_counts": self.counts, "dead_reckoned": payload.get("dead_reckoned"),
                "frame_stamp_ns": payload.get("sample_time_ns")})

    rclpy.init(args=args)
    node = RunLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
