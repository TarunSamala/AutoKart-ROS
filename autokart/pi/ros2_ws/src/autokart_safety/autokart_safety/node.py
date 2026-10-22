"""ROS 2 node: sensor-freshness + motion supervision, startup stages, arm/stop authority."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from autokart_msgs.msg import (ConeDetectionArray, ControllerState, LocalMap, PlannedPath, SafetyState,
                               StartupStage, VehicleOdom, VehicleTelemetry)

from .core import Config, Inputs, SafetyLogic, TelView
from autokart_vehicle_bridge.protocol import FAULT_BITS


class SafetyNode(Node):
    def __init__(self):
        super().__init__("autokart_safety")
        p = self.declare_parameter
        self.cfg = Config(
            overspeed_mps=float(p("overspeed_mps", 2.5).value),
            stop_ineffective_s=float(p("stop_ineffective_s", 4.0).value),
        )
        self.vehicle_config_verified = bool(p("vehicle_config_verified", False).value)
        self.braking_verified = bool(p("braking_verified", False).value)
        self.logic = SafetyLogic(self.cfg)

        self.last = {}                 # topic -> monotonic receive time
        self.tel = None
        self.det = None
        self.ctrl = None
        self.requests = []
        self.estop_confirmed = False

        self.create_subscription(CompressedImage, "/autokart/camera/compressed", lambda m: self.touch("image"), 1)
        self.create_subscription(ConeDetectionArray, "/autokart/detections", self.on_det, 1)
        self.create_subscription(LocalMap, "/autokart/map", lambda m: self.touch("map"), 1)
        self.create_subscription(VehicleOdom, "/autokart/odom", lambda m: self.touch("odom"), 5)
        self.create_subscription(PlannedPath, "/autokart/planned_path", lambda m: self.touch("planner"), 5)
        self.create_subscription(ControllerState, "/autokart/controller_state", self.on_ctrl, 5)
        self.create_subscription(VehicleTelemetry, "/autokart/vehicle_telemetry", self.on_tel, 10)
        self.create_subscription(String, "/autokart/safety/request", self.on_request, 10)

        self.pub = self.create_publisher(SafetyState, "/autokart/safety/state", 10)
        self.pub_req = self.create_publisher(String, "/autokart/bridge/vcu_request", 10)
        self.create_timer(0.1, self.tick)

    def touch(self, key):
        self.last[key] = time.monotonic()

    def on_det(self, m):
        self.touch("detections")
        self.det = m

    def on_ctrl(self, m):
        self.touch("controller")
        self.ctrl = m

    def on_tel(self, m):
        self.touch("telemetry")
        names = [n for b, n in FAULT_BITS.items() if m.fault & b]
        if not m.estop_ok:
            self.estop_confirmed = False   # an E-stop event requires the operator to re-confirm the test
        self.tel = TelView(m.state, m.fault, names, m.estop_ok, m.steer_pwm_ok, m.brake_pwm_ok,
                           m.axle_motion_seen, m.protocol_ok, m.fw_version, m.speed_mps)

    def on_request(self, m):
        self.requests.append(m.data)
        if m.data == "confirm_estop_tested":
            self.estop_confirmed = True

    def tick(self):
        now = time.monotonic()
        ages = {k: now - self.last[k] for k in self.last}
        inp = Inputs(
            ages=ages, tel=self.tel,
            detections_depth_valid=bool(self.det and self.det.depth_valid),
            detections_inference_ms=float(self.det.inference_ms) if self.det else 0.0,
            calibration_verified=bool(self.det and self.det.calibration_verified),
            vehicle_config_verified=self.vehicle_config_verified,
            braking_verified=self.braking_verified,
            estop_operator_confirmed=self.estop_confirmed,
            controller_state=self.ctrl.state if self.ctrl else "IDLE",
            controller_stopped=bool(self.ctrl and self.ctrl.stopped),
        )
        reqs, self.requests = self.requests, []
        out = self.logic.step(now, inp, reqs)

        for e in out.events:
            self.get_logger().info(e)
        if out.vcu_request:
            self.pub_req.publish(String(data=out.vcu_request))

        m = SafetyState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state, m.arm_allowed, m.vcu_heartbeat_ok = out.state, out.arm_allowed, out.vcu_heartbeat_ok
        m.startup_ready, m.stop_latched, m.reasons = out.startup_ready, out.stop_latched, out.reasons
        for s in out.stages:
            st = StartupStage()
            st.id, st.name, st.state, st.detail = s["id"], s["name"], s["state"], s["detail"]
            m.stages.append(st)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
