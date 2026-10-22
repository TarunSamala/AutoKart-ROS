"""ROS 2 node: Pure Pursuit (observed) + speed control -> VehicleCommand at 20 Hz."""
import time

import rclpy
from rclpy.node import Node

from autokart_msgs.msg import ControllerState, PlannedPath, SafetyState, VehicleCommand, VehicleOdom

from .core import THROTTLE_V_MAX, Controller, ControllerConfig


class ControllerNode(Node):
    def __init__(self):
        super().__init__("autokart_controller")
        p = self.declare_parameter
        self.cfg = ControllerConfig(
            wheelbase_m=float(p("wheelbase_m", 1.0).value),
            lookahead_min_m=float(p("lookahead_min_m", 0.8).value),
            lookahead_max_m=float(p("lookahead_max_m", 2.5).value),
            max_steer_deg=float(p("max_steer_deg", 25.0).value),
            volts_per_mps=float(p("volts_per_mps", 1.0).value),
            throttle_v_max=min(float(p("throttle_v_max", 2.0).value), THROTTLE_V_MAX),
            throttle_run_v=float(p("throttle_run_v", 2.0).value),
        )
        self.ctl = Controller(self.cfg)
        self.plan, self.plan_t = None, None
        self.pose, self.speed = None, 0.0
        self.armed = False
        self.seq = 0
        self.create_subscription(PlannedPath, "/autokart/planned_path", self.on_plan, 1)
        self.create_subscription(VehicleOdom, "/autokart/odom", self.on_odom, 10)
        self.create_subscription(SafetyState, "/autokart/safety/state", self.on_safety, 5)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/autokart/vehicle_command", 1)
        self.pub_state = self.create_publisher(ControllerState, "/autokart/controller_state", 1)
        self.create_timer(0.05, self.tick)

    def on_plan(self, m):
        self.plan, self.plan_t = m, time.monotonic()

    def on_odom(self, m):
        self.pose, self.speed = (m.x, m.y, m.yaw_rad), m.speed_mps

    def on_safety(self, m):
        self.armed = m.state == "ARMED"

    def tick(self):
        if self.pose is None:
            return
        now = time.monotonic()
        fresh = self.plan is not None and now - self.plan_t < 0.5
        decision = self.plan.decision if fresh else "BRAKE"     # no fresh plan while armed means brake
        target = self.plan.target_speed_mps if fresh else 0.0
        path = [(q.x, q.y) for q in self.plan.path] if fresh else []
        out = self.ctl.step(now, self.armed, decision, target, path, self.pose, self.speed)

        stamp = self.get_clock().now().to_msg()
        cmd = VehicleCommand()
        cmd.header.stamp = stamp
        self.seq += 1
        cmd.seq = self.seq
        cmd.throttle_v, cmd.steer_deg, cmd.brake_pct = float(out.throttle_v), float(out.steer_deg), float(out.brake_pct)
        self.pub_cmd.publish(cmd)

        st = ControllerState()
        st.header.stamp = stamp
        st.state, st.target_speed_mps, st.measured_speed_mps = out.state, float(target), float(self.speed)
        st.throttle_v, st.brake_pct, st.steer_deg_cmd = float(out.throttle_v), float(out.brake_pct), float(out.steer_deg)
        st.steer_actuated = False
        st.alpha_rad, st.lookahead_m, st.wheelbase_m = float(out.steer.alpha_rad), float(out.steer.lookahead_m), float(self.cfg.wheelbase_m)
        st.lookahead_x, st.lookahead_y, st.lookahead_valid = float(out.steer.lookahead_xy[0]), float(out.steer.lookahead_xy[1]), out.steer.valid
        st.stopped = out.stopped
        self.pub_state.publish(st)


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
