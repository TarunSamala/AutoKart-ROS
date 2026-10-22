"""ROS 2 node: straight path + stop target 15 cm before the cone ahead + braking decision."""
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from autokart_msgs.msg import LocalMap, PlannedPath, SafetyState, VehicleOdom

from .core import Planner, PlannerConfig


class PlannerNode(Node):
    def __init__(self):
        super().__init__("autokart_planner")
        p = self.declare_parameter
        self.cfg = PlannerConfig(
            stop_gap_m=float(p("stop_gap_m", 0.15).value),
            front_bumper_x_m=float(p("front_bumper_x_m", 0.6).value),
            cruise_speed_mps=float(p("cruise_speed_mps", 1.0).value),
            a_verified_mps2=float(p("a_verified_mps2", 0.5).value),
            t_latency_s=float(p("t_latency_s", 0.25).value),
            d_margin_m=float(p("d_margin_m", 0.10).value),
            stop_lateral_tol_m=float(p("stop_lateral_tol_m", 0.35).value),
            stop_class=str(p("stop_class", "").value),
            max_run_m=float(p("max_run_m", 15.0).value),
        )
        self.planner = Planner(self.cfg)
        self.map = None
        self.map_t = None
        self.pose = None
        self.speed = 0.0
        self.armed = False
        self.last_decision = ""
        self.create_subscription(LocalMap, "/autokart/map", self.on_map, 1)
        self.create_subscription(VehicleOdom, "/autokart/odom", self.on_odom, 10)
        self.create_subscription(SafetyState, "/autokart/safety/state", self.on_safety, 5)
        self.pub = self.create_publisher(PlannedPath, "/autokart/planned_path", 1)
        self.create_timer(0.1, self.tick)

    def on_map(self, m):
        self.map, self.map_t = m, time.monotonic()

    def on_odom(self, m):
        self.pose, self.speed = (m.x, m.y, m.yaw_rad), m.speed_mps

    def on_safety(self, m):
        self.armed = m.state == "ARMED"

    def tick(self):
        if self.pose is None:
            return
        now = time.monotonic()
        self.planner.set_active(self.armed)
        have = self.map is not None
        res = self.planner.plan(
            now,
            [(p.x, p.y) for p in self.map.centerline] if have else [],
            bool(have and self.map.valid), (now - self.map_t) if have else 1e9,
            [(c.cls, c.x, c.y) for c in self.map.cones] if have else [],
            self.pose, self.speed)

        if res.decision != self.last_decision:
            self.get_logger().info(f"decision {self.last_decision or '-'} -> {res.decision}")
            self.last_decision = res.decision
        if res.decision == "STOP" and self.planner.final_gap_m is not None:
            self.get_logger().info(f"stopped with {self.planner.final_gap_m * 100:.0f} cm to the cone (target {self.cfg.stop_gap_m * 100:.0f} cm)")
            self.planner.final_gap_m = None

        out = PlannedPath()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"
        out.frame_id = self.map.frame_id if have else 0
        out.path_valid, out.stop_valid, out.decision = res.path_valid, res.stop_valid, res.decision
        out.nearest_cone_m, out.available_m = float(res.nearest_cone_m), float(res.available_m)
        out.required_stop_m, out.target_speed_mps = float(res.required_stop_m), float(res.target_speed_mps)
        out.stop_target = Point(x=float(res.stop_target[0]), y=float(res.stop_target[1]), z=0.0)
        for x, y in res.path:
            out.path.append(Point(x=float(x), y=float(y), z=0.0))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
