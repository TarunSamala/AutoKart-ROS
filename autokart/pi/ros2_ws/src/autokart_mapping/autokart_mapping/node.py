"""ROS 2 node: tracks cones in the odom frame and builds the straight-corridor centerline."""
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from autokart_msgs.msg import ConeDetection, ConeDetectionArray, LocalMap, VehicleOdom

from .core import ConeTracker, MapConfig, build_corridor


class Mapping(Node):
    def __init__(self):
        super().__init__("autokart_mapping")
        p = self.declare_parameter
        self.cfg = MapConfig(
            left_class=p("left_class", "blue").value, right_class=p("right_class", "yellow").value,
            min_width_m=float(p("min_width_m", 1.0).value), max_width_m=float(p("max_width_m", 5.0).value),
            nominal_width_m=float(p("nominal_width_m", 3.0).value), min_hits=int(p("min_hits", 3).value),
            max_age_s=float(p("max_age_s", 20.0).value), max_cone_range_m=float(p("max_cone_range_m", 10.0).value),
        )
        self.tracker = ConeTracker(self.cfg)
        self.pose = None
        self.last_frame = 0
        self.create_subscription(VehicleOdom, "/autokart/odom", self.on_odom, 10)
        self.create_subscription(ConeDetectionArray, "/autokart/detections", self.on_det, 1)
        self.pub = self.create_publisher(LocalMap, "/autokart/map", 1)
        self.create_timer(0.1, self.tick)

    def on_odom(self, m: VehicleOdom):
        self.pose = (m.x, m.y, m.yaw_rad)

    def on_det(self, m: ConeDetectionArray):
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        cones = [(c.cls, c.x, c.y) for c in m.cones if c.depth_ok]
        self.tracker.update(cones, self.pose, now)
        self.last_frame = m.frame_id

    def tick(self):
        out = LocalMap()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"
        out.frame_id = self.last_frame
        if self.pose is None:
            out.valid, out.status = False, "NO_ODOMETRY"
            self.pub.publish(out)
            return
        tracks = self.tracker.confirmed()
        corridor = build_corridor(tracks, self.pose, self.cfg)
        out.valid, out.status, out.corridor_width_m = corridor.valid, corridor.status, float(corridor.width_m)
        for t in tracks:
            c = ConeDetection()
            c.cls, c.x, c.y, c.depth_ok, c.conf = t.cls, float(t.x), float(t.y), True, float(t.hits)
            out.cones.append(c)
        for x, y in corridor.centerline:
            out.centerline.append(Point(x=float(x), y=float(y), z=0.0))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Mapping()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
