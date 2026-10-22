#!/usr/bin/env python3
"""ROS 2 -> WebSocket gateway for the Mission Control dashboard.

    python3 gateway.py --port 8765 --http-port 8080
    then open http://<laptop-ip>:8080/index.html

The gateway only OBSERVES ROS topics. The one thing it can send into ROS is a request string
("arm", "stop", "reset_fault", "confirm_estop_tested") to /autokart/safety/request; the safety node decides.
It never publishes actuator commands.
"""
import argparse
import functools
import http.server
import os
import threading

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from autokart_msgs.msg import (ConeDetectionArray, ControllerState, LocalMap, PlannedPath, SafetyState, VehicleOdom,
                               VehicleTelemetry)

from gateway_core import GatewayCore, GatewayServer


class Gateway(Node):
    def __init__(self, args):
        super().__init__("autokart_dashboard_gateway")
        self.pub_req = self.create_publisher(String, "/autokart/safety/request", 10)
        self.server = GatewayServer(args.host, args.port, self.on_request)
        self.core = GatewayCore(self.server.emit, mode=args.mode, throttle_cap_v=args.throttle_cap_v)
        c = self.core
        self.create_subscription(CompressedImage, "/autokart/camera/compressed", c.on_image, 1)
        self.create_subscription(ConeDetectionArray, "/autokart/detections", c.on_detections, 1)
        self.create_subscription(LocalMap, "/autokart/map", c.on_map, 1)
        self.create_subscription(PlannedPath, "/autokart/planned_path", c.on_plan, 5)
        self.create_subscription(ControllerState, "/autokart/controller_state", c.on_ctrl, 5)
        self.create_subscription(VehicleOdom, "/autokart/odom", c.on_odom, 10)
        self.create_subscription(VehicleTelemetry, "/autokart/vehicle_telemetry", c.on_telemetry, 10)
        self.create_subscription(SafetyState, "/autokart/safety/state", c.on_safety, 5)
        self.create_subscription(Log, "/rosout", c.on_rosout, 50)
        self.server.start()
        self.get_logger().info(f"gateway listening on ws://{args.host}:{args.port} (mode {args.mode})")

    def on_request(self, action: str, msg: dict):
        self.pub_req.publish(String(data=action))
        self.get_logger().info(f"dashboard request: {action}")


def serve_frontend(port: int, directory: str):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=directory)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--http-port", type=int, default=8080, help="0 disables the static file server")
    ap.add_argument("--mode", default="LIVE", choices=["LIVE", "REPLAY", "SIMULATED"])
    ap.add_argument("--throttle-cap-v", type=float, default=2.0,
                     help="must match autokart.yaml throttle_v_max; shown on the drive panel, never hardcoded")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Gateway(args)
    if args.http_port:
        here = os.path.dirname(os.path.abspath(__file__))
        serve_frontend(args.http_port, os.path.join(here, "..", "frontend"))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
