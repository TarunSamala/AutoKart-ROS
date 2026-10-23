"""Existing Mission Control adapter, permanently read-only in this integration.

The reused gateway request publishers are destroyed and no HTTP/WebSocket
request sinks are installed. Canonical payloads retain their original source
claims through /integration/api/state; no camera or VCU device is opened.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from .adapter import GatewayAdapterError, load_gateway
from .readonly import make_readonly_node, attach_state_endpoint


class _ParamBootstrapNode(Node):
    """Exists only to declare and read ROS parameters from a launch config
    file, exactly like every other node in this library. Destroyed (or kept
    alive and reused, in "live" mode) immediately after parameters are read."""

    def __init__(self, **node_kwargs):
        super().__init__("autokart_dashboard_gateway", **node_kwargs)
        self.declare_parameter("gateway_package_path", "")
        self.declare_parameter("perception_package_path", "")
        self.declare_parameter("enable_camera_feed", False)
        self.declare_parameter("mode", "replay")  # "replay" | "live"
        self.declare_parameter("source", "SIMULATION")  # "SIMULATION" | "REAL_KART"
        self.declare_parameter("run_id", "ros-live")
        self.declare_parameter("scenario", "UNKNOWN")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("static_root", "")
        self.declare_parameter("display_only", True)
        self.declare_parameter("fsds", False)
        self.declare_parameter("replay_path", "")
        self.declare_parameter("readiness_path", "")

    def read(self) -> dict:
        return {
            "gateway_package_path": str(self.get_parameter("gateway_package_path").value),
            "perception_package_path": str(self.get_parameter("perception_package_path").value),
            "enable_camera_feed": bool(self.get_parameter("enable_camera_feed").value),
            "mode": str(self.get_parameter("mode").value),
            "source": str(self.get_parameter("source").value),
            "run_id": str(self.get_parameter("run_id").value),
            "scenario": str(self.get_parameter("scenario").value),
            "host": str(self.get_parameter("host").value),
            "port": int(self.get_parameter("port").value),
            "static_root": str(self.get_parameter("static_root").value),
            "display_only": bool(self.get_parameter("display_only").value),
            "fsds": bool(self.get_parameter("fsds").value),
            "replay_path": str(self.get_parameter("replay_path").value),
            "readiness_path": str(self.get_parameter("readiness_path").value),
        }


def _run_degraded_app(host: str, port: int, reason: str) -> None:
    """Dependency unavailable: serve ONLY a degraded /health endpoint, never
    crash, never fabricate ROS data, never accept a request. This has no
    ROS node, no publisher, and therefore categorically no actuation path --
    the strongest possible version of "read-only" when the real gateway
    cannot even be loaded."""
    from aiohttp import web

    async def health(_):
        return web.json_response({
            "status": "degraded", "gateway_available": False, "reason": reason,
            "vehicle_authority": "OFF", "actuator_authority": "NONE",
        })

    app = web.Application()
    app.router.add_get("/health", health)
    web.run_app(app, host=host, port=port)


def main(args=None):
    rclpy.init(args=args)
    bootstrap = _ParamBootstrapNode()
    params = bootstrap.read()
    bootstrap.destroy_node()

    try:
        gateway = load_gateway(params["gateway_package_path"], params["perception_package_path"])
    except GatewayAdapterError as exc:
        rclpy.logging.get_logger("autokart_dashboard_gateway").error(
            f"autokart_dashboard_gateway startup failed, real gateway unavailable: {exc}. "
            "Serving a degraded, read-only /health endpoint only -- no ROS node, no actuation path.")
        rclpy.shutdown()
        _run_degraded_app(params["host"], params["port"], str(exc))
        return

    store = gateway.MissionStore(params["source"], params["run_id"], scenario=params["scenario"])
    camera = None
    if params["enable_camera_feed"]:
        camera = gateway.CameraFeed()

    replay = params["mode"] == "replay"
    if replay and params["replay_path"]:
        gateway.load_replay(Path(params["replay_path"]), store)

    if replay:
        # No rclpy node, no subscriptions, no publishers at all in this mode
        # -- matches the upstream gateway's own --replay behavior exactly.
        rclpy.shutdown()
        from aiohttp import web
        static_root = Path(params["static_root"]) if params["static_root"] else None
        app = gateway.create_app(store, static_root, replay=True, camera=camera)
        web.run_app(app, host=params["host"], port=params["port"])
        return

    node, published_state = make_readonly_node(gateway, store, camera=camera)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=False)
    thread.start()

    request_sink = demo_drive_sink = demo_drive_ping = None

    from aiohttp import web
    static_root = Path(params["static_root"]) if params["static_root"] else None
    readiness_path = Path(params["readiness_path"]) if params["readiness_path"] else None
    app = gateway.create_app(
        store, static_root, replay=False, camera=camera,
        request_sink=request_sink, readiness_path=readiness_path,
        demo_drive_sink=demo_drive_sink, demo_drive_ping=demo_drive_ping,
    )
    attach_state_endpoint(app, published_state)
    try:
        web.run_app(app, host=params["host"], port=params["port"])
    finally:
        executor.shutdown()
        thread.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
