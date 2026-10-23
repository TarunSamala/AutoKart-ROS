"""ROS 2 telemetry to a bounded, read-only Mission Control WebSocket.

The node subscribes to existing autonomy/VCU topics. An optional MissionDebug
topic carries explanations produced by autonomy nodes; this gateway never
calculates or publishes an autonomy or actuator command. The only thing a
browser can send is one of a fixed whitelist of REQUEST strings
(ALLOWED_SAFETY_REQUESTS), forwarded as-is to /autokart/safety/request; only
autokart_vcu_safety decides what, if anything, happens as a result -- this
gateway has no actuator publisher and cannot bypass that decision.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import time
from math import atan2, hypot, isfinite
from pathlib import Path
from threading import Thread
from typing import Any, Callable, Optional

from .core import MissionStore, valid_frame
from .camera_feed import CameraFeed

ALLOWED_SAFETY_REQUESTS = {"arm", "stop", "reset_fault", "confirm_manual_power_cut",
                           "demo_drive_start", "demo_drive_stop"}
DEMO_DRIVE_DIRECTIONS = {"forward"}  # "reverse" is refused -- see autokart_vehicle_bridge/demo_drive.py
DEMO_DRIVE_MAX_DURATION_S = 10.0


def validate_demo_drive_request(action: str, direction: str, duration_s: float) -> bool:
    """Pure, no-ROS validation shared by send_demo_drive_message and its tests. A "drive" action
    needs a supported direction and an in-range duration; "stop" needs nothing. Anything else is
    rejected."""
    if action == "stop":
        return True
    if action != "drive":
        return False
    if direction not in DEMO_DRIVE_DIRECTIONS:
        return False
    return isinstance(duration_s, (int, float)) and 0.0 < duration_s <= DEMO_DRIVE_MAX_DURATION_S


def ros_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def ros_nanoseconds(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(quaternion: Any) -> float:
    return atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


def cone_payload(message: Any) -> list[dict[str, Any]]:
    labels = {0: "UNKNOWN", 1: "BLUE", 2: "YELLOW", 3: "ORANGE", 4: "BIG_ORANGE"}
    return [
        {
            "id": str(cone.id), "class": labels.get(cone.semantic_class, str(cone.semantic_class)),
            "confidence": float(cone.confidence), "x_m": float(cone.position.x),
            "y_m": float(cone.position.y), "z_m": float(cone.position.z),
        }
        for cone in message.cones
    ]


def safety_envelope_payload(message: Any) -> dict[str, Any]:
    """Normalize a diagnostic-only /safety/envelope sample for display.

    message.limiting_reason is "<STATE>:<reason>[:CALIBRATION_UNVERIFIED]",
    written by the front safety envelope monitor (DIAGNOSTIC_ONLY).
    """
    state, _, reason = message.limiting_reason.partition(":")
    data_source = str(getattr(message, "data_source", "") or "UNVERIFIED")
    return {
        "state": state or "UNKNOWN",
        "reason": reason,
        "valid": bool(message.valid),
        "calibration_verified": "CALIBRATION_UNVERIFIED" not in reason,
        "clear_distance_m": message.clear_distance_m if isfinite(message.clear_distance_m) else None,
        "stopping_distance_m": message.stopping_distance_m if isfinite(message.stopping_distance_m) else None,
        "guaranteed_deceleration_mps2": float(message.guaranteed_deceleration_mps2),
        # Self-reported by the publishing node's own configuration -- not
        # cryptographic proof of a physical sensor. Display, never trust alone.
        "data_source": data_source if data_source in {"LIVE", "SIMULATED", "REPLAY"} else "UNVERIFIED",
    }


def fsds_track_payload(message: Any) -> list[dict[str, Any]]:
    """Map FSDS's latched fs_msgs/Track to display-only world cones."""
    labels = {0: "BLUE", 1: "YELLOW", 2: "BIG_ORANGE", 3: "ORANGE", 4: "UNKNOWN"}
    return [
        {"id": f"FSDS_{index}", "class": labels.get(int(cone.color), "UNKNOWN"),
         "confidence": 1.0, "x_m": float(cone.location.x),
         "y_m": float(cone.location.y), "z_m": float(cone.location.z)}
        for index, cone in enumerate(message.track)
    ]


def make_ros_node(store: MissionStore, fsds: bool = False, camera: CameraFeed | None = None) -> Any:
    try:
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import Odometry, Path as RosPath
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from autokart_msgs.msg import ActuatorFeedback, ConeArray, MissionDebug, MotionCommand, SafetyEnvelope, VcuState
        if fsds:
            from fs_msgs.msg import Track
    except ImportError as exc:
        raise RuntimeError(f"ROS 2 interfaces unavailable: {exc}") from exc

    class DashboardGatewayNode(Node):
        def __init__(self) -> None:
            super().__init__("dashboard_gateway")
            sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Odometry, "/fsds/testing_only/odom" if fsds else "/vehicle/odometry", self._odometry, sensor_qos)
            if store.source == "SIMULATION" and not fsds:
                self.create_subscription(Odometry, "/autokart/odom", self._odometry, sensor_qos)
            self.create_subscription(ConeArray, "/perception/cones", self._cones, sensor_qos)
            self.create_subscription(ConeArray, "/autokart/mapping/cones", self._tracked_cones, sensor_qos)
            if store.source == "SIMULATION":
                self.create_subscription(ConeArray, "/dashboard/ground_truth/cones", self._ground_truth, sensor_qos)
            if fsds:
                track_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
                self.create_subscription(Track, "/fsds/testing_only/track", self._fsds_track, track_qos)
            self.create_subscription(RosPath, "/planning/path", self._path, sensor_qos)
            self.create_subscription(RosPath, "/autokart/planner/path_candidate", self._path, sensor_qos)
            self.create_subscription(MotionCommand, "/control/motion_command", self._command, 10)
            self.create_subscription(ActuatorFeedback, "/vehicle/actuator_feedback", self._feedback, 10)
            self.create_subscription(VcuState, "/vcu/state", self._vcu, 10)
            self.create_subscription(SafetyEnvelope, "/safety/envelope", self._safety_envelope, 10)
            self.create_subscription(MissionDebug, "/dashboard/debug", self._debug, 10)
            if camera is not None:
                self.create_subscription(Image, "/autokart/perception/rgb", self._rgb, sensor_qos)
                self.create_subscription(Image, "/autokart/perception/depth", self._depth, sensor_qos)
            self.request_pub = self.create_publisher(String, "/autokart/safety/request", 10)
            # DEMO DRIVE only: narrow JSON control messages (action/direction/duration_s), never a
            # throttle voltage or motor command -- this gateway still has no actuator publisher in
            # that sense. autokart_vcu_safety's demo_drive_active (ARMED-gated) and the vehicle
            # bridge's own CommandGate are what actually permit motion; see demo_drive.py.
            self.demo_drive_pub = self.create_publisher(String, "/autokart/demo_drive/control", 10)

        def send_request(self, action: str) -> bool:
            """Forward one whitelisted request string. Returns False (and publishes nothing) for anything
            outside ALLOWED_SAFETY_REQUESTS -- this is the only effect a browser can have on the vehicle, and
            even this is only a request autokart_vcu_safety may refuse."""
            if action not in ALLOWED_SAFETY_REQUESTS:
                return False
            self.request_pub.publish(String(data=action))
            self.get_logger().info(f"dashboard request: {action}")
            return True

        def send_demo_drive_ping(self) -> None:
            """Backend-owned heartbeat: called on a timer while the dashboard's demo-drive panel
            has an active websocket, independent of exact browser message timing. See
            receive_demo_drive_message's docstring for why this is NOT itself sufficient to start
            or continue motion -- it only ever refreshes the dead-man clock."""
            self.demo_drive_pub.publish(String(data=json.dumps({"action": "ping"})))

        def send_demo_drive_message(self, action: str, direction: str = "", duration_s: float = 0.0) -> bool:
            """Validates strictly and forwards one demo-drive control message. Returns False (and
            publishes nothing) for anything outside the fixed action/direction/duration contract --
            this is the only drive-parameter effect a browser can have, and autokart_vehicle_bridge's
            DemoDriveController still refuses it unless demo_drive_active and heartbeat-fresh."""
            if not validate_demo_drive_request(action, direction, duration_s):
                return False
            if action == "stop":
                self.demo_drive_pub.publish(String(data=json.dumps({"action": "stop"})))
            else:
                self.demo_drive_pub.publish(String(data=json.dumps(
                    {"action": "drive", "direction": direction, "duration_s": float(duration_s)})))
            return True

        def _rgb(self, message: Any) -> None:
            camera.update_image("rgb", ros_nanoseconds(message.header.stamp), message)

        def _depth(self, message: Any) -> None:
            camera.update_image("depth", ros_nanoseconds(message.header.stamp), message)

        def _odometry(self, message: Any) -> None:
            if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
                self.get_logger().warning("Dropped odometry outside odom/base_link contract")
                return
            pose = message.pose.pose
            twist = message.twist.twist
            try:
                store.capture(ros_seconds(message.header.stamp), {
                    "x_m": float(pose.position.x), "y_m": float(pose.position.y),
                    "z_m": float(pose.position.z), "yaw_rad": yaw_from_quaternion(pose.orientation),
                    "speed_mps": hypot(twist.linear.x, twist.linear.y),
                })
            except ValueError as exc:
                self.get_logger().warning(f"Dropped invalid odometry: {exc}")

        def _cones(self, message: Any) -> None:
            name = "ground_truth_cones" if message.ground_truth else "cones"
            try:
                if message.frame_id != "odom":
                    # Only odom coordinates may enter the common world view;
                    # mapping publishes transformed observations separately.
                    return
                store.update(name, ros_seconds(message.measurement_time), cone_payload(message))
            except ValueError as exc:
                self.get_logger().warning(f"Dropped cone sample: {exc}")

        def _tracked_cones(self, message: Any) -> None:
            if message.frame_id != "odom" or message.ground_truth:
                return
            try:
                store.update("tracked_cones", ros_seconds(message.measurement_time), cone_payload(message))
            except ValueError as exc:
                self.get_logger().warning(f"Dropped tracked cone sample: {exc}")

        def _ground_truth(self, message: Any) -> None:
            try:
                store.update("ground_truth_cones", ros_seconds(message.measurement_time), cone_payload(message))
            except ValueError as exc:
                self.get_logger().warning(f"Dropped ground truth sample: {exc}")

        def _fsds_track(self, message: Any) -> None:
            try:
                store.update("ground_truth_cones", 0.0, fsds_track_payload(message))
            except ValueError as exc:
                self.get_logger().warning(f"Dropped FSDS track: {exc}")

        def _path(self, message: Any) -> None:
            if message.header.frame_id != "odom":
                self.get_logger().warning("Dropped path outside odom frame")
                return
            points = [
                {"x_m": float(item.pose.position.x), "y_m": float(item.pose.position.y), "z_m": float(item.pose.position.z)}
                for item in message.poses
            ]
            store.update("path", ros_seconds(message.header.stamp), points)

        def _command(self, message: Any) -> None:
            if not message.valid:
                return
            store.update("command", ros_seconds(message.timestamp), {
                "target_speed_mps": float(message.target_speed_mps),
                "requested_steering_rad": float(message.steering_angle_rad),
            })

        def _feedback(self, message: Any) -> None:
            if not message.valid:
                return
            store.update("feedback", ros_seconds(message.measurement_time), {
                "measured_steering_rad": float(message.road_wheel_angle_rad),
                "brake_feedback": float(message.brake_actuator_position_m),
            })

        def _vcu(self, message: Any) -> None:
            store.update("vcu", ros_seconds(message.timestamp), {
                "state": message.state,
                "physical_estop": bool(message.emergency_stop_active) if message.estop_input_fitted else None,
                "estop_input_fitted": bool(message.estop_input_fitted),
                "estop_input_status": ("ACTIVE" if message.emergency_stop_active else "OK") if message.estop_input_fitted else "NOT FITTED - MANUAL POWER CUT ONLY",
                "motor_enable": bool(message.motor_enable),
                "faults": list(message.active_faults),
            })

        def _safety_envelope(self, message: Any) -> None:
            # Front safety envelope monitor is DIAGNOSTIC_ONLY (vehicle_authority=OFF);
            # this gateway only displays its classification, never acts on it.
            try:
                store.update("safety_envelope", ros_seconds(message.timestamp), safety_envelope_payload(message))
            except ValueError as exc:
                self.get_logger().warning(f"Dropped invalid safety envelope sample: {exc}")

        def _debug(self, message: Any) -> None:
            try:
                payload = json.loads(message.payload_json)
                store.update_debug(message.stage.lower(), ros_seconds(message.measurement_time), payload)
                if camera is not None and message.stage.lower() == "perception":
                    camera.update_status(ros_nanoseconds(message.measurement_time), payload)
            except (ValueError, json.JSONDecodeError) as exc:
                self.get_logger().warning(f"Dropped invalid MissionDebug: {exc}")

    return DashboardGatewayNode()


def create_app(store: MissionStore, static_root: Path | None = None, replay: bool = False,
               camera: CameraFeed | None = None,
               request_sink: Optional[Callable[[str], bool]] = None,
               readiness_path: Path | None = None,
               demo_drive_sink: Optional[Callable[..., bool]] = None,
               demo_drive_ping: Optional[Callable[[], None]] = None) -> Any:
    try:
        from aiohttp import web
    except ImportError as exc:
        raise RuntimeError("aiohttp is required for the dashboard gateway") from exc

    @web.middleware
    async def read_only(request: Any, handler: Any) -> Any:
        if request.method not in {"GET", "HEAD"}:
            return web.json_response({"error": "read-only gateway", "actuator_authority": "NONE"}, status=405)
        return await handler(request)

    app = web.Application(middlewares=[read_only])
    started_at = time.monotonic()
    readiness_file = readiness_path or (Path(os.environ.get("AUTOKART_ROOT", ".")) /
                                        "results" / "readiness_report_latest.json")

    async def readiness(_: Any) -> Any:
        """Read-only passthrough of tools/readiness_report.py's last saved gate table. Never
        computes or re-derives a gate itself; reports UNAVAILABLE rather than a fabricated table
        if the file is missing, stale JSON, or has never been generated."""
        try:
            text = readiness_file.read_text(encoding="utf-8")
            mtime = readiness_file.stat().st_mtime
        except OSError:
            return web.json_response({"available": False, "reason": "READINESS_REPORT_NOT_GENERATED",
                                      "command": "python3 tools/readiness_report.py"})
        try:
            report = json.loads(text)
        except ValueError:
            return web.json_response({"available": False, "reason": "READINESS_REPORT_UNREADABLE"})
        return web.json_response({"available": True, "generated_unix_s": mtime,
                                  "age_s": round(time.time() - mtime, 1), **report})

    async def health(_: Any) -> Any:
        """Process-liveness probe only: confirms this gateway process is up and serving, not that
        any ROS topic is fresh (use /mission/api/status for that). Never actuates anything."""
        return web.json_response({
            "status": "ok", "uptime_s": round(time.monotonic() - started_at, 1),
            "mode": "REPLAY" if replay else "LIVE", "source": store.source,
            "ros_connected": store.live_status().get("ros_connected", False) if not replay else None,
            "vehicle_authority": "OFF",
        })

    async def manifest(_: Any) -> Any:
        return web.json_response({**store.manifest(), "mode": "REPLAY" if replay else "LIVE"})

    async def replay_data(_: Any) -> Any:
        return web.json_response({"manifest": store.manifest(), "frames": store.all_frames()})

    async def status(_: Any) -> Any:
        snapshot = store.live_status()
        vcu_payload = snapshot["vcu"]["payload"] or {}
        estop_status = (vcu_payload.get("estop_input_status") or
                        ("NOT FITTED - MANUAL POWER CUT ONLY" if store.source == "REAL_KART" else "NOT APPLICABLE"))
        return web.json_response({**snapshot, "mode": "REPLAY" if replay else "LIVE",
                                  "safety_requests_enabled": bool(not replay and request_sink is not None),
                                  "estop_input_status": estop_status})

    async def camera_status(_: Any) -> Any:
        return web.json_response(camera.snapshot() if camera is not None else
                                 {"available": False, "reason": "CAMERA_NOT_CONNECTED"})

    async def camera_image(request: Any) -> Any:
        if camera is None:
            raise web.HTTPNotFound()
        payload = camera.image(request.match_info["frame_id"], request.match_info["kind"])
        if payload is None:
            raise web.HTTPNotFound()
        return web.Response(body=payload, content_type="image/jpeg",
                            headers={"Cache-Control": "no-store"})

    async def receive_requests(socket: Any) -> None:
        """Read the browser's one permitted outbound message shape and forward it through request_sink.
        Never replayed (no live safety node to forward to) and never anything but a plain whitelisted string.
        """
        if request_sink is None and demo_drive_sink is None:
            return
        async for msg in socket:
            if msg.type != web.WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except (ValueError, TypeError):
                continue
            if not isinstance(data, dict):
                continue
            if data.get("type") == "request" and request_sink is not None:
                action = data.get("action")
                if isinstance(action, str):
                    if not request_sink(action):
                        await socket.send_json({"type": "request_rejected", "action": action})
            elif data.get("type") == "demo_drive" and demo_drive_sink is not None:
                action = data.get("action")
                if action == "ping":
                    if demo_drive_ping is not None:
                        demo_drive_ping()
                elif isinstance(action, str):
                    if not demo_drive_sink(action, direction=data.get("direction", ""),
                                           duration_s=data.get("duration_s", 0.0)):
                        await socket.send_json({"type": "demo_drive_rejected", "action": action})

    async def websocket(_: Any) -> Any:
        socket = web.WebSocketResponse(heartbeat=None if replay else 10, max_msg_size=256 * 1024)
        await socket.prepare(_)
        await socket.send_json({"type": "manifest", "manifest": {**store.manifest(), "mode": "REPLAY" if replay else "LIVE"}})
        receiver = (None if (replay or (request_sink is None and demo_drive_sink is None)) else
                   asyncio.ensure_future(receive_requests(socket)))
        if replay:
            for frame in store.all_frames():
                if socket.closed:
                    break
                await socket.send_json({"type": "frame", "frame": frame})
                await asyncio.sleep(0.05)
        else:
            last_sequence = -1
            while not socket.closed:
                pending = store.since(last_sequence)
                if pending:
                    # Latest frame wins under browser backpressure. The store
                    # itself remains bounded and ROS callbacks never wait here.
                    frame = pending[-1]
                    await socket.send_json({"type": "frame", "frame": frame})
                    last_sequence = frame["sequence"]
                await asyncio.sleep(0.05)
        if receiver is not None:
            receiver.cancel()
        await socket.close()
        return socket

    app.router.add_get("/health", health)
    app.router.add_get("/mission/api/readiness", readiness)
    app.router.add_get("/mission/api/manifest", manifest)
    app.router.add_get("/mission/api/replay", replay_data)
    app.router.add_get("/mission/api/status", status)
    app.router.add_get("/mission/api/camera", camera_status)
    app.router.add_get("/mission/api/camera/{frame_id}/{kind}", camera_image)
    app.router.add_get("/mission/ws", websocket)
    if static_root is not None:
        static_root = static_root.resolve()
        if not (static_root / "index.html").is_file():
            raise ValueError("static root must contain the built frontend index.html")

        async def index(_: Any) -> Any:
            return web.FileResponse(static_root / "index.html")

        app.router.add_get("/", index)
        app.router.add_get("/demo", index)
        app.router.add_get("/ops", index)
        app.router.add_get("/live", index)
        app.router.add_get("/shared", index)
        app.router.add_static("/assets", static_root / "assets")
    return app


def load_replay(path: Path, store: MissionStore) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    payload = json.loads("\n".join(lines)) if path.suffix == ".json" else [json.loads(line) for line in lines if line.strip()]
    frames = payload if isinstance(payload, list) else payload.get("frames", [])
    previous_time = -1.0
    for frame in frames:
        if not valid_frame(frame):
            raise ValueError("replay contains an invalid Mission Control frame")
        if frame["timestamp_s"] < previous_time:
            raise ValueError("replay frames must have monotonic timestamps")
        previous_time = frame["timestamp_s"]
        store.frames.append(frame)
    if store.frames:
        store.source = store.frames[0]["source"]
        store.run_id = store.frames[0]["run_id"]
        store.scenario = store.frames[0].get("scenario", "UNKNOWN")
        store._sequence = store.frames[-1]["sequence"] + 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only AutoKart Mission Control gateway")
    parser.add_argument("--source", choices=("SIMULATION", "REAL_KART"), default="SIMULATION")
    parser.add_argument("--run-id", default="ros-live")
    parser.add_argument("--scenario", default="UNKNOWN", help="scenario name carried in each normalized WorldFrame")
    parser.add_argument("--fsds", action="store_true", help="subscribe directly to FSDS testing-only odometry and track for SIM display")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--static-root", type=Path)
    parser.add_argument("--display-only", action="store_true",
                        help="disable browser safety requests; display ROS data only")
    parser.add_argument("--replay", type=Path, help="serve normalized Mission Control .jsonl or .json without ROS")
    args, ros_args = parser.parse_known_args()
    if args.fsds and args.source != "SIMULATION":
        parser.error("--fsds requires --source SIMULATION")
    store = MissionStore(args.source, args.run_id, scenario=args.scenario)
    camera = None if args.replay else CameraFeed()
    if args.replay:
        load_replay(args.replay, store)
    else:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
        except ImportError as exc:
            raise SystemExit(f"NOT_EXECUTED_EXTERNAL_DEPENDENCY: {exc}") from exc
        rclpy.init(args=ros_args)
        node = make_ros_node(store, fsds=args.fsds, camera=camera)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = Thread(target=executor.spin, daemon=False)
        thread.start()
    from aiohttp import web
    request_sink = None if args.replay or args.display_only else node.send_request
    demo_drive_sink = None if args.replay or args.display_only else node.send_demo_drive_message
    demo_drive_ping = None if args.replay or args.display_only else node.send_demo_drive_ping
    try:
        web.run_app(create_app(store, args.static_root, replay=bool(args.replay), camera=camera,
                               request_sink=request_sink, demo_drive_sink=demo_drive_sink,
                               demo_drive_ping=demo_drive_ping), host=args.host, port=args.port)
    finally:
        if not args.replay:
            executor.shutdown()
            thread.join(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
