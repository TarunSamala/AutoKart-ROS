"""Gateway logic with no ROS imports: converts message-like objects into dashboard envelopes and serves WebSockets.

The dashboard only ever sees data that arrived from ROS. If an input is stale the envelope is marked
valid=false (or not sent at all), so the dashboard shows INVALID/STALE and never LIVE for old data.
"""
from __future__ import annotations

import asyncio
import base64
import json
import threading
import time
from collections import OrderedDict
from typing import Callable, Dict, Optional

ALLOWED_ACTIONS = {"arm", "stop", "reset_fault", "confirm_estop_tested"}
MIN_INTERVAL = {"camera": 0.09, "map": 0.09, "odom": 0.09, "steering": 0.09, "drive": 0.09, "brake": 0.09,
                "obstacle": 0.09, "safety": 0.19, "startup": 0.45}


def jpeg_size(data: bytes):
    """(width, height) from a JPEG's SOF marker, or (0, 0)."""
    i = 2
    n = len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            return (data[i + 7] << 8) | data[i + 8], (data[i + 5] << 8) | data[i + 6]
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        i += 2 + ((data[i + 2] << 8) | data[i + 3])
    return 0, 0


def stamp_of(msg) -> float:
    try:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except AttributeError:
        return time.time()


def nz(v):
    """JSON-safe float: NaN/inf -> None."""
    try:
        return None if v != v or v in (float("inf"), float("-inf")) else float(v)
    except (TypeError, ValueError):
        return None


class GatewayCore:
    def __init__(self, emit: Callable[[dict], None], mode: str = "LIVE", clock: Callable[[], float] = time.monotonic,
                 throttle_cap_v: float = 2.0):
        self.emit_fn = emit
        self.mode = mode
        self.clock = clock
        self.throttle_cap_v = throttle_cap_v  # from config (autokart.yaml throttle_v_max), not a baked-in literal
        self.last_emit: Dict[str, float] = {}
        self.det_by_id: "OrderedDict[int, dict]" = OrderedDict()
        self.latest: Dict[str, tuple] = {}          # name -> (msg, receive time)
        self.fps_times = []
        self.last_speed = None                      # (t, speed) for the acceleration estimate
        self.accel = 0.0

    # ---------- helpers ----------
    def env(self, topic, msg_or_stamp, data, source, frame_id=None, valid=True):
        stamp = msg_or_stamp if isinstance(msg_or_stamp, float) else stamp_of(msg_or_stamp)
        e = {"topic": topic, "stamp": stamp, "mode": self.mode, "valid": bool(valid), "source": source, "data": data}
        if frame_id is not None:
            e["frame_id"] = int(frame_id)
        return e

    def throttle(self, topic: str) -> bool:
        now = self.clock()
        if now - self.last_emit.get(topic, -1e9) < MIN_INTERVAL.get(topic, 0.0):
            return False
        self.last_emit[topic] = now
        return True

    def fresh(self, name: str, limit: float) -> Optional[object]:
        e = self.latest.get(name)
        return e[0] if e and self.clock() - e[1] <= limit else None

    def remember(self, name, msg):
        self.latest[name] = (msg, self.clock())

    # ---------- ROS callbacks (duck-typed) ----------
    def on_detections(self, m):
        items = []
        for c in m.cones:
            items.append({"cls": c.cls, "conf": nz(c.conf), "bbox": [c.x1, c.y1, c.x2, c.y2], "dist_m": nz(c.dist_m),
                          "x": nz(c.x), "y": nz(c.y), "depth_ok": bool(c.depth_ok)})
        self.det_by_id[int(m.frame_id)] = self.env(
            "detections", m, {"inference_ms": nz(m.inference_ms), "items": items, "depth_valid": bool(m.depth_valid),
                              "calibration_verified": bool(m.calibration_verified)},
            "/autokart/detections", m.frame_id, valid=m.depth_valid)
        while len(self.det_by_id) > 40:
            self.det_by_id.popitem(last=False)
        self.remember("detections", m)

    def on_image(self, m):
        """Image and its detections leave together, so the browser always has matching frames."""
        try:
            fid = int(m.header.frame_id)
        except ValueError:
            return
        now = self.clock()
        self.fps_times = [t for t in self.fps_times if now - t < 2.0] + [now]
        if not self.throttle("camera"):
            return
        w, h = jpeg_size(bytes(m.data))
        fps = (len(self.fps_times) - 1) / (self.fps_times[-1] - self.fps_times[0]) if len(self.fps_times) > 1 else 0.0
        det = self.det_by_id.get(fid)
        self.emit_fn(self.env("camera", m, {
            "jpeg_b64": base64.b64encode(bytes(m.data)).decode("ascii"), "width": w, "height": h,
            "fps": round(fps, 1), "depth_valid": bool(det and det["data"]["depth_valid"]) if det else None},
            "/autokart/camera/compressed", fid))
        if det:
            self.emit_fn(det)

    def on_map(self, m):
        self.remember("map", m)
        if not self.throttle("map"):
            return
        plan = self.fresh("plan", 0.5)
        ctl = self.fresh("ctrl", 0.5)
        pp = {}
        data = {"cones": [{"cls": c.cls, "x": nz(c.x), "y": nz(c.y)} for c in m.cones],
                "path": [[p.x, p.y] for p in plan.path] if plan else [], "status": m.status,
                "corridor_width_m": nz(m.corridor_width_m)}
        if plan and plan.stop_valid:
            data["stop_target"] = {"x": plan.stop_target.x, "y": plan.stop_target.y}
        if ctl:
            if ctl.lookahead_valid:
                data["lookahead"] = {"x": ctl.lookahead_x, "y": ctl.lookahead_y}
            pp = {"wheelbase_m": nz(ctl.wheelbase_m), "lookahead_m": nz(ctl.lookahead_m), "alpha_rad": nz(ctl.alpha_rad),
                  "delta_deg": nz(ctl.steer_deg_cmd)}
        data["pure_pursuit"] = pp
        self.emit_fn(self.env("map", m, data, "/autokart/map+planned_path+controller_state", m.frame_id,
                              valid=bool(m.valid and plan is not None and plan.path_valid)))

    def on_plan(self, m):
        self.remember("plan", m)
        if not self.throttle("obstacle"):
            return
        self.emit_fn(self.env("obstacle", m, {
            "nearest_m": nz(m.nearest_cone_m), "required_stop_m": nz(m.required_stop_m), "available_m": nz(m.available_m),
            "decision": m.decision, "target_speed_mps": nz(m.target_speed_mps)}, "/autokart/planned_path", m.frame_id))

    def on_ctrl(self, m):
        self.remember("ctrl", m)

    def on_odom(self, m):
        if not self.throttle("odom"):
            return
        self.emit_fn(self.env("odom", m, {
            "x": nz(m.x), "y": nz(m.y), "yaw_rad": nz(m.yaw_rad), "speed_mps": nz(m.speed_mps),
            "yaw_rate_dps": nz(m.yaw_rate_dps), "axle_counts": int(m.axle_counts), "status": m.status},
            "/autokart/odom"))

    def on_telemetry(self, m):
        now = self.clock()
        if self.last_speed is not None and now - self.last_speed[0] >= 0.2:
            self.accel = (m.speed_mps - self.last_speed[1]) / (now - self.last_speed[0])
            self.last_speed = (now, m.speed_mps)
        elif self.last_speed is None:
            self.last_speed = (now, m.speed_mps)
        ctl = self.fresh("ctrl", 0.5)
        if self.throttle("steering"):
            self.emit_fn(self.env("steering", m, {
                "target_deg": nz(ctl.steer_deg_cmd) if ctl else None,
                "actual_deg": nz(m.steer_deg) if m.steer_valid else None,
                "state": "OBSERVE ONLY (steering not actuated)",
                "fault": None if m.steer_pwm_ok else "AS5600 steering PWM invalid"}, "/autokart/vehicle_telemetry"))
        if self.throttle("drive"):
            self.emit_fn(self.env("drive", m, {
                "target_kmh": nz(ctl.target_speed_mps * 3.6) if ctl else None, "actual_kmh": nz(m.speed_mps * 3.6),
                "accel_mps2": nz(self.accel), "cmd": f"{m.dac_v:.2f} V (cap {self.throttle_cap_v:.2f} V)",
                "state": f"{m.state_name}, EN={'on' if m.en else 'off'}"}, "/autokart/vehicle_telemetry"))
        if self.throttle("brake"):
            self.emit_fn(self.env("brake", m, {
                "target": nz(ctl.brake_pct) if ctl else None, "actual": 100.0 if m.brk else 0.0, "unit": "%",
                "state": "BLD-750 BRK line " + ("asserted" if m.brk else "released"),
                "wheel_speed_zero": bool(abs(m.speed_mps) < 0.03)}, "/autokart/vehicle_telemetry"))

    def on_safety(self, m):
        if self.throttle("safety"):
            self.emit_fn(self.env("safety", m, {
                "state": m.state, "vcu_heartbeat_ok": bool(m.vcu_heartbeat_ok), "arm_allowed": bool(m.arm_allowed),
                "stop_latched": bool(m.stop_latched), "reasons": list(m.reasons)}, "/autokart/safety/state"))
        if self.throttle("startup"):
            self.emit_fn(self.env("startup", m, {
                "ready": bool(m.startup_ready),
                "stages": [{"id": s.id, "name": s.name, "state": s.state, "detail": s.detail} for s in m.stages]},
                "/autokart/safety/state"))

    def on_rosout(self, m):
        if not str(m.name).startswith("autokart"):
            return
        level = "fault" if m.level >= 40 else "warn" if m.level >= 30 else "info"
        self.emit_fn({"topic": "log", "stamp": time.time(), "mode": self.mode, "valid": True, "source": m.name,
                      "data": {"level": level, "text": m.msg}})

    def log(self, text, level="info", source="gateway"):
        self.emit_fn({"topic": "log", "stamp": time.time(), "mode": self.mode, "valid": True, "source": source,
                      "data": {"level": level, "text": text}})


class GatewayServer:
    """WebSocket server running in its own thread. emit() is safe to call from any thread."""

    def __init__(self, host: str, port: int, on_request: Callable[[str, dict], None]):
        self.host, self.port, self.on_request = host, port, on_request
        self.loop = asyncio.new_event_loop()
        self.clients = set()
        self.ready = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        self.ready.wait(5.0)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._serve())

    async def _serve(self):
        from websockets.asyncio.server import serve
        async with serve(self._handler, self.host, self.port, max_size=2 ** 20):
            self.ready.set()
            await asyncio.Future()

    async def _handler(self, ws):
        self.clients.add(ws)
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                if msg.get("type") == "request" and msg.get("action") in ALLOWED_ACTIONS:
                    self.on_request(msg["action"], msg)
        finally:
            self.clients.discard(ws)

    def _send(self, text: str):
        import websockets
        websockets.broadcast(self.clients, text)

    def emit(self, env: dict):
        text = json.dumps(env, separators=(",", ":"), allow_nan=False)
        self.loop.call_soon_threadsafe(self._send, text)
