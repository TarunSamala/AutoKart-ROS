"""Read-only HTTP/WebSocket service for the engineering dashboard.

The service accepts no actuator, command, reset, or VCU mutation endpoint.
Telemetry is copied from an immutable replay or tailed JSONL source into a
bounded per-client queue so browser backpressure cannot stall physics.
"""

from __future__ import annotations

import asyncio
import json
from dataclasses import dataclass
from pathlib import Path
from time import monotonic
from typing import Any

from .config import load_kart_config
from .dashboard import (
    ReplayDataset,
    dashboard_kart_payload,
    dashboard_track_payload,
    normalize_dashboard_record,
)
from .track import Track
from .types import SimulationError

try:
    from aiohttp import WSMsgType, web
except ImportError:  # pragma: no cover - exercised only on minimal installs.
    WSMsgType = None
    web = None


@dataclass(frozen=True)
class DashboardServerConfig:
    telemetry: Path
    track: Path
    kart: Path
    static_root: Path
    host: str = "127.0.0.1"
    port: int = 8000
    telemetry_rate_hz: float = 30.0
    maximum_replay_points: int = 20_000
    queue_size: int = 64
    live_tail: bool = False

    def validate(self) -> None:
        if not self.telemetry.is_file():
            raise SimulationError(f"dashboard telemetry file not found: {self.telemetry}")
        if not self.track.is_file():
            raise SimulationError(f"dashboard track file not found: {self.track}")
        if not self.kart.is_file():
            raise SimulationError(f"dashboard kart file not found: {self.kart}")
        if not self.static_root.is_dir():
            raise SimulationError(f"dashboard frontend directory not found: {self.static_root}")
        if not 1 <= self.port <= 65535:
            raise SimulationError("dashboard port must be in [1, 65535]")
        if not 1.0 <= self.telemetry_rate_hz <= 120.0:
            raise SimulationError("dashboard telemetry rate must be in [1, 120] Hz")
        if self.maximum_replay_points <= 1 or self.queue_size <= 0:
            raise SimulationError("dashboard buffer limits must be positive")


class DashboardService:
    """Normalized run/track/kart data used by both HTTP and WebSocket routes."""

    def __init__(self, config: DashboardServerConfig) -> None:
        config.validate()
        self.config = config
        self.track_model = Track.from_yaml(config.track)
        self.kart_config = load_kart_config(config.kart)
        self.track = dashboard_track_payload(self.track_model)
        self.kart = dashboard_kart_payload(self.kart_config)
        self.dataset = ReplayDataset.from_jsonl(config.telemetry, self.track_model)
        if len(self.dataset.records) > config.maximum_replay_points:
            self.dataset.records = self.dataset.records[-config.maximum_replay_points :]
        self._live_offset = config.telemetry.stat().st_size
        self._live_fragment = ""
        self._next_sequence = (
            self.dataset.records[-1]["sequence"] + 1 if self.dataset.records else 0
        )
        self.metadata = self._metadata()
        self.started_monotonic = monotonic()
        self.websocket_connections = 0
        self.sent_messages = 0
        self.dropped_messages = 0

    def _metadata(self) -> dict[str, Any]:
        metadata_path = self.config.telemetry.parent / "metadata.json"
        metadata = (
            json.loads(metadata_path.read_text(encoding="utf-8"))
            if metadata_path.is_file()
            else {}
        )
        return {
            "run_id": self.config.telemetry.parent.name,
            "scenario": metadata.get("scenario", self.config.telemetry.parent.name),
            "track": self.track_model.name,
            "kart": self.kart_config.kart.name,
            "physics_dt_s": metadata.get("physics_dt_s"),
            "seed": metadata.get("seed"),
            "control_mode": metadata.get("control_mode", "unknown"),
            "samples": len(self.dataset.records),
            "mode": "live" if self.config.live_tail else "replay",
            "telemetry_rate_hz": self.config.telemetry_rate_hz,
            "read_only": True,
            "actuator_authority": "NONE",
        }

    def refresh(self) -> None:
        if not self.config.live_tail:
            return
        try:
            size = self.config.telemetry.stat().st_size
            if size < self._live_offset:
                self.dataset.records.clear()
                self._live_offset = 0
                self._live_fragment = ""
                self._next_sequence = 0
            with self.config.telemetry.open("r", encoding="utf-8") as stream:
                stream.seek(self._live_offset)
                chunk = stream.read()
                self._live_offset = stream.tell()
            if not chunk:
                return
            text = self._live_fragment + chunk
            lines = text.splitlines(keepends=True)
            self._live_fragment = ""
            if lines and not lines[-1].endswith(("\n", "\r")):
                self._live_fragment = lines.pop()
            for line in lines:
                if not line.strip():
                    continue
                row = json.loads(line)
                self.dataset.records.append(
                    normalize_dashboard_record(
                        row,
                        track=self.track_model,
                        sequence=self._next_sequence,
                    )
                )
                self._next_sequence += 1
            overflow = len(self.dataset.records) - self.config.maximum_replay_points
            if overflow > 0:
                del self.dataset.records[:overflow]
            self.metadata["samples"] = len(self.dataset.records)
        except (json.JSONDecodeError, OSError):
            # A live writer may be between bytes of its next complete JSONL
            # record. Retain the last valid immutable dataset and retry.
            return

    def manifest(self) -> dict[str, Any]:
        first = self.dataset.records[0]["timestamp_s"] if self.dataset.records else None
        last = self.dataset.records[-1]["timestamp_s"] if self.dataset.records else None
        return {
            **self.metadata,
            "start_time_s": first,
            "end_time_s": last,
            "endpoints": {
                "track": "/api/track",
                "kart": "/api/kart",
                "telemetry": "/api/telemetry",
                "comparison": "/api/comparison",
                "performance": "/api/performance",
                "websocket": "/ws",
            },
        }

    def telemetry(self, maximum_points: int | None = None) -> list[dict[str, Any]]:
        self.refresh()
        return self.dataset.decimate(
            min(maximum_points or self.config.maximum_replay_points, self.config.maximum_replay_points)
        )

    def performance(self) -> dict[str, Any]:
        elapsed = max(1e-9, monotonic() - self.started_monotonic)
        run_directory = self.config.telemetry.parent
        timing_path = run_directory / "timing_report.json"
        native_path = run_directory / "native_results.json"
        summary_path = run_directory / "summary.json"
        timing = (
            json.loads(timing_path.read_text(encoding="utf-8"))
            if timing_path.is_file() else {}
        )
        native = (
            json.loads(native_path.read_text(encoding="utf-8"))
            if native_path.is_file() else {}
        )
        summary = (
            json.loads(summary_path.read_text(encoding="utf-8"))
            if summary_path.is_file() else {}
        )
        tasks = timing.get("tasks", {})
        rtf = float(timing.get(
            "aggregate_rtf", summary.get("real_time_factor", 0.0)
        ))
        safety_misses = sum(
            int(tasks.get(task, {}).get("deadline_misses", 0))
            for task in ("vcu", "independent_safety")
        )
        if rtf >= 1.0 and safety_misses == 0:
            qualification = "REALTIME_QUALIFIED"
        elif rtf >= 0.5:
            qualification = "REALTIME_DEGRADED"
        else:
            qualification = "OFFLINE_ONLY"
        sensor_results = native.get("sensors", {})
        return {
            "uptime_s": elapsed,
            "websocket_connections": self.websocket_connections,
            "messages_sent": self.sent_messages,
            "message_rate_hz": self.sent_messages / elapsed,
            "dropped_messages": self.dropped_messages,
            "queue_bound": self.config.queue_size,
            "replay_records": len(self.dataset.records),
            "qualification": qualification,
            "simulation_rtf": rtf,
            "minimum_window_rtf":
                float(timing.get("minimum_window_rtf", 0.0)),
            "task_timing": tasks,
            "deadline_misses": {
                task: int(values.get("deadline_misses", 0))
                for task, values in tasks.items()
            },
            "gazebo_follower_latency_s":
                float(native.get("maximum_acknowledgement_lag_s", 0.0)),
            "sensor_latency_s": {
                sensor: float(values.get("maximum_latency_s", 0.0))
                for sensor, values in sensor_results.items()
            },
            "sensor_queue_depth": {
                sensor: int(values.get("maximum_queue_depth", 0))
                for sensor, values in sensor_results.items()
            },
            "sensor_drops_by_reason": {
                sensor: values.get("drops_by_reason", {})
                for sensor, values in sensor_results.items()
            },
            "cpu_time_s": float(summary.get("cpu_time_s", 0.0)),
            "memory_mib": float(summary.get("memory_mib", 0.0)),
            "gpu": "NOT_EXECUTED_EXTERNAL_DEPENDENCY"
                if not (run_directory / "gpu_after.txt").is_file()
                else (run_directory / "gpu_after.txt").read_text(
                    encoding="utf-8"
                ).strip(),
            "ros_callback_latency_s": None,
        }


def create_dashboard_app(config: DashboardServerConfig):
    if web is None:
        raise SimulationError("dashboard live server requires optional dependency aiohttp")
    service = DashboardService(config)

    @web.middleware
    async def read_only(request, handler):
        if request.method not in {"GET", "HEAD"}:
            return web.json_response(
                {
                    "error": "ENGINEERING VISUALIZATION ONLY",
                    "actuator_authority": "NONE",
                    "detail": "mutation endpoints are intentionally unavailable",
                },
                status=405,
            )
        return await handler(request)

    app = web.Application(middlewares=[read_only])
    app["dashboard_service"] = service

    async def index(_: Any) -> Any:
        return web.FileResponse(config.static_root / "index.html")

    async def manifest(_: Any) -> Any:
        return web.json_response(service.manifest())

    async def track(_: Any) -> Any:
        return web.json_response(service.track)

    async def kart(_: Any) -> Any:
        return web.json_response(service.kart)

    async def telemetry(request: Any) -> Any:
        try:
            maximum = int(request.query.get("max_points", config.maximum_replay_points))
        except ValueError:
            return web.json_response({"error": "max_points must be an integer"}, status=400)
        if maximum <= 0:
            return web.json_response({"error": "max_points must be positive"}, status=400)
        return web.json_response({"records": service.telemetry(maximum)})

    async def comparison(request: Any) -> Any:
        # The default comparison is the current run against an optional second
        # immutable JSONL path. Paths are restricted to the workspace/runs tree
        # by resolving beneath the current telemetry parent.
        other = request.query.get("other")
        if not other:
            return web.json_response({"records": [], "reason": "no comparison run selected"})
        candidate = (config.telemetry.parent.parent / other / "telemetry.jsonl").resolve()
        allowed = config.telemetry.parent.parent.resolve()
        if allowed not in candidate.parents or not candidate.is_file():
            return web.json_response({"error": "comparison run is not available"}, status=404)
        right = ReplayDataset.from_jsonl(candidate, service.track_model)
        left_records, right_records = service.dataset.records, right.records
        result = []
        for index, record in enumerate(left_records):
            s_m = record["track"]["s_m"]
            nearest = min(right_records, key=lambda item: abs(item["track"]["s_m"] - s_m))
            result.append(
                {
                    "s_m": s_m,
                    "left_speed_mps": record["motion"]["speed_mps"],
                    "right_speed_mps": nearest["motion"]["speed_mps"],
                    "speed_delta_mps": record["motion"]["speed_mps"] - nearest["motion"]["speed_mps"],
                    "left_x_m": record["pose"]["x_m"],
                    "left_y_m": record["pose"]["y_m"],
                    "right_x_m": nearest["pose"]["x_m"],
                    "right_y_m": nearest["pose"]["y_m"],
                }
            )
        return web.json_response({"records": result})

    async def performance(_: Any) -> Any:
        return web.json_response(service.performance())

    async def websocket(request: Any) -> Any:
        socket = web.WebSocketResponse(heartbeat=10.0, max_msg_size=256 * 1024)
        await socket.prepare(request)
        service.websocket_connections += 1
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=config.queue_size)

        async def producer() -> None:
            if not config.live_tail:
                records = service.dataset.records
                if not records:
                    await queue.put({"type": "end_of_replay"})
                    return
                next_measurement = records[0]["timestamp_s"]
                period = 1.0 / config.telemetry_rate_hz
                selected: list[dict[str, Any]] = []
                for record in records:
                    if record["timestamp_s"] + 1e-12 >= next_measurement:
                        selected.append(record)
                        next_measurement = record["timestamp_s"] + period
                if selected[-1]["sequence"] != records[-1]["sequence"]:
                    selected.append(records[-1])
                for record in selected:
                    if socket.closed:
                        return
                    await queue.put({"type": "telemetry", "record": record})
                    await asyncio.sleep(period)
                await queue.put({"type": "end_of_replay"})
                return
            last_sequence = -1
            period = 1.0 / config.telemetry_rate_hz
            while not socket.closed:
                service.refresh()
                records = service.dataset.records
                pending = [record for record in records if record["sequence"] > last_sequence]
                if pending:
                    # Downsample by authoritative measurement time; always keep
                    # the newest record and drop oldest browser-bound data first.
                    next_time = pending[0]["timestamp_s"]
                    selected = []
                    for record in pending:
                        if record["timestamp_s"] + 1e-12 >= next_time:
                            selected.append(record)
                            next_time = record["timestamp_s"] + period
                    if selected[-1]["sequence"] != pending[-1]["sequence"]:
                        selected.append(pending[-1])
                    for record in selected:
                        if queue.full():
                            queue.get_nowait()
                            service.dropped_messages += 1
                        queue.put_nowait({"type": "telemetry", "record": record})
                    last_sequence = pending[-1]["sequence"]
                await asyncio.sleep(period)

        producer_task = asyncio.create_task(producer())
        try:
            await socket.send_json({"type": "manifest", "manifest": service.manifest()})
            while not socket.closed:
                item = await queue.get()
                await socket.send_json(item)
                service.sent_messages += 1
                if item["type"] == "end_of_replay":
                    break
        finally:
            producer_task.cancel()
            service.websocket_connections -= 1
            await socket.close()
        return socket

    app.router.add_get("/", index)
    app.router.add_get("/dashboard.html", index)
    app.router.add_get("/api/manifest", manifest)
    app.router.add_get("/api/track", track)
    app.router.add_get("/api/kart", kart)
    app.router.add_get("/api/telemetry", telemetry)
    app.router.add_get("/api/comparison", comparison)
    app.router.add_get("/api/performance", performance)
    app.router.add_get("/ws", websocket)
    app.router.add_static("/css", config.static_root / "css")
    app.router.add_static("/js", config.static_root / "js")
    app.router.add_static("/assets", config.static_root / "assets")
    return app


def serve_dashboard(config: DashboardServerConfig) -> None:
    if web is None:
        raise SimulationError("dashboard live server requires optional dependency aiohttp")
    app = create_dashboard_app(config)
    web.run_app(app, host=config.host, port=config.port, print=lambda line: print(line, flush=True))
