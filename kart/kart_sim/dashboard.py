"""Headless engineering-dashboard data service and static report exporter.

The module deliberately consumes telemetry as a read-only source. It has no
actuator command API, which prevents a visualization client from becoming a
safety authority.
"""

from __future__ import annotations

import csv
import html
import json
import shutil
from dataclasses import asdict
from dataclasses import dataclass
from math import atan2, hypot, isfinite
from pathlib import Path
from typing import Any, Iterable

from .config import FullKartConfig
from .track import Track, segment_corners
from .types import SimulationError


@dataclass(frozen=True)
class TelemetryRecord:
    timestamp_s: float
    source: str
    values: dict[str, Any]
    valid: bool = True
    stale: bool = False
    frame_id: str = "vehicle_body"
    configuration_hash: str = ""


def _timestamp(row: dict[str, Any]) -> float:
    state = row.get("state", {})
    timestamp = state.get(
        "time_s",
        row.get("timestamp_s", row.get("simulation_time_s")),
    )
    if timestamp is None or not isfinite(float(timestamp)):
        raise SimulationError("dashboard telemetry record lacks a finite simulation timestamp")
    return float(timestamp)


class TelemetryStore:
    """Bounded normalized replay/live cache with explicit data-gap semantics."""

    def __init__(self, maximum_records: int = 100_000) -> None:
        if maximum_records <= 0:
            raise SimulationError("dashboard cache bound must be positive")
        self.maximum_records = maximum_records
        self.records: list[TelemetryRecord] = []
        self.connection_alive = True

    def append(self, record: TelemetryRecord) -> None:
        if not isfinite(record.timestamp_s):
            raise SimulationError("dashboard record timestamp is invalid")
        self.records.append(record)
        overflow = len(self.records) - self.maximum_records
        if overflow > 0:
            del self.records[:overflow]

    def connection_lost(self) -> None:
        self.connection_alive = False

    @classmethod
    def from_jsonl(cls, path: str | Path, maximum_records: int = 100_000) -> "TelemetryStore":
        store = cls(maximum_records)
        previous_time = -float("inf")
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            timestamp = _timestamp(row)
            # A backward sample is invalid rather than silently interpolated.
            stale = timestamp < previous_time
            store.append(TelemetryRecord(timestamp, "simulator", row, valid=not stale, stale=stale))
            previous_time = max(previous_time, timestamp)
        return store

    def decimate(self, maximum_points: int) -> list[TelemetryRecord]:
        if maximum_points <= 0:
            raise SimulationError("maximum visible point count must be positive")
        if len(self.records) <= maximum_points:
            return list(self.records)
        # Preserve first/last and take deterministic uniformly spaced samples.
        indices = {round(index * (len(self.records) - 1) / (maximum_points - 1)) for index in range(maximum_points)}
        return [self.records[index] for index in sorted(indices)]

    def fault_events(self, severity: str | None = None) -> list[TelemetryRecord]:
        events = []
        for record in self.records:
            faults = record.values.get("vcu", {}).get("faults", ())
            if faults and (severity is None or severity in " ".join(faults)):
                events.append(record)
        return events

    def at(self, timestamp_s: float) -> TelemetryRecord | None:
        past = [record for record in self.records if record.timestamp_s <= timestamp_s]
        return past[-1] if past else None


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if isfinite(number) else default


def normalize_dashboard_record(
    row: dict[str, Any],
    *,
    track: Track | None = None,
    sequence: int = 0,
) -> dict[str, Any]:
    """Convert simulator JSONL into the stable browser-facing data contract.

    The original engineering payload is retained under subsystem groups. Missing
    signals stay ``None``; the normalizer never synthesizes motion.
    """
    timestamp = _timestamp(row)
    state = row.get("state", {})
    if not state and "simulation_time_s" in row:
        # Native Gazebo-follower evidence is an immutable authoritative
        # snapshot stream. Map its measured state into the existing read-only
        # dashboard contract without synthesizing commands or dynamics.
        state = {
            "time_s": row.get("simulation_time_s"),
            "x_m": row.get("x_m"),
            "y_m": row.get("y_m"),
            "z_m": row.get("z_m"),
            "roll_rad": row.get("roll_rad"),
            "pitch_rad": row.get("pitch_rad"),
            "yaw_rad": row.get("yaw_rad"),
            "u_mps": row.get("speed_mps"),
            "v_mps": 0.0,
        }
    x_m, y_m = _finite(state.get("x_m")), _finite(state.get("y_m"))
    u_mps, v_mps = _finite(state.get("u_mps")), _finite(state.get("v_mps"))
    projection = row.get("track", {}).get("projection")
    geometry = row.get("track", {}).get("geometry")
    if track is not None and not projection:
        projected = track.project(x_m, y_m, _finite(state.get("z_m")))
        projection, geometry = asdict(projected), asdict(track.sample(projected.s_m))
    yaw = _finite(state.get("yaw_rad"))
    heading_error = None
    if geometry:
        heading_error = (yaw - _finite(geometry.get("heading_rad")) + 3.141592653589793) % (
            2.0 * 3.141592653589793
        ) - 3.141592653589793
    vcu = row.get("vcu", {})
    if not vcu and "vcu_state" in row:
        vcu = {
            "state": row.get("vcu_state"),
            "faults": tuple(row.get("faults", ())),
        }
    subsystems = row.get("subsystems", {})
    commands = row.get("commands", {})
    faults = tuple(vcu.get("faults", ())) or tuple(subsystems.get("faults", ()))
    final_command = commands.get("final", vcu.get("final_command", {}))
    return {
        "schema_version": 1,
        "sequence": sequence,
        "timestamp_s": timestamp,
        "valid": all(isfinite(value) for value in (timestamp, x_m, y_m, u_mps, v_mps, yaw)),
        "stale": False,
        "pose": {
            "x_m": x_m,
            "y_m": y_m,
            "z_m": _finite(state.get("z_m")),
            "roll_rad": _finite(state.get("roll_rad")),
            "pitch_rad": _finite(state.get("pitch_rad")),
            "yaw_rad": yaw,
        },
        "motion": {
            "speed_mps": hypot(u_mps, v_mps),
            "longitudinal_speed_mps": u_mps,
            "lateral_speed_mps": v_mps,
            "longitudinal_acceleration_mps2": _finite(state.get("ax_mps2")),
            "lateral_acceleration_mps2": _finite(state.get("ay_mps2")),
            "yaw_rate_radps": _finite(state.get("yaw_rate_radps")),
            "sideslip_rad": atan2(v_mps, max(abs(u_mps), 1e-9)),
        },
        "track": {
            "s_m": _finite((projection or {}).get("s_m"), _finite(row.get("track_s_m"))),
            "lateral_m": _finite((projection or {}).get("lateral_m"), _finite(row.get("track_lateral_m"))),
            "heading_error_rad": heading_error,
            "curvature_1pm": _finite((geometry or {}).get("curvature_1pm")),
            "grade": _finite((geometry or {}).get("grade")),
            "bank_rad": _finite((geometry or {}).get("bank_rad")),
            "left_margin_m": (projection or {}).get("left_margin_m"),
            "right_margin_m": (projection or {}).get("right_margin_m"),
            "wheel_surfaces": row.get("track", {}).get("wheel_surfaces", {}),
        },
        "commands": {
            "requested": commands.get("requested"),
            "validated": commands.get("validated"),
            "safety_limited": commands.get("safety_limited"),
            "final": final_command,
        },
        "vcu": {
            **vcu,
            "faults": faults,
        },
        "subsystems": subsystems,
        "forces": row.get("forces", {}),
        "race_control": row.get("race_control", {}),
        "event": row.get("event"),
    }


def dashboard_track_payload(track: Track, spacing_m: float = 0.5) -> dict[str, Any]:
    if spacing_m <= 0:
        raise SimulationError("dashboard track spacing must be positive")
    count = max(2, int(track.length_m / spacing_m) + 1)
    points = []
    for index in range(count):
        s_m = min(track.length_m, index * track.length_m / (count - 1))
        point = track.sample(s_m)
        left = track.world_from_track(s_m, point.left_width_m)
        right = track.world_from_track(s_m, -point.right_width_m)
        surface = track.surface_at(s_m, 0.0)
        points.append(
            {
                "s_m": point.s_m,
                "x_m": point.x_m,
                "y_m": point.y_m,
                "z_m": point.z_m,
                "heading_rad": point.heading_rad,
                "curvature_1pm": point.curvature_1pm,
                "grade": point.grade,
                "bank_rad": point.bank_rad,
                "left_width_m": point.left_width_m,
                "right_width_m": point.right_width_m,
                "surface_id": surface.material.id,
                "left": {"x_m": left[0], "y_m": left[1], "z_m": left[2]},
                "right": {"x_m": right[0], "y_m": right[1], "z_m": right[2]},
            }
        )
    return {
        "schema_version": 1,
        "name": track.name,
        "closed": track.closed,
        "length_m": track.length_m,
        "coordinate_system": "local_enu",
        "points": points,
        "surface_regions": track.regions,
        "materials": {name: asdict(material) for name, material in track.materials.items()},
        "corners": segment_corners(track),
        "start_finish_s_m": 0.0,
        "placeholder": False,
    }


def dashboard_kart_payload(config: FullKartConfig) -> dict[str, Any]:
    kart, tire = config.kart, config.tires
    cg = kart.cg.model_dump() if hasattr(kart.cg, "model_dump") else kart.cg.dict()
    return {
        "schema_version": 1,
        "name": kart.name,
        "dimensions": {
            "wheelbase_m": kart.wheelbase_m,
            "front_track_m": kart.front_track_m,
            "rear_track_m": kart.rear_track_m,
            "wheel_radius_m": tire.radius_m,
            "wheel_width_m": tire.width_m,
            "chassis_length_m": kart.wheelbase_m + 0.42,
            "chassis_width_m": max(kart.front_track_m, kart.rear_track_m) + 0.12,
        },
        "cg": cg,
        "mass_kg": kart.mass_kg,
        "sensor_mounts": [],
        "model": "procedural_engineering_placeholder",
        "placeholder": True,
        "replacement_asset": "dashboard/assets/kart/kart.glb",
    }


class ReplayDataset:
    """Immutable normalized replay with deterministic point reduction."""

    def __init__(self, store: TelemetryStore, track: Track | None = None) -> None:
        self.records = [
            normalize_dashboard_record(record.values, track=track, sequence=index)
            for index, record in enumerate(store.records)
        ]

    @classmethod
    def from_jsonl(cls, path: str | Path, track: Track | None = None) -> "ReplayDataset":
        return cls(TelemetryStore.from_jsonl(path), track)

    def decimate(self, maximum_points: int) -> list[dict[str, Any]]:
        if maximum_points <= 0:
            raise SimulationError("dashboard replay point bound must be positive")
        if len(self.records) <= maximum_points:
            return list(self.records)
        if maximum_points == 1:
            return [self.records[-1]]
        indices = {
            round(index * (len(self.records) - 1) / (maximum_points - 1))
            for index in range(maximum_points)
        }
        return [self.records[index] for index in sorted(indices)]


def compare_by_track_distance(left: Iterable[TelemetryRecord], right: Iterable[TelemetryRecord]) -> list[dict[str, float]]:
    """Compare available vehicle speed samples by nearest normalized progress.

    Phase 5 telemetry is incomplete in this repository, therefore this uses
    `track_s_m` when supplied and otherwise deterministic sample progress.
    """
    a, b = list(left), list(right)
    if not a or not b:
        return []
    result: list[dict[str, float]] = []
    for index, record in enumerate(a):
        progress = record.values.get("track_s_m", index / max(1, len(a) - 1))
        nearest = min(enumerate(b), key=lambda candidate: abs(candidate[1].values.get("track_s_m", candidate[0] / max(1, len(b) - 1)) - progress))[1]
        au = float(record.values.get("state", {}).get("u_mps", 0.0))
        bu = float(nearest.values.get("state", {}).get("u_mps", 0.0))
        result.append({"track_s_m": float(progress), "speed_delta_mps": au - bu})
    return result


class DashboardExporter:
    """Static HTML/CSV/JSON exports retaining critical safety visibility."""

    @staticmethod
    def export_json(store: TelemetryStore, destination: str | Path) -> Path:
        output = Path(destination)
        output.write_text(json.dumps([record.__dict__ for record in store.records], indent=2, default=str) + "\n", encoding="utf-8")
        return output

    @staticmethod
    def export_csv(store: TelemetryStore, destination: str | Path) -> Path:
        output = Path(destination)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=("timestamp_s", "speed_mps", "vcu_state", "faults", "valid", "stale"))
            writer.writeheader()
            for record in store.records:
                vcu = record.values.get("vcu", {})
                writer.writerow({"timestamp_s": record.timestamp_s, "speed_mps": record.values.get("state", {}).get("u_mps", ""),
                                 "vcu_state": vcu.get("state", ""), "faults": ";".join(vcu.get("faults", ())),
                                 "valid": record.valid, "stale": record.stale})
        return output

    @staticmethod
    def export_html(store: TelemetryStore, destination: str | Path, *, title: str = "Kart engineering dashboard") -> Path:
        output = Path(destination)
        current = store.records[-1] if store.records else None
        vcu = current.values.get("vcu", {}) if current else {}
        faults = ", ".join(vcu.get("faults", ())) or "none"
        emergency = vcu.get("state") == "EMERGENCY_STOP"
        body = f"<h1>{html.escape(title)}</h1><p>simulation samples: {len(store.records)}</p>"
        body += f"<p>connection: {'live' if store.connection_alive else 'LOST / STALE'}</p>"
        body += f"<p>latest simulation time [s]: {current.timestamp_s if current else 'n/a'}</p>"
        body += f"<p class='{'critical' if emergency else ''}'>VCU state: {html.escape(str(vcu.get('state', 'n/a')))}</p>"
        body += f"<p class='critical'>Active faults: {html.escape(faults)}</p>"
        body += "<p>Warning: analysis only. No dashboard action has actuator authority.</p>"
        output.write_text("<!doctype html><html><head><style>.critical{color:#b00020;font-weight:bold}</style></head><body>" + body + "</body></html>\n", encoding="utf-8")
        return output


def export_dashboard_bundle(
    store: TelemetryStore,
    destination: str | Path,
    *,
    track: Track,
    kart: FullKartConfig,
    static_root: str | Path,
    run_id: str = "exported-run",
) -> Path:
    """Export the same modular UI with immutable local replay data.

    The bundle remains read-only and can be served with ``python -m
    http.server``. It does not include WebSocket live mode.
    """
    output = Path(destination)
    source = Path(static_root)
    output.mkdir(parents=True, exist_ok=True)
    for directory in ("css", "js", "assets"):
        shutil.copytree(source / directory, output / directory, dirs_exist_ok=True)
    index = (source / "index.html").read_text(encoding="utf-8")
    (output / "index.html").write_text(index, encoding="utf-8")
    (output / "dashboard.html").write_text(index, encoding="utf-8")
    data = output / "data"
    data.mkdir(exist_ok=True)
    replay = ReplayDataset(store, track)
    first = replay.records[0]["timestamp_s"] if replay.records else None
    last = replay.records[-1]["timestamp_s"] if replay.records else None
    manifest = {
        "run_id": run_id,
        "scenario": run_id,
        "track": track.name,
        "kart": kart.kart.name,
        "samples": len(replay.records),
        "mode": "replay",
        "read_only": True,
        "actuator_authority": "NONE",
        "start_time_s": first,
        "end_time_s": last,
    }
    (data / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (data / "track.json").write_text(json.dumps(dashboard_track_payload(track)) + "\n", encoding="utf-8")
    (data / "kart.json").write_text(json.dumps(dashboard_kart_payload(kart)) + "\n", encoding="utf-8")
    (data / "telemetry.json").write_text(json.dumps({"records": replay.records}) + "\n", encoding="utf-8")
    return output / "dashboard.html"
