"""Transport-neutral, read-only normalization for Mission Control.

Only odometry creates a frame. Other ROS samples are attached when their
measurement timestamps are no newer than that odometry sample. Ground truth
has a separate display-only slot and cannot enter planner debug fields.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from math import atan2, hypot, isfinite
from threading import Lock
import time
from typing import Any

STAGES = frozenset({"delaunay", "gabriel", "gates", "candidates", "mission", "obstacle", "control", "metrics", "safety", "perception", "mapping", "planning"})
PLANNING_FIELDS = {
    "delaunay": "delaunay_edges",
    "gabriel": "gabriel_edges",
    "gates": "gates",
    "candidates": "candidate_paths",
    "mission": "skidpad",
}


def target_on_path(target: Any, path: list[dict], tolerance_m: float = 0.75) -> bool:
    """Gate display of a controller target against the current odom path."""
    if not isinstance(target, dict) or not path:
        return False
    try:
        x, y = float(target["x_m"]), float(target["y_m"])
        points = [(float(point["x_m"]), float(point["y_m"])) for point in path]
    except (KeyError, TypeError, ValueError):
        return False
    if not all(isfinite(value) for point in [(x, y), *points] for value in point):
        return False
    if len(points) == 1:
        return hypot(x - points[0][0], y - points[0][1]) <= tolerance_m
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        fraction = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / max(dx * dx + dy * dy, 1e-12)))
        if hypot(x - (ax + fraction * dx), y - (ay + fraction * dy)) <= tolerance_m:
            return True
    return False


def mapping_visualization(payload: Any) -> dict[str, Any]:
    """Normalize published observed geometry; do not calculate a new map."""
    empty = {"cones": [], "delaunay_edges": [], "gabriel_edges": [],
             "gates": [], "centerline": [], "boundary_left": [],
             "boundary_right": [], "boundary_status": "UNAVAILABLE",
             "skidpad_topology": {}}
    if not isinstance(payload, dict) or payload.get("valid") is not True or payload.get("frame_id") != "odom":
        return empty
    geometry = payload.get("geometry") or {}
    raw_cones = geometry.get("observation_cones_odom") or []
    cones = []
    for cone in raw_cones:
        try:
            x, y, confidence = float(cone["x_m"]), float(cone["y_m"]), float(cone["confidence"])
            if not all(isfinite(value) for value in (x, y, confidence)) or not 0 <= confidence <= 1:
                return empty
            cones.append({"id": str(cone["id"]), "class": str(cone["label"]).upper(),
                          "confidence": confidence, "x_m": x, "y_m": y})
        except (KeyError, TypeError, ValueError):
            return empty
    by_id = {cone["id"]: cone for cone in cones}
    if len(by_id) != len(cones):
        return empty
    def pairs(name: str) -> set[tuple[str, str]]:
        result = set()
        for pair in geometry.get(name) or []:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                continue
            a, b = str(pair[0]), str(pair[1])
            if a in by_id and b in by_id and a != b:
                result.add(tuple(sorted((a, b))))
        return result
    delaunay = pairs("delaunay_edges")
    accepted = pairs("gabriel_edges")
    gates = []
    for gate in geometry.get("gates") or []:
        if not isinstance(gate, dict):
            continue
        a, b = by_id.get(str(gate.get("left_id"))), by_id.get(str(gate.get("right_id")))
        if a is None or b is None:
            continue
        dx, dy = b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]
        gates.append({"id": str(gate.get("id")), "midpoint": {
            "x_m": (a["x_m"] + b["x_m"]) / 2, "y_m": (a["y_m"] + b["y_m"]) / 2},
            "width_m": hypot(dx, dy), "heading_rad": atan2(dx, -dy),
            "confidence": gate.get("quality"), "valid": gate.get("status") == "VALID",
            "reason": None if gate.get("status") == "VALID" else str(gate.get("status", "UNQUALIFIED"))})
    centerline = []
    for item in geometry.get("centerline_odom") or []:
        try:
            if isinstance(item, (list, tuple)) and len(item) == 2:
                x, y = float(item[0]), float(item[1])
                if isfinite(x) and isfinite(y):
                    centerline.append({"x_m": x, "y_m": y})
        except (TypeError, ValueError):
            continue
    boundaries = geometry.get("boundaries") or {}
    if not isinstance(boundaries, dict):
        boundaries = {}
    def boundary(name: str) -> list[dict[str, float]]:
        return [{"x_m": by_id[str(identifier)]["x_m"], "y_m": by_id[str(identifier)]["y_m"]}
                for identifier in boundaries.get(name) or [] if str(identifier) in by_id]
    return {"cones": cones,
            "delaunay_edges": [{"id": f"D_{a}_{b}", "a": a, "b": b} for a, b in sorted(delaunay)],
            "gabriel_edges": [{"id": f"G_{a}_{b}", "a": a, "b": b,
                               "accepted": (a, b) in accepted,
                               "reason": None if (a, b) in accepted else "GABRIEL_REJECTED"}
                              for a, b in sorted(delaunay)],
            "gates": gates, "centerline": centerline,
            "boundary_left": boundary("left_ids"),
            "boundary_right": boundary("right_ids"),
            "boundary_status": str(boundaries.get("status", "UNAVAILABLE")),
            "skidpad_topology": geometry.get("skidpad_topology") or {}}


def finite_time(value: Any) -> float:
    number = float(value)
    if not isfinite(number) or number < 0:
        raise ValueError("measurement time must be finite and nonnegative")
    return number


def valid_frame(frame: Any) -> bool:
    if not isinstance(frame, dict) or frame.get("schema_version") != 1:
        return False
    try:
        finite_time(frame["timestamp_s"])
        vehicle = frame["vehicle"]
        return all(isfinite(float(vehicle[key])) for key in ("x_m", "y_m", "yaw_rad", "speed_mps")) and isinstance(frame["perception"]["cones"], list) and isinstance(frame["planning"]["delaunay_edges"], list)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


@dataclass(frozen=True)
class Stamped:
    time_s: float
    value: Any


class MissionStore:
    """Bounded common timebase; no physics, planning, or command decisions."""

    def __init__(self, source: str = "SIMULATION", run_id: str = "ros-live", limit: int = 6000, max_age_s: float = 0.5, scenario: str = "UNKNOWN"):
        if source not in {"SIMULATION", "REAL_KART"}:
            raise ValueError("source must be SIMULATION or REAL_KART")
        if limit < 1 or max_age_s <= 0:
            raise ValueError("invalid store bounds")
        self.source, self.run_id, self.max_age_s, self.scenario = source, run_id, max_age_s, scenario
        self.frames: deque[dict[str, Any]] = deque(maxlen=limit)
        self._parts: dict[str, Stamped] = {}
        self._received_at: dict[str, float] = {}
        self._lock = Lock()
        self._sequence = 0

    def update(self, name: str, measurement_time_s: float, value: Any) -> None:
        if name not in {"cones", "tracked_cones", "ground_truth_cones", "path", "command", "feedback", "vcu", "safety", "safety_envelope", *STAGES}:
            raise ValueError("unsupported input channel")
        if name == "ground_truth_cones" and self.source != "SIMULATION":
            raise ValueError("real kart frames have no simulator ground truth")
        when = finite_time(measurement_time_s)
        with self._lock:
            previous = self._parts.get(name)
            if previous is None or when >= previous.time_s:
                self._parts[name] = Stamped(when, deepcopy(value))
                self._received_at[name] = time.monotonic()

    def update_debug(self, stage: str, measurement_time_s: float, payload: Any) -> None:
        if stage not in STAGES or not isinstance(payload, dict):
            raise ValueError("unknown or malformed debug stage")
        self.update(stage, measurement_time_s, payload)

    def capture(self, measurement_time_s: float, vehicle: dict[str, Any]) -> dict[str, Any]:
        when = finite_time(measurement_time_s)
        if not all(key in vehicle and isfinite(float(vehicle[key])) for key in ("x_m", "y_m", "yaw_rad", "speed_mps")):
            raise ValueError("odometry lacks measured pose or speed")
        with self._lock:
            if self.frames and when < self.frames[-1]["timestamp_s"]:
                # /clock resets start a new replay epoch; old samples cannot
                # attach to the new one.
                self.frames.clear()
                self._parts.clear()
                self._received_at.clear()
                # Keep the transport sequence monotonic so connected
                # WebSocket clients receive the first frame after /clock reset.

            def sample(name: str) -> Any:
                item = self._parts.get(name)
                # FSDS publishes its physical track once with transient-local
                # durability. Keep it for display until a /clock reset.
                if item is None or item.time_s > when or (name != "ground_truth_cones" and when - item.time_s > self.max_age_s):
                    return None
                return deepcopy(item.value)

            command = sample("command") or {}
            feedback = sample("feedback") or {}
            vcu = sample("vcu") or {}
            safety = sample("safety") or {}
            safety_envelope = sample("safety_envelope") or {}
            control = sample("control") or {}
            obstacle = sample("obstacle") or {}
            metrics = sample("metrics") or {}
            expected_origin = "LIVE" if self.source == "REAL_KART" else "SIMULATED"
            def debug_current(payload: dict[str, Any]) -> bool:
                expiry = payload.get("expires_ns")
                return (payload.get("source") == expected_origin and
                        isinstance(expiry, int) and expiry >= round(when * 1e9))
            if self.source == "REAL_KART":
                control = control if debug_current(control) else {}
                obstacle = obstacle if debug_current(obstacle) else {}
                metrics = metrics if debug_current(metrics) else {}
                safety = safety if debug_current(safety) else {}
            map_payload = sample("mapping") or {}
            map_part = self._parts.get("mapping")
            map_valid = (map_payload.get("valid") is True and debug_current(map_payload)
                         and map_payload.get("frame_id") == "odom")
            mapped = mapping_visualization(map_payload) if map_valid else mapping_visualization(None)
            tracked_part = self._parts.get("tracked_cones")
            tracked = (sample("tracked_cones") or []) if (map_valid and map_part and tracked_part and
                abs(map_part.time_s - tracked_part.time_s) < 0.001) else []
            planner_payload = sample("planning") or {}
            planner_part = self._parts.get("planning")
            path_part = self._parts.get("path")
            planner_current = debug_current(planner_payload)
            planner_valid = (planner_payload.get("valid") is True and planner_current and
                             planner_payload.get("frame_id") == "odom")
            selected_path = (sample("path") or []) if (planner_valid and planner_part and path_part and
                abs(planner_part.time_s - path_part.time_s) < 0.001) else []
            mission_state = planner_payload.get("mission_state") if planner_current else None
            topology = mapped["skidpad_topology"]
            skidpad = (sample("mission") or {}).get("skidpad") if self.source == "SIMULATION" else None
            if isinstance(mission_state, dict):
                skidpad = {"state": mission_state.get("state"),
                           "w_right": mission_state.get("right_laps"),
                           "w_left": mission_state.get("left_laps"),
                           "branch": "RIGHT" if str(mission_state.get("state", "")).startswith("RIGHT") else
                                     "LEFT" if str(mission_state.get("state", "")).startswith("LEFT") else
                                     mission_state.get("state"),
                           "progress_index": mission_state.get("progress_turns"),
                           "progress_unit": "turns",
                           "target_winding": 2,
                           "reason": ", ".join(planner_payload.get("reason_codes") or [])}
                if topology.get("valid"):
                    skidpad.update({"left_center": {"x_m": topology["left"]["center_x_m"],
                                                       "y_m": topology["left"]["center_y_m"]},
                                    "right_center": {"x_m": topology["right"]["center_x_m"],
                                                        "y_m": topology["right"]["center_y_m"]},
                                    "left_radius_m": topology["left"]["radius_m"],
                                    "right_radius_m": topology["right"]["radius_m"]})
            frame = {
                "schema_version": 1,
                "source": self.source,
                "run_id": self.run_id,
                "scenario": self.scenario,
                "sequence": self._sequence,
                "timestamp_s": when,
                "vehicle": {
                    "x_m": vehicle["x_m"], "y_m": vehicle["y_m"], "z_m": vehicle.get("z_m", 0.0),
                    "yaw_rad": vehicle["yaw_rad"], "speed_mps": vehicle["speed_mps"],
                    "measured_steering_rad": feedback.get("measured_steering_rad"),
                    "requested_steering_rad": command.get("requested_steering_rad"),
                    "target_speed_mps": command.get("target_speed_mps"),
                    "brake_command": control.get("brake_command"),
                    "brake_feedback": feedback.get("brake_feedback"),
                },
                "perception": {
                    "cones": mapped["cones"] if map_valid else [] if self.source == "REAL_KART" else sample("cones") or [],
                    "tracked_cones": tracked,
                    "ground_truth_cones": sample("ground_truth_cones") or [],
                    "obstacles": obstacle.get("obstacles", []),
                },
                "planning": {
                    "delaunay_edges": mapped["delaunay_edges"] if map_valid else (sample("delaunay") or {}).get("edges", []) if self.source == "SIMULATION" else [],
                    "gabriel_edges": mapped["gabriel_edges"] if map_valid else (sample("gabriel") or {}).get("edges", []) if self.source == "SIMULATION" else [],
                    "gates": mapped["gates"] if map_valid else (sample("gates") or {}).get("gates", []) if self.source == "SIMULATION" else [],
                    "candidate_paths": (sample("candidates") or {}).get("paths", []) if self.source == "SIMULATION" else [],
                    "selected_path": selected_path,
                    "lookahead": control.get("lookahead") if target_on_path(control.get("lookahead"), selected_path) else None,
                    "predicted_path": control.get("predicted_path", []),
                    "skidpad": skidpad,
                },
                "visualization": {"course_guide": mapped["centerline"],
                                  "boundary_left": mapped["boundary_left"],
                                  "boundary_right": mapped["boundary_right"],
                                  "boundary_status": mapped["boundary_status"]} if map_valid else {},
                "safety": {
                    "state": vcu.get("state"), "priority": safety.get("priority"),
                    "physical_estop": vcu.get("physical_estop"),
                    "estop_input_status": vcu.get("estop_input_status", "NOT FITTED - MANUAL POWER CUT ONLY"),
                    "remote_estop": safety.get("remote_estop"),
                    "lora_heartbeat": safety.get("lora_heartbeat"),
                    "vcu_watchdog": safety.get("vcu_watchdog"),
                    "rpi_heartbeat": safety.get("rpi_heartbeat"),
                    "command_fresh": safety.get("command_fresh"),
                    "sensor_fresh": safety.get("sensor_fresh"),
                    "motor_enable": vcu.get("motor_enable"),
                    "faults": vcu.get("faults", []),
                    "obstacle": obstacle.get("safety"),
                    # Front safety envelope monitor (DIAGNOSTIC_ONLY, vehicle_authority=OFF):
                    # read-only CLEAR/BRAKING/CRITICAL/UNKNOWN classification from
                    # /safety/envelope. Distinct from vcu-derived "state" above.
                    "envelope": safety_envelope or None,
                },
                "metrics": metrics,
                "diagnostics": {name: payload if self.source == "SIMULATION" or debug_current(payload) else None
                                for name in ("perception", "mapping", "planning")
                                if (payload := sample(name)) is not None},
            }
            self.frames.append(frame)
            self._received_at["odometry"] = time.monotonic()
            self._sequence += 1
            return deepcopy(frame)

    def live_status(self, freshness_s: float = 1.0) -> dict[str, Any]:
        """Current ROS arrival health, including when no odometry frame exists."""
        now = time.monotonic()
        with self._lock:
            def channel(name: str) -> dict[str, Any]:
                received = self._received_at.get(name)
                age_ms = (now - received) * 1000 if received is not None else None
                current = age_ms is not None and age_ms <= freshness_s * 1000
                value = self._parts.get(name)
                return {"current": current,
                        "state": "CURRENT" if current else "STALE" if received is not None else "MISSING",
                        "age_ms": age_ms,
                        "payload": deepcopy(value.value) if current and value is not None else None}

            stages = {name: channel(name) for name in ("perception", "mapping", "planning")}
            odometry = channel("odometry")
            vcu = channel("vcu")
            safety = channel("safety")
            safety_envelope = channel("safety_envelope")
            return {"source": self.source, "run_id": self.run_id, "scenario": self.scenario,
                    "ros_connected": any(item["current"] for item in (odometry, vcu, *stages.values())),
                    "odometry": odometry, "vcu": vcu, "safety": safety, "safety_envelope": safety_envelope,
                    "stages": stages, "read_only": True, "vehicle_authorized": False}

    def since(self, sequence: int) -> list[dict[str, Any]]:
        with self._lock:
            return deepcopy([frame for frame in self.frames if frame["sequence"] > sequence])

    def all_frames(self) -> list[dict[str, Any]]:
        with self._lock:
            return deepcopy(list(self.frames))

    def manifest(self) -> dict[str, Any]:
        with self._lock:
            frames = list(self.frames)
            return {
                "schema_version": 1, "run_id": self.run_id, "source": self.source, "scenario": self.scenario,
                "samples": len(frames), "start_time_s": frames[0]["timestamp_s"] if frames else None,
                "end_time_s": frames[-1]["timestamp_s"] if frames else None,
                "read_only": True, "actuator_authority": "NONE",
            }
