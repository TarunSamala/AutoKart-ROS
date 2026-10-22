"""Straight-corridor planner: path, stop target 15 cm before the cone ahead, stopping-distance logic (no ROS)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

Pose = Tuple[float, float, float]


@dataclass
class PlannerConfig:
    stop_gap_m: float = 0.15            # final distance from the front of the kart to the cone ahead
    front_bumper_x_m: float = 0.6       # MEASURE: vehicle-frame x of the front of the kart
    cruise_speed_mps: float = 1.0
    a_verified_mps2: float = 0.5        # MEASURE with tools/braking_test.py. Conservative placeholder.
    t_latency_s: float = 0.25           # perception + control + actuator delay. MEASURE.
    d_margin_m: float = 0.10
    stop_lateral_tol_m: float = 0.35    # a cone this close to the centerline is the "center cone" ahead
    stop_class: str = ""                # optional: only this class counts as the stop cone
    max_run_m: float = 15.0             # without ever seeing a stop cone, stop after this distance
    stopped_speed_mps: float = 0.03
    map_timeout_s: float = 0.5


@dataclass
class PlanResult:
    path: List[Tuple[float, float]]
    path_valid: bool
    stop_valid: bool
    stop_target: Tuple[float, float]
    nearest_cone_m: float
    available_m: float
    required_stop_m: float
    target_speed_mps: float
    decision: str                       # HOLD NO_PATH CRUISE BRAKE STOP


def required_stop_distance(v: float, a: float, t_latency: float, margin: float) -> float:
    v = max(0.0, v)
    return v * v / (2.0 * a) + v * t_latency + margin


def point_on_path(path: List[Tuple[float, float]], pose: Pose, dist: float) -> Tuple[float, float]:
    """Point at arc length `dist` along the path measured from the kart's projection on it."""
    if len(path) < 2:
        return path[0] if path else (pose[0], pose[1])
    # start index: nearest point
    i0 = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - pose[0], path[i][1] - pose[1]))
    remaining = max(0.0, dist)
    for i in range(i0, len(path) - 1):
        seg = math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if seg >= remaining and seg > 0:
            f = remaining / seg
            return path[i][0] + f * (path[i + 1][0] - path[i][0]), path[i][1] + f * (path[i + 1][1] - path[i][1])
        remaining -= seg
    return path[-1]


def dist_to_polyline(px: float, py: float, path: List[Tuple[float, float]]) -> float:
    best = float("inf")
    for i in range(len(path) - 1):
        x1, y1 = path[i]; x2, y2 = path[i + 1]
        vx, vy = x2 - x1, y2 - y1
        l2 = vx * vx + vy * vy or 1e-9
        t = max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / l2))
        best = min(best, math.hypot(px - (x1 + t * vx), py - (y1 + t * vy)))
    return best


class Planner:
    def __init__(self, cfg: PlannerConfig):
        self.cfg = cfg
        self.mission_active = False
        self.braking_latched = False
        self.run_origin: Optional[Tuple[float, float]] = None
        self.slow_since: Optional[float] = None
        self.final_gap_m: Optional[float] = None   # measured gap to the cone once stopped (for the log)

    def reset(self):
        self.braking_latched = False
        self.run_origin = None
        self.slow_since = None
        self.final_gap_m = None

    def set_active(self, active: bool):
        if active and not self.mission_active:
            self.reset()
        if not active:
            self.reset()
        self.mission_active = active

    def plan(self, now: float, centerline: List[Tuple[float, float]], map_valid: bool, map_age_s: float,
             cones_odom: List[Tuple[str, float, float]], pose: Pose, speed: float) -> PlanResult:
        c = self.cfg
        px, py, yaw = pose
        path_ok = map_valid and len(centerline) >= 2 and map_age_s <= c.map_timeout_s

        nearest = float("nan"); available = float("nan"); stop_xy = (px, py)
        stop_found = False
        if path_ok:
            cdx, cdy = math.cos(yaw), math.sin(yaw)
            best_fwd = None
            for cls, ox, oy in cones_odom:
                if c.stop_class and cls != c.stop_class:
                    continue
                fwd = (ox - px) * cdx + (oy - py) * cdy
                if fwd <= 0.0:
                    continue
                if dist_to_polyline(ox, oy, centerline) <= c.stop_lateral_tol_m and (best_fwd is None or fwd < best_fwd):
                    best_fwd = fwd
            if best_fwd is not None:
                stop_found = True
                nearest = best_fwd - c.front_bumper_x_m
                available = nearest - c.stop_gap_m
                stop_xy = point_on_path(centerline, pose, max(0.0, available))

        required = required_stop_distance(speed, c.a_verified_mps2, c.t_latency_s, c.d_margin_m)

        if self.mission_active and self.run_origin is None:
            self.run_origin = (px, py)
        run_m = math.hypot(px - self.run_origin[0], py - self.run_origin[1]) if self.run_origin else 0.0

        if not self.mission_active:
            decision, target = "HOLD", 0.0
        elif self.braking_latched:
            decision, target = "BRAKE", 0.0
        elif not path_ok:
            if speed > c.stopped_speed_mps:
                decision, target = "BRAKE", 0.0
                self.braking_latched = True
            else:
                decision, target = "NO_PATH", 0.0
        elif stop_found and available <= required:
            decision, target = "BRAKE", 0.0
            self.braking_latched = True
        elif not stop_found and run_m > c.max_run_m:
            decision, target = "BRAKE", 0.0
            self.braking_latched = True
        else:
            decision, target = "CRUISE", c.cruise_speed_mps

        if decision == "BRAKE":
            if speed <= c.stopped_speed_mps:
                self.slow_since = self.slow_since if self.slow_since is not None else now
                if now - self.slow_since >= 0.5:
                    decision = "STOP"
                    if stop_found:
                        self.final_gap_m = nearest
            else:
                self.slow_since = None

        return PlanResult(centerline if path_ok else [], path_ok, stop_found, stop_xy, nearest, available,
                          required, target, decision)
