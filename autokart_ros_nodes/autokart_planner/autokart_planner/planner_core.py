"""ROS-independent planner core: path-candidate validity, per-point curvature-capped speed
profile, obstacle clearance, and the braking stop envelope.

Scope boundary (read this before changing anything here): this module is deliberately
**planner-proper only** -- curvature, speed profile, collision clearance, stop envelope. It has
no mission state machine (no SKIDPAD/BRAKING_TEST progress FSM, no gate-crossing logic). That FSM
lives in `kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py` and is
being extracted into a separate standalone node (06/07) by another agent in this session -- see
`../README.md` and `../../AUDIT_PHASE0.md`. This module does not import anything from that file
and does not depend on its internal shape, so it is unaffected by however that extraction turns
out. A caller (this package's own `node.py`, or a future mission node) supplies an already-chosen
reference path; this module only evaluates whether following it is currently safe and at what
speed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot, isfinite, sqrt

from .algorithms import AlgorithmBackend, AlgorithmBackendError

DEFAULT_MAX_ODOM_AGE_NS = 150_000_000  # 150 ms, matches this repo's other nodes' pose-freshness gate
DEFAULT_MAX_PATH_AGE_NS = 500_000_000  # 500 ms


def _closest_point_on_polyline(x: float, y: float, points: list):
    """Point-to-polyline distance (standard computational geometry: nearest point on any of the
    polyline's segments), returning ``(distance_m, arc_length_m_to_closest_point)``.

    Written fresh rather than reused from `mission.py`'s private `_polyline_closest` helper: that
    helper lives inside the mission-FSM module another agent in this session may be restructuring
    (see the module docstring above), and this is textbook point-segment distance math, not a
    tuned or domain-specific algorithm the mission statement asks us to preserve verbatim.
    """
    best_distance, best_arc = float("inf"), 0.0
    cumulative = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        seg_len = hypot(dx, dy)
        if seg_len <= 0:
            continue
        fraction = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (seg_len * seg_len)))
        px, py = ax + fraction * dx, ay + fraction * dy
        distance = hypot(x - px, y - py)
        arc = cumulative + fraction * seg_len
        if distance < best_distance:
            best_distance, best_arc = distance, arc
        cumulative += seg_len
    return best_distance, best_arc


def _path_length(points) -> float:
    return sum(hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:]))


@dataclass
class PlannerCore:
    algorithm: AlgorithmBackend
    a_lat_max_mps2: float
    a_brake_min_mps2: float
    t_total_latency_s: float
    d_margin_m: float
    footprint_width_m: float
    speed_limit_mps: float
    curvature_eps_m_inv: float = 1e-3
    max_path_age_ns: int = DEFAULT_MAX_PATH_AGE_NS
    _sequence: int = field(default=0, init=False, repr=False)

    def reject(self, reason: str, stamp_ns: int, *, decision: str = "HOLD") -> dict:
        self._sequence += 1
        return {
            "valid": False, "reason_codes": [reason], "decision": decision,
            "path_odom": [], "speed_profile": [], "curvature_profile": [],
            "sample_time_ns": stamp_ns, "sequence": self._sequence,
            "maximum_speed_mps": 0.0, "clear_distance_m": 0.0,
            "stopping_distance_m": None, "limiting_reason": reason,
        }

    def evaluate(self, *, path_points: list, path_stamp_ns: int, now_ns: int,
                 pose_x: float, pose_y: float, pose_speed_mps: float, pose_fresh: bool,
                 obstacles: list) -> dict:
        if not pose_fresh:
            return self.reject("ODOM_MISSING_OR_STALE", path_stamp_ns)
        if now_ns < path_stamp_ns:
            return self.reject("PATH_TIMESTAMP_INVALID", path_stamp_ns)
        if now_ns - path_stamp_ns > self.max_path_age_ns:
            return self.reject("PATH_EXPIRED", path_stamp_ns)
        if len(path_points) < 2:
            return self.reject("PATH_TOO_SHORT", path_stamp_ns)
        if not all(isfinite(c) for p in path_points for c in p):
            return self.reject("PATH_NONFINITE", path_stamp_ns)
        if not isfinite(pose_speed_mps) or pose_speed_mps < 0:
            return self.reject("ODOM_SPEED_INVALID", path_stamp_ns)
        if not (isfinite(self.a_lat_max_mps2) and self.a_lat_max_mps2 > 0):
            return self.reject("LATERAL_LIMIT_UNQUALIFIED", path_stamp_ns)
        if not (isfinite(self.footprint_width_m) and self.footprint_width_m > 0):
            return self.reject("FOOTPRINT_WIDTH_UNQUALIFIED", path_stamp_ns)
        if not (isfinite(self.curvature_eps_m_inv) and self.curvature_eps_m_inv > 0):
            # A misconfigured curvature_eps_m_inv<=0 would otherwise divide by exactly zero at any
            # perfectly straight path point (kappa=0) inside the speed-profile comprehension below
            # -- an uncaught ZeroDivisionError that would crash this evaluate() call (and, in the
            # ROS node, the subscription callback that invoked it). Found by this review with a
            # reproduction test before this guard existed; see test_planner_core.py.
            return self.reject("CURVATURE_EPSILON_UNQUALIFIED", path_stamp_ns)

        try:
            kappas = self.algorithm.curvature_profile(path_points)
        except AlgorithmBackendError as exc:
            return self.reject(f"CURVATURE_BACKEND_ERROR:{exc}", path_stamp_ns)

        speed_profile = [
            min(self.speed_limit_mps, sqrt(self.a_lat_max_mps2 / max(abs(k), self.curvature_eps_m_inv)))
            for k in kappas
        ]

        try:
            stopping_distance_m = self.algorithm.stopping_distance(
                pose_speed_mps, a_brake_min_mps2=self.a_brake_min_mps2,
                t_total_latency_s=self.t_total_latency_s, d_margin_m=self.d_margin_m)
        except AlgorithmBackendError as exc:
            return self.reject(f"STOPPING_DISTANCE_BACKEND_ERROR:{exc}", path_stamp_ns)

        clear_distance_m, blocking_reason = self._clearance(path_points, pose_x, pose_y, obstacles)
        if blocking_reason == "INVALID_OBSTACLE":
            return self.reject("INVALID_OBSTACLE", path_stamp_ns)

        if clear_distance_m < stopping_distance_m:
            self._sequence += 1
            reason = blocking_reason or "STOP_ENVELOPE_VIOLATED"
            return {
                "valid": False, "reason_codes": [reason], "decision": "STOP_CANDIDATE",
                "path_odom": [], "speed_profile": [0.0] * len(path_points),
                "curvature_profile": kappas, "sample_time_ns": path_stamp_ns, "sequence": self._sequence,
                "maximum_speed_mps": 0.0, "clear_distance_m": clear_distance_m,
                "stopping_distance_m": stopping_distance_m, "limiting_reason": reason,
            }

        self._sequence += 1
        return {
            "valid": True, "reason_codes": [], "decision": "PATH_CANDIDATE",
            "path_odom": path_points, "speed_profile": speed_profile, "curvature_profile": kappas,
            "sample_time_ns": path_stamp_ns, "sequence": self._sequence,
            "maximum_speed_mps": min(speed_profile) if speed_profile else 0.0,
            "clear_distance_m": clear_distance_m, "stopping_distance_m": stopping_distance_m,
            "limiting_reason": "",
        }

    def _clearance(self, path_points: list, pose_x: float, pose_y: float, obstacles: list):
        """Returns (clear_distance_m, reason). ``reason`` is ``"CORRIDOR_BLOCKED"`` when a
        forward obstacle intersects the footprint-expanded path, ``"INVALID_OBSTACLE"`` when an
        obstacle entry is malformed/non-finite, or ``None`` when the path is clear (in which case
        ``clear_distance_m`` is the remaining path length ahead of the vehicle)."""
        _, pose_arc = _closest_point_on_polyline(pose_x, pose_y, path_points)
        total_length = _path_length(path_points)
        nearest_block_arc = None
        for obstacle in obstacles:
            try:
                ox, oy, radius = (float(obstacle[key]) for key in ("x_m", "y_m", "radius_m"))
            except (KeyError, TypeError, ValueError):
                return 0.0, "INVALID_OBSTACLE"
            if not all(isfinite(v) for v in (ox, oy, radius)) or radius < 0:
                return 0.0, "INVALID_OBSTACLE"
            distance, arc = _closest_point_on_polyline(ox, oy, path_points)
            if arc < pose_arc:
                continue  # behind the vehicle along the path: not a forward obstacle
            if distance <= radius + self.footprint_width_m / 2.0:
                if nearest_block_arc is None or arc < nearest_block_arc:
                    nearest_block_arc = arc
        if nearest_block_arc is not None:
            return max(0.0, nearest_block_arc - pose_arc), "CORRIDOR_BLOCKED"
        return max(0.0, total_length - pose_arc), None
