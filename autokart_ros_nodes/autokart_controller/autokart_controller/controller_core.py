"""ROS-independent controller core: freshness/validity gating around the Pure Pursuit + PID
pursuit backend. No vehicle authority -- see CONTRACT.md: this node's output is always a
candidate, never dispatched to an actuator transport.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, isfinite

from .pursuit_backends import PursuitBackend, PursuitBackendError

DEFAULT_MAX_POSE_AGE_NS = 150_000_000  # 150 ms, matches this repo's other nodes' pose-freshness gate
DEFAULT_MAX_PATH_AGE_NS = 500_000_000  # 500 ms


def _headings_for(points: list) -> list:
    """Per-point heading, same convention as the existing
    `kart/ros2_ws/.../autokart_controller/engine.py::ControllerEngine._build_trajectory`: forward
    difference to the next point, or the heading of the last segment for the final point."""
    headings = [0.0] * len(points)
    previous = None
    for index, (x, y) in enumerate(points):
        if index + 1 < len(points):
            nxt_x, nxt_y = points[index + 1]
            headings[index] = atan2(nxt_y - y, nxt_x - x)
        elif previous is not None:
            headings[index] = atan2(y - previous[1], x - previous[0])
        previous = (x, y)
    return headings


@dataclass
class ControllerCore:
    backend: PursuitBackend
    maximum_speed_mps: float
    max_pose_age_ns: int = DEFAULT_MAX_POSE_AGE_NS
    max_path_age_ns: int = DEFAULT_MAX_PATH_AGE_NS
    _sequence: int = field(default=0, init=False, repr=False)

    def reject(self, reason: str, stamp_ns: int) -> dict:
        # A rejected cycle publishes steering=0/speed=0; drop the backend's rate-limiter/PID memory
        # so the next valid command is computed fresh rather than from pre-fault state.
        reset = getattr(self.backend, "reset", None)
        if callable(reset):
            reset()
        self._sequence += 1
        return {
            "valid": False, "reason_codes": [reason], "sample_time_ns": stamp_ns,
            "sequence": self._sequence, "steering_rad": 0.0, "speed_mps": 0.0, "brake": 0.0,
            "drive": 0.0, "cross_track_error_m": None, "heading_error_rad": None,
            "speed_tracking_error_mps": None,
        }

    def evaluate(self, *, path_points: list, path_stamp_ns: int, now_ns: int,
                 envelope_valid: bool, envelope_max_speed_mps: float, speed_profile: list,
                 pose_x: float, pose_y: float, pose_yaw_rad: float, pose_speed_mps: float,
                 pose_fresh: bool, dt_s: float, brake_stuck: bool = False) -> dict:
        if not envelope_valid:
            return self.reject("PLANNER_INVALID", path_stamp_ns)
        if not pose_fresh:
            return self.reject("POSE_MISSING_OR_STALE", path_stamp_ns)
        if now_ns < path_stamp_ns:
            return self.reject("PATH_TIMESTAMP_INVALID", path_stamp_ns)
        if now_ns - path_stamp_ns > self.max_path_age_ns:
            return self.reject("PATH_EXPIRED", path_stamp_ns)
        if len(path_points) < 2:
            return self.reject("PATH_TOO_SHORT", path_stamp_ns)
        if not all(isfinite(c) for p in path_points for c in p):
            return self.reject("PATH_NONFINITE", path_stamp_ns)
        if not isfinite(pose_speed_mps) or pose_speed_mps < 0:
            return self.reject("POSE_SPEED_INVALID", path_stamp_ns)
        if dt_s <= 0.0 or not isfinite(dt_s):
            return self.reject("CONTROLLER_TIMING_INVALID", path_stamp_ns)
        if not isfinite(envelope_max_speed_mps) or envelope_max_speed_mps < 0:
            return self.reject("ENVELOPE_SPEED_INVALID", path_stamp_ns)

        headings = _headings_for(path_points)
        if speed_profile and len(speed_profile) == len(path_points):
            speeds = speed_profile
        else:
            # No per-point profile available (or it doesn't align with this path): fall back to
            # the envelope's flat scalar cap -- never to an un-capped/last-known speed.
            speeds = [envelope_max_speed_mps] * len(path_points)
        points_with_heading_speed = [
            (x, y, headings[i], speeds[i]) for i, (x, y) in enumerate(path_points)
        ]

        safety_speed_limit_mps = min(self.maximum_speed_mps, max(0.0, envelope_max_speed_mps))
        try:
            output = self.backend.compute(
                pose_x=pose_x, pose_y=pose_y, pose_yaw_rad=pose_yaw_rad, pose_speed_mps=pose_speed_mps,
                path_points_with_heading_speed=points_with_heading_speed, dt_s=dt_s,
                safety_speed_limit_mps=safety_speed_limit_mps, brake_stuck=brake_stuck,
            )
        except PursuitBackendError as exc:
            return self.reject(f"CONTROLLER_COMPUTE_ERROR:{exc}", path_stamp_ns)

        self._sequence += 1
        return {
            "valid": True, "reason_codes": [], "sample_time_ns": path_stamp_ns,
            "sequence": self._sequence, "steering_rad": output["steering_rad"],
            "speed_mps": output["speed_mps"], "brake": output["brake"], "drive": output["drive"],
            "cross_track_error_m": output["cross_track_error_m"],
            "heading_error_rad": output["heading_error_rad"],
            "speed_tracking_error_mps": output["speed_tracking_error_mps"],
        }
