"""ROS-independent gate around the existing Pure Pursuit command candidate.

This module never produces vehicle authority. It consumes an already-gated
planner path/status pair and a fresh vehicle pose, and emits a bounded,
expiring ``MotionCommand`` *candidate* using the existing
``kart_sim.autonomy.ControllerManager`` / ``PurePursuitController`` and
``kart_sim.vcu.MotionCommand`` implementations. A missing, stale, mismatched
or invalid input always clears the previous candidate and returns
``valid=False`` with a reason code; it never reuses an old command.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, hypot, isfinite

from kart_sim.autonomy import (
    ControllerManager,
    LongitudinalPidController,
    MessageHeader,
    PurePursuitController,
    Trajectory,
    TrajectoryPoint,
)
from kart_sim.types import PlanarKartState, SimulationError

MAX_POSE_AGE_NS = 150_000_000
MINIMUM_TRAJECTORY_SPEED_MPS = 0.1


@dataclass
class ControllerEngine:
    """Wraps the existing baseline controller with freshness/validity gates."""

    source: str = "SIMULATED"
    wheelbase_m: float = 1.0
    lookahead_m: float = 1.5
    maximum_steering_rad: float = 0.6
    maximum_steering_rate_radps: float = 3.0
    maximum_speed_mps: float = 5.0
    acceleration_limit_mps2: float = 3.0
    deceleration_limit_mps2: float = 5.0
    max_pose_age_ns: int = MAX_POSE_AGE_NS
    _sequence: int = field(default=0, init=False, repr=False)
    manager: ControllerManager = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if self.wheelbase_m <= 0.0 or not isfinite(self.wheelbase_m):
            raise SimulationError("controller wheelbase must be positive and finite")
        self.manager = ControllerManager(
            PurePursuitController(
                lookahead_m=self.lookahead_m,
                wheelbase_m=self.wheelbase_m,
                maximum_steering_rad=self.maximum_steering_rad,
                maximum_steering_rate_radps=self.maximum_steering_rate_radps,
            ),
            LongitudinalPidController(),
            source_id=f"autokart_controller/{self.source}",
        )

    def reject(self, reason: str, stamp_ns: int) -> dict:
        self._sequence += 1
        return {
            "valid": False,
            "reason_codes": [reason],
            "sample_time_ns": stamp_ns,
            "sequence": self._sequence,
            "source": self.source,
            "target_speed_mps": 0.0,
            "steering_angle_rad": 0.0,
            "acceleration_limit_mps2": 0.0,
            "deceleration_limit_mps2": self.deceleration_limit_mps2,
            "integrity": 0,
            "mission_state": None,
            "decision": None,
        }

    def evaluate(
        self,
        *,
        path_points: list[tuple[float, float]],
        path_stamp_ns: int,
        planner_status: dict | None,
        pose: PlanarKartState | None,
        pose_valid: bool,
        pose_time_ns: int | None,
        now_ns: int,
        dt_s: float,
        brake_stuck: bool = False,
    ) -> dict:
        if not planner_status or not planner_status.get("valid"):
            return self.reject("PLANNER_INVALID", path_stamp_ns)
        if planner_status.get("sample_time_ns") != path_stamp_ns:
            return self.reject("PATH_STATUS_MISMATCH", path_stamp_ns)
        expires_ns = planner_status.get("expires_ns")
        if expires_ns is None or now_ns > expires_ns:
            return self.reject("PLANNER_STATUS_EXPIRED", path_stamp_ns)
        if len(path_points) < 2:
            return self.reject("PATH_TOO_SHORT", path_stamp_ns)
        if pose is None or not pose_valid or pose_time_ns is None:
            return self.reject("POSE_INVALID", path_stamp_ns)
        if abs(path_stamp_ns - pose_time_ns) > self.max_pose_age_ns:
            return self.reject("POSE_STALE", path_stamp_ns)
        target_speed = planner_status.get("target_speed_mps")
        if target_speed is None or not isfinite(target_speed) or target_speed < 0.0:
            return self.reject("TARGET_SPEED_INVALID", path_stamp_ns)
        if dt_s <= 0.0 or not isfinite(dt_s):
            return self.reject("CONTROLLER_TIMING_INVALID", path_stamp_ns)

        safety_speed_limit = min(self.maximum_speed_mps, max(target_speed, 0.0))
        try:
            trajectory = self._build_trajectory(path_points, path_stamp_ns, target_speed)
            motion = self.manager.command(
                pose,
                trajectory,
                timestamp_s=path_stamp_ns / 1e9,
                dt_s=dt_s,
                safety_speed_limit_mps=safety_speed_limit,
                brake_stuck=brake_stuck,
            )
        except SimulationError:
            return self.reject("CONTROLLER_COMPUTE_ERROR", path_stamp_ns)

        self._sequence += 1
        return {
            "valid": bool(motion.valid),
            "reason_codes": [] if motion.valid else ["CONTROLLER_OUTPUT_INVALID"],
            "sample_time_ns": path_stamp_ns,
            "sequence": self._sequence,
            "source": self.source,
            "target_speed_mps": float(motion.speed_mps),
            "steering_angle_rad": float(motion.steering_rad),
            "acceleration_limit_mps2": float(motion.acceleration_limit_mps2),
            "deceleration_limit_mps2": float(motion.deceleration_limit_mps2),
            "brake": float(motion.brake),
            "integrity": int(motion.checksum),
            "mission_state": planner_status.get("mission_state"),
            "decision": planner_status.get("decision"),
        }

    def _build_trajectory(
        self, path_points: list[tuple[float, float]], stamp_ns: int, target_speed_mps: float
    ) -> Trajectory:
        speed = max(target_speed_mps, MINIMUM_TRAJECTORY_SPEED_MPS)
        points = []
        time_s = 0.0
        previous = None
        for index, (x, y) in enumerate(path_points):
            if previous is not None:
                time_s += max(hypot(x - previous[0], y - previous[1]), 1e-3) / speed
            heading = 0.0
            if index + 1 < len(path_points):
                nxt = path_points[index + 1]
                heading = atan2(nxt[1] - y, nxt[0] - x)
            elif previous is not None:
                heading = atan2(y - previous[1], x - previous[0])
            points.append(
                TrajectoryPoint(
                    time_s=time_s,
                    x_m=float(x),
                    y_m=float(y),
                    heading_rad=heading,
                    speed_mps=min(target_speed_mps, self.maximum_speed_mps),
                )
            )
            previous = (x, y)
        header = MessageHeader(stamp_s=stamp_ns / 1e9, frame_id="odom", sequence=self._sequence, source=self.source)
        return Trajectory(header=header, points=tuple(points), valid_until_s=header.stamp_s + 1.0)
