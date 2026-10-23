"""Fast-racing planners and gap-following baseline controllers."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable

from .autonomy import MessageHeader, Trajectory, TrajectoryPoint
from .cones import Cone, ConeClass
from .track import Track
from .types import SimulationError


@dataclass(frozen=True)
class GapCommand:
    target_speed_mps: float
    steering_rad: float
    gap_start_index: int
    gap_end_index: int
    minimum_clearance_m: float


@dataclass
class FollowTheGapController:
    field_of_view_rad: float = 4.71238898038
    clearance_threshold_m: float = 1.5
    maximum_speed_mps: float = 6.0
    maximum_steering_rad: float = 0.5585053606

    def compute(self, ranges_m: Iterable[float]) -> GapCommand:
        ranges = tuple(float(value) for value in ranges_m)
        if len(ranges) < 3:
            raise SimulationError("gap controller requires at least three beams")
        free = [isfinite(value) and value >= self.clearance_threshold_m for value in ranges]
        best = (0, -1)
        start = None
        for index, available in enumerate(free + [False]):
            if available and start is None:
                start = index
            elif not available and start is not None:
                if index - start > best[1] - best[0] + 1:
                    best = (start, index - 1)
                start = None
        if best[1] < best[0]:
            return GapCommand(0.0, 0.0, 0, 0, 0.0)
        target = (best[0] + best[1]) // 2
        angle = -self.field_of_view_rad / 2 + self.field_of_view_rad * target / (len(ranges) - 1)
        steering = max(-self.maximum_steering_rad, min(self.maximum_steering_rad, angle))
        clearance = min(ranges[index] for index in range(best[0], best[1] + 1))
        speed = self.maximum_speed_mps * max(0.15, 1.0 - abs(steering) / self.maximum_steering_rad)
        return GapCommand(speed, steering, best[0], best[1], clearance)


def track_trajectory(
    track: Track,
    *,
    timestamp_s: float,
    speed_mps: float = 4.0,
    spacing_m: float = 0.5,
    source: str = "centreline_planner",
) -> Trajectory:
    points = []
    count = max(2, int(track.length_m / spacing_m))
    for index in range(count + 1):
        s_m = min(track.length_m, index * track.length_m / count)
        point = track.sample(s_m)
        points.append(TrajectoryPoint(
            timestamp_s + index * spacing_m / max(speed_mps, 0.1),
            point.x_m,
            point.y_m,
            point.z_m,
            point.heading_rad,
            speed_mps,
            curvature_1pm=point.curvature_1pm,
            track_s_m=s_m,
        ))
    return Trajectory(
        MessageHeader(timestamp_s, "map", 1, source),
        tuple(points),
        timestamp_s + max(0.2, track.length_m / max(speed_mps, 0.1)),
    )


def cone_midpoint_path(cones: Iterable[Cone], *, timestamp_s: float, speed_mps: float = 3.0) -> Trajectory:
    left = sorted((cone for cone in cones if cone.semantic_class == ConeClass.BLUE), key=lambda cone: cone.cone_id)
    right = sorted((cone for cone in cones if cone.semantic_class == ConeClass.YELLOW), key=lambda cone: cone.cone_id)
    if len(left) < 2 or len(left) != len(right):
        raise SimulationError("cone midpoint planner requires paired blue/yellow boundaries")
    points = tuple(
        TrajectoryPoint(
            timestamp_s + index * 0.2,
            (blue.x_m + yellow.x_m) / 2,
            (blue.y_m + yellow.y_m) / 2,
            speed_mps=speed_mps,
        )
        for index, (blue, yellow) in enumerate(zip(left, right))
    )
    return Trajectory(MessageHeader(timestamp_s, "map", 1, "cone_midpoint_planner"), points, timestamp_s + len(points) * 0.2)

