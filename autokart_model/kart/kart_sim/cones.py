"""Semantic cone ground truth and deterministic noisy perception."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from math import atan2, cos, hypot, sin
from random import Random
from typing import Iterable

from .types import PlanarKartState, SimulationError


class ConeClass(str, Enum):
    BLUE = "blue"
    YELLOW = "yellow"
    SMALL_ORANGE = "small_orange"
    LARGE_ORANGE = "large_orange"
    UNKNOWN = "unknown"
    FALLEN = "fallen"
    MOVED = "moved"


@dataclass(frozen=True)
class Cone:
    cone_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    semantic_class: ConeClass
    collision_radius_m: float = 0.15
    visible: bool = True
    lidar_reflectivity: float = 0.8
    displaced: bool = False
    knocked_down: bool = False

    def displaced_to(self, x_m: float, y_m: float, *, knocked_down: bool = False) -> "Cone":
        return replace(
            self,
            x_m=x_m,
            y_m=y_m,
            semantic_class=ConeClass.FALLEN if knocked_down else ConeClass.MOVED,
            displaced=True,
            knocked_down=knocked_down,
        )


@dataclass(frozen=True)
class ConeDetection:
    detection_id: str
    source_cone_id: str | None
    range_m: float
    bearing_rad: float
    semantic_class: ConeClass
    position_std_m: float
    confidence: float


@dataclass(frozen=True)
class ConeNoise:
    position_std_m: float = 0.03
    range_std_gain: float = 0.004
    misclassification_probability: float = 0.01
    missed_detection_probability: float = 0.02
    false_positive_rate: float = 0.0
    maximum_range_m: float = 35.0
    field_of_view_rad: float = 2.4


class ConePerception:
    def __init__(self, config: ConeNoise = ConeNoise(), *, seed: int = 1) -> None:
        if not all(0 <= value <= 1 for value in (
            config.misclassification_probability,
            config.missed_detection_probability,
        )):
            raise SimulationError("cone probabilities must lie in [0, 1]")
        self.config = config
        self.random = Random(seed)
        self.sequence = 0

    def detect(self, state: PlanarKartState, cones: Iterable[Cone]) -> tuple[ConeDetection, ...]:
        detections: list[ConeDetection] = []
        classes = tuple(ConeClass)
        for cone in sorted(cones, key=lambda item: item.cone_id):
            dx, dy = cone.x_m - state.x_m, cone.y_m - state.y_m
            distance = hypot(dx, dy)
            bearing = (atan2(dy, dx) - state.yaw_rad + 3.141592653589793) % (2 * 3.141592653589793) - 3.141592653589793
            if (
                not cone.visible
                or distance > self.config.maximum_range_m
                or abs(bearing) > self.config.field_of_view_rad / 2
                or self.random.random() < self.config.missed_detection_probability
            ):
                continue
            sigma = self.config.position_std_m + distance * self.config.range_std_gain
            noisy_x = dx + self.random.gauss(0, sigma)
            noisy_y = dy + self.random.gauss(0, sigma)
            semantic = cone.semantic_class
            if self.random.random() < self.config.misclassification_probability:
                semantic = classes[self.random.randrange(len(classes))]
            self.sequence += 1
            detections.append(ConeDetection(
                f"detection_{self.sequence:06d}",
                cone.cone_id,
                hypot(noisy_x, noisy_y),
                (atan2(noisy_y, noisy_x) - state.yaw_rad + 3.141592653589793) % (2 * 3.141592653589793) - 3.141592653589793,
                semantic,
                sigma,
                max(0.05, 1.0 - distance / self.config.maximum_range_m),
            ))
        expected_false = self.config.false_positive_rate
        if self.random.random() < expected_false:
            self.sequence += 1
            detections.append(ConeDetection(
                f"false_positive_{self.sequence:06d}",
                None,
                self.random.uniform(1.0, self.config.maximum_range_m),
                self.random.uniform(-self.config.field_of_view_rad / 2, self.config.field_of_view_rad / 2),
                ConeClass.UNKNOWN,
                0.5,
                0.2,
            ))
        return tuple(detections)


def boundary_cones(track: object, spacing_m: float = 3.0) -> tuple[Cone, ...]:
    """Generate deterministic blue-left/yellow-right cones from a Track."""
    if spacing_m <= 0:
        raise SimulationError("cone spacing must be positive")
    cones: list[Cone] = []
    count = max(2, int(track.length_m / spacing_m))
    for index in range(count + (0 if track.closed else 1)):
        s_m = min(track.length_m, index * track.length_m / count)
        point = track.sample(s_m)
        for side, lateral, semantic in (
            ("L", point.left_width_m, ConeClass.BLUE),
            ("R", -point.right_width_m, ConeClass.YELLOW),
        ):
            x_m, y_m, _ = track.world_from_track(s_m, lateral)
            cones.append(Cone(f"{side}_{index:04d}", x_m, y_m, point.heading_rad, semantic))
    return tuple(cones)

