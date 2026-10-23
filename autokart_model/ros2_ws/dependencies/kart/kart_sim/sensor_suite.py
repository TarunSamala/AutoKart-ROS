"""Configurable deterministic racing sensor-suite models."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin
from random import Random
from typing import Iterable

from .cones import Cone
from .sensors import AsyncSensor
from .types import SimulationError


@dataclass(frozen=True)
class LidarConfiguration:
    beams: int = 108
    field_of_view_rad: float = 4.71238898038
    minimum_range_m: float = 0.05
    maximum_range_m: float = 30.0
    range_noise_std_m: float = 0.01
    dropout_probability: float = 0.0
    motion_distortion: bool = True
    fog_attenuation: float = 0.0


class PlanarLidar(AsyncSensor):
    """2D LiDAR against track corridor and circular actors.

    This is the Tier-1 deterministic scan model. Gazebo-rendered ray sensors
    use the same timing envelope but are an external Tier-3 implementation.
    """

    def __init__(self, sensor_id: str, frame_id: str, rate_hz: float, config: LidarConfiguration, **kwargs: object) -> None:
        if config.beams < 2 or config.minimum_range_m < 0 or config.maximum_range_m <= config.minimum_range_m:
            raise SimulationError("invalid LiDAR geometry")
        super().__init__(sensor_id, "lidar", frame_id, rate_hz, **kwargs)
        self.config = config

    def measure(self, truth: dict) -> dict:
        left = float(truth.get("left_margin_m", self.config.maximum_range_m))
        right = float(truth.get("right_margin_m", self.config.maximum_range_m))
        actors: Iterable[tuple[float, float, float, float]] = truth.get("actors_local", ())
        ranges: list[float] = []
        intensities: list[float] = []
        for index in range(self.config.beams):
            angle = -self.config.field_of_view_rad / 2 + self.config.field_of_view_rad * index / (self.config.beams - 1)
            lateral_ray = sin(angle)
            distance = self.config.maximum_range_m
            intensity = 0.2
            if lateral_ray > 1e-9:
                distance = min(distance, left / lateral_ray)
            elif lateral_ray < -1e-9:
                distance = min(distance, right / -lateral_ray)
            for x_m, y_m, radius_m, reflectivity in actors:
                along = x_m * cos(angle) + y_m * sin(angle)
                cross = -x_m * sin(angle) + y_m * cos(angle)
                if along > 0 and abs(cross) <= radius_m:
                    distance = min(distance, max(0.0, along - (radius_m * radius_m - cross * cross) ** 0.5))
                    intensity = reflectivity
                else:
                    intensity = 0.2
            sigma = self.config.range_noise_std_m * (1.0 + distance * 0.02)
            measured = max(self.config.minimum_range_m, min(self.config.maximum_range_m, distance + self.random.gauss(0, sigma)))
            if self.random.random() < self.config.dropout_probability:
                measured = float("inf")
                intensity = 0.0
            fog_loss = max(0.0, 1.0 - self.config.fog_attenuation * min(distance, self.config.maximum_range_m))
            ranges.append(measured)
            intensities.append(intensity * fog_loss)
        return {
            "ranges_m": ranges,
            "intensities": intensities,
            "angle_min_rad": -self.config.field_of_view_rad / 2,
            "angle_max_rad": self.config.field_of_view_rad / 2,
            "range_min_m": self.config.minimum_range_m,
            "range_max_m": self.config.maximum_range_m,
        }


@dataclass(frozen=True)
class CameraConfiguration:
    width: int = 640
    height: int = 480
    horizontal_fov_rad: float = 1.3962634
    exposure_s: float = 0.005
    rolling_shutter_s: float = 0.0
    distortion: tuple[float, float, float, float, float] = (0, 0, 0, 0, 0)
    motion_blur: bool = True


class SemanticCamera(AsyncSensor):
    """Headless camera contract with visible semantic features, not fake pixels."""

    def __init__(self, sensor_id: str, frame_id: str, rate_hz: float, config: CameraConfiguration, **kwargs: object) -> None:
        if config.width <= 0 or config.height <= 0 or not 0 < config.horizontal_fov_rad < pi:
            raise SimulationError("invalid camera configuration")
        super().__init__(sensor_id, "camera", frame_id, rate_hz, acquisition_s=config.exposure_s, **kwargs)
        self.config = config

    def measure(self, truth: dict) -> dict:
        features = tuple(truth.get("visible_features", ()))
        return {
            "encoding": "semantic_feature_frame",
            "width": self.config.width,
            "height": self.config.height,
            "features": features,
            "intrinsics": {
                "fx": self.config.width / (2.0 * __import__("math").tan(self.config.horizontal_fov_rad / 2)),
                "fy": self.config.width / (2.0 * __import__("math").tan(self.config.horizontal_fov_rad / 2)),
                "cx": self.config.width / 2,
                "cy": self.config.height / 2,
            },
            "distortion": self.config.distortion,
            "rendered_pixels_available": False,
        }


@dataclass(frozen=True)
class DepthConfiguration:
    minimum_range_m: float = 0.2
    maximum_range_m: float = 20.0
    quantization_m: float = 0.001
    invalid_probability: float = 0.0
    alignment_error_px: float = 0.0


class DepthSampler:
    def __init__(self, config: DepthConfiguration, *, seed: int = 1) -> None:
        self.config = config
        self.random = Random(seed)

    def sample(self, depths_m: Iterable[float]) -> tuple[float | None, ...]:
        result: list[float | None] = []
        for depth in depths_m:
            if (
                depth < self.config.minimum_range_m
                or depth > self.config.maximum_range_m
                or self.random.random() < self.config.invalid_probability
            ):
                result.append(None)
                continue
            noisy = depth + self.random.gauss(0, 0.002 + 0.001 * depth)
            result.append(round(noisy / self.config.quantization_m) * self.config.quantization_m)
        return tuple(result)
