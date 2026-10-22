"""Tire-force interfaces used by the internal analytical backends."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from .config import TireConfig


@dataclass(frozen=True)
class TireInput:
    vertical_load_n: float
    slip_ratio: float
    slip_angle_rad: float
    surface_friction_scale: float


@dataclass(frozen=True)
class TireForce:
    longitudinal_n: float
    lateral_n: float
    utilisation: float


class TireModel:
    """Common internal contract; model inputs and outputs are all SI."""

    def force(self, request: TireInput) -> TireForce:
        raise NotImplementedError


class LinearFrictionCircleTire(TireModel):
    """Bounded linear tire force with friction-circle combined-slip saturation."""

    def __init__(self, config: TireConfig) -> None:
        self.config = config

    def force(self, request: TireInput) -> TireForce:
        load = max(0.0, request.vertical_load_n)
        limit = self.config.friction_peak * request.surface_friction_scale * load
        raw_longitudinal = self.config.longitudinal_stiffness_n * request.slip_ratio
        raw_lateral = -self.config.cornering_stiffness_n_per_rad * request.slip_angle_rad
        raw_magnitude = sqrt(raw_longitudinal**2 + raw_lateral**2)
        if limit <= 0.0 or raw_magnitude <= 1e-12:
            return TireForce(0.0, 0.0, 0.0)
        scale = min(1.0, limit / raw_magnitude)
        longitudinal, lateral = raw_longitudinal * scale, raw_lateral * scale
        utilisation = sqrt(longitudinal**2 + lateral**2) / limit
        return TireForce(longitudinal, lateral, utilisation)
