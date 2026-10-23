"""Shared SI-unit types used by the deterministic headless core."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


class SimulationError(RuntimeError):
    """Raised when a simulation invariant would be violated."""


@dataclass(frozen=True)
class VehicleCommand:
    """Actuator-level command after external validation; dimensionless inputs are [-1, 1]."""

    drive: float = 0.0
    brake: float = 0.0
    steering_rad: float = 0.0

    def validated(self) -> "VehicleCommand":
        if not all(isfinite(value) for value in (self.drive, self.brake, self.steering_rad)):
            raise SimulationError("command contains a non-finite value")
        if not -1.0 <= self.drive <= 1.0:
            raise SimulationError("drive command must be in [-1, 1]")
        if not 0.0 <= self.brake <= 1.0:
            raise SimulationError("brake command must be in [0, 1]")
        return self


@dataclass
class PlanarKartState:
    """World pose and body-frame planar state in SI units."""

    time_s: float = 0.0
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    u_mps: float = 0.0
    v_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    ax_mps2: float = 0.0
    ay_mps2: float = 0.0
    # The analytical dynamics remain planar. These explicit pose channels are
    # populated from the track surface frame when a scenario selects a Phase-5
    # track, and remain zero otherwise. Keeping them in the authoritative state
    # prevents visualization code from inventing a separate pose.
    z_m: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0

    def assert_finite(self) -> None:
        if not all(isfinite(value) for value in self.__dict__.values()):
            raise SimulationError("vehicle state contains NaN or infinity")
