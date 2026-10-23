"""Backend-neutral vehicle physics interface and optional Project Chrono gate."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from importlib.util import find_spec
from pathlib import Path
from typing import Protocol
import yaml

from .config import FullKartConfig
from .dynamic_kart import DynamicFourWheelKart
from .types import PlanarKartState, SimulationError, VehicleCommand


@dataclass(frozen=True)
class WheelPhysicsState:
    wheel_id: str
    normal_load_n: float
    longitudinal_force_n: float
    lateral_force_n: float
    angular_speed_radps: float


class VehiclePhysicsBackend(Protocol):
    name: str
    def reset(self, state: PlanarKartState) -> None: ...
    def step(self, command: VehicleCommand, dt_s: float, *, motor_enable: bool = True, contactor_open: bool = False) -> PlanarKartState: ...
    def chassis_state(self) -> PlanarKartState: ...
    def wheel_states(self) -> tuple[WheelPhysicsState, ...]: ...
    def diagnostics(self) -> dict: ...
    def snapshot(self) -> dict: ...


class AnalyticalBackend:
    """Adapter retaining the validated dynamic analytical kart unchanged."""
    name = "internal_analytical"

    def __init__(self, configuration: FullKartConfig, initial_state: PlanarKartState | None = None) -> None:
        self.kart = DynamicFourWheelKart(configuration, initial_state or PlanarKartState())

    def reset(self, state: PlanarKartState) -> None:
        self.kart = DynamicFourWheelKart(self.kart.config, state)

    def step(self, command: VehicleCommand, dt_s: float, *, motor_enable: bool = True, contactor_open: bool = False) -> PlanarKartState:
        return self.kart.step(command, dt_s, motor_enable=motor_enable, contactor_open=contactor_open)

    def chassis_state(self) -> PlanarKartState:
        return self.kart.state

    def wheel_states(self) -> tuple[WheelPhysicsState, ...]:
        return tuple(WheelPhysicsState(name, wheel.vertical_load_n, wheel.longitudinal_force_n,
                                       wheel.lateral_force_n, wheel.angular_speed_radps)
                     for name, wheel in self.kart.wheels.items())

    def diagnostics(self) -> dict:
        return self.kart.telemetry()

    def snapshot(self) -> dict:
        return {"backend": self.name, "state": asdict(self.kart.state), "diagnostics": self.diagnostics()}


@dataclass(frozen=True)
class ChronoAvailability:
    available: bool
    module: str | None
    detail: str


def detect_chrono() -> ChronoAvailability:
    for name in ("pychrono", "chrono"):
        if find_spec(name) is not None:
            return ChronoAvailability(True, name, "Python bindings detected; native adapter validation required")
    return ChronoAvailability(False, None, "Project Chrono Python bindings were not found")


class ChronoMultibodyBackend:
    """Explicit optional-backend placeholder; never silently falls back."""
    name = "chrono_multibody"

    def __init__(self, configuration: FullKartConfig, initial_state: PlanarKartState | None = None, *, fidelity: str = "smoke") -> None:
        availability = detect_chrono()
        if not availability.available:
            raise SimulationError("chrono_multibody was explicitly requested but Project Chrono bindings are unavailable")
        raise SimulationError("Project Chrono bindings were detected but this repository has no validated native adapter yet")


def create_backend(name: str, configuration: FullKartConfig, initial_state: PlanarKartState | None = None) -> VehiclePhysicsBackend:
    if name == "internal_analytical":
        return AnalyticalBackend(configuration, initial_state)
    if name == "chrono_multibody":
        return ChronoMultibodyBackend(configuration, initial_state)
    raise SimulationError(f"unknown physics backend: {name}")


@dataclass(frozen=True)
class BackendConfiguration:
    backend: str
    fidelity: str
    allow_fallback: bool = False


def load_backend_configuration(path: str | Path) -> BackendConfiguration:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    configuration = BackendConfiguration(
        str(payload.get("backend", "")),
        str(payload.get("fidelity", "")),
        bool(payload.get("allow_fallback", False)),
    )
    if configuration.backend not in {"internal_analytical", "chrono_multibody"}:
        raise SimulationError("unknown configured physics backend")
    if configuration.fidelity not in {"smoke", "realtime", "validation"}:
        raise SimulationError("unknown backend fidelity")
    if configuration.backend == "chrono_multibody" and configuration.allow_fallback:
        raise SimulationError("explicit Chrono mode cannot silently fall back")
    return configuration
