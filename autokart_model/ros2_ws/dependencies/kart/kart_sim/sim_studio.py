"""Platform-level mode selection and single-physics-authority enforcement."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import yaml

from .types import SimulationError


class SimulationMode(str, Enum):
    FAST = "fast"
    ENGINEERING = "engineering"
    GAZEBO = "gazebo"
    FORMULA_STUDENT = "formula_student"
    SIL_HIL = "sil_hil"


class PhysicsAuthority(str, Enum):
    INTERNAL = "internal"
    CHRONO = "chrono"
    GAZEBO = "gazebo"


class GazeboRole(str, Enum):
    DISABLED = "disabled"
    VISUAL_FOLLOWER = "visual_follower"
    AUTHORITY = "authority"


@dataclass(frozen=True)
class AuthorityLease:
    backend: PhysicsAuthority
    namespace: str
    epoch: int
    fixed_step_s: float
    owner_id: str


class AuthorityRegistry:
    """Reject a second integrator for one kart namespace and reset epoch."""

    def __init__(self) -> None:
        self._leases: dict[str, AuthorityLease] = {}

    def acquire(
        self,
        backend: PhysicsAuthority,
        namespace: str,
        epoch: int,
        fixed_step_s: float,
        owner_id: str,
    ) -> AuthorityLease:
        if not namespace.startswith("/") or not owner_id or epoch < 0 or fixed_step_s <= 0:
            raise SimulationError("invalid physics authority lease")
        requested = AuthorityLease(backend, namespace.rstrip("/") or "/", epoch, fixed_step_s, owner_id)
        existing = self._leases.get(requested.namespace)
        if existing is not None and existing != requested:
            raise SimulationError(
                f"double physics prohibited for {requested.namespace}: "
                f"{existing.owner_id}/{existing.backend.value} already owns integration"
            )
        self._leases[requested.namespace] = requested
        return requested

    def release(self, lease: AuthorityLease) -> None:
        if self._leases.get(lease.namespace) != lease:
            raise SimulationError("only the current owner may release a physics lease")
        del self._leases[lease.namespace]

    def assert_owner(self, lease: AuthorityLease, *, timestamp_s: float) -> None:
        if timestamp_s < 0 or self._leases.get(lease.namespace) != lease:
            raise SimulationError("physics step attempted without the active authority lease")


@dataclass(frozen=True)
class StudioPreset:
    name: str
    mode: SimulationMode
    physics_backend: PhysicsAuthority
    gazebo_role: GazeboRole
    fidelity_tier: int
    sensor_suite: str
    vcu_enabled: bool
    independent_safety_enabled: bool
    rendering_enabled: bool
    ros_enabled: bool
    dashboard_enabled: bool
    real_actuator_outputs_enabled: bool
    expected_performance: str
    dependencies: tuple[str, ...]

    def validate(self) -> None:
        valid = {
            (PhysicsAuthority.INTERNAL, GazeboRole.DISABLED),
            (PhysicsAuthority.INTERNAL, GazeboRole.VISUAL_FOLLOWER),
            (PhysicsAuthority.CHRONO, GazeboRole.DISABLED),
            (PhysicsAuthority.CHRONO, GazeboRole.VISUAL_FOLLOWER),
            (PhysicsAuthority.GAZEBO, GazeboRole.AUTHORITY),
        }
        if (self.physics_backend, self.gazebo_role) not in valid:
            raise SimulationError("invalid physics authority/Gazebo role combination")
        if self.fidelity_tier not in range(5):
            raise SimulationError("fidelity tier must be 0 through 4")
        if self.mode != SimulationMode.ENGINEERING and not (
            self.vcu_enabled and self.independent_safety_enabled
        ):
            raise SimulationError("integrated modes require VCU and independent safety")
        if self.real_actuator_outputs_enabled:
            raise SimulationError("repository presets must keep real actuator outputs disabled")
        if self.physics_backend == PhysicsAuthority.GAZEBO and "gazebo_system_plugin_reviewed" not in self.dependencies:
            raise SimulationError("Gazebo authority requires a reviewed custom system plugin")


def load_preset(path: str | Path) -> StudioPreset:
    payload: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    simulation = payload["simulation"]
    features = payload["features"]
    preset = StudioPreset(
        name=str(payload["name"]),
        mode=SimulationMode(str(payload["mode"])),
        physics_backend=PhysicsAuthority(str(simulation["physics_backend"])),
        gazebo_role=GazeboRole(str(simulation.get("gazebo_role", "disabled"))),
        fidelity_tier=int(simulation["fidelity_tier"]),
        sensor_suite=str(features["sensor_suite"]),
        vcu_enabled=bool(features["vcu_enabled"]),
        independent_safety_enabled=bool(features["independent_safety_enabled"]),
        rendering_enabled=bool(features["rendering_enabled"]),
        ros_enabled=bool(features["ros_enabled"]),
        dashboard_enabled=bool(features["dashboard_enabled"]),
        real_actuator_outputs_enabled=bool(features.get("real_actuator_outputs_enabled", False)),
        expected_performance=str(payload["expected_performance"]),
        dependencies=tuple(map(str, payload.get("dependencies", []))),
    )
    preset.validate()
    return preset


@dataclass(frozen=True)
class VisualState:
    epoch: int
    timestamp_s: float
    owner_id: str
    pose_xyz_yaw: tuple[float, float, float, float]
    joint_positions: dict[str, float]


class GazeboVisualFollower:
    """Read-only state sink. It intentionally has no actuator command method."""

    def __init__(self, lease: AuthorityLease) -> None:
        if lease.backend == PhysicsAuthority.GAZEBO:
            raise SimulationError("Gazebo authority cannot also be a visual follower")
        self.lease = lease
        self.last_state: VisualState | None = None

    def accept(self, state: VisualState) -> None:
        if state.epoch != self.lease.epoch or state.owner_id != self.lease.owner_id:
            raise SimulationError("visual state authority or reset epoch mismatch")
        if self.last_state is not None and state.timestamp_s < self.last_state.timestamp_s:
            raise SimulationError("visual state time moved backwards")
        forbidden = {"motor_command", "brake_command", "steering_command"}
        if forbidden.intersection(state.joint_positions):
            raise SimulationError("visual follower input contains a prohibited actuator command")
        self.last_state = state

