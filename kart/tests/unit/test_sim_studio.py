from pathlib import Path

import pytest

from kart_sim.sim_studio import (
    AuthorityRegistry,
    GazeboRole,
    GazeboVisualFollower,
    PhysicsAuthority,
    VisualState,
    load_preset,
)
from kart_sim.types import SimulationError


ROOT = Path(__file__).parents[2]


def test_all_presets_are_safe_and_explicit() -> None:
    presets = [load_preset(path) for path in sorted((ROOT / "configs/presets").glob("*.yaml"))]
    assert len(presets) >= 14
    assert all(preset.vcu_enabled and preset.independent_safety_enabled for preset in presets)
    assert all(not preset.real_actuator_outputs_enabled for preset in presets)
    gazebo = next(preset for preset in presets if preset.name == "gazebo_sensor_development")
    assert gazebo.physics_backend == PhysicsAuthority.INTERNAL
    assert gazebo.gazebo_role == GazeboRole.VISUAL_FOLLOWER


def test_authority_registry_rejects_double_physics_and_visual_commands() -> None:
    registry = AuthorityRegistry()
    lease = registry.acquire(PhysicsAuthority.INTERNAL, "/kart_01", 1, 0.001, "internal_core")
    registry.assert_owner(lease, timestamp_s=0.0)
    with pytest.raises(SimulationError, match="double physics"):
        registry.acquire(PhysicsAuthority.GAZEBO, "/kart_01", 1, 0.001, "gazebo_plugin")
    follower = GazeboVisualFollower(lease)
    follower.accept(VisualState(1, 0.0, "internal_core", (0, 0, 0, 0), {"rear_solid_axle_joint": 0.0}))
    with pytest.raises(SimulationError, match="prohibited actuator"):
        follower.accept(VisualState(1, 0.1, "internal_core", (0, 0, 0, 0), {"motor_command": 1.0}))
