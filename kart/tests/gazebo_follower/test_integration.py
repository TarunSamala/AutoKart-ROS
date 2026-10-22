from __future__ import annotations

import json
from pathlib import Path

from kart_sim.gazebo_full_lap import execute_full_lap


ROOT = Path(__file__).parents[2]


def test_headless_follower_completes_lap_and_controlled_stop(tmp_path: Path) -> None:
    output = tmp_path / "full_lap"
    result = execute_full_lap(
        ROOT / "configs/scenarios/gazebo_follower_full_lap.yaml",
        output,
        follower_hz=100.0,
    )
    assert result.completed
    assert result.controlled_stop_complete
    assert result.final_vcu_state == "SAFE"
    assert result.physics_steps > result.follower_updates > 100
    assert not result.faults
    assert result.double_physics_invariant_violations == 0
    manifest = json.loads((output / "run_manifest.json").read_text(encoding="utf-8"))
    assert manifest["physics_authority"] == "internal"
    assert manifest["gazebo_role"] == "visual_sensor_follower"
    assert len(list((output / "plots").glob("*.svg"))) == 12


def test_render_update_rate_does_not_mutate_short_internal_trajectory() -> None:
    from kart_sim.autonomy import PurePursuitController
    from kart_sim.fast_env import AutoKartFastEnv
    from kart_sim.gazebo_follower import (
        AuthoritativeSnapshotBuffer,
        AuthoritativeSnapshotFactory,
        ReadOnlyGazeboFollower,
    )
    from kart_sim.racing_autonomy import track_trajectory

    def run(follower_rate_hz: float | None) -> tuple[tuple[float, ...], ...]:
        env = AutoKartFastEnv(
            ROOT / "configs/tracks/research_circle.yaml",
            ROOT / "configs/karts/prototype_dynamic.yaml",
        )
        env.arm_autonomy(operator_acknowledged=True)
        trajectory = track_trajectory(env.track, timestamp_s=0.0, speed_mps=3.0, spacing_m=0.25)
        controller = PurePursuitController(
            lookahead_m=4.0, maximum_steering_rate_radps=0.5
        )
        if follower_rate_hz is not None:
            factory = AuthoritativeSnapshotFactory("b" * 64, env._epoch)
            buffer = AuthoritativeSnapshotBuffer()
            follower = ReadOnlyGazeboFollower(buffer)
            period = 1.0 / follower_rate_hz
            next_update = 0.0

            def observe(current: AutoKartFastEnv) -> None:
                nonlocal next_update
                from dataclasses import asdict
                state = factory.from_record({
                    "state": asdict(current.state),
                    "subsystems": current.kart.telemetry(),
                })
                follower.ingest(state)
                if state.simulation_time_s >= next_update:
                    follower.update(state.simulation_time_s)
                    next_update += period

            env.add_physics_observer(observe)
        result = []
        for _ in range(100):
            output = controller.compute(env.state, trajectory, 0.02)
            env.step((output.target_speed_mps, output.steering_rad))
            result.append((
                env.state.x_m, env.state.y_m, env.state.yaw_rad,
                env.state.u_mps, env.state.v_mps,
            ))
        return tuple(result)

    disabled = run(None)
    assert run(20.0) == disabled
    assert run(100.0) == disabled
    assert run(200.0) == disabled

