"""Namespaced multi-kart fast racing orchestration."""

from __future__ import annotations

from math import hypot
from pathlib import Path
from typing import Any, Iterable

from .fast_env import AutoKartFastEnv
from .race_control import RaceControl
from .types import SimulationError


class MultiKartRace:
    def __init__(
        self,
        track: str | Path,
        kart: str | Path,
        *,
        agent_count: int = 2,
        control_dt_s: float = 0.02,
        collision_radius_m: float = 0.55,
    ) -> None:
        if not 1 <= agent_count <= 32:
            raise SimulationError("multi-kart race supports 1 through 32 karts")
        self.environments = tuple(
            AutoKartFastEnv(
                track,
                kart,
                control_dt_s=control_dt_s,
                namespace=f"/kart_{index + 1:02d}",
            )
            for index in range(agent_count)
        )
        self.collision_radius_m = collision_radius_m
        self.race_control = RaceControl(tuple(env.namespace for env in self.environments))
        self._previous_s = {env.namespace: 0.0 for env in self.environments}

    def reset(self, *, seed: int = 1) -> dict[str, tuple[dict, dict]]:
        result = {}
        for index, env in enumerate(self.environments):
            observation, info = env.reset(
                seed=seed + index,
                options={"arm": True, "operator_acknowledged": True},
            )
            # Deterministic staggered grid along the source centreline.
            grid_s = index * 1.5
            point = env.track.sample(grid_s)
            env.kart.state.x_m, env.kart.state.y_m = point.x_m, point.y_m
            env.kart.state.yaw_rad = point.heading_rad
            env.projection = env.track.project(point.x_m, point.y_m, point.z_m)
            self._previous_s[env.namespace] = env.projection.s_m
            result[env.namespace] = (observation, info)
        return result

    def step(self, actions: dict[str, Any]) -> dict[str, tuple[dict, float, bool, bool, dict]]:
        if set(actions) != {env.namespace for env in self.environments}:
            raise SimulationError("multi-kart action namespaces do not match spawned karts")
        for env in self.environments:
            env.set_local_actors(
                (
                    (other.state.x_m, other.state.y_m, self.collision_radius_m, 0.7)
                    for other in self.environments
                    if other is not env
                )
            )
        results = {env.namespace: env.step(actions[env.namespace]) for env in self.environments}
        collisions: set[str] = set()
        for index, env in enumerate(self.environments):
            for other in self.environments[index + 1:]:
                if hypot(env.state.x_m - other.state.x_m, env.state.y_m - other.state.y_m) <= 2 * self.collision_radius_m:
                    collisions.update((env.namespace, other.namespace))
        for env in self.environments:
            result = list(results[env.namespace])
            if env.namespace in collisions:
                result[1] -= 10.0
                result[2] = True
                result[4] = {**result[4], "collision": True}
                self.race_control.penalty(env.namespace, "collision", env.state.time_s)
            previous = self._previous_s[env.namespace]
            current = env.projection.s_m
            if env.track.closed and previous > 0.8 * env.track.length_m and current < 0.2 * env.track.length_m:
                self.race_control.lap_crossing(env.namespace, env.state.time_s)
            self._previous_s[env.namespace] = current
            self.race_control.update_safety(
                env.namespace,
                env.vcu.state.value,
                tuple(env.vcu.faults),
                env.vcu.state.value == "EMERGENCY_STOP",
            )
            results[env.namespace] = tuple(result)
        snapshot = self.race_control.snapshot()
        for namespace, result_tuple in tuple(results.items()):
            result = list(result_tuple)
            result[4] = {**result[4], "race_control": snapshot}
            results[namespace] = tuple(result)
        return results
