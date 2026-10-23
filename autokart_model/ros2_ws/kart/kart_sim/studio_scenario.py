"""Schema-driven reproducible scenario editor backend."""

from __future__ import annotations

from hashlib import sha256
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field, confloat, conint

from .sim_studio import load_preset
from .track_formats import validate_track_source
from .types import SimulationError


class FaultEvent(BaseModel):
    fault: str
    time_s: confloat(ge=0)
    duration_s: confloat(gt=0) | None = None
    target_namespace: str = "/kart_01"


class StudioScenario(BaseModel):
    name: str
    preset: str
    track: str
    kart: str = "configs/karts/prototype_dynamic.yaml"
    controller: str = "configs/controllers/baseline_racing.yaml"
    sensor_suite: str = "configs/sensors/planned_kart.yaml"
    seed: int = 1
    duration_s: confloat(gt=0) = 60.0
    start: dict[str, float] = Field(default_factory=lambda: {"s_m": 0.0, "lateral_m": 0.0})
    target_speed_mps: confloat(ge=0, le=15) = 4.0
    weather: dict[str, Any] = Field(default_factory=lambda: {"condition": "clear", "temperature_c": 25.0})
    obstacles: list[dict[str, Any]] = Field(default_factory=list)
    faults: list[FaultEvent] = Field(default_factory=list)
    agents: conint(ge=1, le=32) = 1
    session_rules: dict[str, Any] = Field(default_factory=dict)
    logging: dict[str, Any] = Field(default_factory=lambda: {"telemetry": True, "sensors": False})


def validate_studio_scenario(path: str | Path, *, root: str | Path = ".") -> dict[str, Any]:
    workspace = Path(root)
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    scenario = StudioScenario.model_validate(payload) if hasattr(StudioScenario, "model_validate") else StudioScenario.parse_obj(payload)
    preset_path = workspace / scenario.preset
    track_path = workspace / scenario.track
    for required in (preset_path, track_path, workspace / scenario.kart, workspace / scenario.controller, workspace / scenario.sensor_suite):
        if not required.is_file():
            raise SimulationError(f"scenario dependency missing: {required}")
    preset = load_preset(preset_path)
    if not (preset.vcu_enabled and preset.independent_safety_enabled):
        raise SimulationError("integrated scenario requires VCU and independent safety")
    if preset.real_actuator_outputs_enabled:
        raise SimulationError("scenario cannot enable real actuator output")
    if scenario.agents > 1 and scenario.start.get("grid_spacing_m", 1.5) < 1.0:
        raise SimulationError("multi-kart grid spacing is unsafe/overlapping")
    normalized = scenario.model_dump(mode="json") if hasattr(scenario, "model_dump") else scenario.dict()
    canonical = yaml.safe_dump(normalized, sort_keys=True)
    return {
        "scenario": normalized,
        "scenario_sha256": sha256(canonical.encode("utf-8")).hexdigest(),
        "preset": preset.name,
        "physics_authority": preset.physics_backend.value,
        "track": validate_track_source(track_path),
        "valid": True,
    }

