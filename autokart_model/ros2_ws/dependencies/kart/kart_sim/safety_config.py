"""Schema-validated simulation-only VCU safety configuration."""
from __future__ import annotations
from pathlib import Path
from pydantic import BaseModel, Field
import yaml
from .types import SimulationError


class CommunicationSafetyConfig(BaseModel):
    maximum_age_s: float = Field(gt=0)
    timeout_s: float = Field(gt=0)
    counter_bits: int = Field(ge=2, le=32)


class StopSafetyConfig(BaseModel):
    target_deceleration_mps2: float = Field(gt=0)
    stopped_speed_threshold_mps: float = Field(ge=0)
    stopped_confirmation_s: float = Field(gt=0)
    timeout_s: float = Field(gt=0)


class SafetyEnvelopeConfig(BaseModel):
    fixed_margin_m: float = Field(ge=0)
    guaranteed_deceleration_mps2: float = Field(gt=0)
    maximum_autonomous_speed_mps: float = Field(gt=0)
    maximum_degraded_speed_mps: float = Field(ge=0)


class SafetyConfiguration(BaseModel):
    simulation_use_only: bool
    vcu_rate_hz: float = Field(gt=0)
    independent_safety_rate_hz: float = Field(gt=0)
    communication: CommunicationSafetyConfig
    controlled_stop: StopSafetyConfig
    safety_envelope: SafetyEnvelopeConfig
    automatic_autonomous_restart: bool = False


def load_safety_configuration(path: str | Path) -> SafetyConfiguration:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    try:
        config = SafetyConfiguration.model_validate(payload)
    except AttributeError:
        config = SafetyConfiguration.parse_obj(payload)
    if not config.simulation_use_only:
        raise SimulationError("safety configuration must be explicitly marked simulation-only")
    if config.automatic_autonomous_restart:
        raise SimulationError("automatic autonomous restart is prohibited")
    if config.communication.maximum_age_s > config.communication.timeout_s:
        raise SimulationError("command maximum age cannot exceed timeout")
    if config.safety_envelope.maximum_degraded_speed_mps > config.safety_envelope.maximum_autonomous_speed_mps:
        raise SimulationError("degraded speed cannot exceed autonomous speed")
    return config
