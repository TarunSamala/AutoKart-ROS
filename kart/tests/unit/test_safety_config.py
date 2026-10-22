from pathlib import Path
import pytest
from kart_sim.safety_config import load_safety_configuration
from kart_sim.types import SimulationError

ROOT = Path(__file__).parents[2]


def test_safety_configuration_validates_and_prohibits_auto_restart(tmp_path):
    config = load_safety_configuration(ROOT / "configs/safety/prototype_safety.yaml")
    assert config.vcu_rate_hz == 250
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text((ROOT / "configs/safety/prototype_safety.yaml").read_text().replace(
        "automatic_autonomous_restart: false", "automatic_autonomous_restart: true"
    ))
    with pytest.raises(SimulationError, match="restart"):
        load_safety_configuration(invalid)
