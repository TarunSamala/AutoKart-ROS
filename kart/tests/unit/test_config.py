from pathlib import Path

import pytest

from kart_sim.config import load_kart_config


ROOT = Path(__file__).parents[2]


def test_provisional_kart_configuration_is_valid() -> None:
    config = load_kart_config(ROOT / "configs/karts/prototype.yaml")
    assert config.kart.mass_kg == 145.0
    assert config.simulation.physics_dt_s == 0.001


def test_configuration_rejects_negative_mass(tmp_path: Path) -> None:
    invalid = (ROOT / "configs/karts/prototype.yaml").read_text(encoding="utf-8").replace("mass_kg: 145.0", "mass_kg: -1.0")
    target = tmp_path / "invalid.yaml"
    target.write_text(invalid, encoding="utf-8")
    with pytest.raises(ValueError, match="mass_kg"):
        load_kart_config(target)
