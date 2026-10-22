from pathlib import Path

import pytest

from kart_sim.backends import AnalyticalBackend, create_backend, detect_chrono
from kart_sim.config import load_kart_config
from kart_sim.types import PlanarKartState, SimulationError, VehicleCommand

ROOT = Path(__file__).parents[2]


def test_analytical_backend_exposes_backend_neutral_state_wheels_and_snapshot():
    backend = AnalyticalBackend(load_kart_config(ROOT / "configs/karts/prototype_dynamic.yaml"), PlanarKartState())
    backend.step(VehicleCommand(drive=.2), .001)
    assert backend.name == "internal_analytical"
    assert len(backend.wheel_states()) == 4
    assert backend.snapshot()["backend"] == backend.name


def test_explicit_chrono_request_never_silently_falls_back():
    config = load_kart_config(ROOT / "configs/karts/prototype_dynamic.yaml")
    if not detect_chrono().available:
        with pytest.raises(SimulationError, match="unavailable"):
            create_backend("chrono_multibody", config)
