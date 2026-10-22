from pathlib import Path

import pytest

from kart_sim.config import load_kart_config
from kart_sim.types import PlanarKartState, VehicleCommand
from kart_sim.vehicle import PlanarBicycleKart

ROOT = Path(__file__).parents[2]


def model() -> PlanarBicycleKart:
    return PlanarBicycleKart(load_kart_config(ROOT / "configs/karts/prototype.yaml"))


def test_stationary_equilibrium_has_no_spurious_motion() -> None:
    kart = model()
    for _ in range(1_000):
        kart.step(VehicleCommand())
    assert kart.state.u_mps == pytest.approx(0.0, abs=1e-12)
    assert kart.state.v_mps == pytest.approx(0.0, abs=1e-12)
    assert kart.state.yaw_rate_radps == pytest.approx(0.0, abs=1e-12)


def test_zero_force_constant_velocity() -> None:
    kart = model()
    kart.config.environment.rolling_resistance_coefficient = 0.0
    kart.config.environment.drag_coefficient_area_m2 = 0.0
    kart.state = PlanarKartState(u_mps=4.0)
    for _ in range(1_000):
        kart.step(VehicleCommand())
    assert kart.state.u_mps == pytest.approx(4.0, abs=1e-10)
    assert kart.state.x_m == pytest.approx(4.0, abs=0.01)


def test_constant_longitudinal_force_accelerates_forward() -> None:
    kart = model()
    kart.config.environment.rolling_resistance_coefficient = 0.0
    kart.config.environment.drag_coefficient_area_m2 = 0.0
    for _ in range(1_000):
        kart.step(VehicleCommand(drive=0.5))
    expected_speed = 0.5 * kart.config.motor.max_wheel_force_n / kart.config.kart.mass_kg
    assert kart.state.u_mps == pytest.approx(expected_speed, rel=2e-3)


def test_braking_opposes_forward_motion() -> None:
    kart = model()
    kart.config.environment.rolling_resistance_coefficient = 0.0
    kart.config.environment.drag_coefficient_area_m2 = 0.0
    kart.state = PlanarKartState(u_mps=5.0)
    kart.step(VehicleCommand(brake=1.0))
    assert kart.state.ax_mps2 < 0.0


def test_constant_radius_response_turns_left_for_positive_steer() -> None:
    kart = model()
    kart.config.environment.rolling_resistance_coefficient = 0.0
    kart.config.environment.drag_coefficient_area_m2 = 0.0
    kart.state = PlanarKartState(u_mps=3.0)
    for _ in range(2_000):
        kart.step(VehicleCommand(steering_rad=0.1))
    assert kart.state.y_m > 0.2
    assert kart.state.yaw_rad > 0.0
