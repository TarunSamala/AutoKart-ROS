from pathlib import Path

import pytest

from kart_sim.config import load_kart_config
from kart_sim.four_wheel import FourWheelKart
from kart_sim.types import PlanarKartState, VehicleCommand

ROOT = Path(__file__).parents[2]


def model(state: PlanarKartState | None = None) -> FourWheelKart:
    return FourWheelKart(load_kart_config(ROOT / "configs/karts/prototype_four_wheel.yaml"), state)


def test_solid_rear_axle_reports_equal_rear_wheel_speed() -> None:
    kart = model(PlanarKartState(u_mps=3.0, yaw_rate_radps=0.5))
    kart.step(VehicleCommand(steering_rad=0.1))
    assert kart.wheels["rear_left"].angular_speed_radps == pytest.approx(kart.wheels["rear_right"].angular_speed_radps)


def test_solid_axle_creates_geometric_rear_scrub_in_turn() -> None:
    kart = model(PlanarKartState(u_mps=3.0, yaw_rate_radps=0.5))
    kart.step(VehicleCommand(steering_rad=0.1))
    assert kart.wheels["rear_left"].slip_ratio != pytest.approx(kart.wheels["rear_right"].slip_ratio)


def test_wheel_loads_are_never_negative_and_wheel_lift_is_exposed() -> None:
    kart = model(PlanarKartState(u_mps=2.0, ay_mps2=80.0))
    kart.step(VehicleCommand())
    assert all(wheel.vertical_load_n >= 0.0 for wheel in kart.wheels.values())
    assert any(not wheel.contact_valid for wheel in kart.wheels.values())
    assert kart.diagnostics.inside_rear_unloaded


def test_four_wheel_skidpad_step_remains_finite() -> None:
    kart = model(PlanarKartState(u_mps=3.0))
    for _ in range(500):
        kart.step(VehicleCommand(steering_rad=0.1))
    kart.state.assert_finite()
