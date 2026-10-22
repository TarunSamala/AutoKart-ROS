"""Acceptance tests for the "shared" and "built" kart profiles (Prompt 1).

Validates: straight-line acceleration, steady-state turning radius against the
kinematic L/tan(delta) formula, brake stopping distance against
v*tau + v^2/(2a), and that switching profiles changes results in the expected
direction (shorter wheelbase -> tighter turning radius for the same steer).
"""

from math import tan
from pathlib import Path

import pytest

from kart_sim.config import load_kart_config
from kart_sim.types import PlanarKartState, VehicleCommand
from kart_sim.vehicle import PlanarBicycleKart

ROOT = Path(__file__).parents[2]
PROFILES = ("shared", "built")


def model(name: str) -> PlanarBicycleKart:
    config = load_kart_config(ROOT / "configs/karts" / f"{name}.yaml")
    config.environment.rolling_resistance_coefficient = 0.0
    config.environment.drag_coefficient_area_m2 = 0.0
    return PlanarBicycleKart(config)


@pytest.mark.parametrize("profile", PROFILES)
def test_straight_line_acceleration_matches_force_over_mass(profile: str) -> None:
    kart = model(profile)
    for _ in range(1_000):
        kart.step(VehicleCommand(drive=0.5))
    expected_speed = 0.5 * kart.config.motor.max_wheel_force_n / kart.config.kart.mass_kg
    assert kart.state.u_mps == pytest.approx(expected_speed, rel=2e-3)


@pytest.mark.parametrize("profile", PROFILES)
def test_steady_state_circle_matches_bicycle_radius_formula(profile: str) -> None:
    kart = model(profile)
    kart.state = PlanarKartState(u_mps=1.0)
    steer_rad = 0.03
    for _ in range(4_000):
        kart.step(VehicleCommand(steering_rad=steer_rad))
    radius_m = kart.state.u_mps / kart.state.yaw_rate_radps
    expected_radius_m = kart.config.kart.wheelbase_m / tan(steer_rad)
    assert radius_m == pytest.approx(expected_radius_m, rel=0.02)


@pytest.mark.parametrize("profile", PROFILES)
def test_brake_stop_distance_matches_stopping_distance_formula(profile: str) -> None:
    kart = model(profile)
    v0_mps = 2.0
    kart.state = PlanarKartState(u_mps=v0_mps)
    dt = kart.config.simulation.physics_dt_s
    decel_mps2 = kart.config.brake.max_brake_force_n / kart.config.kart.mass_kg
    brake_lag = kart.config.actuator_lag.brake
    # Effective actuation delay: the dead time, plus half the ramp-up duration
    # (average deceleration during a linear ramp is half the peak).
    tau_s = brake_lag.dead_time_s + 0.5 / brake_lag.ramp_rate_per_s
    expected_distance_m = v0_mps * tau_s + v0_mps**2 / (2 * decel_mps2)
    stop_time_s = tau_s + v0_mps / decel_mps2

    distance_m = 0.0
    for _ in range(int(stop_time_s / dt) + 500):
        speed_before = kart.state.u_mps
        kart.step(VehicleCommand(brake=1.0))
        distance_m += speed_before * dt

    assert kart.state.u_mps < 0.1
    assert distance_m == pytest.approx(expected_distance_m, rel=0.05)


def test_profile_switch_changes_turning_radius_in_expected_direction() -> None:
    """The "built" kart has a shorter wheelbase than "shared", so for the same
    steering angle and speed it must turn a tighter (smaller) circle."""
    steer_rad = 0.03
    radii_m = {}
    for profile in PROFILES:
        kart = model(profile)
        kart.state = PlanarKartState(u_mps=1.0)
        for _ in range(4_000):
            kart.step(VehicleCommand(steering_rad=steer_rad))
        radii_m[profile] = kart.state.u_mps / kart.state.yaw_rate_radps

    assert kart_shared_wheelbase_longer_than_built()
    assert radii_m["built"] < radii_m["shared"]


def kart_shared_wheelbase_longer_than_built() -> bool:
    shared_wheelbase_m = load_kart_config(ROOT / "configs/karts/shared.yaml").kart.wheelbase_m
    built_wheelbase_m = load_kart_config(ROOT / "configs/karts/built.yaml").kart.wheelbase_m
    return shared_wheelbase_m > built_wheelbase_m
