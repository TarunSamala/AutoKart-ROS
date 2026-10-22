import pytest

from kart_sim.config import TireConfig
from kart_sim.tires import LinearFrictionCircleTire, TireInput


def test_combined_tire_force_never_exceeds_friction_limit() -> None:
    model = LinearFrictionCircleTire(TireConfig(
        model="linear_friction_circle", radius_m=0.14, friction_peak=1.0,
        cornering_stiffness_n_per_rad=8500.0, longitudinal_stiffness_n=9000.0,
    ))
    force = model.force(TireInput(vertical_load_n=500.0, slip_ratio=2.0, slip_angle_rad=1.0, surface_friction_scale=0.7))
    assert force.utilisation == pytest.approx(1.0)
    assert force.longitudinal_n**2 + force.lateral_n**2 <= (350.0 + 1e-10) ** 2


def test_unloaded_tire_generates_no_force() -> None:
    model = LinearFrictionCircleTire(TireConfig(
        model="linear_friction_circle", radius_m=0.14, friction_peak=1.0,
        cornering_stiffness_n_per_rad=8500.0, longitudinal_stiffness_n=9000.0,
    ))
    assert model.force(TireInput(0.0, 0.1, 0.1, 1.0)).utilisation == 0.0
