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


@pytest.mark.parametrize("profile", ["shared", "built"])
def test_kart_profile_dimensions_match_the_dimension_sheet(profile: str) -> None:
    config = load_kart_config(ROOT / "configs/karts" / f"{profile}.yaml")
    assert config.kart.length_m == 2.400
    assert config.kart.width_m == 1.181
    assert config.kart.height_m == 1.118
    assert config.kart.ground_clearance_m == 0.381
    assert config.kart.rear_track_m == 1.1811


def test_shared_and_built_profiles_differ_only_in_wheelbase_and_front_track() -> None:
    shared = load_kart_config(ROOT / "configs/karts/shared.yaml")
    built = load_kart_config(ROOT / "configs/karts/built.yaml")
    assert shared.kart.wheelbase_m == pytest.approx(1.422)
    assert built.kart.wheelbase_m == pytest.approx(1.1176)
    assert shared.kart.front_track_m == pytest.approx(1.0648)
    assert built.kart.front_track_m == pytest.approx(1.0668)
    assert shared.kart.mass_kg == built.kart.mass_kg
    assert shared.kart.length_m == built.kart.length_m


def test_footprint_must_sum_to_length(tmp_path: Path) -> None:
    invalid = (ROOT / "configs/karts/shared.yaml").read_text(encoding="utf-8").replace(
        "front_extent_m: 2.00", "front_extent_m: 1.50"
    )
    target = tmp_path / "invalid.yaml"
    target.write_text(invalid, encoding="utf-8")
    with pytest.raises(ValueError, match="rear_overhang_m \\+ front_extent_m"):
        load_kart_config(target)


def test_mechanical_steering_limit_must_not_be_tighter_than_software_limit(tmp_path: Path) -> None:
    invalid = (ROOT / "configs/karts/shared.yaml").read_text(encoding="utf-8").replace(
        "mechanical_max_road_wheel_angle_rad: 0.4363323130",
        "mechanical_max_road_wheel_angle_rad: 0.1",
    )
    target = tmp_path / "invalid.yaml"
    target.write_text(invalid, encoding="utf-8")
    with pytest.raises(ValueError, match="mechanical_max_road_wheel_angle_rad"):
        load_kart_config(target)


def test_actuator_lag_defaults_are_effectively_instantaneous() -> None:
    config = load_kart_config(ROOT / "configs/karts/prototype.yaml")
    assert config.actuator_lag.steering.lag_s == 0.0
    assert config.actuator_lag.brake.dead_time_s == 0.0


def test_my_track_profiles_set_assumed_actuator_lag() -> None:
    for profile in ("shared", "built"):
        config = load_kart_config(ROOT / "configs/karts" / f"{profile}.yaml")
        assert config.actuator_lag.steering.lag_s == pytest.approx(0.12)
        assert config.actuator_lag.brake.dead_time_s == pytest.approx(0.15)
