from pathlib import Path

import pytest

from kart_sim.cones import Cone, ConeClass
from kart_sim.hp60c import ConeMap, HP60CConfig, HP60CSensor, load_hp60c_config
from kart_sim.types import PlanarKartState

ROOT = Path(__file__).parents[2]


def cone_at(range_m: float, *, cone_id: str = "c1") -> Cone:
    return Cone(cone_id, range_m, 0.0, 0.0, ConeClass.BLUE)


@pytest.mark.parametrize("range_m", [2.0, 3.0, 3.9])
def test_cone_within_datasheet_range_is_geometrically_visible(range_m: float) -> None:
    sensor = HP60CSensor(HP60CConfig(), camera_x_m=0.0, seed=1)
    frame = sensor.maybe_capture(1.0, PlanarKartState(), (0.0, 0.0), [cone_at(range_m)])
    assert frame is not None
    assert {d.cone_id for d in frame.detections} == {"c1"}


def test_cone_beyond_datasheet_max_range_is_not_visible() -> None:
    sensor = HP60CSensor(HP60CConfig(), camera_x_m=0.0, seed=1)
    frame = sensor.maybe_capture(1.0, PlanarKartState(), (0.0, 0.0), [cone_at(4.5)])
    assert frame is not None
    assert frame.detections == ()


def test_detection_recall_at_close_range_is_near_the_configured_peak() -> None:
    cfg = HP60CConfig()
    detected_count = 0
    trials = 500
    for seed in range(trials):
        sensor = HP60CSensor(cfg, camera_x_m=0.0, seed=seed)
        frame = sensor.maybe_capture(1.0, PlanarKartState(), (0.0, 0.0), [cone_at(2.0)])
        assert frame is not None and len(frame.detections) == 1
        detected_count += frame.detections[0].detected
    observed_recall = detected_count / trials
    assert observed_recall == pytest.approx(cfg.recall_peak, abs=0.05)


def test_frame_delivery_is_delayed_by_latency() -> None:
    cfg = HP60CConfig(latency_s=0.29, latency_jitter_s=0.0)
    sensor = HP60CSensor(cfg, camera_x_m=0.0, seed=1)
    frame = sensor.maybe_capture(1.0, PlanarKartState(), (0.0, 0.0), [cone_at(2.0)])
    assert frame is not None
    assert frame.deliver_time_s == pytest.approx(frame.capture_time_s + 0.29)


def test_sunlight_degradation_shrinks_effective_max_range() -> None:
    nominal = HP60CConfig(sunlight_degradation=False)
    degraded = HP60CConfig(sunlight_degradation=True)
    assert degraded.effective_zmax_m < nominal.effective_zmax_m


def test_hp60c_config_loads_measured_datasheet_values() -> None:
    config = load_hp60c_config(ROOT / "configs/sensors/hp60c.yaml")
    assert config.hfov_deg == 64.4
    assert config.vfov_deg == 51.7
    assert config.zmin_m == 0.20
    assert config.zmax_m == 4.0
    assert config.rate_hz == 4.2
    assert config.latency_s == 0.29


def _detection_frame(t_s: float, colour: str = "blue") -> "object":
    from kart_sim.hp60c import CameraDetection, CameraFrame
    return CameraFrame(t_s, t_s, (0.0, 0.0, 0.0), (
        CameraDetection("c1", 320.0, 240.0, 40.0, colour, 2.0, True, colour, 2.0, 0.0),
    ))


def _empty_frame(t_s: float) -> "object":
    from kart_sim.hp60c import CameraFrame
    return CameraFrame(t_s, t_s, (0.0, 0.0, 0.0), ())


def test_cone_map_confirms_after_two_hits_and_expires_after_ten_seconds() -> None:
    cone_map = ConeMap(association_radius_m=0.6, expiry_s=10.0)
    cone_map.update(_detection_frame(0.0))
    assert cone_map.confirmed_tracks() == []  # one hit is not yet confirmed
    cone_map.update(_detection_frame(0.1))
    assert len(cone_map.confirmed_tracks()) == 1
    assert cone_map.confirmed_tracks()[0].colour == "blue"
    # No further detections arrive; time passes past the last hit at t=0.1.
    cone_map.update(_empty_frame(10.2))
    assert cone_map.confirmed_tracks() == []
