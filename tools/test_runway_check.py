import pytest

from runway_check import calculate


def test_runway_ladder_uses_bumper_geometry_and_measured_stop_terms():
    result = calculate(1.0, 1.0, 0.2, 0.15, 4.0, camera_to_bumper_m=1.0)
    assert result["max_bumper_to_cone_at_first_detection_m"] == 3.0
    assert result["stopping_distance_m"] == pytest.approx(0.85)
    assert [row["stop_gap_m"] for row in result["rows"]] == [1.0, 0.6, 0.4, 0.25, 0.15]
    assert result["rows"][0]["runway_m"] == pytest.approx(1.15)
    assert all(not row["less_than_0_5_m"] for row in result["rows"])


def test_short_runway_and_bad_inputs():
    result = calculate(1.5, 1.0, 0.2, 0.2, 4.0, camera_to_bumper_m=1.0668)
    assert result["rows"][0]["less_than_0_5_m"]
    with pytest.raises(ValueError):
        calculate(float("nan"), 1.0, 0.2, 0.1)
    with pytest.raises(ValueError):
        calculate(1.0, 0, 0.2, 0.1)
    with pytest.raises(ValueError):
        calculate(1.0, 1.0, -0.1, 0.1)
