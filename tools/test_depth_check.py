import math

import pytest

from camera_visibility import visibility
from depth_check import budget_m, fit_pitch, projected_forward_m


def test_saved_sdk_intrinsics_put_stop_point_inside_nominal_ground_view():
    view = visibility()
    assert view["nearest_ground_forward_m"] == pytest.approx(0.99805, abs=0.001)
    assert view["cone_base_below_axis_deg"] == pytest.approx(16.761, abs=0.01)
    assert view["cone_base_nominally_in_image"]
    assert view["cone_base_within_configured_depth_range"]


def test_depth_budget_and_surface_projection():
    assert budget_m(1.2) == pytest.approx(0.05)
    assert budget_m(5.0) == pytest.approx(0.25)
    assert projected_forward_m(2.0, 240.0, 585.0, 240.0, 17.8) == pytest.approx(
        2.0 * math.cos(math.radians(17.8)))


def test_pitch_fit_recovers_synthetic_pitch_and_reports_reference_residuals():
    true_pitch = 14.2
    stations = []
    for target in (1.2, 2.0, 3.0, 4.0):
        # A center-row ray has forward distance depth*cos(pitch).
        depth = target / math.cos(math.radians(true_pitch))
        stations.append({"target_m": target,
                         "raw_samples": [{"depth_m": depth, "pixel_v": 240.0,
                                          "fy": 585.0, "cy": 240.0}] * 3})
    result = fit_pitch(stations)
    assert result["fitted"]["pitch_deg"] == pytest.approx(true_pitch, abs=0.01)
    assert result["fitted"]["rmse_m"] < 1e-8
    assert result["configured"]["rmse_m"] > result["fitted"]["rmse_m"]
    assert result["old_ground_plane"]["rmse_m"] > result["fitted"]["rmse_m"]
    assert result["recommendation"] == "PROVISIONAL_FIT_REVIEW"


def test_pitch_fit_needs_four_stations_and_rejects_boundary_fit():
    stations = [{"target_m": value, "raw_samples": [{"depth_m": value, "pixel_v": 240.0,
                "fy": 585.0, "cy": 240.0}]} for value in (1.2, 2.0, 3.0)]
    with pytest.raises(ValueError, match="four distinct"):
        fit_pitch(stations)
    stations.append({"target_m": 4.0, "raw_samples": [{"depth_m": 4.0, "pixel_v": 240.0,
                     "fy": 585.0, "cy": 240.0}]})
    assert fit_pitch(stations)["recommendation"] == "NO_CONFIG_CHANGE_FIT_UNRELIABLE"
