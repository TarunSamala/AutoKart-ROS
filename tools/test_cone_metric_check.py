"""Pure calculation tests for cone_metric_check.py."""
import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    "cone_metric_check", Path(__file__).with_name("cone_metric_check.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def row(x, y=0.0, depth=1.0):
    return {"cone_forward_m": x, "cone_left_m": y, "surface_depth_m": depth}


def test_station_reports_camera_and_bumper_metrics():
    result = module.summarize_station([row(1.54), row(1.55), row(1.56)], 1.55, 1.30)
    assert result["cone_forward_m"] == 1.55
    assert result["bumper_gap_m"] == 0.25
    assert result["expected_bumper_gap_m"] == 0.25
    assert result["error_m"] == 0.0
    assert result["status"] == "PASS_DIAGNOSTIC"


def test_station_fails_large_metric_error_and_empty_input():
    result = module.summarize_station([row(1.30), row(1.31), row(1.32)], 1.55, 1.30)
    assert result["status"] == "FAIL_DIAGNOSTIC"
    assert module.summarize_station([], 1.55, 1.30)["status"] == "NO_VALID_METRIC_SAMPLES"
