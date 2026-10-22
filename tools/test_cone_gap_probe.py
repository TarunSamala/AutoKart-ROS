"""Safety boundaries for the read-only camera gap probe."""
import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location("cone_gap_probe", Path(__file__).with_name("cone_gap_probe.py"))
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def status(valid=True):
    return {"schema_version": 1, "source": "LIVE", "sample_time_ns": 100,
            "frame_id": "base_link", "valid": valid}


def cone(x):
    return {"id": 7, "x_m": x, "y_m": 0.0, "position_std_m": 0.02,
            "confidence": 0.9}


def test_bumper_to_center_target():
    offset = probe.GEOMETRY.camera_to_front_bumper_horizontal_m
    target = probe.GEOMETRY.stop_gap_from_bumper_m
    row = probe.evaluate(status(), [cone(offset + target)], stamp_ns=100, cone_id=7)
    assert row["decision"] == "AT_OR_INSIDE_TARGET"
    assert abs(row["bumper_to_center_m"] - target) < 1e-9


def test_missing_or_unqualified_cone_fails_closed():
    assert probe.evaluate(status(), [], stamp_ns=100, cone_id=7)["decision"] == "STOP"
    assert probe.evaluate(status(False), [cone(2.0)], stamp_ns=100, cone_id=7)["decision"] == "STOP"
    assert probe.evaluate(status(), [cone(2.0)], stamp_ns=101, cone_id=7)["decision"] == "STOP"
    assert probe.evaluate(status(), [cone(2.0), cone(2.1)], stamp_ns=100, cone_id=7)["decision"] == "STOP"
