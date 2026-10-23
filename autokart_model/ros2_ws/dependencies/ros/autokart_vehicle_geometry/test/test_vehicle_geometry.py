import math
import os
import re

import pytest
import yaml

from autokart_vehicle_geometry.geometry import GEOMETRY, VehicleGeometry, load_geometry

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
BRINGUP_CONFIG = os.path.join(REPO_ROOT, "kart", "ros2_ws", "src", "autokart_bringup", "config",
                              "vehicle_geometry.yaml")
SAFETY_BRIDGE_CONFIG = os.path.join(REPO_ROOT, "kart", "ros2_ws", "src", "autokart_safety_bridge", "config",
                                    "front_safety_envelope_monitor.yaml")
SAFETY_BRIDGE_ENVELOPE_PY = os.path.join(REPO_ROOT, "kart", "ros2_ws", "src", "autokart_safety_bridge",
                                         "autokart_safety_bridge_ros", "envelope.py")
SAFETY_BRIDGE_NODE_PY = os.path.join(REPO_ROOT, "kart", "ros2_ws", "src", "autokart_safety_bridge",
                                     "autokart_safety_bridge_ros", "node.py")
URDF_XACRO = os.path.join(REPO_ROOT, "kart", "ros2_ws", "src", "autokart_description", "urdf",
                          "autokart.urdf.xacro")
PERCEPTION_CALIBRATION = os.path.join(REPO_ROOT, "calibration", "kart_hp60c_2026-10",
                                      "perception_kart_unverified.yaml")


def test_raw_values_match_the_user_measured_spec():
    # These are the exact numbers given for this kart; a change here means the physical measurement changed,
    # not a drift to silently accept.
    assert GEOMETRY.camera_model == "HP60C"
    assert GEOMETRY.camera_height_m == pytest.approx(0.8382)
    assert GEOMETRY.camera_pitch_down_deg == pytest.approx(17.8)
    assert GEOMETRY.camera_to_front_bumper_horizontal_m == pytest.approx(1.0668)
    assert GEOMETRY.bumper_width_m == pytest.approx(1.016)
    assert GEOMETRY.stop_gap_from_bumper_m == pytest.approx(0.15)


def test_derived_values_are_computed_not_hardcoded():
    g = GEOMETRY
    assert g.camera_to_bumper_diagonal_m == pytest.approx(math.hypot(0.8382, 1.0668))
    assert g.camera_to_bumper_diagonal_m == pytest.approx(1.357, abs=0.001)
    assert g.half_bumper_width_m == pytest.approx(0.508)
    assert g.camera_forward_distance_at_stop_m == pytest.approx(1.0668 + 0.15)
    assert g.camera_forward_distance_at_stop_m == pytest.approx(1.2168)
    assert g.corridor_minimum_width_m == pytest.approx(1.016 + 2 * 0.15)


def test_reloading_from_the_same_file_is_stable():
    reloaded = load_geometry(GEOMETRY.source_path)
    assert reloaded.camera_height_m == GEOMETRY.camera_height_m
    assert reloaded.camera_to_bumper_diagonal_m == GEOMETRY.camera_to_bumper_diagonal_m


def test_format_table_contains_every_value():
    table = GEOMETRY.format_table()
    for expected in ("camera_height_m", "0.8382", "17.80", "1.0668", "1.0160", "0.1500",
                     "camera_to_bumper_diagonal_m"):
        assert expected in table


def test_a_bad_config_file_is_rejected_not_silently_defaulted(tmp_path):
    bad = tmp_path / "vehicle_geometry.yaml"
    bad.write_text("vehicle_geometry:\n  camera_model: HP60C\n")  # missing required keys
    with pytest.raises(KeyError):
        load_geometry(str(bad))


# ---- cross-module consistency: fails if any consumer's own copy of these numbers drifts ----

def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_safety_bridge_config_matches_geometry():
    data = _load_yaml(SAFETY_BRIDGE_CONFIG)
    params = data["front_safety_envelope_monitor"]["ros__parameters"]
    assert "bumper_setback_m" not in params
    assert "corridor_half_width_m" not in params


def test_safety_bridge_code_sources_defaults_from_geometry_not_a_literal():
    envelope_src = open(SAFETY_BRIDGE_ENVELOPE_PY, encoding="utf-8").read()
    node_src = open(SAFETY_BRIDGE_NODE_PY, encoding="utf-8").read()
    assert "autokart_vehicle_geometry" in envelope_src
    assert "autokart_vehicle_geometry" in node_src
    # The old hardcoded literals must be gone from the Python source.
    assert "1.0668" not in envelope_src and "1.0668" not in node_src
    assert "0.508" not in envelope_src and "0.508" not in node_src


def test_urdf_camera_and_bumper_load_shared_geometry():
    xacro_src = open(URDF_XACRO, encoding="utf-8").read()
    assert "xacro.load_yaml('$(find autokart_bringup)/config/vehicle_geometry.yaml')" in xacro_src
    for key in ("camera_height_m", "camera_pitch_down_deg", "camera_to_front_bumper_horizontal_m", "bumper_width_m"):
        assert f"measured_geometry['{key}']" in xacro_src
        assert not re.search(rf'name="{key}"\s+value="[\d.]+"', xacro_src)
    assert "front_bumper_link" in xacro_src


def test_perception_calibration_matches_geometry():
    data = _load_yaml(PERCEPTION_CALIBRATION)
    mount = data["camera_mount"]
    assert float(mount["height_m"]) == pytest.approx(GEOMETRY.camera_height_m)
    assert float(mount["pitch_down_deg"]) == pytest.approx(GEOMETRY.camera_pitch_down_deg)


def test_bringup_config_is_the_only_place_the_raw_numbers_are_defined_as_yaml_floats():
    data = _load_yaml(BRINGUP_CONFIG)
    values = data["vehicle_geometry"]
    assert float(values["camera_height_m"]) == pytest.approx(0.8382)
    assert float(values["bumper_width_m"]) == pytest.approx(1.016)
