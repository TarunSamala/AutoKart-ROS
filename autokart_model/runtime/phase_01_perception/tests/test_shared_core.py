import math
from pathlib import Path

import pytest
import yaml

from autokart_perception.shared_core import (
    AlgorithmCore,
    BrakeCapability,
    PoseEstimate,
    WindingTracker,
    fit_skidpad_topology,
    LocalConeMap,
)
from autokart_perception.types import Cone


@pytest.fixture
def config():
    return yaml.safe_load((Path(__file__).parents[1] / "config/perception.yaml").read_text())


def cone(i, x, y, label):
    return Cone(i, label, 0.95, (x, y), 0.05, math.hypot(x, y), (0, 0, 1, 1))


def straight_cones():
    return [cone(i, x, y, label) for i, (x, y, label) in enumerate((
        (2, 1.5, "blue"), (2, -1.5, "yellow"),
        (5, 1.5, "blue"), (5, -1.5, "yellow"),
        (8, 1.5, "blue"), (8, -1.5, "yellow"),
    ))]


def test_shared_core_corridor_and_fail_closed_pose(config):
    core = AlgorithmCore(config, wheelbase_m=1.05)
    now = 1_000_000_000
    pose = PoseEstimate(0, 0, 0, 0, now, 0.05)
    result = core.evaluate(straight_cones(), pose, "STRAIGHT_TRACK", now_ns=now)
    assert result["algorithm_version"]
    assert result["geometry"]["delaunay_edges"]
    assert result["geometry"]["gabriel_edges"]
    assert result["planning"]["path_valid"]
    assert result["control_candidate"]["valid"]
    assert result["vehicle_authorized"] is False

    stale = core.evaluate(straight_cones(), pose, "STRAIGHT_TRACK", now_ns=now + 300_000_000)
    assert stale["planning"]["path_valid"] is False
    assert stale["control_candidate"]["valid"] is False
    assert "INVALID_POSE" in stale["reasons"]


def test_skidpad_winding_rejects_oscillation_and_counts_direction():
    tracker = WindingTracker(required_left=1, required_right=1)
    center = (0.0, 0.0)
    for angle in (0.0, 0.2, 0.0, 0.2, 0.0):
        tracker.update(center, (4 * math.cos(angle), 4 * math.sin(angle)), "LEFT")
    assert tracker.left_laps == 0
    for step in range(1, 73):
        angle = 2 * math.pi * step / 72
        tracker.update(center, (4 * math.cos(angle), 4 * math.sin(angle)), "LEFT")
    assert tracker.left_laps == 1
    assert tracker.state == "RIGHT"
    right = (10.0, 0.0)
    tracker.update(right, (14.0, 0.0), "RIGHT")
    for step in range(1, 73):
        angle = -2 * math.pi * step / 72
        tracker.update(right, (10 + 4 * math.cos(angle), 4 * math.sin(angle)), "RIGHT")
    assert tracker.right_laps == 1
    assert tracker.state == "COMPLETE"


def test_braking_capability_requires_measured_or_simulated_scope(config):
    core = AlgorithmCore(config, wheelbase_m=1.05)
    now = 1_000_000_000
    pose = PoseEstimate(0, 0, 0, 2, now, 0.05)
    no_brake = core.evaluate(straight_cones(), pose, "BRAKING_TEST", now_ns=now)
    assert not no_brake["control_candidate"]["valid"]
    assert "BRAKE_CAPABILITY_UNQUALIFIED" in no_brake["reasons"]
    capability = BrakeCapability(2.0, 0.2, "SIMULATED")
    simulated = core.evaluate(straight_cones(), pose, "BRAKING_TEST", now_ns=now,
                              brake_capability=capability, stop_distance_m=5.0)
    assert simulated["planning"]["stopping_distance_m"] > 0
    assert simulated["vehicle_authorized"] is False


def test_repeated_frame_and_blocked_path_withhold_candidate(config):
    now = 1_000_000_000
    pose = PoseEstimate(0, 0, 0, 1, now, 0.05)
    core = AlgorithmCore(config, wheelbase_m=1.15)
    assert core.evaluate(straight_cones(), pose, "STRAIGHT_TRACK", now_ns=now)["planning"]["path_valid"]
    repeated = core.evaluate(straight_cones(), pose, "STRAIGHT_TRACK", now_ns=now)
    assert "STALE_FRAME" in repeated["reasons"]
    assert not repeated["control_candidate"]["valid"]
    fresh_core = AlgorithmCore(config, wheelbase_m=1.15)
    blocked = fresh_core.evaluate(straight_cones(), pose, "STRAIGHT_TRACK", now_ns=now,
                                  obstacles=[{"x_m": 5.0, "y_m": 0.0, "radius_m": 0.5}])
    assert "PATH_BLOCKED" in blocked["reasons"]
    assert not blocked["vehicle_authorized"]


def test_pose_aware_map_expires_stale_cones():
    now = 1_000_000_000
    mapping = LocalConeMap(maximum_cones=8, expiry_ns=1_000_000_000)
    mapping.update(straight_cones(), PoseEstimate(0, 0, 0, 0, now, 0.05), now)
    assert len(mapping.snapshot()) == 6
    mapping.update([], PoseEstimate(0, 0, 0, 0, now, 0.05), now + 1_000_000_001)
    assert mapping.snapshot() == []


def test_geometry_path_does_not_require_optional_controller(config, monkeypatch):
    import builtins
    original_import = builtins.__import__

    def without_simulator(name, *args, **kwargs):
        if name.startswith("kart_sim"):
            raise ImportError("optional simulator package absent")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", without_simulator)
    now = 1_000_000_000
    result = AlgorithmCore(config, wheelbase_m=1.15).evaluate(
        straight_cones(), PoseEstimate(0, 0, 0, 0, now, 0.05),
        "STRAIGHT_TRACK", now_ns=now)
    assert result["geometry"]["centerline"]
    assert result["planning"]["path_valid"]
    assert not result["control_candidate"]["valid"]
    assert not result["vehicle_authorized"]


def test_unstamped_live_observations_cannot_form_fresh_candidate(config):
    now = 1_000_000_000
    result = AlgorithmCore(config, wheelbase_m=1.15).evaluate(
        straight_cones(), PoseEstimate(0, 0, 0, 0, now, 0.05),
        "STRAIGHT_TRACK", now_ns=now, source="LIVE")
    assert "STALE_FRAME" in result["reasons"]
    assert not result["planning"]["path_valid"]
    assert not result["control_candidate"]["valid"]


@pytest.mark.parametrize("bad_id", [-1, True, 1])
def test_invalid_or_duplicate_cone_id_rejected(config, bad_id):
    observations = straight_cones()
    first = observations[0]
    observations[0] = Cone(bad_id, first.label, first.confidence, first.xy,
                           first.sigma_m, first.distance_m, first.box)
    now = 1_000_000_000
    result = AlgorithmCore(config, wheelbase_m=1.15).evaluate(
        observations, PoseEstimate(0, 0, 0, 0, now, .05),
        "STRAIGHT_TRACK", now_ns=now)
    assert "INVALID_GEOMETRY" in result["reasons"]
    assert not result["planning"]["path_valid"]
    assert not result["control_candidate"]["valid"]


def test_duplicate_coordinates_rejected_even_with_distinct_ids(config):
    observations = straight_cones()
    first, second = observations[:2]
    observations[1] = Cone(second.id, second.label, second.confidence, first.xy,
                           second.sigma_m, second.distance_m, second.box)
    now = 1_000_000_000
    result = AlgorithmCore(config, wheelbase_m=1.15).evaluate(
        observations, PoseEstimate(0, 0, 0, 0, now, .05),
        "STRAIGHT_TRACK", now_ns=now)
    assert "INVALID_GEOMETRY" in result["reasons"]
    assert not result["planning"]["path_valid"]


def test_invalid_cone_id_remains_rejected_in_skidpad(config):
    observations = straight_cones()
    first = observations[0]
    observations[0] = Cone(-1, first.label, first.confidence, first.xy,
                           first.sigma_m, first.distance_m, first.box)
    now = 1_000_000_000
    result = AlgorithmCore(config, wheelbase_m=1.15).evaluate(
        observations, PoseEstimate(0, 0, 0, 0, now, .05),
        "SKIDPAD", now_ns=now)
    assert "INVALID_GEOMETRY" in result["reasons"]
    assert not result["planning"]["path_valid"]


def test_skidpad_topology_comes_from_observed_cones_not_course_truth():
    observations = []
    for center_x in (-9.0, 9.0):
        for radius, label in ((5.0, "blue"), (8.0, "yellow")):
            for index in range(16):
                angle = 2 * math.pi * index / 16
                observations.append({"x_m": center_x + radius * math.cos(angle),
                                     "y_m": radius * math.sin(angle), "label": label,
                                     "sigma_m": 0.05})
    topology = fit_skidpad_topology(observations)
    assert topology["valid"]
    assert topology["left"]["center_x_m"] == pytest.approx(-9.0, abs=0.1)
    assert topology["right"]["center_x_m"] == pytest.approx(9.0, abs=0.1)
    assert topology["left"]["track_width_m"] == pytest.approx(3.0, abs=0.1)
    assert not fit_skidpad_topology(observations[:5])["valid"]
    assert not fit_skidpad_topology([item for item in observations if item["label"] == "blue"])["valid"]


def test_skidpad_topology_does_not_require_world_x_alignment():
    observations = []
    for center_y in (-9.0, 9.0):
        for radius, label in ((5.0, "blue"), (8.0, "yellow")):
            for index in range(16):
                angle = 2 * math.pi * index / 16
                observations.append({"x_m": radius * math.cos(angle),
                                     "y_m": center_y + radius * math.sin(angle),
                                     "label": label, "sigma_m": .05})
    topology = fit_skidpad_topology(observations)
    assert topology["valid"]
    assert {round(topology["left"]["center_y_m"]),
            round(topology["right"]["center_y_m"])} == {-9, 9}


def test_skidpad_topology_handles_overlapping_outer_boundaries_at_crossover():
    observations = []
    for center_x, blue_radius, yellow_radius in ((-9.125, 7.625, 10.625),
                                                  (9.125, 10.625, 7.625)):
        for label, radius in (("blue", blue_radius), ("yellow", yellow_radius)):
            for index in range(24):
                angle = 2 * math.pi * index / 24
                observations.append({"x_m": center_x + radius * math.cos(angle),
                                     "y_m": radius * math.sin(angle),
                                     "label": label, "sigma_m": 0.05})
    topology = fit_skidpad_topology(observations)
    assert topology["valid"]
    assert topology["left"]["center_x_m"] == pytest.approx(-9.125, abs=0.1)
    assert topology["right"]["center_x_m"] == pytest.approx(9.125, abs=0.1)
