"""Unit tests for MappingEngine, ported in behavior from the existing, reused adapter's own
test_mapping_adapter.py (kart/ros2_ws/src/autokart_nodes/autokart_mapping/test/). These exercise the
REAL autokart_perception algorithm (Delaunay/Gabriel/gates/centerline, fit_skidpad_topology) through
our thin wrapper -- not a reimplementation -- so a real regression in that reused algorithm would show
up here too, not just a bug in our own glue code.

No rclpy/ROS runtime required: MappingEngine and ConeObservation are plain Python.
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from autokart_mapping.algorithm import MappingAlgorithm  # noqa: E402
from autokart_mapping.engine import ConeObservation, MappingEngine, observed_exit_centerline  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_ALGORITHM_PATH = REPO_ROOT / "autonomy_stack_phase1" / "phase_01_perception" / "src"

pytestmark = pytest.mark.skipif(
    not REAL_ALGORITHM_PATH.is_dir(),
    reason="autonomy_stack_phase1 checkout not present in this environment")


@pytest.fixture
def algorithm():
    return MappingAlgorithm(str(REAL_ALGORITHM_PATH))


def cones(stamp=1_000_000_000):
    return [ConeObservation(id=index, semantic_class=klass, x_m=float(x), y_m=float(y),
                            confidence=0.95, sigma_m=0.05)
            for index, (x, y, klass) in enumerate(((2, 1.5, 1), (2, -1.5, 2),
                                                    (5, 1.5, 1), (5, -1.5, 2),
                                                    (8, 1.5, 1), (8, -1.5, 2)))]


def test_mapping_requires_paired_status_and_fresh_pose(algorithm):
    engine = MappingEngine(algorithm)
    status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "SIMULATED",
              "frame_id": "base_link", "ground_truth": False}
    pose = algorithm.PoseEstimate(1, 2, 0, 0, 1_000_000_000, 0.05)
    result = engine.update(cones(), status, pose)
    assert result["valid"] and len(result["tracked_cones"]) == 6
    assert result["tracked_cones"][0]["x_m"] == 3.0
    duplicate = engine.update(cones(), status, pose)
    assert not duplicate["valid"] and duplicate["tracked_cones"] == []
    stale = engine.update(cones(2_000_000_000),
                          {**status, "sample_time_ns": 2_000_000_000}, pose)
    assert not stale["valid"] and stale["tracked_cones"] == []


def test_mapping_rejects_unqualified_frame_and_ground_truth(algorithm):
    engine = MappingEngine(algorithm)
    status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "LIVE",
              "frame_id": "camera_optical", "ground_truth": False}
    result = engine.update(cones(), status, algorithm.PoseEstimate(0, 0, 0, 0, 1_000_000_000, 0.05))
    assert not result["valid"] and not result["tracked_cones"]
    status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "SIMULATED",
              "frame_id": "base_link", "ground_truth": True}
    result = engine.update(cones(), status, algorithm.PoseEstimate(0, 0, 0, 0, 1_000_000_000, 0.05))
    assert not result["valid"]


def test_mapping_clears_on_pose_reset_and_duplicate_cone_ids(algorithm):
    engine = MappingEngine(algorithm)
    status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "SIMULATED",
              "frame_id": "base_link", "ground_truth": False}
    assert engine.update(cones(), status,
                         algorithm.PoseEstimate(0, 0, 0, 1, 1_000_000_000, 0.05))["valid"]
    reset_stamp = 1_020_000_000
    result = engine.update(cones(reset_stamp), {**status, "sample_time_ns": reset_stamp},
                           algorithm.PoseEstimate(10, 0, 0, 1, reset_stamp, 0.05))
    assert not result["valid"] and "ODOM_DISCONTINUITY" in result["reason_codes"]
    assert result["tracked_cones"] == []
    duplicate = cones(2_000_000_000)
    duplicate[1] = ConeObservation(id=duplicate[0].id, semantic_class=duplicate[1].semantic_class,
                                   x_m=duplicate[1].x_m, y_m=duplicate[1].y_m,
                                   confidence=duplicate[1].confidence, sigma_m=duplicate[1].sigma_m)
    result = engine.update(duplicate, {**status, "sample_time_ns": 2_000_000_000},
                           algorithm.PoseEstimate(0, 0, 0, 0, 2_000_000_000, 0.05))
    assert not result["valid"] and "INVALID_CONE" in result["reason_codes"]


def test_exit_corridor_requires_observed_ordered_blue_yellow_pairs(algorithm):
    rings = []
    for cx, blue_radius, yellow_radius in ((-9.125, 7.625, 10.625),
                                           (9.125, 10.625, 7.625)):
        for label, radius in (("blue", blue_radius), ("yellow", yellow_radius)):
            for index in range(24):
                angle = index * 2 * math.pi / 24
                rings.append({"x_m": cx + radius * math.cos(angle),
                              "y_m": radius * math.sin(angle), "label": label,
                              "sigma_m": 0.05})
    topology = algorithm.fit_skidpad_topology(rings)
    assert topology["valid"]
    assert observed_exit_centerline(rings, topology) == []
    exit_cones = [{"x_m": x, "y_m": y, "label": label, "sigma_m": 0.05}
                  for y in (5.0, 8.0, 11.0)
                  for x, label in ((-1.5, "blue"), (1.5, "yellow"))]
    assert observed_exit_centerline(rings + exit_cones, topology) == [
        (0.0, 5.0), (0.0, 8.0), (0.0, 11.0)]
    assert observed_exit_centerline(rings + exit_cones[:-1], topology) == []
