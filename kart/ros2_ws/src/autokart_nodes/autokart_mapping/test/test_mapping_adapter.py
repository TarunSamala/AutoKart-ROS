from autokart_msgs.msg import Cone, ConeArray
from autokart_mapping.node import MappingEngine, observed_exit_centerline, ros_time
from autokart_perception.shared_core import PoseEstimate, fit_skidpad_topology
import math


def cones(stamp=1_000_000_000):
    result = ConeArray()
    result.measurement_time = ros_time(stamp)
    result.frame_id = "base_link"
    for index, (x, y, klass) in enumerate(((2, 1.5, 1), (2, -1.5, 2),
                                            (5, 1.5, 1), (5, -1.5, 2),
                                            (8, 1.5, 1), (8, -1.5, 2))):
        item = Cone()
        item.id = index
        item.position.x, item.position.y = float(x), float(y)
        item.semantic_class = klass
        item.confidence = 0.95
        item.position_std_m = 0.05
        result.cones.append(item)
    return result


def test_mapping_requires_paired_status_and_fresh_pose():
    engine = MappingEngine()
    message = cones()
    status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "SIMULATED"}
    pose = PoseEstimate(1, 2, 0, 0, 1_000_000_000, 0.05)
    result = engine.update(message, status, pose)
    assert result["valid"] and len(result["tracked_cones"]) == 6
    assert result["tracked_cones"][0]["x_m"] == 3.0
    duplicate = engine.update(message, status, pose)
    assert not duplicate["valid"] and duplicate["tracked_cones"] == []
    stale = engine.update(cones(2_000_000_000),
                          {**status, "sample_time_ns": 2_000_000_000}, pose)
    assert not stale["valid"] and stale["tracked_cones"] == []


def test_mapping_rejects_unqualified_frame_and_ground_truth():
    engine = MappingEngine()
    message = cones()
    message.frame_id = "camera_optical"
    result = engine.update(message, {"sample_time_ns": 1_000_000_000,
                                     "valid": True, "source": "LIVE"},
                           PoseEstimate(0, 0, 0, 0, 1_000_000_000, 0.05))
    assert not result["valid"] and not result["tracked_cones"]
    message.frame_id = "base_link"
    message.ground_truth = True
    result = engine.update(message, {"sample_time_ns": 1_000_000_000,
                                     "valid": True, "source": "SIMULATED"},
                           PoseEstimate(0, 0, 0, 0, 1_000_000_000, 0.05))
    assert not result["valid"]


def test_mapping_clears_on_pose_reset_and_duplicate_cone_ids():
    engine = MappingEngine()
    status = {"sample_time_ns": 1_000_000_000, "valid": True, "source": "SIMULATED"}
    assert engine.update(cones(), status,
                         PoseEstimate(0, 0, 0, 1, 1_000_000_000, 0.05))["valid"]
    reset_stamp = 1_020_000_000
    result = engine.update(cones(reset_stamp),
                           {**status, "sample_time_ns": reset_stamp},
                           PoseEstimate(10, 0, 0, 1, reset_stamp, 0.05))
    assert not result["valid"] and "ODOM_DISCONTINUITY" in result["reason_codes"]
    assert result["tracked_cones"] == []
    duplicate = cones(2_000_000_000)
    duplicate.cones[1].id = duplicate.cones[0].id
    result = engine.update(duplicate, {**status, "sample_time_ns": 2_000_000_000},
                           PoseEstimate(0, 0, 0, 0, 2_000_000_000, 0.05))
    assert not result["valid"] and "INVALID_CONE" in result["reason_codes"]


def test_exit_corridor_requires_observed_ordered_blue_yellow_pairs():
    rings = []
    for cx, blue_radius, yellow_radius in ((-9.125, 7.625, 10.625),
                                           (9.125, 10.625, 7.625)):
        for label, radius in (("blue", blue_radius), ("yellow", yellow_radius)):
            for index in range(24):
                angle = index * 2 * math.pi / 24
                rings.append({"x_m": cx + radius * math.cos(angle),
                              "y_m": radius * math.sin(angle), "label": label,
                              "sigma_m": 0.05})
    topology = fit_skidpad_topology(rings)
    assert topology["valid"]
    assert observed_exit_centerline(rings, topology) == []
    exit_cones = [{"x_m": x, "y_m": y, "label": label, "sigma_m": 0.05}
                  for y in (5.0, 8.0, 11.0)
                  for x, label in ((-1.5, "blue"), (1.5, "yellow"))]
    assert observed_exit_centerline(rings + exit_cones, topology) == [
        (0.0, 5.0), (0.0, 8.0), (0.0, 11.0)]
    assert observed_exit_centerline(rings + exit_cones[:-1], topology) == []
