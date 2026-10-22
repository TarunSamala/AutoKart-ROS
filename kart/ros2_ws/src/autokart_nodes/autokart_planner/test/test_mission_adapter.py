import math

import pytest

from autokart_perception.shared_core import BrakeCapability, PoseEstimate, fit_skidpad_topology
from autokart_planner.mission import PlannerEngine
from autokart_vehicle_geometry.geometry import GEOMETRY


def topology():
    cones = []
    for center_x, blue, yellow in ((-9.125, 7.625, 10.625),
                                   (9.125, 10.625, 7.625)):
        for label, radius in (("blue", blue), ("yellow", yellow)):
            for index in range(24):
                angle = 2 * math.pi * index / 24
                cones.append({"x_m": center_x + radius * math.cos(angle),
                              "y_m": radius * math.sin(angle), "label": label,
                              "sigma_m": 0.05})
    result = fit_skidpad_topology(cones)
    assert result["valid"]
    return result


def map_sample(t, stamp):
    return {"valid": True, "source": "SIMULATED", "frame_id": "odom",
            "sample_time_ns": stamp, "geometry": {"skidpad_topology": t}}


def pose_on(cx, angle, stamp):
    return PoseEstimate(cx + 9.125 * math.cos(angle), 9.125 * math.sin(angle),
                        angle - math.pi / 2 if cx > 0 else angle + math.pi / 2,
                        1.0, stamp, 0.05)


def test_right_right_left_left_requires_all_four_gate_crossings():
    t = topology()
    engine = PlannerEngine("SKIDPAD", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    decisions = []
    for index in range(161):
        angle = math.pi - index * 2 * math.pi / 80
        result = engine.evaluate(map_sample(t, stamp), pose_on(9.125, angle, stamp))
        decisions.append(result)
        stamp += 20_000_000
    assert engine.progress.right_laps == 2
    assert engine.progress.left_laps == 0
    assert engine.progress.state == "TRANSITION"
    assert decisions[-1]["valid"] is False
    for index in range(1, 163):
        angle = index * 2 * math.pi / 80
        result = engine.evaluate(map_sample(t, stamp), pose_on(-9.125, angle, stamp))
        stamp += 20_000_000
    assert engine.progress.right_laps == 2
    assert engine.progress.left_laps == 2
    assert engine.progress.state == "EXIT"
    assert result["valid"] is False
    assert result["reason_codes"] == ["EXIT_CORRIDOR_UNOBSERVED"]
    exit_map = map_sample(t, stamp)
    exit_map["geometry"]["exit_centerline_odom"] = [(0, 5), (0, 8), (0, 11)]
    result = engine.evaluate(exit_map,
                             PoseEstimate(0, 0, math.pi / 2, 0.5, stamp, 0.05))
    assert result["valid"] and result["decision"] == "EXIT_PATH_CANDIDATE"
    assert result["path_odom"][0][1] > 0
    stamp += 20_000_000
    exit_map["sample_time_ns"] = stamp
    result = engine.evaluate(exit_map,
                             PoseEstimate(0, 8, math.pi / 2, 0.5, stamp, 0.05))
    assert result["valid"] and all(y >= 8 for _, y in result["path_odom"])
    stamp += 20_000_000
    exit_map["sample_time_ns"] = stamp
    result = engine.evaluate(exit_map,
                             PoseEstimate(0, 11, math.pi / 2, 0.0, stamp, 0.05),
                             stop_feedback_valid=True)
    assert engine.progress.state == "STOP"
    assert not result["valid"]


def test_exit_rejects_backward_progress_and_wrong_branch():
    t = topology()
    engine = PlannerEngine("SKIDPAD", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    engine.progress.state = "EXIT"
    mapping = map_sample(t, 1_000_000_000)
    mapping["geometry"]["exit_centerline_odom"] = [(0, 5), (0, 8), (0, 11)]
    first = engine.evaluate(mapping, PoseEstimate(0, 6, math.pi / 2, 0.5,
                                                 1_000_000_000, 0.05))
    assert first["valid"]
    mapping["sample_time_ns"] += 20_000_000
    back = engine.evaluate(mapping, PoseEstimate(0, 5, math.pi / 2, 0.5,
                                                mapping["sample_time_ns"], 0.05))
    assert not back["valid"] and back["reason_codes"] == ["EXIT_PROGRESS_INVALID"]
    mapping["sample_time_ns"] += 20_000_000
    wrong = engine.evaluate(mapping, PoseEstimate(-8, 6, math.pi / 2, 0.5,
                                                 mapping["sample_time_ns"], 0.05))
    assert not wrong["valid"] and wrong["reason_codes"] == ["EXIT_POSE_OFF_CORRIDOR"]


def test_exit_accepts_slight_pose_error_but_requires_terminal_feedback():
    t = topology()
    engine = PlannerEngine("SKIDPAD", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    engine.progress.state = "EXIT"
    stamp = 1_000_000_000
    mapping = map_sample(t, stamp)
    mapping["geometry"]["exit_centerline_odom"] = [(0, 5), (0, 8), (0, 11)]
    candidate = engine.evaluate(mapping, PoseEstimate(0.2, 6, math.pi / 2, 0.5, stamp, 0.05))
    assert candidate["valid"] and candidate["decision"] == "EXIT_PATH_CANDIDATE"
    assert candidate["path_odom"][-1] == (0.0, 11.0)
    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    pending = engine.evaluate(mapping, PoseEstimate(0.1, 10.9, math.pi / 2, 0.0, stamp, 0.05))
    assert not pending["valid"] and pending["decision"] == "HOLD"
    assert engine.progress.state == "STOP_PENDING"
    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    stopped = engine.evaluate(mapping, PoseEstimate(0.1, 10.9, math.pi / 2, 0.0, stamp, 0.05),
                              stop_feedback_valid=True)
    assert not stopped["valid"] and stopped["decision"] == "STOP"
    assert engine.progress.state == "STOP"


def test_missing_or_stale_observations_never_publish_skidpad_path():
    t = topology()
    engine = PlannerEngine("SKIDPAD", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    missing = map_sample(t, stamp)
    missing["geometry"]["skidpad_topology"] = {"valid": False}
    held = engine.evaluate(missing, pose_on(9.125, math.pi, stamp))
    assert not held["valid"] and held["path_odom"] == []
    assert held["reason_codes"] == ["OBSERVED_LOOPS_UNAVAILABLE"]
    stamp += 20_000_000
    stale = engine.evaluate(map_sample(t, stamp), pose_on(9.125, math.pi, stamp - 200_000_000))
    assert not stale["valid"] and stale["path_odom"] == []
    assert stale["reason_codes"] == ["MAP_OR_ODOM_INVALID"]


def test_wrong_direction_and_unqualified_brake_hold():
    t = topology()
    engine = PlannerEngine("SKIDPAD", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    engine.evaluate(map_sample(t, stamp), pose_on(9.125, math.pi, stamp))
    stamp += 20_000_000
    result = engine.evaluate(map_sample(t, stamp), pose_on(9.125, math.pi + 0.1, stamp))
    assert not result["valid"]
    assert "WRONG_DIRECTION_OR_POSE_JUMP" in result["reason_codes"]
    live = PlannerEngine("SKIDPAD", "LIVE", None)
    result = live.evaluate({**map_sample(t, stamp), "source": "LIVE"},
                           pose_on(9.125, math.pi, stamp))
    assert not result["valid"]
    assert result["path_odom"] == []


def test_braking_corridor_obstacle_and_missing_horizon():
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom",
               "sample_time_ns": stamp,
               "geometry": {"centerline_status": "PROVISIONAL",
                            "centerline": [(float(x), 0.0) for x in range(1, 11)]}}
    pose = PoseEstimate(0, 0, 0, 2.0, stamp, 0.05)
    blocked = engine.evaluate(mapping, pose, [{"x_m": 4.0, "y_m": 0.0, "radius_m": 0.3}])
    assert blocked["decision"] == "STOP_CANDIDATE"
    assert not blocked["valid"] and blocked["path_odom"] == []
    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    clear = engine.evaluate(mapping, PoseEstimate(0, 0, 0, 2.0, stamp, 0.05),
                            [{"x_m": 8.0, "y_m": 0.0, "radius_m": 0.3}])
    assert clear["valid"]
    stamp += 20_000_000
    mapping["sample_time_ns"] = stamp
    mapping["geometry"]["centerline"] = [(0.5, 0.0), (1.0, 0.0), (2.0, 0.0)]
    short = engine.evaluate(mapping, PoseEstimate(0, 0, 0, 2.0, stamp, 0.05), [])
    assert not short["valid"] and "STOP_HORIZON_INSUFFICIENT" in short["reason_codes"]


def test_dead_reckoning_tracks_a_cone_that_left_the_field_of_view():
    # The cone is seen once close enough to be about to trigger a stop, then vanishes from `obstacles`
    # (e.g. it left the depth-valid FOV near the stop point, see camera_visibility.py) while the kart keeps
    # moving. Dead reckoning must still catch the stop using the last sighting advanced by the odometry-
    # measured travel, not just silently resume cruising because nothing is currently observed.
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", BrakeCapability(3.0, 0.0, "SIMULATED", margin_m=0.1),
                           dead_reckoning_max_m=2.0, dead_reckoning_margin_rate=0.0)
    stamp = 1_000_000_000
    centerline = [(float(i), 0.0) for i in range(1, 11)]
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
              "geometry": {"centerline_status": "PROVISIONAL", "centerline": centerline}}
    # Last genuine sighting is within the expected camera-forward range at the stop point.
    seen = engine.evaluate(mapping, PoseEstimate(0.0, 0.0, 0, 0.0, stamp, 0.05),
                           [{"x_m": 1.2, "y_m": 0.0, "radius_m": 0.0}])
    assert seen["decision"] == "STOP_CANDIDATE"

    # Cone vanishes at the expected stop range, so the conservative dead-reckoning path remains active.
    stamp += 100_000_000
    mapping["sample_time_ns"] = stamp
    result = engine.evaluate(mapping, PoseEstimate(0.1, 0.0, 0, 0.0, stamp, 0.05), [])
    assert result["decision"] == "STOP_CANDIDATE", result
    assert result.get("dead_reckoned") is True


def test_dead_reckoning_expires_and_fails_safe_beyond_the_blind_travel_limit():
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", BrakeCapability(3.0, 0.0, "SIMULATED", margin_m=0.1),
                           dead_reckoning_max_m=1.0)
    stamp = 1_000_000_000
    centerline = [(float(i), 0.0) for i in range(1, 11)]
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
              "geometry": {"centerline_status": "PROVISIONAL", "centerline": centerline}}
    seen = engine.evaluate(mapping, PoseEstimate(0.0, 0.0, 0, 0.0, stamp, 0.05),
                           [{"x_m": 1.2, "y_m": 0.0, "radius_m": 0.0}])
    assert seen["decision"] == "STOP_CANDIDATE"

    # Travel 1.5 m blind, beyond dead_reckoning_max_m=1.0: no longer trusted, must fail safe to STOP rather
    # than silently resuming PATH_CANDIDATE.
    stamp += 100_000_000
    mapping["sample_time_ns"] = stamp
    result = engine.evaluate(mapping, PoseEstimate(1.5, 0.0, 0, 0.0, stamp, 0.05), [])
    assert result["decision"] == "STOP_CANDIDATE" and "DEAD_RECKONING_EXPIRED" in result["reason_codes"]


def test_lost_target_farther_than_visible_at_stop_stops_immediately():
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", BrakeCapability(3.0, 0.0, "SIMULATED", margin_m=0.1))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
               "tracked_count": 2,
               "geometry": {"centerline_status": "PROVISIONAL",
                            "centerline": [(float(i), 0.0) for i in range(1, 11)]}}
    assert engine.evaluate(mapping, PoseEstimate(0, 0, 0, 0, stamp, 0.05),
                           [{"x_m": 3.0, "y_m": 0.0, "radius_m": 0.0}])["valid"]
    mapping["sample_time_ns"] = stamp + 100_000_000
    lost = engine.evaluate(mapping, PoseEstimate(0.1, 0, 0, 0, stamp + 100_000_000, 0.05),
                           [{"x_m": 2.0, "y_m": 5.0, "radius_m": 0.0}])
    assert lost["decision"] == "STOP_CANDIDATE"
    assert lost["reason_codes"] == ["STOP_CONE_LOST_BEYOND_VISIBLE_STOP_RANGE"]


def test_no_valid_map_cones_never_yields_path_candidate():
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", BrakeCapability(3.0, 0.0, "SIMULATED"))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
               "tracked_count": 0,
               "geometry": {"centerline_status": "PROVISIONAL",
                            "centerline": [(float(i), 0.0) for i in range(1, 11)]}}
    result = engine.evaluate(mapping, PoseEstimate(0, 0, 0, 0, stamp, 0.05), [])
    assert not result["valid"] and result["target_speed_mps"] == 0
    assert result["reason_codes"] == ["NO_VALID_MAP_CONES"]


def test_braking_checks_swept_segments_not_only_waypoints():
    # Obstacle at x=2.0 (close enough that bumper_to_cone - stop_gap <= the stationary stopping distance,
    # 1.5 m margin at v=0) still off-centerline at y=0.4: this must be caught by the swept-segment distance
    # to the (2,0)-(5,0) polyline leg, not just distance to the nearest waypoint.
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom",
               "sample_time_ns": stamp,
               "geometry": {"centerline_status": "PROVISIONAL",
                            "centerline": [(2.0, 0.0), (5.0, 0.0), (8.0, 0.0)]}}
    result = engine.evaluate(mapping, PoseEstimate(0, 0, 0, 0, stamp, 0.05),
                             [{"x_m": 2.0, "y_m": 0.4, "radius_m": 0.3}])
    assert result["decision"] == "STOP_CANDIDATE"
    assert not result["path_odom"]
    assert result["stop_point_m"][1] == 0.0


def _run_fixed_speed_braking(a_true, a_cfg, margin, v_run=1.0, stop_x=9.0, latency=0.2, dt=0.02, horizon=40.0):
    """Kinematic closed-loop check (no ROS): cruise a fixed-speed kart at v_run, let the obstacle "enter view"
    exactly brake.stopping_distance(v_run) ahead (as a real perception pipeline with that much range would),
    decelerate at the TRUE a_true from the moment the engine first reports STOP_CANDIDATE, and return the final
    gap between the kart and the cone. All coordinates passed to the engine are vehicle-frame (forward distance
    from the kart), matching how _braking() uses them -- see its final odom transform using pose.yaw_rad.
    """
    brake = BrakeCapability(a_cfg, latency, "SIMULATED", margin_m=margin)
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", brake, speed_limit_mps=v_run)
    stamp = 1_000_000_000
    x, v = 0.0, v_run
    centerline = [(float(i), 0.0) for i in range(1, int(horizon) + 1)]
    braking = False
    # Simulated sensor range: must cover the full BUMPER-referenced trigger threshold (stopping distance
    # plus the camera-to-bumper offset plus the stop gap), not just the raw stopping-distance physics term,
    # since the engine's actual trigger is bumper-referenced (see mission.py PlannerEngine._braking).
    detect_ahead_m = (brake.stopping_distance(v_run) + engine.camera_to_bumper_m + engine.stop_gap_from_bumper_m)
    for _ in range(20_000):
        stamp += int(dt * 1e9)
        remaining = stop_x - x
        if remaining <= 0:
            break
        mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
                   "geometry": {"centerline_status": "PROVISIONAL", "centerline": centerline}}
        pose = PoseEstimate(x, 0.0, 0.0, v, stamp, 0.05)
        obstacles = [{"x_m": remaining, "y_m": 0.0, "radius_m": 0.0}] if remaining <= detect_ahead_m else []
        result = engine.evaluate(mapping, pose, obstacles)
        if result["decision"] == "STOP_CANDIDATE":
            braking = True
        if braking:
            v = max(0.0, v - a_true * dt)
        x += v * dt
        if braking and v <= 0.0:
            break
    return stop_x - x


def test_fixed_speed_kart_keeps_at_least_the_margin_for_several_distances():
    # Accurately-configured deceleration (a_cfg == a_true): the realized gap should land at ~margin for any
    # starting distance, since detection range is computed from the same stopping_distance() formula.
    # Camera-frame gap (includes the fixed ~1.2168 m camera-to-bumper-plus-stop-gap offset baked into the
    # trigger); see test_fixed_speed_kart_keeps_the_bumper_stop_gap_for_several_distances below for the
    # bumper-referenced version, which is the number that actually matters physically.
    for stop_x in (5.0, 7.0, 9.0, 12.0, 15.0, 20.0):
        gap = _run_fixed_speed_braking(a_true=0.6, a_cfg=0.6, margin=1.0, stop_x=stop_x)
        assert gap >= 1.0 - 0.05, f"stop_x={stop_x}: gap {gap:.3f} m < the configured 1.0 m margin"


def test_fixed_speed_kart_keeps_the_bumper_stop_gap_for_several_distances():
    # Bumper-referenced, using the real GEOMETRY.stop_gap_from_bumper_m (0.15 m), not camera-frame distance.
    for stop_x in (5.0, 7.0, 9.0, 12.0, 15.0, 20.0):
        camera_gap = _run_fixed_speed_braking(a_true=0.6, a_cfg=0.6, margin=0.1, stop_x=stop_x)
        bumper_gap = camera_gap - GEOMETRY.camera_to_front_bumper_horizontal_m
        assert bumper_gap >= GEOMETRY.stop_gap_from_bumper_m - 0.05, \
            f"stop_x={stop_x}: bumper gap {bumper_gap:.3f} m < the {GEOMETRY.stop_gap_from_bumper_m} m stop gap"


def test_margin_covers_an_overestimated_deceleration_at_the_bumper():
    # Same overestimate as before (0.6 configured vs 0.4 actual), but the assertion is on the BUMPER gap,
    # the number that actually matters physically, not the inflated camera-frame distance.
    camera_gap_no_margin = _run_fixed_speed_braking(a_true=0.4, a_cfg=0.6, margin=0.0)
    bumper_gap_no_margin = camera_gap_no_margin - GEOMETRY.camera_to_front_bumper_horizontal_m
    assert bumper_gap_no_margin < GEOMETRY.stop_gap_from_bumper_m, \
        f"expected the unsafe case to actually be unsafe, got bumper gap {bumper_gap_no_margin:.3f} m"
    camera_gap_with_margin = _run_fixed_speed_braking(a_true=0.4, a_cfg=0.6, margin=1.0)
    bumper_gap_with_margin = camera_gap_with_margin - GEOMETRY.camera_to_front_bumper_horizontal_m
    assert bumper_gap_with_margin >= GEOMETRY.stop_gap_from_bumper_m, \
        f"margin did not save the bumper gap: {bumper_gap_with_margin:.3f} m"
    assert bumper_gap_with_margin > bumper_gap_no_margin


def test_dropping_the_camera_to_bumper_offset_would_be_caught():
    # A cone at x=1.30 m (camera-frame, radius 0), speed 0, margin_m=0.1 -> stop_distance = 0.1 m.
    # Correct (bumper-referenced): available = (1.30 - 1.0668) - 0.15 = 0.0832 m <= 0.1 m -> STOP_CANDIDATE.
    # If the camera_to_bumper_m subtraction were ever dropped: available = 1.30 - 0.15 = 1.15 m > 0.1 m ->
    # PATH_CANDIDATE instead. This test fails if that offset is removed or its sign is flipped.
    engine = PlannerEngine("BRAKING_TEST", "SIMULATED", BrakeCapability(3.0, 0.0, "SIMULATED", margin_m=0.1))
    stamp = 1_000_000_000
    mapping = {"valid": True, "source": "SIMULATED", "frame_id": "odom", "sample_time_ns": stamp,
              "geometry": {"centerline_status": "PROVISIONAL",
                           "centerline": [(float(i), 0.0) for i in range(1, 11)]}}
    result = engine.evaluate(mapping, PoseEstimate(0, 0, 0, 0.0, stamp, 0.05),
                             [{"x_m": 1.30, "y_m": 0.0, "radius_m": 0.0}])
    assert result["decision"] == "STOP_CANDIDATE", (
        "a cone 1.30 m ahead in camera coordinates is only 0.0832 m past the bumper's required stopping "
        "distance once the 1.0668 m camera-to-bumper offset is correctly subtracted -- if this is "
        "PATH_CANDIDATE, that offset was dropped or its sign flipped")
    assert result["bumper_available_m"] == pytest.approx(0.0832, abs=1e-6)


def test_skidpad_rejects_color_swapped_loop_and_incomplete_map():
    observed = topology()
    engine = PlannerEngine("SKIDPAD", "SIMULATED", BrakeCapability(3.0, 0.2, "SIMULATED"))
    stamp = 1_000_000_000
    incomplete = {"valid": True, "source": "SIMULATED", "frame_id": "odom",
                  "sample_time_ns": stamp,
                  "geometry": {"skidpad_topology": {"valid": False,
                                                   "reason": "INSUFFICIENT_LOOP_COVERAGE"}}}
    result = engine.evaluate(incomplete, pose_on(9.125, math.pi, stamp))
    assert not result["valid"] and "OBSERVED_LOOPS_UNAVAILABLE" in result["reason_codes"]
    swapped = topology()
    swapped["right"]["boundaries"]["blue"], swapped["right"]["boundaries"]["yellow"] = (
        swapped["right"]["boundaries"]["yellow"], swapped["right"]["boundaries"]["blue"])
    stamp += 20_000_000
    result = engine.evaluate(map_sample(swapped, stamp), pose_on(9.125, math.pi, stamp))
    assert not result["valid"] and "AMBIGUOUS_CROSSOVER_OR_COLORS" in result["reason_codes"]
