from autokart_dashboard_gateway.core import MissionStore, valid_frame
from autokart_dashboard_gateway.gateway import fsds_track_payload
from types import SimpleNamespace


def test_timestamp_join_and_ground_truth_isolation():
    store = MissionStore(max_age_s=0.25)
    store.update("cones", 1.0, [{"id": "detected", "class": "BLUE", "x_m": 1, "y_m": 2}])
    store.update("ground_truth_cones", 1.0, [{"id": "truth", "class": "BLUE", "x_m": 8, "y_m": 9}])
    store.update_debug("delaunay", 1.1, {"edges": [{"id": "e1", "a": "detected", "b": "another"}]})
    vehicle = {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 1}
    before = store.capture(1.05, vehicle)
    assert before["planning"]["delaunay_edges"] == []
    joined = store.capture(1.15, vehicle)
    assert valid_frame(joined)
    assert joined["perception"]["cones"][0]["id"] == "detected"
    assert joined["perception"]["ground_truth_cones"][0]["id"] == "truth"
    assert "truth" not in str(joined["planning"])
    stale = store.capture(1.5, vehicle)
    assert stale["perception"]["cones"] == []
    assert stale["planning"]["delaunay_edges"] == []
    assert stale["perception"]["ground_truth_cones"][0]["id"] == "truth"


def test_fsds_track_is_normalized_only_as_display_ground_truth():
    point = SimpleNamespace(x=2.0, y=-3.0, z=0.0)
    track = SimpleNamespace(track=[SimpleNamespace(color=0, location=point), SimpleNamespace(color=2, location=point)])
    cones = fsds_track_payload(track)
    assert [cone["class"] for cone in cones] == ["BLUE", "BIG_ORANGE"]
    store = MissionStore(source="SIMULATION", max_age_s=0.1)
    store.update("ground_truth_cones", 0, cones)
    frame = store.capture(20, {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0})
    assert frame["perception"]["ground_truth_cones"] == cones
    assert frame["perception"]["cones"] == []
    assert frame["planning"]["delaunay_edges"] == []


def test_real_source_rejects_simulator_ground_truth():
    store = MissionStore(source="REAL_KART")
    try:
        store.update("ground_truth_cones", 1, [])
    except ValueError:
        pass
    else:
        assert False, "real source accepted simulator ground truth"


def test_sim_scenario_uses_same_world_frame_contract():
    store = MissionStore(source="SIMULATION", scenario="SKIDPAD")
    frame = store.capture(0, {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0})
    assert frame["source"] == "SIMULATION"
    assert frame["scenario"] == "SKIDPAD"
    assert frame["planning"]["delaunay_edges"] == []
    assert store.manifest()["scenario"] == "SKIDPAD"


def test_reset_epoch_clears_stale_debug_and_bounds_history():
    store = MissionStore(limit=2)
    vehicle = {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0}
    store.update_debug("gabriel", 2, {"edges": [{"id": "old"}]})
    store.capture(2, vehicle)
    store.capture(3, vehicle)
    store.capture(4, vehicle)
    assert len(store.all_frames()) == 2
    reset = store.capture(0.1, vehicle)
    assert reset["sequence"] == 3
    assert reset["planning"]["gabriel_edges"] == []


def test_rc03_rc05_status_and_observed_map_remain_read_only_display_data():
    store = MissionStore(source="SIMULATION", max_age_s=0.25)
    store.update_debug("perception", 1.0, {"source": "SIMULATED", "valid": True,
                                             "sample_time_ns": 1_000_000_000})
    store.update_debug("mapping", 1.0, {"source": "SIMULATED", "valid": True,
                                          "frame_id": "odom", "tracked_count": 1,
                                          "expires_ns": 1_200_000_000})
    store.update_debug("planning", 1.0, {"source": "SIMULATED", "valid": False,
                                           "reason_codes": ["OBSERVED_LOOPS_UNAVAILABLE"]})
    store.update("tracked_cones", 1.0, [{"id": "1", "class": "BLUE", "x_m": 2, "y_m": 1}])
    vehicle = {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0}
    frame = store.capture(1.1, vehicle)
    assert frame["diagnostics"]["perception"]["source"] == "SIMULATED"
    assert frame["diagnostics"]["planning"]["valid"] is False
    assert len(frame["perception"]["tracked_cones"]) == 1
    assert frame["vehicle"]["target_speed_mps"] is None
    store.update("tracked_cones", 1.2, [])
    cleared = store.capture(1.25, vehicle)
    assert cleared["perception"]["tracked_cones"] == []


def test_observed_odom_geometry_and_planner_path_reach_same_world_frame():
    store = MissionStore(source="SIMULATION", scenario="SKIDPAD")
    geometry = {"observation_cones_odom": [
        {"id": 10, "label": "blue", "confidence": .9, "x_m": 2.0, "y_m": 1.5},
        {"id": 11, "label": "yellow", "confidence": .9, "x_m": 2.0, "y_m": -1.5}],
        "delaunay_edges": [[10, 11]], "gabriel_edges": [],
        "boundaries": {"left_ids": [10], "right_ids": [11], "status": "CANDIDATE"},
        "gates": [{"id": 1, "left_id": 10, "right_id": 11, "status": "CANDIDATE", "quality": .8}],
        "centerline_odom": [[2.0, 0.0], [3.0, 0.0]],
        "skidpad_topology": {"valid": False}}
    store.update_debug("mapping", 1.0, {"source": "SIMULATED", "valid": True,
                                        "frame_id": "odom", "expires_ns": 1_200_000_000,
                                        "geometry": geometry})
    store.update("tracked_cones", 1.0, [{"id": "21", "class": "BLUE", "x_m": 2, "y_m": 1.5}])
    store.update_debug("planning", 1.0, {"source": "SIMULATED", "valid": True,
                                         "frame_id": "odom", "expires_ns": 1_200_000_000,
                                         "mission_state": {
                                             "state": "RIGHT_1", "right_laps": 0, "left_laps": 0}})
    store.update("path", 1.0, [{"x_m": 2.0, "y_m": 0.0}])
    frame = store.capture(1.1, {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0})
    assert [cone["id"] for cone in frame["perception"]["cones"]] == ["10", "11"]
    assert frame["perception"]["tracked_cones"][0]["id"] == "21"
    assert frame["planning"]["delaunay_edges"][0]["a"] == "10"
    assert frame["planning"]["gabriel_edges"][0]["accepted"] is False
    assert frame["planning"]["gates"][0]["reason"] == "CANDIDATE"
    assert frame["planning"]["selected_path"] == [{"x_m": 2.0, "y_m": 0.0}]
    assert frame["visualization"]["course_guide"][0]["x_m"] == 2.0
    assert frame["visualization"]["boundary_left"][0]["y_m"] == 1.5
    assert frame["visualization"]["boundary_right"][0]["y_m"] == -1.5
    assert frame["visualization"]["boundary_status"] == "CANDIDATE"
    assert frame["planning"]["skidpad"]["state"] == "RIGHT_1"
    store.update_debug("mapping", 1.2, {"source": "SIMULATED", "valid": False,
                                        "frame_id": "odom", "geometry": {}})
    store.update_debug("planning", 1.2, {"source": "SIMULATED", "valid": False,
                                         "frame_id": "odom"})
    invalid = store.capture(1.25, {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0})
    assert invalid["perception"]["cones"] == []
    assert invalid["perception"]["tracked_cones"] == []
    assert invalid["planning"]["selected_path"] == []


def test_real_world_frame_never_accepts_simulated_mapping_visualization():
    store = MissionStore(source="REAL_KART")
    store.update_debug("mapping", 1, {"source": "SIMULATED", "valid": True,
                                      "frame_id": "odom", "geometry": {"observation_cones_odom": [
                                          {"id": 1, "label": "blue", "confidence": 1,
                                           "x_m": 1, "y_m": 1}]}})
    frame = store.capture(1, {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0})
    assert frame["perception"]["cones"] == []
    assert frame["planning"]["delaunay_edges"] == []


def test_observed_geometry_moves_with_new_published_map():
    store = MissionStore(source="REAL_KART")
    vehicle = {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0}
    def publish(time_s, x_m):
        store.update_debug("mapping", time_s, {"source": "LIVE", "valid": True,
                           "frame_id": "odom", "expires_ns": round((time_s + .2) * 1e9),
                           "geometry": {"observation_cones_odom": [
                               {"id": 1, "label": "blue", "confidence": .8,
                                "x_m": x_m, "y_m": 1}],
                               "centerline_odom": [[x_m, 0]]}})
        return store.capture(time_s, vehicle)
    first, second = publish(1.0, 2.0), publish(1.1, 3.0)
    assert first["perception"]["cones"][0]["x_m"] == 2.0
    assert second["perception"]["cones"][0]["x_m"] == 3.0
    assert second["visualization"]["course_guide"][0]["x_m"] == 3.0


def test_expired_map_and_planner_targets_are_not_rendered():
    store = MissionStore(source="REAL_KART", max_age_s=.5)
    store.update_debug("mapping", 1.0, {"source": "LIVE", "valid": True,
                       "frame_id": "odom", "expires_ns": 1_100_000_000,
                       "geometry": {"observation_cones_odom": [
                           {"id": 1, "label": "blue", "confidence": .9,
                            "x_m": 2, "y_m": 1}]}})
    store.update_debug("planning", 1.0, {"source": "LIVE", "valid": True,
                       "frame_id": "odom", "expires_ns": 1_100_000_000,
                       "mission_state": {"state": "EXIT", "right_laps": 2, "left_laps": 2}})
    store.update("path", 1.0, [{"x_m": 2, "y_m": 0}])
    vehicle = {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0}
    fresh = store.capture(1.05, vehicle)
    assert fresh["perception"]["cones"] and fresh["planning"]["selected_path"]
    expired = store.capture(1.15, vehicle)
    assert expired["perception"]["cones"] == []
    assert expired["planning"]["selected_path"] == []
    assert expired["planning"]["skidpad"] is None


def test_control_lookahead_is_hidden_when_no_current_selected_path():
    store = MissionStore(source="SIMULATION", max_age_s=.5)
    store.update("control", 1.0, {"lookahead": {"x_m": 1, "y_m": 1}})
    store.update_debug("planning", 1.0, {"source": "SIMULATED", "valid": True,
                       "frame_id": "odom", "expires_ns": 1_100_000_000,
                       "mission_state": {"state": "EXIT", "right_laps": 2, "left_laps": 2,
                                         "progress_turns": 2.0}})
    store.update("path", 1.0, [{"x_m": 1, "y_m": 1}])
    vehicle = {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 0}
    fresh = store.capture(1.05, vehicle)
    assert fresh["planning"]["lookahead"] == {"x_m": 1, "y_m": 1}
    assert fresh["planning"]["skidpad"]["progress_index"] == 2.0
    expired = store.capture(1.15, vehicle)
    assert expired["planning"]["selected_path"] == []
    assert expired["planning"]["lookahead"] is None
    store.update("control", 1.2, {"lookahead": {"x_m": 9, "y_m": 9}})
    store.update_debug("planning", 1.2, {"source": "SIMULATED", "valid": True,
                       "frame_id": "odom", "expires_ns": 1_300_000_000})
    store.update("path", 1.2, [{"x_m": 1, "y_m": 1}])
    wrong_branch = store.capture(1.25, vehicle)
    assert wrong_branch["planning"]["lookahead"] is None


def test_safety_envelope_channel_is_displayed_read_only_and_independently_fresh():
    """RC-06: /safety/envelope (DIAGNOSTIC_ONLY) reaches the frame and live_status,
    never the vcu-derived safety.state, and goes stale independently of odometry."""
    store = MissionStore(max_age_s=0.5)
    vehicle = {"x_m": 0, "y_m": 0, "yaw_rad": 0, "speed_mps": 2}
    store.update("vcu", 1.0, {"state": "ARMED"})
    store.update("safety_envelope", 1.0, {
        "state": "BRAKING", "reason": "OBSTACLE_WITHIN_BRAKING_ENVELOPE", "valid": True,
        "calibration_verified": False, "clear_distance_m": 2.0, "stopping_distance_m": 2.8,
        "guaranteed_deceleration_mps2": 2.0})
    fresh = store.capture(1.1, vehicle)
    assert fresh["safety"]["state"] == "ARMED"
    assert fresh["safety"]["envelope"] == {
        "state": "BRAKING", "reason": "OBSTACLE_WITHIN_BRAKING_ENVELOPE", "valid": True,
        "calibration_verified": False, "clear_distance_m": 2.0, "stopping_distance_m": 2.8,
        "guaranteed_deceleration_mps2": 2.0}
    stale = store.capture(1.7, vehicle)
    assert stale["safety"]["envelope"] is None
    status = store.live_status()
    assert status["safety_envelope"]["current"] is True
    assert status["safety_envelope"]["payload"]["state"] == "BRAKING"
