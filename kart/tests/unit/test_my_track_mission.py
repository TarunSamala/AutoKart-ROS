import pytest

from kart_sim.hp60c import TrackedCone
from kart_sim.my_track_mission import MissionConfig, MissionController, _max_speed_for_stop_distance_m, _stopping_distance_m


def controller() -> MissionController:
    return MissionController(
        MissionConfig(), wheelbase_m=1.422, front_extent_m=2.0, rear_overhang_m=0.4, half_width_m=0.5905,
        brake_delay_s=0.17, brake_decel_mps2=2.5,
    )


def no_centroid(_component):
    return (0.0, 0.0)


def gate_tracks(xs: tuple[float, ...] = (1.0, 2.0, 3.0, 4.0, 5.0)) -> list[TrackedCone]:
    tracks = []
    track_id = 1
    for x in xs:
        tracks.append(TrackedCone(track_id, x, 1.5, 3, {"blue": 3}, 0.0)); track_id += 1
        tracks.append(TrackedCone(track_id, x, -1.5, 3, {"yellow": 3}, 0.0)); track_id += 1
    return tracks


def test_mission_starts_ready_and_moves_to_run_on_first_step() -> None:
    mc = controller()
    assert mc.state == "READY"
    cmd = mc.step(tracks=gate_tracks(), pose=(0.0, 0.0, 0.0), speed_mps=0.0, odometry_m=0.0, dt_s=0.05, timestamp_s=0.05, world_cluster_centroid=no_centroid)
    assert mc.state == "RUN"
    assert cmd.state == "RUN"


def test_blind_creep_when_too_few_cones_near_the_start() -> None:
    mc = controller()
    cmd = mc.step(tracks=[], pose=(0.0, 0.0, 0.0), speed_mps=0.0, odometry_m=1.0, dt_s=0.05, timestamp_s=0.05, world_cluster_centroid=no_centroid)
    assert cmd.valid is False
    assert cmd.target_speed_mps == pytest.approx(mc.config.blind_creep_speed_mps)
    assert cmd.state == "RUN"


def test_path_lost_after_two_seconds_with_no_usable_gates_past_the_blind_distance() -> None:
    mc = controller()
    for step in range(60):  # 3.0 s at dt=0.05, well past blind_creep_distance_m=6.0 odometry
        cmd = mc.step(tracks=[], pose=(0.0, 0.0, 0.0), speed_mps=0.0, odometry_m=7.0, dt_s=0.05, timestamp_s=step * 0.05, world_cluster_centroid=no_centroid)
    assert cmd.path_lost is True
    assert cmd.state == "CONTROLLED_STOP"


def test_valid_plan_with_a_clear_corridor_steers_straight_and_targets_cruise_speed() -> None:
    mc = controller()
    cmd = mc.step(tracks=gate_tracks(), pose=(0.0, 0.0, 0.0), speed_mps=0.3, odometry_m=1.0, dt_s=0.05, timestamp_s=0.05, world_cluster_centroid=no_centroid)
    assert cmd.valid is True
    assert cmd.gates > 0
    assert abs(cmd.steering_rad) < 0.05  # straight corridor centred on the kart
    assert cmd.target_speed_mps == pytest.approx(mc.config.cruise_speed_mps)


def test_stopping_distance_formula_round_trips() -> None:
    distance_m = _stopping_distance_m(2.0, 0.17, 2.5, 1.5)
    assert distance_m == pytest.approx(2.0 * 0.17 + 2.0**2 / (2 * 2.5) + 1.5)
    recovered_speed_mps = _max_speed_for_stop_distance_m(distance_m, 0.17, 2.5, 1.5)
    assert recovered_speed_mps == pytest.approx(2.0, abs=1e-9)
