from kart_sim.my_track_algorithm import (
    AlgorithmConfig,
    add_single_side_fallback_gates,
    build_gates,
    classify_obstacles,
    detect_finish,
    evaluate_centreline,
    fit_centreline,
    footprint_clearance_m,
)

CFG = AlgorithmConfig()


def gate_pair(x: float, half_width: float = 1.5) -> list[tuple[float, float, str, int]]:
    return [(x, half_width, "blue", 0), (x, -half_width, "yellow", 0)]


def test_build_gates_connects_opposite_side_cones_within_width_and_skew() -> None:
    points = []
    for index, x in enumerate((0.0, 1.0, 2.0, 3.0)):
        points += [(x, 1.5, "blue", index * 2), (x, -1.5, "yellow", index * 2 + 1)]
    gates = build_gates(points, CFG)
    assert len(gates) >= 4
    for gate in gates:
        assert CFG.gate_width_m[0] <= gate.width_m <= CFG.gate_width_m[1]


def test_build_gates_rejects_width_outside_the_configured_band() -> None:
    # 6 m apart: wider than the 2.0-3.8 m gate band, must not form a gate.
    points = [(0.0, 3.0, "blue", 0), (0.0, -3.0, "yellow", 1)]
    assert build_gates(points, CFG) == []


def test_build_gates_rejects_same_side_pairs() -> None:
    points = [(0.0, 1.5, "blue", 0), (1.0, 1.4, "blue", 1)]
    assert build_gates(points, CFG) == []


def test_single_side_fallback_gate_added_when_one_boundary_missing() -> None:
    # Only the left (blue) boundary is visible, e.g. the right boundary left
    # the 64.4 deg HFOV during a bend.
    points = [(x, 1.5, "blue", i) for i, x in enumerate((0.0, 1.0, 2.0, 3.0))]
    real_gates = build_gates(points, CFG)
    assert real_gates == []  # no opposite-side cones at all
    fallback = add_single_side_fallback_gates(points, real_gates, CFG)
    assert len(fallback) > 0
    for gate in fallback:
        assert gate.width_m == CFG.nominal_lane_width_m


def test_fallback_gate_not_added_when_a_real_gate_is_already_close() -> None:
    points = gate_pair(0.0) + gate_pair(1.0) + [(2.0, 1.5, "blue", 9)]
    real_gates = build_gates(points, CFG)
    assert len(real_gates) >= 1
    fallback = add_single_side_fallback_gates(points, real_gates, CFG)
    # The lone extra blue cone at x=2.0 is within 0.8 m of a real gate's
    # midpoint from x=1.0, so no fallback gate should be synthesized there.
    for gate in fallback:
        assert gate.midpoint[0] != 2.0 or abs(gate.midpoint[1]) > 1.0


def test_obstacle_classifier_flags_compact_orange_cluster_not_a_straight_wall() -> None:
    cluster = [(5.0, 0.5, "large_orange", 0), (5.5, 0.5, "large_orange", 1),
               (5.0, 1.0, "large_orange", 2), (5.5, 1.0, "large_orange", 3)]
    wall = [(0.0, 1.5, "large_orange", 10), (1.0, 1.5, "large_orange", 11),
            (2.0, 1.5, "large_orange", 12), (3.0, 1.5, "large_orange", 13)]
    obstacle_indices = classify_obstacles(cluster + wall, set(), CFG)
    assert obstacle_indices == {0, 1, 2, 3}  # the wall (indices 4-7) must not be flagged


def test_finish_line_requires_40m_of_progress() -> None:
    points = [(5.0, 1.4, "large_orange", 0), (5.1, 1.0, "large_orange", 1)]
    assert detect_finish(points, odometry_m=30.0, config=CFG) == (None, set())
    finish_x, indices = detect_finish(points, odometry_m=45.0, config=CFG)
    assert finish_x is not None
    assert indices == {0, 1}


def test_centreline_fit_tracks_a_simple_curve() -> None:
    from kart_sim.my_track_algorithm import Gate
    gates = [Gate((0, 1), (0, -1), (0.0, 0.0), 2.0),
              Gate((2, 1.5), (2, -0.5), (2.0, 0.5), 2.0),
              Gate((4, 2), (4, 0), (4.0, 1.0), 2.0)]
    fit = fit_centreline(gates)
    assert fit is not None
    import numpy as np
    y_at_2 = evaluate_centreline(fit, np.array([2.0]))[0]
    assert abs(y_at_2 - 0.5) < 0.2


def test_footprint_clearance_is_zero_inside_the_rectangle_and_positive_outside() -> None:
    inside = footprint_clearance_m(1.0, 0.0, 0.0, 0.0, 0.0, rear_overhang_m=0.4, front_extent_m=2.0, half_width_m=0.6)
    outside = footprint_clearance_m(3.0, 0.0, 0.0, 0.0, 0.0, rear_overhang_m=0.4, front_extent_m=2.0, half_width_m=0.6)
    assert inside == 0.0
    assert outside == 1.0
