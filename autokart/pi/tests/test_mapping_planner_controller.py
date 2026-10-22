import math
from autokart_mapping.core import ConeTracker, MapConfig, build_corridor, odom_to_vehicle, vehicle_to_odom
from autokart_planner.core import Planner, PlannerConfig, point_on_path, required_stop_distance
from autokart_controller.core import (Controller, ControllerConfig, SpeedController, pure_pursuit, THROTTLE_V_MAX,
                                      THROTTLE_RUN_V)


def make_map(pose=(0.0, 0.0, 0.0), n=4, width=3.0, stop_x=None, hits=3):
    cfg = MapConfig()
    tr = ConeTracker(cfg)
    cones = []
    for i in range(n):
        cones += [("blue", 2.0 + i * 2.0, width / 2), ("yellow", 2.0 + i * 2.0, -width / 2)]
    if stop_x is not None:
        cones.append(("orange", stop_x, 0.0))
    for k in range(hits):
        tr.update(cones, pose, float(k))
    return cfg, tr


def test_frame_roundtrip():
    pose = (1.0, 2.0, 0.7)
    x, y = vehicle_to_odom(3.0, -1.0, pose)
    vx, vy = odom_to_vehicle(x, y, pose)
    assert abs(vx - 3.0) < 1e-9 and abs(vy + 1.0) < 1e-9


def test_corridor_straight():
    cfg, tr = make_map()
    cor = build_corridor(tr.confirmed(), (0, 0, 0), cfg)
    assert cor.valid and cor.status == "4_PAIRS" and abs(cor.width_m - 3.0) < 1e-6
    assert all(abs(y) < 1e-6 for _, y in cor.centerline) and cor.centerline[-1][0] > 8.0


def test_corridor_rotated_kart():
    # kart yawed 10 degrees but the corridor is straight ahead in the odom frame
    cfg, tr = make_map(pose=(0, 0, math.radians(10)))
    cor = build_corridor(tr.confirmed(), (0, 0, math.radians(10)), cfg)
    assert cor.valid
    # centerline runs along the corridor direction, which in vehicle frame is the -10 degree ray
    (x0, y0), (x1, y1) = cor.centerline[0], cor.centerline[-1]
    ang = math.atan2(y1 - y0, x1 - x0)
    assert abs(ang) < math.radians(15)


def test_corridor_one_pair_and_invalid():
    cfg, tr = make_map(n=1)
    cor = build_corridor(tr.confirmed(), (0, 0, 0), cfg)
    assert cor.valid and cor.status.startswith("ONE_PAIR")
    cfg2, tr2 = make_map(width=0.3)
    assert not build_corridor(tr2.confirmed(), (0, 0, 0), cfg2).valid
    assert not build_corridor([], (0, 0, 0), cfg).valid


def test_tracker_needs_hits_and_merges():
    cfg = MapConfig()
    tr = ConeTracker(cfg)
    tr.update([("blue", 3.0, 1.0)], (0, 0, 0), 0.0)
    assert tr.confirmed() == []
    tr.update([("blue", 3.05, 1.0)], (0, 0, 0), 0.1); tr.update([("blue", 2.95, 1.0)], (0, 0, 0), 0.2)
    assert len(tr.tracks) == 1 and len(tr.confirmed()) == 1


def plan_for(stop_x, v, pose=(0, 0, 0), cfg=None, active=True):
    mcfg, tr = make_map(stop_x=stop_x)
    cor = build_corridor(tr.confirmed(), pose, mcfg)
    pl = Planner(cfg or PlannerConfig(front_bumper_x_m=0.6))
    pl.set_active(active)
    return pl, pl.plan(10.0, cor.centerline, cor.valid, 0.0, [(t.cls, t.x, t.y) for t in tr.confirmed()], pose, v)


def test_planner_stop_target_is_15cm_before_cone():
    pl, r = plan_for(stop_x=9.0, v=0.0)
    assert r.stop_valid and abs(r.nearest_cone_m - (9.0 - 0.6)) < 0.05
    assert abs(r.available_m - (r.nearest_cone_m - 0.15)) < 1e-9
    assert abs(r.stop_target[0] - r.available_m) < 0.3 and r.decision == "CRUISE"


def test_planner_blue_yellow_are_not_stop_cones():
    pl, r = plan_for(stop_x=None, v=0.0)
    assert not r.stop_valid and r.decision == "CRUISE"


def test_planner_brakes_at_required_distance_and_latches():
    cfg = PlannerConfig(front_bumper_x_m=0.6, a_verified_mps2=0.5, t_latency_s=0.25, d_margin_m=0.10)
    v = 1.0
    req = required_stop_distance(v, 0.5, 0.25, 0.10)
    assert abs(req - (1.0 + 0.25 + 0.10)) < 1e-9
    # place the cone so that available is slightly less than required
    stop_x = 0.6 + 0.15 + req - 0.05
    pl, r = plan_for(stop_x=stop_x, v=v, cfg=cfg)
    assert r.decision == "BRAKE" and r.target_speed_mps == 0.0
    # latch: even if speed numbers change, it keeps braking and then STOPs once stationary
    mcfg, tr = make_map(stop_x=stop_x)
    cor = build_corridor(tr.confirmed(), (0, 0, 0), mcfg)
    cones = [(t.cls, t.x, t.y) for t in tr.confirmed()]
    assert pl.plan(11.0, cor.centerline, True, 0.0, cones, (0, 0, 0), 0.0).decision == "BRAKE"
    assert pl.plan(11.7, cor.centerline, True, 0.0, cones, (0, 0, 0), 0.0).decision == "STOP"


def test_planner_hold_when_not_armed_and_no_path_braking():
    pl, r = plan_for(stop_x=9.0, v=0.0, active=False)
    assert r.decision == "HOLD" and r.target_speed_mps == 0.0
    pl2 = Planner(PlannerConfig()); pl2.set_active(True)
    assert pl2.plan(0.0, [], False, 0.0, [], (0, 0, 0), 0.0).decision == "NO_PATH"
    assert pl2.plan(0.1, [], False, 0.0, [], (0, 0, 0), 0.5).decision == "BRAKE"


def test_planner_stale_map_brakes():
    mcfg, tr = make_map(stop_x=9.0)
    cor = build_corridor(tr.confirmed(), (0, 0, 0), mcfg)
    pl = Planner(PlannerConfig()); pl.set_active(True)
    r = pl.plan(0.0, cor.centerline, True, 2.0, [], (0, 0, 0), 0.8)
    assert r.decision == "BRAKE"


def test_pure_pursuit_signs_and_limits():
    cfg = ControllerConfig(wheelbase_m=1.0)
    straight = [(x * 0.25, 0.0) for x in range(40)]
    assert abs(pure_pursuit(straight, (0, 0, 0), 1.0, cfg).delta_deg) < 1e-6
    left_path = [(x * 0.25, 0.5) for x in range(40)]            # path is to the LEFT of the kart
    r = pure_pursuit(left_path, (0, 0, 0), 1.0, cfg)
    assert r.valid and r.delta_deg > 0
    right_path = [(x * 0.25, -0.5) for x in range(40)]
    assert pure_pursuit(right_path, (0, 0, 0), 1.0, cfg).delta_deg < 0
    far = [(x * 0.25, 5.0) for x in range(40)]
    assert abs(pure_pursuit(far, (0, 0, 0), 1.0, cfg).delta_deg) <= cfg.max_steer_deg
    # formula check: delta = atan(2 L sin(alpha) / Ld)
    r = pure_pursuit(left_path, (0, 0, 0), 1.0, cfg)
    assert abs(r.delta_deg - math.degrees(math.atan2(2 * 1.0 * math.sin(r.alpha_rad), r.lookahead_m))) < 1e-6
    assert not pure_pursuit([], (0, 0, 0), 0.0, cfg).valid


def test_speed_controller_never_exceeds_2v():
    cfg = ControllerConfig(volts_per_mps=10.0, throttle_v_max=9.0)
    sc = SpeedController(cfg)
    for k in range(200):
        assert 0.0 <= sc.step(k * 0.05, 5.0, 0.0) <= THROTTLE_V_MAX
    assert sc.step(20.0, 0.0, 0.0) == 0.0


def test_speed_controller_run_voltage_never_below_floor():
    # Default config: throttle_run_v == throttle_v_max == 2.0, so any RUN command is exactly 2.0 V (on/off).
    cfg = ControllerConfig()
    sc = SpeedController(cfg)
    for k in range(100):
        out = sc.step(k * 0.05, 1.0, 0.3 * math.sin(k * 0.1))   # varying measured speed, varying error sign
        assert out == THROTTLE_RUN_V == cfg.throttle_v_max
    assert sc.step(10.0, 0.0, 0.0) == 0.0   # target <= 0 still drops straight to 0 V

    # A wider band (run floor below the ceiling): output is 0, or clamped into [run, max], never in between.
    cfg2 = ControllerConfig(volts_per_mps=0.2, kp_v_per_mps=0.05, ki_v_per_mps_s=0.0, pi_trim_max_v=0.05,
                             throttle_run_v=1.0, throttle_v_max=2.0)
    sc2 = SpeedController(cfg2)
    for k in range(50):
        out = sc2.step(k * 0.05, 1.0, 0.0)
        assert out == 0.0 or cfg2.throttle_run_v <= out <= cfg2.throttle_v_max


def test_speed_controller_integral_does_not_wind_up_against_the_cap():
    # Degenerate on/off case: the output can never move, so the integral must not grow without bound while
    # held there — otherwise a later legitimate target change would be fighting a huge stale integral.
    cfg = ControllerConfig(volts_per_mps=1.0, ki_v_per_mps_s=0.5)
    sc = SpeedController(cfg)
    for k in range(2000):
        sc.step(k * 0.02, 1.0, 0.0)   # measured speed never catches up -> err stays positive forever
    assert abs(sc.integral) < 10.0   # would be ~20 after 2000 steps of dt=0.02 if it wound up unchecked


def test_controller_state_machine():
    c = Controller(ControllerConfig())
    path = [(x * 0.25, 0.0) for x in range(40)]
    o = c.step(0.0, False, "CRUISE", 1.0, path, (0, 0, 0), 0.0)
    assert o.state == "IDLE" and o.throttle_v == 0.0 and o.brake_pct == 100.0
    o = c.step(0.1, True, "CRUISE", 1.0, path, (0, 0, 0), 0.0)
    assert o.state == "RUN" and 0.0 < o.throttle_v <= 2.0 and o.brake_pct == 0.0
    o = c.step(0.2, True, "BRAKE", 0.0, path, (0, 0, 0), 0.5)
    assert o.state == "STOPPING" and o.throttle_v == 0.0 and o.brake_pct == 100.0
    o = c.step(0.3, True, "STOP", 0.0, path, (0, 0, 0), 0.0)
    o = c.step(1.5, True, "STOP", 0.0, path, (0, 0, 0), 0.0)
    assert o.state == "STOPPED" and o.stopped
    # a stopped controller never drives again, even if the plan says CRUISE
    o = c.step(2.0, True, "CRUISE", 1.0, path, (0, 0, 0), 0.0)
    assert o.state == "STOPPED" and o.throttle_v == 0.0
    o = c.step(2.1, False, "CRUISE", 1.0, path, (0, 0, 0), 0.0)    # disarm resets
    assert o.state == "IDLE"
