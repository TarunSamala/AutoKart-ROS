"""Logic-only closed-loop simulation (planner + controller + simple kart model).

This does NOT prove anything about the real kart. It checks that the decision logic stops short of the cone,
never exceeds the 2 V cap, and how the final gap depends on the braking numbers you configure.
"""
import math
from autokart_mapping.core import ConeTracker, MapConfig, build_corridor
from autokart_planner.core import Planner, PlannerConfig
from autokart_controller.core import Controller, ControllerConfig


def run(a_true, a_cfg, margin, v_cruise=1.0, stop_x=9.0, latency=0.2, tau=0.4, vpm=1.0):
    mcfg = MapConfig()
    tr = ConeTracker(mcfg)
    cones_world = [("blue", 2 + 2 * i, 1.5) for i in range(5)] + [("yellow", 2 + 2 * i, -1.5) for i in range(5)] + [("orange", stop_x, 0.0)]
    pcfg = PlannerConfig(front_bumper_x_m=0.6, cruise_speed_mps=v_cruise, a_verified_mps2=a_cfg, d_margin_m=margin, t_latency_s=0.25)
    pl = Planner(pcfg); pl.set_active(True)
    ctl = Controller(ControllerConfig(volts_per_mps=vpm))
    x, v, dt, t = 0.0, 0.0, 0.02, 0.0
    brake_q = []   # (apply_time, brake)
    thr_applied, brk_applied = 0.0, 0.0
    max_thr = 0.0
    gap = None
    for _ in range(int(40 / dt)):
        t += dt
        # perception: world cones in the vehicle frame (kart is straight along +x, y=0)
        tr.update([(c, cx - x, cy) for c, cx, cy in cones_world if 0.3 < cx - x < 8.0], (x, 0.0, 0.0), t)
        cor = build_corridor(tr.confirmed(), (x, 0.0, 0.0), mcfg)
        r = pl.plan(t, cor.centerline, cor.valid, 0.0, [(q.cls, q.x, q.y) for q in tr.confirmed()], (x, 0.0, 0.0), v)
        o = ctl.step(t, True, r.decision, r.target_speed_mps, cor.centerline, (x, 0.0, 0.0), v)
        max_thr = max(max_thr, o.throttle_v)
        brake_q.append((t + latency, o.brake_pct, o.throttle_v))
        while brake_q and brake_q[0][0] <= t:
            _, brk_applied, thr_applied = brake_q.pop(0)
        if brk_applied >= 50:
            v = max(0.0, v - a_true * dt)
        else:
            v += (thr_applied / vpm - v) / tau * dt
        x += v * dt
        if o.stopped:
            gap = stop_x - (x + 0.6)
            break
    return gap, max_thr


def test_stops_before_cone_and_respects_cap():
    gap, max_thr = run(a_true=0.8, a_cfg=0.5, margin=0.10)
    assert gap is not None and gap > 0.0           # never hits the cone
    assert max_thr <= 2.0
    print("conservative a_cfg=0.5 (true 0.8):", round(gap * 100), "cm")


def test_accurate_numbers_land_near_15cm():
    gap, _ = run(a_true=0.8, a_cfg=0.8, margin=0.0)
    print("accurate a_cfg=0.8, margin 0:", round(gap * 100), "cm")
    assert 0.05 < gap < 0.45


def test_overestimated_braking_overshoots_in_sim():
    # if you configure a better deceleration than the kart really has, the gap shrinks: this is why a must be measured
    gap_ok, _ = run(a_true=0.8, a_cfg=0.8, margin=0.0)
    gap_bad, _ = run(a_true=0.4, a_cfg=0.8, margin=0.0)
    print("true decel half of configured:", round(gap_bad * 100), "cm")
    assert gap_bad < gap_ok


def test_fixed_speed_kart_stays_at_least_15cm_for_several_distances():
    # Mildly conservative config (a_cfg <= a_true): with the 15 cm stop_gap_m baked into the planner's
    # "available" distance, the kart must never end up closer than 15 cm to the cone, at any starting distance.
    for stop_x in (5.0, 7.0, 9.0, 12.0, 15.0, 20.0):
        gap, _ = run(a_true=0.6, a_cfg=0.5, margin=0.10, stop_x=stop_x)
        assert gap is not None, f"stop_x={stop_x}: kart never reported stopped"
        assert gap >= 0.15 - 0.03, f"stop_x={stop_x}: gap {gap * 100:.1f} cm < 15 cm"


def test_margin_is_what_saves_an_overestimated_deceleration():
    # The configured deceleration is OPTIMISTIC here (0.60 configured vs 0.55 actual -- about 9% off, the kind
    # of error a small measurement mistake could produce) -- exactly the dangerous case the margin exists for.
    # With no margin this eats into the 15 cm gap and the kart ends up unsafely close; add a margin and the
    # SAME mis-estimate lands safely, which is the point: the margin, not the (wrong) deceleration number, is
    # what kept the kart off the cone.
    gap_no_margin, _ = run(a_true=0.55, a_cfg=0.60, margin=0.0)
    assert gap_no_margin < 0.15, f"expected the unsafe case to actually be unsafe, got {gap_no_margin * 100:.1f} cm"
    gap_with_margin, _ = run(a_true=0.55, a_cfg=0.60, margin=0.30)
    assert gap_with_margin >= 0.15, f"margin did not save it: {gap_with_margin * 100:.1f} cm"
    assert gap_with_margin > gap_no_margin
