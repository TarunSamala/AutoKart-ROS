import os
#!/usr/bin/env python3
"""
my_track_sim.py  --  "MY TRACK" scenario for AutoKart (standalone, software-in-the-loop).

Hybrid autocross + obstacle track (Rev 6 layout): orange straights, blue-left / yellow-right S-bends,
two orange obstacle clusters, start line, finish line.  141 cones owned, 120 deployed.

Chain simulated at 100 Hz physics / 20 Hz control / 4.2 Hz perception:
  kart dynamics (dynamic bicycle + tyre relaxation, kinematic blend below 0.5 m/s)
  -> steering (stepper, rate+lag) / brake (stepper, dead-time) / BLD-750 drive (DAC volts)
  -> encoder + gyro odometry (noisy)
  -> HP60C pinhole camera model (FOV, depth range, foot-visibility, detector recall, depth noise, latency)
  -> persistent cone map -> obstacle classifier -> Delaunay gates (Gabriel = evidence only)
  -> centreline fit -> swept-footprint detour admission -> Pure Pursuit + speed/stop planner
  -> VCU speed PI, brake logic -> mission state machine.

Every number marked ASSUMED below is NOT measured on your kart.  Replace them in KartProfile / CamCfg.
Usage:  python my_track_sim.py --profile shared --scenario nominal --out out_dir
"""
import numpy as np, math, json, csv, argparse, os
from dataclasses import dataclass, field, replace
from collections import deque
from scipy.spatial import Delaunay

# ============================================================== TRACK (Rev 7, 3.0 m corridor)
W = 3.0; HW = W / 2; TH = math.radians(20); L_TOT = 50.0
SECT = {"A": (0, 4), "B": (4, 14), "C": (14, 24), "D": (24, 34), "E": (34, 44), "F": (44, 50)}
def ss(t): t = np.clip(t, 0, 1); return t * t * (3 - 2 * t)
def _theta(s): return TH * (ss((s - 4) / 5) - ss((s - 9) / 5)) - TH * (ss((s - 24) / 5) - ss((s - 29) / 5))
_S = np.linspace(0, L_TOT, 50001); _TH = _theta(_S); _DS = _S[1] - _S[0]
_X = np.cumsum(np.cos(_TH)) * _DS; _Y = np.cumsum(np.sin(_TH)) * _DS; _X -= _X[0]; _Y -= _Y[0]
CL = np.c_[_X[::20], _Y[::20]]
def cl_pt(sv, lat=0.0):
    x = np.interp(sv, _S, _X); y = np.interp(sv, _S, _Y); t = np.interp(sv, _S, _TH)
    return float(x - lat * math.sin(t)), float(y + lat * math.cos(t))
def make_track():
    cones = []
    def add(cid, col, role, sv, lat):
        x, y = cl_pt(sv, lat); cones.append(dict(id=cid, color=col, role=role, station=sv, lat=lat, x=x, y=y))
    for name in "ACEF":
        a, b = SECT[name]; n = int(round(b - a))
        for i in range(n):
            add(f"OL-{name}{i+1:02d}", "ORANGE", "straight boundary L", a + .5 + i, +HW)
            add(f"OR-{name}{i+1:02d}", "ORANGE", "straight boundary R", a + .5 + i, -HW)
    for name in "BD":
        a, b = SECT[name]
        for i in range(10):
            sv = a + .5 + i
            add(f"B-{name}{i+1:02d}", "BLUE", "bend boundary L", sv, +HW)
            add(f"Y-{name}{i+1:02d}", "YELLOW", "bend boundary R", sv, -HW)
    for pre, s0, lat0 in (("OB1-", 18.0, +0.875), ("OB2-", 38.0, -0.875)):
        k = 1
        for d_ in (0, .7, 1.4):
            for dl in (-.175, .175): add(f"{pre}{k:02d}", "ORANGE", "obstacle", s0 + d_, lat0 + dl); k += 1
    for j, lat in enumerate((-1.5, -1.15, 1.15, 1.5)):
        add(f"START-{j+1}", "ORANGE", "start line", 0.0, lat); add(f"FINISH-{j+1}", "ORANGE", "finish line", L_TOT, lat)
    return cones

# ============================================================== KART PROFILES
@dataclass
class KartProfile:
    name: str
    L: float            # wheelbase [m]
    track_f: float; track_r: float
    length: float; width: float; height: float; ground_clr: float
    rear_ext: float = 0.40                 # rear axle -> rear bumper   ASSUMED
    mass: float = 160.0                    # ASSUMED (kart + packs + electronics, no driver)
    wf: float = 0.45                       # front weight fraction       ASSUMED
    Caf: float = 7000.0; Car: float = 9000.0   # tyre cornering stiffness N/rad ASSUMED
    mu: float = 0.7                        # ASSUMED
    sigma_relax: float = 0.25              # tyre relaxation length m    ASSUMED
    steer_mech: float = 25.0               # deg, reported (road-wheel angle UNVERIFIED)
    steer_sw: float = 20.0                 # deg, software limit (project docs)
    steer_rate: float = 40.0               # deg/s, NEMA34 2:1 stepper   ASSUMED
    steer_tau: float = 0.12                # s                           ASSUMED
    F_drive_max: float = 150.0             # N, BLD-750 at low speed     ASSUMED
    dac_max_v: float = 5.0
    c_rr: float = 0.02
    brake_amax: float = 2.5                # m/s^2 at full brake (docs illustration, NOT measured)
    brake_deadtime: float = 0.15           # s  ASSUMED (docs use 0.25 s total delay)
    brake_rate: float = 3.0                # 1/s stepper travel rate     ASSUMED
    @property
    def front_ext(self): return self.length - self.rear_ext
    @property
    def b(self): return self.wf * self.L           # CG -> rear axle
    @property
    def a(self): return self.L - self.b            # CG -> front axle
    @property
    def Iz(self): return self.mass * self.a * self.b
PROFILES = {
    "shared": KartProfile("kart from shared dimensions image", L=1.422, track_f=1.0648, track_r=1.1811, length=2.400, width=1.181, height=1.118, ground_clr=0.381),
    "built":  KartProfile("newly built kart (wheelbase 1.1176 m measured)", L=1.1176, track_f=1.0668, track_r=1.1811, length=2.400, width=1.181, height=1.118, ground_clr=0.381),
}
@dataclass
class CamCfg:
    w: int = 640; h: int = 480
    hfov_deg: float = 64.4                 # HP60C datasheet: depth aligned to 640x480 RGB = 64.4 x 51.7 deg
    zmin: float = 0.20; zmax: float = 4.0  # HP60C datasheet operating range 0.2-4 m (best within 2 m)
    height: float = 0.30; pitch_deg: float = 4.0   # config baseline (docs: do not mix with 0.18 m lab value)
    cone_h: float = 0.45                   # ASSUMED
    rate_hz: float = 4.2                   # measured full-pipeline output (10-min diagnostic)
    latency: float = 0.29                  # measured frame age p50 286 ms
    latency_jitter: float = 0.03
    recall_peak: float = 0.90; px_full: float = 35.0; px_min: float = 10.0   # detector recall vs cone pixel height ASSUMED
    label_swap_p: float = 0.02
    sig_r0: float = 0.01; sig_r1: float = 0.008; sig_l0: float = 0.01; sig_l1: float = 0.005   # sigma_r = r0 + r1*range^2 (structured light) ASSUMED
    @property
    def fx(self): return (self.w / 2) / math.tan(math.radians(self.hfov_deg) / 2)
@dataclass
class Cfg:
    profile: str = "shared"; kart: KartProfile = None; cam: CamCfg = field(default_factory=CamCfg)
    v_cruise: float = 0.5; v_blind: float = 0.3; Ld_min: float = 1.8; a_comf: float = 0.4
    map_expiry: float = 10.0; gate_w: tuple = (2.0, 3.8); gate_skew: float = 45.0
    margin: float = 0.20; cone_r: float = 0.15; detour_ramp: float = 4.0
    gyro_bias_sigma: float = 3e-4; gyro_noise: float = 0.004; enc_scale_sigma: float = 0.005
    t_max: float = 400.0; name: str = "nominal"
    enable_detour: bool = True; inject: str = ""
    def __post_init__(self):
        if self.kart is None: self.kart = PROFILES[self.profile]

# ============================================================== helpers
def rect_dist(px, py, xr, yr, psi, K, half_w):
    c, s = math.cos(psi), math.sin(psi); dx = px - xr; dy = py - yr
    bx = c * dx + s * dy; by = -s * dx + c * dy
    ex = max(-K.rear_ext - bx, bx - K.front_ext, 0.0); ey = max(abs(by) - half_w, 0.0)
    return math.hypot(ex, ey)
def project(cfg, K, xr, yr, psi, px, py, z):
    """world point -> (u, v, depth_z) in HP60C pixels; camera on the nose of the kart."""
    cam = cfg.cam; th = math.radians(cam.pitch_deg); xc = K.front_ext - 0.15
    c, s = math.cos(psi), math.sin(psi); dx = px - xr; dy = py - yr
    xb = c * dx + s * dy - xc; yb = -s * dx + c * dy; dz = z - cam.height
    zc = xb * math.cos(th) - dz * math.sin(th)
    yd = -xb * math.sin(th) - dz * math.cos(th)
    xr_ = -yb
    if zc < 1e-3: return None
    f = cam.fx; return cam.w / 2 + f * xr_ / zc, cam.h / 2 + f * yd / zc, zc
COLS = {"BLUE": 0, "YELLOW": 1, "ORANGE": 2}

# ============================================================== simulation
def simulate(cfg: Cfg, seed=1, log_dt=0.1, view_dt=0.2):
    rng = np.random.default_rng(seed); K = cfg.kart; cam = cfg.cam; dt = 0.01
    cones = make_track(); CP = np.array([[c["x"], c["y"]] for c in cones])
    if cfg.inject == "no_obstacle_class": pass
    # state: CG position
    psi = 0.0; x = K.b; y = 0.0; vx = 0.0; vy = 0.0; r = 0.0; Fyf = 0.0; Fyr = 0.0
    delta = 0.0; steer_cmd_v = 0.0; thr = 0.0; thr_cmd = 0.0; brake_pos = 0.0; brake_q = deque([0.0] * int(K.brake_deadtime / dt))
    brake_cmd = 0.0
    # estimator
    bias = rng.normal(0, cfg.gyro_bias_sigma); scale = 1 + rng.normal(0, cfg.enc_scale_sigma)
    xe = 0.0; ye = 0.0; pe = 0.0; odom_dist = 0.0
    pose_hist = deque(maxlen=400)
    # perception / map
    tracks = []; pending = []; next_cap = 0.3; det_ids_t = {}; det_count = {}; vis_count = {}
    stats = dict(frames=0, dets=0, truth_visible=0, delivered=0, age_ms=[])
    # controller
    I = 0.0; state = "READY"; fault = ""; path_lost_t = 0.0; locks = []; last_plan = None
    plan = dict(valid=False, steer=0.0, v_cmd=0.0, gates=0, path=None, state="READY", obstacle=False, finish=False)
    logs = []; views = []; events = []; hit = set(); min_clr = 9.0; clr_now = 9.0
    t = 0.0; next_ctrl = 0.0; next_log = 0.0; next_view = 0.0; stop_since = None; started = False; max_dec = 0.0
    prev_vx = 0.0; rate_pk = 0.0; prev_delta = 0.0; cte_max = 0.0; sq = 0.0; nsq = 0
    steer_max = 0.0; vmax = 0.0; cte_out = 0.0; t_start = None; xr = 0.0; yr = 0.0
    det_recent = []; vx_meas = 0.0

    def rear_axle(): return x - K.b * math.cos(psi), y - K.b * math.sin(psi)

    while t < cfg.t_max:
        xr, yr = rear_axle()
        # ---------------- perception capture
        if t >= next_cap:
            next_cap += 1.0 / cam.rate_hz; stats["frames"] += 1
            dets = []; ids = []
            for c in cones:
                pr = project(cfg, K, xr, yr, psi, c["x"], c["y"], 0.0)
                if pr is None: continue
                u, v, zc = pr
                if not (cam.zmin <= zc <= cam.zmax and 0 <= u < cam.w and 0 <= v < cam.h): continue
                pt = project(cfg, K, xr, yr, psi, c["x"], c["y"], cam.cone_h)
                hpx = v - pt[1] if pt else 0
                stats["truth_visible"] += 1; vis_count[c["id"]] = vis_count.get(c["id"], 0) + 1
                p = cam.recall_peak * float(np.clip((hpx - cam.px_min) / (cam.px_full - cam.px_min), 0, 1))
                if rng.random() > p: continue
                cc, sn = math.cos(psi), math.sin(psi); dx = c["x"] - xr; dy = c["y"] - yr
                xb = cc * dx + sn * dy; yb = -sn * dx + cc * dy; rr = math.hypot(xb, yb)
                sr = cam.sig_r0 + cam.sig_r1 * rr * rr; sl = cam.sig_l0 + cam.sig_l1 * rr
                n1, n2 = rng.normal(0, sr), rng.normal(0, sl)
                xm = xb + n1 * xb / rr - n2 * yb / rr; ym = yb + n1 * yb / rr + n2 * xb / rr
                col = c["color"]
                if rng.random() < cam.label_swap_p: col = {"ORANGE": "YELLOW", "YELLOW": "ORANGE", "BLUE": "UNKNOWN"}[col]
                dets.append((xm, ym, col)); ids.append(c["id"])
            lat = max(0.15, cam.latency + rng.normal(0, cam.latency_jitter))
            pending.append((t + lat, t, dets, ids, (xe, ye, pe)))
        for item in list(pending):
            td, tc, dets, ids, pose_c = item
            if t >= td:
                pending.remove(item); stats["delivered"] += 1; stats["age_ms"].append((td - tc) * 1000); stats["dets"] += len(dets)
                for i_ in ids: det_ids_t[i_] = t; det_count[i_] = det_count.get(i_, 0) + 1
                cx, cy, cp = pose_c
                for (xm, ym, col) in dets:
                    if col == "UNKNOWN": continue
                    wx = cx + math.cos(cp) * xm - math.sin(cp) * ym; wy = cy + math.sin(cp) * xm + math.cos(cp) * ym
                    best = None; bd = 0.6
                    for tr in tracks:
                        d = math.hypot(tr["x"] - wx, tr["y"] - wy)
                        if d < bd: bd = d; best = tr
                    if best is None: tracks.append(dict(x=wx, y=wy, n=1, votes={col: 1}, last=t))
                    else:
                        a = max(1.0 / (best["n"] + 1), 0.2); best["x"] += a * (wx - best["x"]); best["y"] += a * (wy - best["y"])
                        best["n"] += 1; best["votes"][col] = best["votes"].get(col, 0) + 1; best["last"] = t
                tracks = [tr for tr in tracks if t - tr["last"] <= cfg.map_expiry]
        # ---------------- control cycle (20 Hz): planner + VCU
        if t >= next_ctrl:
            next_ctrl += 0.05
            pose_e = (xe, ye, pe)
            plan = plan_step(cfg, K, tracks, pose_e, vx_meas if t > 0 else 0.0, odom_dist, t, locks, state, path_lost_t) if t > 0 else plan
            if t > 0 and plan.get("path_lost_inc"): path_lost_t += 0.05
            elif t > 0: path_lost_t = 0.0
            # mission state machine
            if state == "READY" and t > 1.0: state = "RUN"; t_start = t; events.append((t, "START", "mission start (rear axle at start line)"))
            if state in ("RUN", "OBSTACLE_ASSESS", "DETOUR", "RECENTER", "FINISH_BRAKING", "CONTROLLED_STOP", "PATH_LOST"):
                ns = plan["state"]
                if path_lost_t > 2.0 and not fault: fault = "PATH_LOST > 2 s"; events.append((t, "FAULT", fault))
                if fault: ns = "CONTROLLED_STOP"
                if ns != state: events.append((t, "STATE", f"{state} -> {ns}"))
                state = ns
            v_cmd = plan["v_cmd"] if state not in ("READY",) else 0.0
            if fault: v_cmd = 0.0
            steer_cmd = plan["steer"] if plan["valid"] or plan.get("blind") else 0.0
            if state == "READY": steer_cmd = 0.0
            steer_cmd = float(np.clip(steer_cmd, -K.steer_sw, K.steer_sw)); steer_cmd_v = steer_cmd
            # speed PI (VCU)
            e = v_cmd - vx_meas if t > 0 else 0.0
            I = float(np.clip(I + e * 0.05, -0.5, 0.8))
            u = 1.4 * e + 0.8 * I + (0.12 if v_cmd > 0.05 else 0.0)
            if v_cmd < 0.02 and vx_meas < 0.04: u = -0.6; I = 0.0
            if u >= -0.03: thr_cmd = float(np.clip(u, 0, 1)); brake_cmd = 0.0
            else: thr_cmd = 0.0; brake_cmd = float(np.clip(-u, 0, 1))
            if v_cmd < 0.02 and vx_meas > 0.15: brake_cmd = max(brake_cmd, 0.5)
            last_plan = plan
            pose_hist.append((t, xe, ye, pe))
        # ---------------- actuators
        tgt = float(np.clip(steer_cmd_v, -K.steer_mech, K.steer_mech))
        step = K.steer_rate * dt; dd = (tgt - math.degrees(delta)) * (dt / K.steer_tau)
        delta_d = math.degrees(delta) + float(np.clip(dd, -step, step)); delta = math.radians(delta_d)
        thr += (thr_cmd - thr) * dt / 0.10
        brake_q.append(brake_cmd); bc = brake_q.popleft()
        brake_pos += float(np.clip(bc - brake_pos, -K.brake_rate * dt, K.brake_rate * dt))
        # ---------------- vehicle physics
        Fzf = K.mass * 9.81 * K.b / K.L; Fzr = K.mass * 9.81 * K.a / K.L
        vxs = max(vx, 0.5)
        af = math.atan2(vy + K.a * r, vxs) - delta; ar = math.atan2(vy - K.b * r, vxs)
        Fyf_ss = -K.mu * Fzf * math.tanh(K.Caf * af / (K.mu * Fzf)); Fyr_ss = -K.mu * Fzr * math.tanh(K.Car * ar / (K.mu * Fzr))
        Fyf += (vxs / K.sigma_relax) * (Fyf_ss - Fyf) * dt; Fyr += (vxs / K.sigma_relax) * (Fyr_ss - Fyr) * dt
        Fdrive = thr * K.F_drive_max
        Fbrake = brake_pos * K.brake_amax * K.mass
        Fres = K.c_rr * K.mass * 9.81 + 0.5 * 1.2 * 0.6 * vx * vx * 0.5
        if vx > 0.001: Fx = Fdrive - Fbrake - Fres
        else: Fx = max(Fdrive - Fres - 0.0, 0.0)
        dvx = (Fx - Fyf * math.sin(delta)) / K.mass + vy * r
        dvy = (Fyf * math.cos(delta) + Fyr) / K.mass - vx * r
        dr = (K.a * Fyf * math.cos(delta) - K.b * Fyr) / K.Iz
        w = float(np.clip((vx - 0.5) / 0.5, 0, 1)); rk = vx * math.tan(delta) / K.L
        r = w * (r + dr * dt) + (1 - w) * rk; vy = w * (vy + dvy * dt) + (1 - w) * K.b * rk
        vx = max(0.0, vx + dvx * dt)
        psi += r * dt; x += (vx * math.cos(psi) - vy * math.sin(psi)) * dt; y += (vx * math.sin(psi) + vy * math.cos(psi)) * dt
        # ---------------- sensors / odometry
        vx_meas = round(vx * scale / 0.005) * 0.005 + rng.normal(0, 0.004)
        gyro = r + bias + rng.normal(0, cfg.gyro_noise)
        pe += gyro * dt; xe += vx_meas * math.cos(pe) * dt; ye += vx_meas * math.sin(pe) * dt; odom_dist += max(vx_meas, 0) * dt
        # ---------------- safety metrics (truth)
        xr, yr = rear_axle()
        if int(round(t / dt)) % 5 == 0:
            m = 9.0
            for c in cones:
                d = rect_dist(c["x"], c["y"], xr, yr, psi, K, K.width / 2) - cfg.cone_r
                m = min(m, d)
                if d < 0.0 and c["id"] not in hit: hit.add(c["id"]); events.append((t, "COLLISION", f"kart footprint touched cone {c['id']}"))
            clr_now = m; min_clr = min(min_clr, m)
            j = int(np.argmin(np.hypot(CL[:, 0] - xr, CL[:, 1] - yr))); cte = float(math.hypot(CL[j, 0] - xr, CL[j, 1] - yr)); stn = j * 20 * _DS
            if not (12.5 < stn < 25.5 or 32.5 < stn < 45.5): cte_out = max(cte_out, cte)
            if started or vx > 0.05: started = True; cte_max = max(cte_max, cte); sq += cte * cte; nsq += 1
        dec = (prev_vx - vx) / dt; max_dec = max(max_dec, dec); prev_vx = vx
        rate_pk = max(rate_pk, abs(delta_d - prev_delta) / dt); prev_delta = delta_d
        steer_max = max(steer_max, abs(delta_d)); vmax = max(vmax, vx)
        # ---------------- logging
        if t >= next_log:
            next_log += log_dt
            s_prog = odom_dist
            sec = next((k for k, (a, b) in SECT.items() if a <= s_prog < b), "F")
            gd = plan.get("gates", 0); age = float(np.mean(stats["age_ms"][-5:])) if stats["age_ms"] else 0.0
            j = int(np.argmin(np.hypot(CL[:, 0] - xr, CL[:, 1] - yr))); sgn = 1 if ((xr - CL[j, 0]) * -math.sin(_TH[min(j*20,len(_TH)-1)]) + (yr - CL[j, 1]) * math.cos(_TH[min(j*20,len(_TH)-1)])) > 0 else -1
            logs.append(dict(t=round(t, 2), dist_m=round(odom_dist, 2), x=round(xr, 3), y=round(yr, 3), heading_deg=round(math.degrees(psi), 2),
                speed_mps=round(vx, 3), speed_kmh=round(vx * 3.6, 2), speed_target_mps=round(plan["v_cmd"] if state != "READY" else 0, 3),
                steer_cmd_deg=round(steer_cmd_v, 2), steer_deg=round(delta_d, 2), steer_rate_dps=round((delta_d - (logs[-1]["steer_deg"] if logs else 0)) / log_dt, 1),
                brake_pct=round(brake_pos * 100, 1), brake_cmd_pct=round(brake_cmd * 100, 1), decel_mps2=round(max(dec, 0) if brake_pos > 0.02 else 0, 2),
                throttle_pct=round(thr * 100, 1), dac_V=round(thr * K.dac_max_v, 2), section=sec, state=state, gates=gd, map_cones=len(tracks),
                frame_age_ms=round(age), centreline_offset_m=round(sgn * float(math.hypot(CL[j, 0] - xr, CL[j, 1] - yr)), 3), clearance_m=round(clr_now, 3),
                obstacle="yes" if plan.get("obstacle") else "no", detour_offset_m=round(plan.get("offset0", 0.0), 2)))
        if t >= next_view:
            next_view += view_dt
            cm = []
            for c in cones:
                pr = project(cfg, K, xr, yr, psi, c["x"], c["y"], 0.0)
                if pr is None: continue
                u, v, zc = pr
                if not (0 <= u < cam.w and -60 <= v < cam.h + 60 and zc < 12): continue
                pt = project(cfg, K, xr, yr, psi, c["x"], c["y"], cam.cone_h)
                cm.append([round(u, 1), round(v, 1), round(v - pt[1], 1), COLS[c["color"]], 1 if t - det_ids_t.get(c["id"], -9) < 1.0 else 0, round(zc, 2)])
            pp = plan.get("path"); pw = []
            if pp is not None:
                c_, s_ = math.cos(pe), math.sin(pe)
                for (px_, py_) in pp[::10][:16]: pw.append([round(xe + c_ * px_ - s_ * py_, 2), round(ye + s_ * px_ + c_ * py_, 2)])
            views.append(dict(t=round(t, 2), x=round(xr, 3), y=round(yr, 3), psi=round(psi, 4), v=round(vx, 3), vt=round(plan["v_cmd"], 3), sc=round(steer_cmd_v, 2), s=round(delta_d, 2),
                b=round(brake_pos * 100, 1), th=round(thr * 100, 1), st=state, sec=sec, cam=cm, path=pw, clr=round(clr_now, 2), g=plan.get("gates", 0), tr=len(tracks)))
        # ---------------- termination
        if state in ("RUN", "OBSTACLE_ASSESS", "DETOUR", "RECENTER", "FINISH_BRAKING") and plan.get("done"):
            state = "STOPPED"; events.append((t, "STOP", "mission complete: vehicle stopped at finish"))
        if state == "CONTROLLED_STOP" and vx < 0.02:
            if stop_since is None: stop_since = t
            if t - stop_since > 2.0: events.append((t, "END", "controlled stop reached, run ended")); break
        else: stop_since = None if state != "STOPPED" else stop_since
        if state == "STOPPED":
            if stop_since is None: stop_since = t
            if t - stop_since > 3.0: break
        t += dt
    # ---- final metrics
    xr, yr = rear_axle()
    front = (xr + K.front_ext * math.cos(psi), yr + K.front_ext * math.sin(psi))
    fin = np.array(cl_pt(L_TOT))
    stop_gap = float(fin[0] - front[0])
    metrics = dict(scenario=cfg.name, kart=K.name, wheelbase_m=K.L, outcome=("COMPLETE" if state == "STOPPED" else "ABORTED: " + (fault or state)),
        run_time_s=round(t - (t_start or 0), 1), collisions=len(hit), min_clearance_m=round(min_clr, 3), max_offset_total_m=round(cte_max, 3), max_offset_outside_detours_m=round(cte_out, 3), max_steer_deg=round(steer_max, 1), peak_steer_rate_dps=round(rate_pk, 1),
        max_speed_mps=round(vmax, 3), max_decel_mps2=round(max_dec, 2), front_bumper_to_finish_m=round(stop_gap, 2),
        frame_age_ms_mean=round(float(np.mean(stats["age_ms"])), 0) if stats["age_ms"] else 0, frames=stats["frames"],
        detection_recall_in_view=round(stats["dets"] / max(stats["truth_visible"], 1), 3))
    return dict(logs=logs, views=views, events=events, metrics=metrics, cones=cones, det_count=det_count, vis_count=vis_count)

vx_meas = 0.0
# ============================================================== planner
def plan_step(cfg, K, tracks, pose_e, v, odom_dist, t, locks, state, path_lost_t):
    global vx_meas
    xe, ye, pe = pose_e; cam = cfg.cam
    out = dict(valid=False, steer=0.0, v_cmd=0.0, gates=0, path=None, state=state if state != "READY" else "RUN", obstacle=False, finish=False)
    c_, s_ = math.cos(pe), math.sin(pe)
    pts = []
    for tr in tracks:
        if tr["n"] < 2: continue
        dx = tr["x"] - xe; dy = tr["y"] - ye; xv = c_ * dx + s_ * dy; yv = -s_ * dx + c_ * dy
        col = max(tr["votes"], key=tr["votes"].get)
        if -2.5 < xv < 12.0 and abs(yv) < 7.0: pts.append([xv, yv, col])
    if len(pts) < 3:
        return _blind(cfg, out, odom_dist)
    P = np.array([[p[0], p[1]] for p in pts]); cols = [p[2] for p in pts]
    # ---- obstacle classifier: compact orange cluster (>=3 orange neighbours within 1.0 m)
    # ---- finish line (evaluated first): orange pair 0.2-0.6 m apart at the outer edge of the lane, only after 40 m of progress
    fx = None; fin_idx = set()
    org0 = [i for i, c in enumerate(cols) if c == "ORANGE"]
    if odom_dist > 40:
        pr = []
        for ii in range(len(org0)):
            for jj in range(ii + 1, len(org0)):
                a, b = org0[ii], org0[jj]
                d = P[a] - P[b]; dist = math.hypot(*d)
                if 0.2 <= abs(d[1]) <= 0.6 and abs(d[0]) < 0.7 and P[a][0] > 0.5 and P[b][0] > 0.5 and abs(P[a][1] + P[b][1]) / 2 > 1.1:
                    pr.append((P[a][0] + P[b][0]) / 2); fin_idx.update((a, b))
        if len(pr) >= 1: fx = float(np.mean(pr))
    org = [i for i in org0 if i not in fin_idx]; isobs = set(); seen_ = set()
    for i in org:
        if i in seen_: continue
        comp = [i]; fr = [i]; seen_.add(i)
        while fr:
            a_ = fr.pop()
            for j in org:
                if j not in seen_ and math.hypot(*(P[a_] - P[j])) <= 0.85: seen_.add(j); comp.append(j); fr.append(j)
        yr_ = float(P[comp][:, 1].max() - P[comp][:, 1].min())
        if len(comp) >= 4 and yr_ >= 0.28: isobs.update(comp)   # collinear wall segments (small lateral range) are boundary, not obstacle
    bidx = [i for i in range(len(pts)) if i not in isobs and i not in fin_idx]
    if len(bidx) < 3: return _blind(cfg, out, odom_dist)
    B = P[bidx]; side = []
    for i in bidx:
        side.append("L" if cols[i] == "BLUE" else "R" if cols[i] == "YELLOW" else ("L" if P[i][1] > 0 else "R"))
    try: tri = Delaunay(B)
    except Exception: return _blind(cfg, out, odom_dist)
    E = set()
    for sx in tri.simplices:
        for a, b in ((0, 1), (1, 2), (0, 2)): E.add(tuple(sorted((sx[a], sx[b]))))
    mids = []; widths = []
    for i, j in E:
        if side[i] == side[j]: continue
        d = B[i] - B[j]; w = math.hypot(*d)
        if not (cfg.gate_w[0] <= w <= cfg.gate_w[1]): continue
        if math.degrees(math.atan2(abs(d[0]), abs(d[1]))) > cfg.gate_skew: continue
        mids.append((B[i] + B[j]) / 2); widths.append(w)
    # ---- single-side fallback gates: boundary cone offset by half the nominal lane width along its local normal
    Wn = 3.0; real = [np.array(m_) for m_ in mids]; syn = 0
    for sd_, sgn in (("L", -1.0), ("R", 1.0)):
        idx = sorted([k for k in range(len(B)) if side[k] == sd_ and B[k][0] > -1.0], key=lambda k: B[k][0])
        for q, k in enumerate(idx):
            a_ = B[idx[max(q - 1, 0)]]; b_ = B[idx[min(q + 1, len(idx) - 1)]]
            tv = b_ - a_
            if np.hypot(*tv) < 0.5 or tv[0] <= 0: continue
            tv = tv / np.hypot(*tv); nv = np.array([tv[1], -tv[0]]) * (-sgn)
            mpt = B[k] + nv * 0.5 * Wn
            if all(np.hypot(*(mpt - r_)) > 0.8 for r_ in real):
                mids.append(mpt); widths.append(Wn); syn += 1
    out["gates"] = len(mids)
    if len(mids) < 2: return _blind(cfg, out, odom_dist)
    M = np.array(mids); M = M[np.argsort(M[:, 0])]
    deg = 3 if len(M) >= 8 else 2 if len(M) >= 5 else 1
    w_ = np.ones(len(M))
    for _ in range(5):
        co = np.polyfit(M[:, 0], M[:, 1], deg, w=w_); res = np.abs(np.polyval(co, M[:, 0]) - M[:, 1]); w_ = 1 / np.maximum(1, res / 0.15)
    xlo, xhi = float(M[0, 0]), float(M[-1, 0]); dco = np.polyder(co)
    def yc(x):
        x = np.asarray(x, float); xc = np.clip(x, xlo, xhi); y = np.polyval(co, xc)
        sl_lo = float(np.clip(np.polyval(dco, xlo), -.6, .6)); sl_hi = float(np.clip(np.polyval(dco, xhi), -.6, .6))
        return np.where(x < xlo, y + sl_lo * (x - xlo), np.where(x > xhi, y + sl_hi * (x - xhi), y))
    Wh = float(np.median(widths)) / 2
    Ld = max(cfg.Ld_min, min(3.0, 1.2 + 2.0 * max(v, 0.0)))
    xs = np.arange(0.0, max(xhi, 1.0) + Ld + 3.0, 0.1); ys = yc(xs); off = np.zeros_like(xs)
    # ---- obstacle clusters -> detour planning
    clusters = []; rem = set(isobs)
    while rem:
        s0 = rem.pop(); comp = [s0]; fr = [s0]
        while fr:
            a = fr.pop()
            for b in list(rem):
                if math.hypot(*(P[a] - P[b])) <= 1.0: rem.remove(b); comp.append(b); fr.append(b)
        clusters.append(comp)
    stop_obs = None; offset_flag = False
    for comp in clusters:
        C = P[comp]; xmin, xmax = float(C[:, 0].min()), float(C[:, 0].max())
        if xmax < -3.0: continue
        l = C[:, 1] - yc(C[:, 0]); sd = 1 if l.mean() > 0 else -1; lcn = float(np.min(np.abs(l)))
        A = (-sd * Wh + sd * lcn) / 2
        out["obstacle"] = xmax > -0.5
        free = Wh + lcn - 2 * cfg.cone_r
        key = None
        for lk in locks:
            if math.hypot(lk["cx"] - (xe + c_ * C[:, 0].mean() - s_ * C[:, 1].mean()), lk["cy"] - (ye + s_ * C[:, 0].mean() + c_ * C[:, 1].mean())) < 2.0: key = lk
        def ramp(D, R=None):
            R = cfg.detour_ramp if R is None else R
            xe_ = xmin - D; xs_ = xe_ - R; xe2 = xmax + 0.4; xd2 = xe2 + R
            return A * (ss((xs - xs_) / (xe_ - xs_)) - ss((xs - xe2) / (xd2 - xe2))), xs_
        ok = False; Dsel = None; Rsel = None
        if cfg.enable_detour and free >= K.width + 2 * 0.25:
            if key is not None: Dsel = key["D"]; Rsel = key.get("R"); ok = True
            else:
                for R in (cfg.detour_ramp, 3.0, 2.4, 2.0):
                  for D in np.arange(0.0, K.front_ext + 3.0, 0.25):
                    o, xs_ = ramp(D, R)
                    if xs_ < -0.2: break
                    yy = ys + o; hd = np.arctan2(np.gradient(yy), np.gradient(xs)); good = True
                    for k in range(0, len(xs), 2):
                        for q in P:
                            if q[0] > xs[k] + K.front_ext + 1 or q[0] < xs[k] - 2: continue
                            if rect_dist(q[0], q[1], xs[k], yy[k], hd[k], K, K.width / 2) - cfg.cone_r < cfg.margin: good = False; break
                        if not good: break
                    if good:
                        ok = True; Dsel = float(D); Rsel = R
                        locks.append(dict(cx=xe + c_ * C[:, 0].mean() - s_ * C[:, 1].mean(), cy=ye + s_ * C[:, 0].mean() + c_ * C[:, 1].mean(), D=Dsel, R=R)); break
                  if ok: break
        if ok:
            o, _ = ramp(Dsel, Rsel); off = off + o; offset_flag = True
        else:
            d_stop = xmin - K.front_ext - 0.5
            stop_obs = d_stop if stop_obs is None else min(stop_obs, d_stop); out["obstacle"] = True
    yy = ys + off; path = np.c_[xs, yy]
    out["offset0"] = float(off[0]); out["path"] = path
    # ---- pure pursuit
    dist = np.hypot(path[:, 0], path[:, 1]); k = int(np.argmax(dist >= Ld)) if (dist >= Ld).any() else len(path) - 1
    tx, ty = path[k]; al = math.atan2(ty, tx); delta = math.degrees(math.atan2(2 * K.L * math.sin(al), max(math.hypot(tx, ty), 0.5)))
    # ---- speed planner
    hd = np.arctan2(np.gradient(yy), np.gradient(xs)); kap = np.abs(np.gradient(hd) / 0.1); kmax = float(kap[: int(5 / 0.1)].max()) if len(kap) else 0
    v_cmd = cfg.v_cruise
    if kmax > 1e-3: v_cmd = min(v_cmd, math.sqrt(0.6 / kmax))
    st = "RUN"
    if out["obstacle"] or offset_flag: st = "OBSTACLE_ASSESS"
    if offset_flag and (abs(off[0]) > 0.05 or (off[:int(Ld / 0.1) + 5] != 0).any()): st = "DETOUR"
    if offset_flag and any(c_k[:, 0].max() < 0 for c_k in [P[c] for c in clusters]) and abs(off[0]) > 0.03: st = "RECENTER"
    if stop_obs is not None:
        v_cmd = min(v_cmd, math.sqrt(2 * cfg.a_comf * max(stop_obs, 0))); st = "CONTROLLED_STOP"
    done = False
    if fx is not None:
        d_stop = fx - K.front_ext - 0.3; v_cmd = min(v_cmd, math.sqrt(2 * cfg.a_comf * max(d_stop, 0))); st = "FINISH_BRAKING"; out["finish"] = True
        if d_stop < 0.1 and v < 0.03: done = True
    out.update(valid=True, steer=delta, v_cmd=v_cmd, state=st, done=done, gates=len(mids))
    return out
def _blind(cfg, out, odom_dist):
    if odom_dist < 6.0 and out.get("gates", 0) < 2:
        out.update(valid=False, blind=True, steer=0.0, v_cmd=cfg.v_blind, state="RUN", path=None)
    else:
        out.update(valid=False, steer=0.0, v_cmd=0.0, state="PATH_LOST", path_lost_inc=True)
    return out
# vx_meas is set by simulate through the module global (kept simple for a single-process SIL)
def _hook():
    pass

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--profile", default="shared"); ap.add_argument("--out", default="out"); ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    res = simulate(Cfg(profile=a.profile), seed=a.seed)
    json.dump(res["metrics"], open(os.path.join(a.out, "metrics.json"), "w"), indent=1)
    print(json.dumps(res["metrics"], indent=1))
