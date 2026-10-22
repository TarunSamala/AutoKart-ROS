#!/usr/bin/env python3
"""Measure braking deceleration, reaction latency and volts-per-(m/s) on the real kart (no ROS).

SAFETY: open, flat, clear area. Someone holds the physical E-stop. Throttle is capped at 2.0 V and the run is aborted
(brake) if the kart travels further than --max-dist. Start with a low voltage (e.g. 0.8 V) and increase slowly.

    python3 tools/braking_test.py /dev/ttyACM0 --volts 1.0 --runs 3
"""
import argparse, os, statistics, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ros2_ws", "src", "autokart_vehicle_bridge"))
import serial
from autokart_vehicle_bridge.protocol import build_cmd, build_simple, parse_frame, parse_message

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--volts", type=float, default=1.0)
ap.add_argument("--runs", type=int, default=3)
ap.add_argument("--max-dist", type=float, default=8.0)
ap.add_argument("--cruise-s", type=float, default=3.0, help="seconds at steady speed before braking")
args = ap.parse_args()
volts = max(0.0, min(2.0, args.volts))

ser = serial.Serial(args.port, 115200, timeout=0.02)
st = {"thr": 0.0, "brake": 100, "seq": 1, "run": True, "tel": None, "t": 0.0}
lock = threading.Lock()


def send(b):
    with lock:
        ser.write(b)


def tx():
    while st["run"]:
        st["seq"] += 1
        send(build_cmd(st["seq"], int(round(st["thr"] * 1000)), 0, st["brake"]))
        time.sleep(0.05)


def rx():
    buf = b""
    while st["run"]:
        buf += ser.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            p = parse_frame(line)
            m = parse_message(p) if p else None
            if m and m[0] == "TEL":
                st["tel"] = m[1]; st["t"] = time.monotonic()


threading.Thread(target=tx, daemon=True).start()
threading.Thread(target=rx, daemon=True).start()


def wait_tel():
    t0 = time.time()
    while st["tel"] is None and time.time() - t0 < 3:
        time.sleep(0.05)
    return st["tel"] is not None


def abort(msg):
    st["thr"] = 0.0; st["brake"] = 100
    send(build_simple("DIS", st["seq"] + 5000))
    print("ABORT:", msg)
    st["run"] = False
    sys.exit(1)


if not wait_tel():
    abort("no telemetry from the VCU")
if input(f"Throttle {volts:.2f} V, {args.runs} runs, max {args.max_dist} m each. E-stop in hand and area clear? Type GO: ").strip() != "GO":
    abort("not confirmed")

results = []
for run in range(1, args.runs + 1):
    st["thr"], st["brake"] = 0.0, 100
    time.sleep(1.0)
    send(build_simple("ARM", st["seq"] + 1000))
    t0 = time.time()
    while st["tel"].state != 1 and time.time() - t0 < 3:
        time.sleep(0.05)
    if st["tel"].state != 1:
        abort("VCU did not arm (check estop, DAC stub, link)")
    c0 = st["tel"].counts
    st["brake"], st["thr"] = 0, volts
    # accelerate and hold
    t_start = time.monotonic(); v_ss = 0.0; steady_since = None
    while True:
        t = st["tel"]; v = t.speed_mmps / 1000.0
        if t.state != 1: abort("VCU left ARMED state")
        if time.monotonic() - t_start > 15: abort("did not reach steady speed")
        if steady_since is None and v > 0.05: steady_since = time.monotonic()
        if steady_since and time.monotonic() - steady_since > args.cruise_s: v_ss = v; break
        time.sleep(0.02)
    # brake
    trace = []; t_b = time.monotonic(); st["thr"], st["brake"] = 0.0, 100
    dist = 0.0; last = t_b; v_prev = v_ss
    while True:
        now = time.monotonic(); v = st["tel"].speed_mmps / 1000.0
        dist += max(v, 0) * (now - last); last = now
        trace.append((now - t_b, v))
        if v < 0.03 and now - t_b > 0.3: break
        if now - t_b > 10: abort("kart did not stop within 10 s")
        time.sleep(0.02)
    # latency: first time speed falls 5% below steady; decel from 80%..20% of v_ss
    lat = next((tt for tt, v in trace if v < 0.95 * v_ss), float("nan"))
    seg = [(tt, v) for tt, v in trace if 0.2 * v_ss <= v <= 0.8 * v_ss]
    if len(seg) >= 3:
        n = len(seg); mt = sum(a for a, _ in seg) / n; mv = sum(b for _, b in seg) / n
        slope = sum((a - mt) * (b - mv) for a, b in seg) / max(sum((a - mt) ** 2 for a, _ in seg), 1e-9)
        a_run = -slope
    else:
        a_run = v_ss / max(trace[-1][0], 1e-3)
    results.append((v_ss, a_run, lat, dist))
    print(f"run {run}: steady {v_ss:.2f} m/s, decel {a_run:.2f} m/s^2, latency {lat*1000:.0f} ms, braking distance {dist:.2f} m")
    send(build_simple("DIS", st["seq"] + 1000))
    time.sleep(1.0)
    input("Reposition the kart, then press Enter for the next run... ") if run < args.runs else None

st["thr"], st["brake"] = 0.0, 100
send(build_simple("DIS", st["seq"] + 9000)); st["run"] = False
va = [r[0] for r in results]; aa = [r[1] for r in results]; la = [r[2] for r in results if r[2] == r[2]]
print("\n=== results ===")
print(f"steady speed at {volts:.2f} V: {statistics.mean(va):.2f} m/s  ->  volts_per_mps = {volts/statistics.mean(va):.2f}")
print(f"deceleration: mean {statistics.mean(aa):.2f}, worst {min(aa):.2f} m/s^2   -> suggested a_verified_mps2 = {0.9*min(aa):.2f} (worst run x 0.9)")
if la:
    print(f"reaction latency (brake command -> speed drops): worst {max(la)*1000:.0f} ms   -> add perception+control delay to get t_latency_s")
print("Put these in config/autokart.yaml, then set braking_verified: true only if the runs are repeatable.")
