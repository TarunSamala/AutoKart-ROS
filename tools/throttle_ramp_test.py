#!/usr/bin/env python3
"""Throttle ramp test (no ROS, no new protocol messages): step the DAC from 0 V up to the configured cap in
0.1 V increments, holding each step for 3 s, watching telemetry and asking about the driver alarm LED at
every step. Stops and returns to 0 V the moment the alarm is reported, the operator hits Ctrl-C, or the VCU
itself faults. Logs the voltage where the motor first turns and where any alarm first appears.

SAFETY: wheels OFF THE GROUND, physical battery/supply power cut within reach. This requires a typed "YES" before it sends anything.

    python3 tools/throttle_ramp_test.py /dev/ttyACM0 [--max-v 2.0] [--step-v 0.1] [--hold-s 3.0]
"""
import argparse, os, sys, threading, time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "kart", "ros2_ws", "src", "autokart_vehicle_bridge"))
import serial
from autokart_vehicle_bridge.protocol import NAK_TEXT, build_cmd, build_simple, parse_frame, parse_message

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--max-v", type=float, default=2.0,
                help="must not exceed the configured cap (vcu_firmware/include/config.h THROTTLE_MAX_MV)")
ap.add_argument("--step-v", type=float, default=0.1)
ap.add_argument("--hold-s", type=float, default=3.0)
args = ap.parse_args()

ser = serial.Serial(args.port, 115200, timeout=0.02)
st = {"thr": 0.0, "brake": 100, "seq": 1, "run": True, "tel": None}
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
            if not m:
                continue
            if m[0] == "TEL":
                st["tel"] = m[1]
            elif m[0] == "NAK":
                print(f"\nVCU refused request: {NAK_TEXT.get(m[1][1], m[1][1])}")


threading.Thread(target=tx, daemon=True).start()
threading.Thread(target=rx, daemon=True).start()


def wait_tel(timeout=3.0):
    t0 = time.time()
    while st["tel"] is None and time.time() - t0 < timeout:
        time.sleep(0.05)
    return st["tel"] is not None


if not wait_tel():
    print("ABORT: no telemetry from the VCU"); st["run"] = False; sys.exit(1)

print("=== Throttle ramp test ===")
print(f"Steps of {args.step_v:.2f} V, holding {args.hold_s:.1f} s each, up to {args.max_v:.2f} V.")
print("The wheels must be OFF THE GROUND and free to spin. Keep the physical battery/supply power cut within reach.")
print("At each step you will be asked whether the BLD-750 driver's alarm LED is on.")
if input("Type YES to confirm wheels are off the ground and physical power cut is within reach: ").strip() != "YES":
    print("ABORT: not confirmed"); st["run"] = False; sys.exit(1)

send(build_simple("ARM", st["seq"] + 1000))
t0 = time.time()
while st["tel"].state != 1 and time.time() - t0 < 3:
    time.sleep(0.05)
if st["tel"].state != 1:
    print("ABORT: VCU did not arm (check E-stop, DAC, command link)")
    st["thr"], st["brake"] = 0.0, 100
    send(build_simple("DIS", st["seq"] + 5000)); time.sleep(0.2); st["run"] = False
    sys.exit(1)

motor_turn_v = None   # first commanded voltage where the axle was observed turning
alarm_v = None        # first commanded voltage where the operator reported the alarm LED on
stop_reason = None
v = 0.0
try:
    while v <= args.max_v + 1e-9:
        if st["tel"].state != 1:
            stop_reason = f"VCU left ARMED state at {v:.2f} V (fault: {st['tel'].fault_names()})"
            break
        st["thr"], st["brake"] = v, 0
        t_step = time.time()
        faulted_mid_hold = False
        while time.time() - t_step < args.hold_s:
            if st["tel"].state != 1:
                faulted_mid_hold = True
                break
            time.sleep(0.05)
        if faulted_mid_hold:
            stop_reason = f"VCU left ARMED state at {v:.2f} V (fault: {st['tel'].fault_names()})"
            break

        tel = st["tel"]
        speed_mps = tel.speed_mmps / 1000.0
        print(f"\n{v:.2f} V commanded -> dac_mv={tel.dac_mv}  speed={speed_mps:.3f} m/s  "
              f"en={int(tel.en)} brk={int(tel.brk)}  fault={tel.fault_names() or '-'}")
        if motor_turn_v is None and abs(tel.speed_mmps) > 10:   # > 1 cm/s counts as "turning"
            motor_turn_v = v
            print(f"  -> motor first turning at {v:.2f} V")

        ans = input("  driver alarm LED on? (y/n): ").strip().lower()
        if ans == "y":
            alarm_v = v
            stop_reason = f"driver alarm reported at {v:.2f} V"
            break
        v = round(v + args.step_v, 10)
    else:
        stop_reason = f"reached {args.max_v:.2f} V with no alarm reported"
except KeyboardInterrupt:
    stop_reason = "Ctrl-C"
finally:
    # Always return to 0 V and disarm, however the loop above ended.
    st["thr"], st["brake"] = 0.0, 100
    send(build_simple("DIS", st["seq"] + 9000))
    time.sleep(0.2)
    st["run"] = False

print(f"\nSTOPPED: {stop_reason}")
print("\n=== results ===")
print(f"motor first turned at: {motor_turn_v:.2f} V" if motor_turn_v is not None
      else "motor first turned at: not observed")
print(f"driver alarm first seen at: {alarm_v:.2f} V" if alarm_v is not None
      else "driver alarm first seen at: not observed in this run")
print("Record both numbers, and any alarm voltage, in docs/BENCH_TESTS.md. An alarm below the configured cap "
      "must be investigated (current-limit setting, soft-start trimmers, supply sag, SV/COM grounding, noise "
      "on SV) before the cap is treated as safe to run at.")
