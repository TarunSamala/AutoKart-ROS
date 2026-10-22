#!/usr/bin/env python3
"""Bench console for the VCU (no ROS). Use with the wheels OFF THE GROUND.

    python3 tools/vcu_console.py /dev/ttyACM0
Commands: arm | dis | rst | thr <volts 0..2> | brake <0..100> | quit
Sends a command frame at 20 Hz (this is the heartbeat) and prints telemetry twice a second.
"""
import os, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ros2_ws", "src", "autokart_vehicle_bridge"))
import serial
from autokart_vehicle_bridge.protocol import build_cmd, build_simple, parse_frame, parse_message, NAK_TEXT

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ser = serial.Serial(port, 115200, timeout=0.05)
lock = threading.Lock()
state = {"thr": 0.0, "brake": 100, "seq": 1, "run": True, "tel": None}


def send(b):
    with lock:
        ser.write(b)


def tx():
    while state["run"]:
        state["seq"] += 1
        send(build_cmd(state["seq"], int(round(min(state["thr"], 2.0) * 1000)), 0, state["brake"]))
        time.sleep(0.05)


def rx():
    buf = b""
    last = 0
    while state["run"]:
        buf += ser.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            p = parse_frame(line)
            m = parse_message(p) if p else None
            if not m:
                continue
            if m[0] == "NAK":
                print(f"\nVCU refused request {m[1][0]}: {NAK_TEXT.get(m[1][1], m[1][1])}")
            elif m[0] == "HELLO":
                print(f"\nHELLO fw {m[1][0]} proto {m[1][1]}")
            elif m[0] == "TEL" and time.time() - last > 0.5:
                last = time.time()
                t = m[1]
                print(f"\r{t.state_name:5s} fault={t.fault_names() or '-'} estop={int(t.estop_ok)} dac={int(t.dac_ok)} en={int(t.en)} brk={int(t.brk)} "
                      f"dac_mv={t.dac_mv:4d} counts={t.counts:8d} speed={t.speed_mmps/1000:5.2f} m/s steer_raw={t.steer_raw} link_age={t.link_age_ms} ms   ", end="")


threading.Thread(target=tx, daemon=True).start()
threading.Thread(target=rx, daemon=True).start()
print("commands: arm | dis | rst | thr <V> | brake <pct> | quit   (throttle is capped at 2.0 V)")
try:
    while True:
        parts = input("\n> ").split()
        if not parts:
            continue
        c = parts[0]
        if c == "arm": state["thr"] = 0.0; send(build_simple("ARM", state["seq"] + 1000))
        elif c == "dis": state["thr"] = 0.0; state["brake"] = 100; send(build_simple("DIS", state["seq"] + 1000))
        elif c == "rst": send(build_simple("RST", state["seq"] + 1000))
        elif c == "thr" and len(parts) == 2: state["thr"] = max(0.0, min(2.0, float(parts[1]))); state["brake"] = 0
        elif c == "brake" and len(parts) == 2: state["brake"] = int(parts[1]); state["thr"] = 0.0 if state["brake"] >= 50 else state["thr"]
        elif c == "quit": break
finally:
    state["thr"] = 0.0; state["brake"] = 100
    send(build_simple("DIS", state["seq"] + 2000)); time.sleep(0.2); state["run"] = False
