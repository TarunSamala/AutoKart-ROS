#!/usr/bin/env python3
"""Operator-run, direct-VCU braking measurement. Never run during a ROS vehicle session.

This program can command BLD-750 motion. It requires a clear 20 m area, a person with
physical battery/supply power cut within reach, and a second observer. It never updates verified flags or firmware.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "kart", "ros2_ws", "src", "autokart_vehicle_bridge"))
from autokart_vehicle_bridge.protocol import build_cmd, build_simple, parse_frame, parse_message

RUN_TIMES_S = (2, 3, 5, 5, 5, 5, 5)
PERIOD_S = 0.05
TEL_MAX_AGE_S = 0.25
STOP_TIMEOUT_S = 6.0
STOP_LIMIT_M = 3.0
RUN_TRAVEL_LIMIT_M = 12.0


class BrakingAbort(RuntimeError):
    pass


class SerialLink:
    def __init__(self, serial_port, clock=time.monotonic):
        self.port = serial_port
        self.clock = clock
        self.buffer = b""
        self.seq = 0
        self.tel = None
        self.tel_at = None
        self.hello = None

    def send(self, kind="CMD", throttle_mv=0, brake_pct=100):
        self.seq += 1
        data = (build_cmd(self.seq, throttle_mv, 0, brake_pct) if kind == "CMD"
                else build_simple(kind, self.seq))
        self.port.write(data)

    def poll(self):
        self.buffer += self.port.read(4096)
        if len(self.buffer) > 8192:
            raise BrakingAbort("serial receive buffer overflow")
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            payload = parse_frame(line)
            message = parse_message(payload) if payload else None
            if message is None:
                continue
            kind, value = message
            if kind == "TEL":
                self.tel = value
                self.tel_at = self.clock()
            elif kind == "HELLO":
                self.hello = value
            elif kind == "NAK":
                raise BrakingAbort(f"VCU NAK {value}")
        return self.tel

    def require_healthy(self, require_armed=False):
        self.poll()
        if self.tel is None or self.tel_at is None or self.clock() - self.tel_at > TEL_MAX_AGE_S:
            raise BrakingAbort("VCU telemetry lost or stale")
        if self.tel.state == 2 or self.tel.fault:
            raise BrakingAbort(f"VCU fault {self.tel.fault_names()}")
        if self.tel.estop_input_fitted and not self.tel.estop_ok:
            raise BrakingAbort("E-stop active")
        if not self.tel.dac_ok:
            raise BrakingAbort("DAC not ready")
        if self.tel.speed_mmps < 0 or self.tel.counts < 0:
            raise BrakingAbort("no axle encoder telemetry; this braking measurement cannot run on this kart")
        if require_armed and self.tel.state != 1:
            raise BrakingAbort("VCU left ARMED state")
        return self.tel

    def software_stop(self):
        # The mission's stop command: zero DAC request with full BLD-750 brake.
        # Firmware applies DAC zero before EN/BRK and latches faults on link loss.
        try:
            self.send("CMD", 0, 100)
            self.send("DIS")
        except Exception:
            pass


def wait_healthy(link, duration_s, *, armed=False, throttle_mv=0, brake_pct=100):
    deadline = link.clock() + duration_s
    while link.clock() < deadline:
        link.send("CMD", throttle_mv, brake_pct)
        link.require_healthy(require_armed=armed)
        time.sleep(PERIOD_S)


def wait_initial_tel(link):
    deadline = link.clock() + 3.0
    while link.clock() < deadline:
        link.send("CMD", 0, 100)
        link.poll()
        if link.tel is not None:
            link.require_healthy()
            if link.tel.state != 0:
                raise BrakingAbort("VCU must begin SAFE")
            return
        time.sleep(PERIOD_S)
    raise BrakingAbort("no VCU telemetry")


def arm(link):
    link.require_healthy()
    if link.tel.state != 0 or link.tel.speed_mmps != 0:
        raise BrakingAbort("VCU must be SAFE and stationary before ARM")
    link.send("ARM")
    deadline = link.clock() + 3.0
    while link.clock() < deadline:
        link.send("CMD", 0, 100)
        tel = link.require_healthy()
        if tel.state == 1:
            return
        time.sleep(PERIOD_S)
    raise BrakingAbort("VCU did not ARM")


def stop_and_measure(link):
    trace = []
    cut = link.clock()
    previous_time = None
    previous_speed = None
    previous_sample_at = None
    distance = 0.0
    stationary_since = None
    speed_at_cut = max(0.0, link.require_healthy(require_armed=True).speed_mmps / 1000.0)
    link.send("CMD", 0, 100)
    link.send("DIS")
    while True:
        link.send("CMD", 0, 100)
        tel = link.require_healthy()
        now = link.clock()
        speed = max(0.0, tel.speed_mmps / 1000.0)
        if link.tel_at != previous_sample_at:
            if previous_time is not None:
                distance += (previous_speed + speed) * 0.5 * (now - previous_time)
            trace.append({"t_s": now - cut, "speed_mps": speed,
                          "encoder_counts": tel.counts, "dac_mv": tel.dac_mv})
            previous_time, previous_speed = now, speed
            previous_sample_at = link.tel_at
        if distance > STOP_LIMIT_M:
            raise BrakingAbort("stopping distance exceeded 3 m")
        if speed <= 0.03:
            stationary_since = now if stationary_since is None else stationary_since
            if now - stationary_since >= 0.3:
                break
        else:
            stationary_since = None
        if now - cut > STOP_TIMEOUT_S:
            raise BrakingAbort("speed did not fall to zero within 6 s")
        time.sleep(PERIOD_S)
    stop_time = now - cut
    mean_decel = speed_at_cut / stop_time if speed_at_cut > 0.03 else 0.0
    if mean_decel <= 0 or not math.isfinite(mean_decel):
        raise BrakingAbort("invalid measured deceleration")
    return {"speed_at_cut_mps": speed_at_cut, "stopping_time_s": stop_time,
            "stopping_distance_m": distance, "mean_deceleration_mps2": mean_decel,
            "trace": trace}


def perform_run(link, hold_s):
    wait_healthy(link, 0.5)
    arm(link)
    distance = 0.0
    last_time = link.clock()

    def drive_tick():
        nonlocal distance, last_time
        link.send("CMD", 2000, 0)
        tel = link.require_healthy(require_armed=True)
        now = link.clock()
        if tel.speed_mmps < 0:
            raise BrakingAbort("encoder reports reverse motion")
        distance += tel.speed_mmps / 1000.0 * (now - last_time)
        last_time = now
        if distance > RUN_TRAVEL_LIMIT_M:
            raise BrakingAbort("run travel exceeded 12 m before braking")
        time.sleep(PERIOD_S)

    # Command 2 V and wait for the firmware's 1000 mV/s slew to finish.
    ramp_start = link.clock()
    while link.clock() - ramp_start < 2.0 or link.tel.dac_mv < 1950:
        if link.clock() - ramp_start > 4.0:
            raise BrakingAbort("2 V ramp did not complete")
        drive_tick()
    hold_until = link.clock() + hold_s
    while link.clock() < hold_until:
        drive_tick()
    result = stop_and_measure(link)
    return result


def summarize(results):
    final = [r for r in results if r["hold_s"] == 5]
    if len(final) < 5:
        raise BrakingAbort("fewer than five final 5 s runs")
    decels = [r["mean_deceleration_mps2"] for r in final]
    return {"v_run_mps": statistics.mean(r["speed_at_cut_mps"] for r in final),
            "worst_case_mean_deceleration_mps2": min(decels),
            "a_verified_mps2_suggestion": 0.8 * min(decels),
            "max_stopping_distance_m": max(r["stopping_distance_m"] for r in final)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="exclusive VCU serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--out-dir", default=os.path.join(ROOT, "results"))
    args = parser.parse_args()
    print("This tool WILL request motion at 2.0 V. Keep ROS vehicle bridge stopped.")
    checks = [
        ("Type CLEAR20 to confirm a straight, unobstructed 20 m area: ", "CLEAR20"),
        ("Type POWERCUT to confirm the physical battery/supply cut is within reach: ", "POWERCUT"),
        ("Type OBSERVER to confirm a second person is watching: ", "OBSERVER"),
    ]
    for prompt, expected in checks:
        if input(prompt).strip() != expected:
            raise SystemExit("No motion requested: confirmation missing")
    import serial
    results = []
    path = os.path.join(args.out_dir, "braking_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
    with serial.Serial(args.port, 115200, timeout=0.02) as port:
        link = SerialLink(port)
        try:
            wait_initial_tel(link)
            for index, hold_s in enumerate(RUN_TIMES_S, 1):
                if results and hold_s > results[-1]["hold_s"]:
                    if results[-1]["stopping_distance_m"] >= STOP_LIMIT_M:
                        raise BrakingAbort("previous stop was not under 3 m")
                if index > 1:
                    input("Reposition at the start; press Enter only when the observer confirms clear: ")
                print(f"Run {index}/{len(RUN_TIMES_S)}: 2 V ramp, then {hold_s} s hold")
                run = perform_run(link, hold_s)
                run.update(run=index, hold_s=hold_s)
                results.append(run)
                print(f"cut speed {run['speed_at_cut_mps']:.3f} m/s, stop time "
                      f"{run['stopping_time_s']:.3f} s, distance {run['stopping_distance_m']:.3f} m, "
                      f"mean decel {run['mean_deceleration_mps2']:.3f} m/s^2")
        except (BrakingAbort, KeyboardInterrupt, OSError) as exc:
            link.software_stop()
            print(f"ABORT: {exc}", file=sys.stderr)
            raise SystemExit(1) from exc
        finally:
            link.software_stop()
            os.makedirs(args.out_dir, exist_ok=True)
            report = {"created_utc": datetime.now(timezone.utc).isoformat(),
                      "port": args.port, "runs": results, "completed": len(results) == len(RUN_TIMES_S),
                      "summary": summarize(results) if len(results) == len(RUN_TIMES_S) else None,
                      "verified_flags_changed": False}
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(report, handle, indent=2, allow_nan=False)
            print(f"Saved {path}")
    summary = summarize(results)
    print(f"v_run at 2.0 V: {summary['v_run_mps']:.3f} m/s")
    print(f"worst-case mean deceleration: {summary['worst_case_mean_deceleration_mps2']:.3f} m/s^2")
    print(f"brake_min_mps2: {summary['a_verified_mps2_suggestion']:.4f}  # a_verified, human review and paste only")
    print("No verified flag was changed. Physical acceptance remains the operator's decision.")


if __name__ == "__main__":
    main()
