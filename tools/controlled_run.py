#!/usr/bin/env python3
"""Operator-run, direct-VCU controlled run demo (deliverable 4b: "CONTROLLED BLDC RUN").

Ramps to the 2.0 V throttle cap (THROTTLE_MAX_MV, the hard design ceiling -- never raised here),
holds for a staged duration starting at 2 s, then runs the exact same stop command the mission's
own command gate sends on a stop decision: CMD throttle_mv=0, brake_pct=100 (see
autokart_vehicle_bridge/core.py's CommandGate.compute(), which returns GateOutput(0, 100, ...) on
every stop reason), followed by DIS to return cleanly to SAFE -- the same two-frame sequence
tools/braking_test.py already uses and calls "the mission's stop command".

No camera, no ROS graph. This tool, like tools/braking_test.py, owns the serial port exclusively
and must never run during a ROS vehicle session -- which also means Mission Control's live
dashboard cannot be watching the SAME serial link while this tool runs (there is only one real
VCU serial port); it only reflects the vehicle's state from before this tool starts and after it
releases the port. This is a real architectural constraint of the existing single-owner serial
link, not a limitation introduced here.

This kart has NO axle encoder (docs/VCU_PROTOCOL.md): counts and speed_mmps are always -1
(unavailable) in telemetry, never zero or fabricated. Every logged CSV row for this hardware will
show -1 in those columns -- reported honestly, never approximated. Because there is no measured
speed, "aborts if speed does not fall to zero within 6 s" is implemented as the closest real,
verifiable substitute: abort if the VCU does not report back to state SAFE with EN de-asserted
within 6 s of the stop command.

The ARM request is sent directly to the firmware exactly as braking_test.py already does; the
firmware's own VCU_OUTPUT_WIRING_VERIFIED=0 lock (NAK 7) still refuses it until that lock is
cleared through the operator's own physical qualification process -- nothing here loosens or
bypasses that gate.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "kart", "ros2_ws", "src", "autokart_vehicle_bridge"))
from autokart_vehicle_bridge.protocol import THROTTLE_MAX_MV, build_cmd, build_simple, parse_frame, parse_message

HOLD_TIMES_S = (2, 5, 10)
PERIOD_S = 0.05
TEL_MAX_AGE_S = 0.25
STOP_CONFIRM_TIMEOUT_S = 6.0
RAMP_TIMEOUT_S = 4.0


class ControlledRunAbort(RuntimeError):
    pass


class ControlledRunLink:
    """Minimal direct-serial VCU link for this tool only. Deliberately NOT shared with
    braking_test.py's SerialLink: that class's require_healthy() hard-refuses any kart without
    axle encoder telemetry (by design, since it measures deceleration), which would make it
    permanently unusable for this no-encoder kart. Reuses only the stable, already-tested wire
    protocol functions (build_cmd/build_simple/parse_frame/parse_message), not a second
    hand-rolled CRC/framing implementation."""

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
            raise ControlledRunAbort("serial receive buffer overflow")
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
                raise ControlledRunAbort(f"VCU NAK {value}")
        return self.tel

    def require_healthy(self, require_armed=False):
        self.poll()
        if self.tel is None or self.tel_at is None or self.clock() - self.tel_at > TEL_MAX_AGE_S:
            raise ControlledRunAbort("VCU telemetry lost or stale")
        if self.tel.state == 2 or self.tel.fault:
            raise ControlledRunAbort(f"VCU fault {self.tel.fault_names()}")
        if self.tel.estop_input_fitted and not self.tel.estop_ok:
            raise ControlledRunAbort("E-stop active")
        if not self.tel.dac_ok:
            raise ControlledRunAbort("DAC not ready")
        if require_armed and self.tel.state != 1:
            raise ControlledRunAbort("VCU left ARMED state")
        return self.tel

    def mission_stop_sequence(self):
        """The same stop command the mission's own command gate sends on a stop decision
        (CommandGate.compute() -> GateOutput(0, 100, ...)), then DIS for a clean terminal state."""
        try:
            self.send("CMD", 0, 100)
            self.send("DIS")
        except Exception:
            pass


def wait_initial_tel(link: ControlledRunLink) -> None:
    deadline = link.clock() + 3.0
    while link.clock() < deadline:
        link.send("CMD", 0, 100)
        link.poll()
        if link.tel is not None:
            link.require_healthy()
            if link.tel.state != 0:
                raise ControlledRunAbort("VCU must begin SAFE")
            return
        time.sleep(PERIOD_S)
    raise ControlledRunAbort("no VCU telemetry")


def arm(link: ControlledRunLink) -> None:
    link.require_healthy()
    if link.tel.state != 0:
        raise ControlledRunAbort("VCU must be SAFE before ARM")
    link.send("ARM")
    deadline = link.clock() + 3.0
    while link.clock() < deadline:
        link.send("CMD", 0, 100)
        tel = link.require_healthy()
        if tel.state == 1:
            return
        time.sleep(PERIOD_S)
    raise ControlledRunAbort("VCU did not ARM")


def confirm_stopped(link: ControlledRunLink, writer, run_index: int, hold_s: float, t0: float) -> None:
    """No encoder exists to confirm speed fell to zero, so this confirms the closest real signal:
    the VCU reporting back to SAFE with EN de-asserted, within the same 6 s budget."""
    deadline = link.clock() + STOP_CONFIRM_TIMEOUT_S
    while link.clock() < deadline:
        link.send("CMD", 0, 100)
        tel = link.require_healthy()
        log_row(writer, run_index, hold_s, link.clock() - t0, tel, "STOP_CONFIRM")
        if tel.state == 0 and not tel.en:
            return
        time.sleep(PERIOD_S)
    raise ControlledRunAbort("VCU did not return to SAFE with EN de-asserted within 6 s of the stop command")


def log_row(writer, run_index: int, hold_s: float, t_s: float, tel, phase: str) -> None:
    writer.writerow({
        "run": run_index, "hold_s": hold_s, "t_s": round(t_s, 3), "phase": phase,
        "state": tel.state_name, "dac_mv": tel.dac_mv, "en": int(tel.en), "brk": int(tel.brk),
        "fault": ",".join(tel.fault_names()) or "NONE",
        "encoder_counts": tel.counts if tel.counts >= 0 else "UNAVAILABLE",
        "speed_mmps": tel.speed_mmps if tel.speed_mmps >= 0 else "UNAVAILABLE",
    })


def perform_stage(link: ControlledRunLink, writer, run_index: int, hold_s: float) -> dict:
    t0 = link.clock()
    arm(link)
    ramp_start = link.clock()
    while link.clock() - ramp_start < RAMP_TIMEOUT_S:
        link.send("CMD", THROTTLE_MAX_MV, 0)
        tel = link.require_healthy(require_armed=True)
        log_row(writer, run_index, hold_s, link.clock() - t0, tel, "RAMP")
        if tel.dac_mv >= THROTTLE_MAX_MV - 50:
            break
        time.sleep(PERIOD_S)
    else:
        raise ControlledRunAbort(f"{THROTTLE_MAX_MV} mV ramp did not complete within {RAMP_TIMEOUT_S} s")
    hold_until = link.clock() + hold_s
    while link.clock() < hold_until:
        link.send("CMD", THROTTLE_MAX_MV, 0)
        tel = link.require_healthy(require_armed=True)
        log_row(writer, run_index, hold_s, link.clock() - t0, tel, "HOLD")
        time.sleep(PERIOD_S)
    link.mission_stop_sequence()
    confirm_stopped(link, writer, run_index, hold_s, t0)
    return {"run": run_index, "hold_s": hold_s, "stopped_cleanly": True}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="exclusive VCU serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--hold-times", nargs="+", type=float, default=list(HOLD_TIMES_S),
                        help="staged hold durations in seconds, ascending; default 2 5 10")
    parser.add_argument("--out-dir", default=os.path.join(ROOT, "results"))
    args = parser.parse_args()
    hold_times = args.hold_times
    if list(hold_times) != sorted(hold_times) or hold_times[0] > 2.0:
        parser.error("--hold-times must be ascending and start at 2 s or less (start short, extend only "
                     "after a clean shorter run)")
    print(f"This tool WILL request motion up to {THROTTLE_MAX_MV} mV ({THROTTLE_MAX_MV / 1000:.1f} V). "
          "Keep the ROS vehicle bridge stopped; this tool owns the serial port exclusively.")
    for prompt, expected in (
        ("Type CLEAR20 to confirm a straight, unobstructed 20 m area: ", "CLEAR20"),
        ("Type POWERCUT to confirm the physical battery/supply cut is within reach and tested: ", "POWERCUT"),
        ("Type OBSERVER to confirm a second person is watching the kart: ", "OBSERVER"),
    ):
        if input(prompt).strip() != expected:
            raise SystemExit("No motion requested: confirmation missing")
    import serial
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, f"controlled_run_{ts}.csv")
    json_path = os.path.join(args.out_dir, f"controlled_run_{ts}.json")
    results: list[dict] = []
    fieldnames = ["run", "hold_s", "t_s", "phase", "state", "dac_mv", "en", "brk", "fault",
                 "encoder_counts", "speed_mmps"]
    with serial.Serial(args.port, 115200, timeout=0.02) as port, open(csv_path, "w", newline="") as csv_handle:
        writer = csv.DictWriter(csv_handle, fieldnames=fieldnames)
        writer.writeheader()
        link = ControlledRunLink(port)
        try:
            wait_initial_tel(link)
            for index, hold_s in enumerate(hold_times, 1):
                if index > 1:
                    input(f"Stage {index}/{len(hold_times)} ({hold_s:g} s hold) next; reposition and press "
                          "Enter only when the observer confirms clear: ")
                print(f"Stage {index}/{len(hold_times)}: ramp to {THROTTLE_MAX_MV} mV, hold {hold_s:g} s, "
                      "then the mission stop sequence")
                result = perform_stage(link, writer, index, hold_s)
                results.append(result)
                print(f"Stage {index} stopped cleanly (VCU back to SAFE, EN de-asserted).")
        except (ControlledRunAbort, KeyboardInterrupt, OSError) as exc:
            link.mission_stop_sequence()
            print(f"ABORT: {exc}", file=sys.stderr)
            report = {"created_utc": datetime.now(timezone.utc).isoformat(), "port": args.port,
                      "hold_times_s": hold_times, "stages_completed": results, "aborted": True,
                      "abort_reason": str(exc), "csv": csv_path, "verified_flags_changed": False}
            with open(json_path, "w", encoding="utf-8") as handle:
                json.dump(report, handle, indent=2, allow_nan=False)
            print(f"Saved {json_path} and {csv_path}")
            return 1
        link.mission_stop_sequence()
        report = {"created_utc": datetime.now(timezone.utc).isoformat(), "port": args.port,
                  "hold_times_s": hold_times, "stages_completed": results, "aborted": False,
                  "csv": csv_path, "verified_flags_changed": False,
                  "note": "No axle encoder on this kart: speed_mmps/counts are always UNAVAILABLE (-1), "
                          "never fabricated. Stop confirmation used VCU state+EN, not measured speed."}
        with open(json_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, allow_nan=False)
    print(f"Saved {json_path} and {csv_path}")
    print(f"All {len(hold_times)} stage(s) completed and stopped cleanly.")
    print("No verified flag was changed. This demo does not by itself authorize a longer or faster run.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
