#!/usr/bin/env python3
"""Read-only, hand-pushed rear-axle calibration. Stop the ROS bridge before opening its port."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kart/ros2_ws/src/autokart_vehicle_bridge"))
from autokart_vehicle_bridge.protocol import parse_frame, parse_message


def counts_per_metre(before: int, after: int, distance_m: float) -> float:
    if not math.isfinite(distance_m) or distance_m <= 0 or before == after:
        raise ValueError("distance must be positive and axle counts must change")
    return abs(after - before) / distance_m


def read_safe_tel(port, timeout_s: float = 3.0):
    port.reset_input_buffer()  # discard TEL accumulated while the operator was pushing/typing
    deadline = time.monotonic() + timeout_s
    buffer = b""
    stable_count = None
    stable_since = None
    while time.monotonic() < deadline:
        buffer += port.read(4096)
        if len(buffer) > 8192:
            buffer = buffer[-8192:]
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            body = parse_frame(line)
            parsed = parse_message(body) if body else None
            if parsed and parsed[0] == "TEL":
                tel = parsed[1]
                if tel.counts < 0 or tel.speed_mmps < 0:
                    raise RuntimeError("no axle encoder telemetry; calibration is unavailable on this kart")
                if tel.state != 0 or tel.en or tel.dac_mv != 0:
                    raise RuntimeError("VCU is not SAFE with enable off and DAC 0; aborting")
                now = time.monotonic()
                if tel.counts != stable_count:
                    stable_count, stable_since = tel.counts, now
                elif stable_since is not None and now - stable_since >= 0.2:
                    return tel
        time.sleep(0.01)
    raise RuntimeError("no stationary VCU TEL sample received")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--trials", type=int, default=3)
    args = parser.parse_args()
    if args.trials < 3:
        parser.error("at least three trials are required")
    try:
        import serial
    except ImportError as exc:
        parser.error(f"pyserial required: {exc}")
    print("Stop the ROS bridge first. Keep traction isolated; hand push only. This tool never writes serial or config.")
    values = []
    signs = []
    with serial.Serial(args.port, args.baud, timeout=0.1, exclusive=True) as port:
        for trial in range(1, args.trials + 1):
            distance = float(input(f"Trial {trial}: tape-measured push distance in metres: "))
            if not math.isfinite(distance) or distance <= 0:
                raise ValueError("distance must be positive and finite")
            input("Place kart at tape start, then press Enter to capture BEFORE counts: ")
            before = read_safe_tel(port)
            input("Hand push to the measured mark, stop, then press Enter to capture AFTER counts: ")
            after = read_safe_tel(port)
            if after.fwseq <= before.fwseq:
                raise RuntimeError("VCU telemetry did not advance")
            value = counts_per_metre(before.counts, after.counts, distance)
            values.append(value)
            signs.append(1 if after.counts > before.counts else -1)
            print(f"Trial {trial}: {before.counts} -> {after.counts}; {distance:.4f} m; {value:.4f} counts/m")
    mean = statistics.mean(values)
    if len(set(signs)) != 1:
        raise RuntimeError("forward push changed encoder count direction between trials")
    print(f"Mean of {len(values)} trials: {mean:.4f} counts/m")
    print("Paste into the localization ROS parameters only after reviewing the trials:")
    print(f"counts_per_m: {mean:.4f}")
    print(f"encoder_forward_sign: {signs[0]}  # verify this with the same forward push direction")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, KeyboardInterrupt) as exc:
        raise SystemExit(f"Calibration aborted: {exc}") from exc
