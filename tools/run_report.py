#!/usr/bin/env python3
"""Print a run's final bumper gap estimate from a real cone sighting and axle count travel."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kart/ros2_ws/src/autokart_vehicle_bridge"))
from autokart_vehicle_bridge.run_log import final_bumper_gap


def configured_calibration() -> tuple[float, int]:
    config = ROOT / "kart/ros2_ws/src/autokart_bringup/config/commissioning.yaml"
    try:
        import yaml
        values = yaml.safe_load(config.read_text())["autokart_localization"]["ros__parameters"]
        return float(values["counts_per_m"]), int(values["encoder_forward_sign"])
    except (ImportError, OSError, KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"cannot read calibrated counts_per_m from {config}: {exc}") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--counts-per-m", type=float, help="override commissioning.yaml after calibration")
    parser.add_argument("--encoder-forward-sign", type=int, choices=(-1, 1),
                        help="override direction recorded in commissioning.yaml")
    args = parser.parse_args()
    with args.csv.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    configured_scale, configured_sign = (configured_calibration()
        if args.counts_per_m is None or args.encoder_forward_sign is None else (0.0, 1))
    value = args.counts_per_m if args.counts_per_m is not None else configured_scale
    sign = args.encoder_forward_sign if args.encoder_forward_sign is not None else configured_sign
    gap = final_bumper_gap(rows, value, sign)
    print(f"Final bumper gap estimate (encoder dead reckoning): {gap:.4f} m")
    print("Compare with the tape measurement; this estimate does not qualify camera or stopping accuracy.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        raise SystemExit(f"NO_ESTIMATE: {exc}") from exc
