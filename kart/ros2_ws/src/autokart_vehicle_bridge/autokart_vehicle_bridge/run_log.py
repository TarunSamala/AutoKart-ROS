"""Append-only commissioning run CSV and count-based final bumper-gap estimate."""
from __future__ import annotations

import csv
import math

FIELDS = ("time", "cone_forward_distance", "bumper_to_cone", "speed", "dac_mv",
          "decision", "state", "axle_counts", "dead_reckoned", "frame_stamp_ns")


class RunCsvWriter:
    def __init__(self, file):
        self.file = file
        self.writer = csv.DictWriter(file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.file.flush()

    def write(self, row: dict) -> None:
        self.writer.writerow({field: row.get(field, "") for field in FIELDS})
        self.file.flush()


def final_bumper_gap(rows: list[dict], counts_per_m: float, encoder_forward_sign: int = 1) -> float:
    if not math.isfinite(counts_per_m) or counts_per_m <= 0:
        raise ValueError("calibrated counts_per_m required")
    if encoder_forward_sign not in (-1, 1):
        raise ValueError("encoder_forward_sign must be -1 or 1")
    sighting = next((row for row in reversed(rows)
                     if row.get("dead_reckoned") not in ("True", "true", True)
                     and row.get("bumper_to_cone") not in ("", None)
                     and row.get("axle_counts") not in ("", None)), None)
    final = next((row for row in reversed(rows) if row.get("axle_counts") not in ("", None)), None)
    if sighting is None or final is None:
        raise ValueError("run lacks a real cone sighting or final axle counts")
    if str(final.get("dac_mv")) != "0":
        raise ValueError("final VCU telemetry does not show dac_mv 0")
    if int(final["axle_counts"]) < 0 or int(sighting["axle_counts"]) < 0:
        raise ValueError("no axle encoder telemetry; final gap cannot be estimated from this run")
    distance = (int(final["axle_counts"]) - int(sighting["axle_counts"])) * encoder_forward_sign / counts_per_m
    return float(sighting["bumper_to_cone"]) - distance
