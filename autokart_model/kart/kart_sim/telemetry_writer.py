"""Writes a my_track run's telemetry to results/<scenario>/<timestamp>_<profile>_<seed>/:
a 10 Hz CSV (the rows RunResult already collects), a 1 Hz CSV downsampled from
it, and a JSON summary (metrics + events). Intended as the one writer used by
all four scenarios (Step 5); only my_track is wired to it so far -- see
docs/MY_TRACK.md for what wiring skidpad/braking/autocross into it still
needs.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from .my_track_runner import RunResult


def write_my_track_run(result: RunResult, *, results_root: Path, profile: str, seed: int, timestamp: str | None = None) -> Path:
    timestamp = timestamp or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = results_root / "my_track" / f"{timestamp}_{profile}_{seed}"
    run_dir.mkdir(parents=True, exist_ok=True)

    rows = result.telemetry_rows
    _write_csv(run_dir / "telemetry_10hz.csv", rows)
    _write_csv(run_dir / "telemetry_1hz.csv", _downsample_to_1hz(rows))

    summary = {
        "metrics": asdict(result.metrics),
        "events": [{"t": t, "kind": kind, "detail": detail} for t, kind, detail in result.events],
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return run_dir


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _downsample_to_1hz(rows: list[dict]) -> list[dict]:
    """Keeps the row whose t is closest to each whole second, assuming the
    10 Hz input is sorted and roughly evenly spaced."""
    downsampled = []
    next_t = 0.0
    for row in rows:
        if row["t"] >= next_t:
            downsampled.append(row)
            next_t = row["t"] + 1.0 - 1e-6
    return downsampled
