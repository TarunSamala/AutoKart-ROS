#!/usr/bin/env python3
"""Read-only: find the latest tools/depth_check.py --fit-pitch result and report its calibration
candidate status (PASS/FAIL/NOT RUN). Never writes perception.yaml or any config; never sets
calibration_verified. This is a status layer over depth_check.py's own fit_pitch(), not a second
fitting algorithm."""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def latest_depth_check(results_dir: Path) -> Path | None:
    candidates = list(results_dir.rglob("depth_check_*.json"))
    return max(candidates, key=lambda path: (path.stat().st_mtime_ns, path.name)) if candidates else None


def evaluate(results_dir: Path) -> dict:
    path = latest_depth_check(results_dir)
    if path is None:
        return {"status": "NOT_RUN", "reason": "NO_DEPTH_CHECK_RESULT",
                "command": "python3 tools/depth_check.py --fit-pitch"}
    try:
        data = json.loads(path.read_text())
    except (ValueError, OSError) as exc:
        return {"status": "FAIL", "reason": f"UNREADABLE_RESULT:{exc}", "source_file": str(path)}
    fit = data.get("pitch_fit")
    if not fit:
        return {"status": "NOT_RUN", "reason": "DEPTH_CHECK_RESULT_HAS_NO_FIT",
                "command": "python3 tools/depth_check.py --fit-pitch", "source_file": str(path)}
    recommendation = fit.get("recommendation")
    if recommendation != "PROVISIONAL_FIT_REVIEW":
        return {"status": "FAIL", "reason": recommendation or "UNKNOWN_FIT_OUTCOME", "source_file": str(path)}
    fitted = fit.get("fitted", {})
    return {
        "status": "PASS", "source_file": str(path), "fitted_pitch_deg": fitted.get("pitch_deg"),
        "fitted_rmse_m": fitted.get("rmse_m"),
        "configured_pitch_deg": fit.get("configured", {}).get("pitch_deg"),
        "old_ground_plane_pitch_deg": fit.get("old_ground_plane", {}).get("pitch_deg"),
        "paste_lines": [
            "autonomy_stack_phase1/phase_01_perception/src/autokart_perception/data/perception.yaml:",
            "  height_m: 0.8382",
            f"  pitch_down_deg: {fitted.get('pitch_deg'):.2f}" if fitted.get("pitch_deg") is not None else
            "  pitch_down_deg: <see fitted_pitch_deg above>",
            "  camera_to_front_bumper_horizontal_m: 1.0668  # operator_measurements",
            "  bumper_width_m: 1.016  # operator_measurements",
        ],
        "note": "Candidate for human review only. This tool does not write the file and does not set "
                "calibration_verified.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", default=str(ROOT / "results"))
    parser.add_argument("--out-dir", default=str(ROOT / "results"))
    args = parser.parse_args()
    result = evaluate(Path(args.results_dir))
    print(json.dumps(result, indent=2))
    if result["status"] == "PASS":
        print("\nCandidate calibration lines for human review (NOT written):")
        for line in result["paste_lines"]:
            print(f"  {line}")
    elif result["status"] == "NOT_RUN":
        print(f"\nNOT RUN. Run: {result['command']}")
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"calibration_candidate_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    out_path.write_text(json.dumps(result, indent=2, sort_keys=True))
    print(f"Saved {out_path}")
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
