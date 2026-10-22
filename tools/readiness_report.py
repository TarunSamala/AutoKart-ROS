#!/usr/bin/env python3
"""Read-only readiness report: reads config and every results/*.json this toolchain produces, prints one
gate table (PASS / FAIL / NOT RUN + value), the exact command for each NOT RUN item, and the exact config
lines a human must paste (never writes them). Ends with READY FOR FIRST RUN or NOT READY + reasons.
Never sets a verified flag; never loosens a gate to make the table look better."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
sys.path.insert(0, str(Path(__file__).resolve().parent))
import model_validation
import calibration_candidate
import runway_check


def _latest(pattern: str) -> Path | None:
    files = sorted(RESULTS.glob(pattern))
    return files[-1] if files else None


def _load(path: Path | None) -> dict | None:
    if path is None:
        return None
    try:
        return json.loads(path.read_text())
    except (ValueError, OSError):
        return None


def _yaml(path: Path) -> dict:
    import yaml
    try:
        return yaml.safe_load(path.read_text()) or {}
    except (OSError, ValueError):
        return {}


def gate_camera_rate() -> dict:
    data = _load(_latest("camera_rate_*.json"))
    if data is None:
        return {"name": "paired_rgb_depth_hz", "status": "NOT_RUN",
                "command": "python3 tools/camera_rate_check.py --duration 15  # graph must already be running"}
    hz = data.get("paired_rgb_depth_hz")
    status = "PASS" if isinstance(hz, (int, float)) and hz >= 9.0 else "FAIL"
    return {"name": "paired_rgb_depth_hz", "status": status, "value": hz,
            "threshold": ">= 9.0 Hz", "source_file": data.get("created_utc")}


def gate_latency() -> dict:
    data = _load(_latest("latency_report_*.json"))
    if data is None:
        return {"name": "latency_p95_ms", "status": "NOT_RUN",
                "command": "python3 tools/latency_report.py --live --seconds 30"}
    return {"name": "latency_p95_ms", "status": "PASS", "value": data.get("p95_ms"),
            "suggested_t_latency_s": data.get("suggested_t_latency_s")}


def gate_depth_error() -> dict:
    data = _load(_latest("depth_check_*.json"))
    if data is None:
        return {"name": "depth_error_vs_budget", "status": "NOT_RUN",
                "command": "python3 tools/depth_check.py --frames 30"}
    stations = []
    all_within_budget = True
    any_station = False
    for row in data.get("results", []):
        if "absolute_error_cm" not in row:
            continue
        any_station = True
        within = row["absolute_error_cm"] <= 100 * row.get("budget_m", 0)
        all_within_budget = all_within_budget and within
        stations.append({"target_m": row.get("target_m"), "error_cm": row.get("absolute_error_cm"),
                         "budget_cm": 100 * row.get("budget_m", 0), "within_budget": within})
    if not any_station:
        return {"name": "depth_error_vs_budget", "status": "NOT_RUN",
                "command": "python3 tools/depth_check.py --frames 30"}
    return {"name": "depth_error_vs_budget", "status": "PASS" if all_within_budget else "FAIL",
            "stations": stations}


def gate_pitch_fit() -> dict:
    result = calibration_candidate.evaluate(RESULTS)
    out = {"name": "calibration_candidate_pitch_fit", "status": result["status"]}
    out.update({k: v for k, v in result.items() if k not in ("status",)})
    return out


def gate_counts_per_m() -> dict:
    config = _yaml(ROOT / "kart/ros2_ws/src/autokart_bringup/config/commissioning.yaml")
    value = config.get("autokart_localization", {}).get("ros__parameters", {}).get("counts_per_m", 0.0)
    status = "PASS" if value and value > 0 else "NOT_RUN"
    return {"name": "counts_per_m", "status": status, "value": value,
            "command": "python3 tools/encoder_calibration.py" if status == "NOT_RUN" else None}


def gate_braking() -> dict:
    data = _load(_latest("braking_*.json"))
    if data is None or not data.get("completed"):
        return {"name": "braking_test", "status": "NOT_RUN",
                "command": "python3 tools/braking_test.py /dev/ttyACM0"}
    summary = data.get("summary") or {}
    return {"name": "braking_test", "status": "PASS",
            "v_run_mps": summary.get("v_run_mps"),
            "a_verified_mps2_suggestion": summary.get("a_verified_mps2_suggestion"),
            "worst_case_mean_deceleration_mps2": summary.get("worst_case_mean_deceleration_mps2")}


def gate_model_validation() -> dict:
    result = model_validation.validate(model_validation.DEFAULT_MODEL, model_validation.DEFAULT_MANIFEST)
    return {"name": "model_validation", "status": result["status"], "stage": result.get("stage"),
            "vehicle_authorization": result.get("vehicle_authorization"),
            "sha256_matches_manifest": result.get("sha256_matches_manifest")}


def gate_runway(braking: dict, latency: dict) -> dict:
    v_run = braking.get("v_run_mps")
    a_verified = braking.get("a_verified_mps2_suggestion")
    t_latency = latency.get("suggested_t_latency_s")
    commissioning = _yaml(ROOT / "kart/ros2_ws/src/autokart_bringup/config/commissioning.yaml")
    margin = commissioning.get("autokart_planner", {}).get("ros__parameters", {}).get("stop_gap_from_bumper_m", 1.0)
    if v_run is None or a_verified is None or t_latency is None:
        return {"name": "runway_per_gap", "status": "NOT_RUN",
                "command": "python3 tools/runway_check.py --v-run <measured> --a-verified <measured> "
                           "--t-latency <measured> --margin <value>  # needs braking + latency measured first"}
    try:
        result = runway_check.calculate(v_run, a_verified, t_latency, margin)
    except ValueError as exc:
        return {"name": "runway_per_gap", "status": "FAIL", "reason": str(exc)}
    any_short = any(row["less_than_0_5_m"] for row in result["rows"])
    return {"name": "runway_per_gap", "status": "FAIL" if any_short else "PASS", "rows": result["rows"]}


def gate_system_check() -> dict:
    proc = subprocess.run([str(ROOT / "scripts/system_check.sh")], text=True, capture_output=True, check=False)
    output = proc.stdout
    go = "FINAL: GO" in output
    return {"name": "system_check", "status": "PASS" if go else "FAIL", "output_tail": output.strip().splitlines()[-5:]}


def gate_flags() -> dict:
    config = _yaml(ROOT / "kart/ros2_ws/src/autokart_vcu_safety/config/vcu_safety.yaml")
    params = config.get("autokart_vcu_safety", {}).get("ros__parameters", {})
    flags = {name: bool(params.get(name, False)) for name in
            ("calibration_verified", "vehicle_config_verified", "braking_verified")}
    status = "PASS" if all(flags.values()) else "NOT_RUN"
    return {"name": "verified_flags", "status": status, **flags}


def build_report() -> dict:
    braking = gate_braking()
    latency = gate_latency()
    gates = [
        gate_camera_rate(), latency, gate_depth_error(), gate_pitch_fit(),
        gate_counts_per_m(), braking, gate_model_validation(),
        gate_runway(braking, latency), gate_system_check(), gate_flags(),
    ]
    not_ready_reasons = [f"{g['name']}: {g['status']}" for g in gates if g["status"] != "PASS"]
    ready = not not_ready_reasons
    return {"gates": gates, "ready": ready, "reasons": not_ready_reasons}


def print_report(report: dict) -> None:
    print(f"{'GATE':<30} {'STATUS':<8} VALUE")
    print("-" * 70)
    for gate in report["gates"]:
        value = gate.get("value")
        if value is None:
            extras = {k: v for k, v in gate.items() if k not in ("name", "status", "command")}
            value = json.dumps(extras, default=str)[:60] if extras else ""
        print(f"{gate['name']:<30} {gate['status']:<8} {value}")
        if gate["status"] == "NOT_RUN" and gate.get("command"):
            print(f"{'':<30} {'':<8} -> {gate['command']}")
    print()
    for gate in report["gates"]:
        if gate.get("status") == "PASS" and "paste_lines" in gate:
            print("Candidate config lines for human review (NOT written):")
            for line in gate["paste_lines"]:
                print(f"  {line}")
    print()
    if report["ready"]:
        print("READY FOR FIRST RUN")
    else:
        print("NOT READY")
        for reason in report["reasons"]:
            print(f"  - {reason}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", help="save the full report as JSON")
    args = parser.parse_args()
    report = build_report()
    print_report(report)
    payload = json.dumps(report, indent=2, sort_keys=True, default=str)
    if args.out:
        Path(args.out).write_text(payload)
        print(f"\nSaved {args.out}")
    # Always refresh this fixed path too, in addition to any --out: the dashboard gateway's
    # /mission/api/readiness endpoint reads it and needs a predictable, always-current location.
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "readiness_report_latest.json").write_text(payload)
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
