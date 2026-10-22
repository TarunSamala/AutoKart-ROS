"""Adapts an existing kart_sim telemetry.jsonl + metadata.json run (skidpad,
braking, autocross, or any other kart_sim.scenario run) to the same replay
JSON schema kart_sim.viewer_export produces for my_track, so the one viewer
(kart/viewer/index.html) can play back all four scenarios.

Scope note (Step 8 / D4): this adapts a run that ALREADY has telemetry.jsonl
on disk. It does not run skidpad/braking/autocross itself -- running skidpad
via scripts/run_formula_student_event.sh currently fails with a PRE-EXISTING
bug unrelated to this branch ("kart is not autonomously armed after reset",
in kart_sim/fast_env.py), reported here rather than fixed, per the ground
rule against touching VCU/arming behaviour. See docs/MY_TRACK.md.
"""

from __future__ import annotations

import json
from math import degrees
from pathlib import Path


def export_generic_replay(telemetry_jsonl: Path, metadata_json: Path, *, scenario_name: str, out_path: Path) -> Path:
    metadata = json.loads(metadata_json.read_text())
    frames = []
    last_t, last_x, last_y = 0.0, None, None
    next_log_t = 0.0
    with telemetry_jsonl.open() as stream:
        for line in stream:
            record = json.loads(line)
            state = record["state"]
            t = state["time_s"]
            if t < next_log_t:
                continue
            next_log_t += 0.1
            commands = record.get("commands", {}).get("final", {})
            dist_m = 0.0
            if last_x is not None:
                from math import hypot
                dist_m = frames[-1]["dist_m"] + hypot(state["x_m"] - last_x, state["y_m"] - last_y) if frames else 0.0
            last_x, last_y, last_t = state["x_m"], state["y_m"], t
            frames.append({
                "t": round(t, 2), "dist_m": round(dist_m, 3), "x": round(state["x_m"], 3), "y": round(state["y_m"], 3),
                "heading_deg": round(degrees(state["yaw_rad"]), 2),
                "speed_mps": round(state["u_mps"], 3), "speed_kmh": round(state["u_mps"] * 3.6, 2),
                "speed_target_mps": 0.0,
                "steer_cmd_deg": round(degrees(commands.get("steering_rad", 0.0)), 2),
                "steer_deg": round(degrees(commands.get("steering_rad", 0.0)), 2),
                "brake_cmd_pct": round(commands.get("brake", 0.0) * 100, 1), "brake_pct": round(commands.get("brake", 0.0) * 100, 1),
                "decel_mps2": round(max(0.0, -state.get("ax_mps2", 0.0)), 2),
                "throttle_pct": round(commands.get("drive", 0.0) * 100, 1), "dac_V": round(commands.get("drive", 0.0) * 5.0, 2),
                "section": "-", "state": "RUN", "gates": 0, "map_cones": 0, "frame_age_ms": 0.0,
                "centreline_offset_m": 0.0, "clearance_m": 9.0, "obstacle": "no", "detour_offset_m": 0.0,
            })

    payload = {
        "scenario": scenario_name, "profile": metadata.get("kart", "?"), "seed": metadata.get("seed", 0),
        "metrics": {"outcome": "N/A (generic adapter, not a my_track run)", "collisions": 0, "min_clearance_m": None,
                     "max_speed_mps": max((f["speed_mps"] for f in frames), default=0.0), "max_steer_deg": max((abs(f["steer_deg"]) for f in frames), default=0.0),
                     "stop_gap_to_finish_m": None},
        "kart_geometry": {"front_extent_m": 1.0, "rear_overhang_m": 0.3, "half_width_m": 0.5},
        "camera": {"hfov_deg": 64.4, "range_m": 4.0},
        "cones": [],
        "events": [],
        "frames": frames,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload), encoding="utf-8")
    return out_path
