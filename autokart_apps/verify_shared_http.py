"""Compare CORE CLI and the running LAB HTTP adapter on identical inputs."""

from __future__ import annotations

import json
from math import cos, hypot, pi, sin
from pathlib import Path
import subprocess
import tempfile
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]


def skidpad_sample() -> dict:
    cones = []
    for center_x in (-9.0, 9.0):
        for radius, label in ((5.0, "blue"), (8.0, "yellow")):
            for index in range(16):
                angle = 2 * pi * index / 16
                world_x = center_x + radius * cos(angle)
                world_y = radius * sin(angle)
                forward, left = world_y, -(world_x + 2.5)
                cones.append({"id": len(cones), "label": label, "confidence": 0.95,
                              "xy": [forward, left], "sigma_m": 0.05,
                              "distance_m": hypot(forward, left), "box": [0, 0, 1, 1]})
    now = 1_000_000_000
    return {"timestamp_ns": now, "source": "SIMULATED", "mission": "SKIDPAD",
            "pose": {"x_m": -2.5, "y_m": 0.0, "yaw_rad": pi / 2, "speed_mps": 1.0,
                     "timestamp_ns": now, "position_std_m": 0.05}, "cones": cones}


def compare(name: str, payload: dict) -> None:
    reset = Request("http://127.0.0.1:8767/api/core/reset", b"{}",
                    {"Content-Type": "application/json"})
    with urlopen(reset, timeout=5):
        pass
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "sample.jsonl"
        path.write_text(json.dumps(payload) + "\n")
        core = json.loads(subprocess.check_output(
            [str(ROOT / "autokart_apps/run_core.sh"), "--source", "sample", "--input", str(path)],
            text=True))
    request = Request("http://127.0.0.1:8767/api/core/evaluate",
                      json.dumps(payload).encode(), {"Content-Type": "application/json"})
    with urlopen(request, timeout=5) as response:
        lab = json.load(response)
    for key in ("geometry", "planning", "mission_state", "control_candidate", "vehicle_authorized"):
        if core[key] != lab[key]:
            raise AssertionError(f"{name}: CORE/LAB {key} mismatch")
    if not core["geometry"]["delaunay_edges"] or not core["geometry"]["gabriel_edges"]:
        raise AssertionError(f"{name}: graph algorithms did not execute")
    if name == "STRAIGHT_TRACK":
        if not (core["geometry"]["boundaries"]["left_ids"] and
                core["geometry"]["boundaries"]["right_ids"] and
                core["geometry"]["gates"] and core["geometry"]["centerline"]):
            raise AssertionError("straight boundary/gate/centerline stages did not execute")
    else:
        if not (core["geometry"]["skidpad_topology"] and
                core["geometry"]["skidpad_topology"]["valid"] and
                core["geometry"]["centerline"] and core["mission_state"]["state"] == "LEFT"):
            raise AssertionError("skidpad topology/mission stages did not execute")
    if not core["planning"]["path_valid"] or not core["control_candidate"]["valid"]:
        raise AssertionError(f"{name}: path/controller candidate did not execute")
    if core["vehicle_authorized"]:
        raise AssertionError(f"{name}: unauthorized vehicle motion")
    print(f"{name}: PASS; {len(core['geometry']['gates'])} corridor gates; "
          f"skidpad topology={bool(core['geometry']['skidpad_topology'] and core['geometry']['skidpad_topology']['valid'])}; "
          f"steering={core['control_candidate']['steering_angle_rad']:.6f} rad")


if __name__ == "__main__":
    straight = json.loads((ROOT / "autokart_apps/examples/straight.jsonl").read_text())
    compare("STRAIGHT_TRACK", straight)
    compare("SKIDPAD", skidpad_sample())
