#!/usr/bin/env python3
"""Read-only: confirm each tracked config file carries its documented schema_version and that it
matches this tool's expectation. Writes nothing, changes no config, sets no verified flag. A
mismatch or missing version is reported as FAIL, never silently ignored.

Two of the four files below are ROS parameter YAML (commissioning.yaml, vcu_safety.yaml): their
schema version is kept as a plain comment, not a ros__parameters key, specifically so adding it
does not change what rclpy loads from the file. The other two carry a real schema_version: N key
at the top level, which is harmless there (their loaders index a specific key/sub-mapping and
ignore unrelated top-level keys -- confirmed by reading autokart_vehicle_geometry/geometry.py and
autokart_perception/sdk.py's validate_config())."""
from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

TRACKED = [
    {"path": ROOT / "autonomy_stack_phase1/phase_01_perception/src/autokart_perception/data/perception.yaml",
     "owner": "autokart_perception", "expected": 1, "kind": "key"},
    {"path": ROOT / "kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml",
     "owner": "autokart_vehicle_geometry", "expected": 1, "kind": "key"},
    {"path": ROOT / "kart/ros2_ws/src/autokart_vcu_safety/config/vcu_safety.yaml",
     "owner": "autokart_vcu_safety", "expected": 1, "kind": "comment"},
    {"path": ROOT / "kart/ros2_ws/src/autokart_bringup/config/commissioning.yaml",
     "owner": "autokart_planner / autokart_localization", "expected": 1, "kind": "comment"},
]

KEY_RE = re.compile(r"^schema_version:\s*(\d+)", re.MULTILINE)
COMMENT_RE = re.compile(r"^#\s*config_schema_version:\s*(\d+)", re.MULTILINE)


def check_one(entry: dict) -> dict:
    path, expected, kind = entry["path"], entry["expected"], entry["kind"]
    result = {"path": str(path), "owner": entry["owner"], "expected_schema_version": expected}
    if not path.is_file():
        return {**result, "status": "FAIL", "reason": "FILE_MISSING"}
    text = path.read_text(encoding="utf-8")
    match = (KEY_RE if kind == "key" else COMMENT_RE).search(text)
    if match is None:
        return {**result, "status": "FAIL", "reason": "SCHEMA_VERSION_NOT_FOUND"}
    found = int(match.group(1))
    status = "PASS" if found == expected else "FAIL"
    return {**result, "status": status, "found_schema_version": found,
            "reason": None if status == "PASS" else "SCHEMA_VERSION_MISMATCH"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(ROOT / "results"))
    args = parser.parse_args()
    results = [check_one(entry) for entry in TRACKED]
    for row in results:
        detail = f"schema_version={row.get('found_schema_version', '—')}" if "found_schema_version" in row else row["reason"]
        print(f"{row['status']}: {row['path']} ({row['owner']}) {detail}")
    report = {"created_utc": datetime.now(timezone.utc).isoformat(), "results": results,
              "all_pass": all(r["status"] == "PASS" for r in results)}
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"config_schema_check_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True))
    print(f"Saved {path}")
    return 0 if report["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
