#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"

python3 -m pytest tests/gazebo_follower -v
./scripts/build_ros2.sh
export GAZEBO_MODEL_PATH="${project_root}/simulation/models${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
export GAZEBO_RESOURCE_PATH="${project_root}/simulation${GAZEBO_RESOURCE_PATH:+:${GAZEBO_RESOURCE_PATH}}"
gz sdf -k simulation/models/autokart/model.sdf
gz sdf -k simulation/worlds/autokart_gazebo_follower_world.sdf

if [[ "${AUTOKART_SKIP_NATIVE:-0}" == "1" ]]; then
  echo "native_gazebo_follower: NOT_EXECUTED_REQUESTED_SKIP"
  exit 0
fi

./scripts/run_gazebo_follower.sh \
  --scenario configs/scenarios/gazebo_follower_full_lap.yaml \
  --output-dir artifacts/gazebo_follower/full_lap

python3 - <<'PY'
import json
from pathlib import Path
root = Path("artifacts/gazebo_follower/full_lap")
summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
native = summary["native_gazebo"]
assert summary["completed"]
assert summary["controlled_stop_complete"]
assert native["follower_updates"] > 100
assert native["double_physics_diagnostics"] == 0
assert native["timestamp_violations"] == 0
for sensor in ("rgb_camera", "depth_camera", "lidar", "imu"):
    assert native["sensors"][sensor]["delivered"] > 0
    assert native["sensors"][sensor]["empty_payloads"] == 0
print("native_gazebo_follower: PASS")
PY
