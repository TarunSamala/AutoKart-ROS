#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scenario="$project_root/configs/scenarios/gazebo_follower_full_lap.yaml"
output_dir="$project_root/artifacts/gazebo_follower/full_lap"
follower_hz=""
preset=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      scenario="$2"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --follower-hz)
      follower_hz="$2"
      shift 2
      ;;
    --preset)
      preset="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

cd "$project_root"
if [[ ! -f /tmp/autokart-ros-install/autokart_gazebo/lib/libautokart_gazebo_follower.so ]]; then
  ./scripts/build_ros2.sh
fi

set +u
source /opt/ros/humble/setup.bash
source /tmp/autokart-ros-install/setup.bash
source /usr/share/gazebo/setup.sh
set -u
world="$(python3 - "$scenario" <<'PY'
from pathlib import Path
import sys
import yaml
payload = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(payload["gazebo"]["world"])
PY
)"
if [[ -z "$preset" ]]; then
  preset="$(python3 - "$scenario" <<'PY'
from pathlib import Path
import sys
import yaml
payload = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(payload.get("preset", ""))
PY
)"
fi
if [[ -z "$follower_hz" ]]; then
  follower_hz="$(python3 - "$scenario" <<'PY'
from pathlib import Path
import sys
import yaml
payload = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(payload.get("rates", {}).get("gazebo_follow_hz", 100))
PY
)"
fi

mkdir -p "$output_dir"
runtime_dir="/tmp/autokart-gazebo-follower"
mkdir -p "$runtime_dir"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"
if [[ -n "$preset" ]] && python3 -m kart_sim.gazebo_preset \
    "$preset" --runtime-dir "$runtime_dir/generated" --project-root "$project_root" \
    >"$runtime_dir/prepared_preset.json" 2>"$runtime_dir/prepared_preset.err"; then
  world="$(python3 - "$runtime_dir/prepared_preset.json" <<'PY'
import json
from pathlib import Path
import sys
print(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))["world_path"])
PY
)"
  generated_model_path="$(python3 - "$runtime_dir/prepared_preset.json" <<'PY'
import json
from pathlib import Path
import sys
print(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))["model_search_path"])
PY
)"
  preset_software="$(python3 - "$runtime_dir/prepared_preset.json" <<'PY'
import json
from pathlib import Path
import sys
print("1" if json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))["software_render"] else "0")
PY
)"
  gazebo_cpu_list="$(python3 - "$runtime_dir/prepared_preset.json" <<'PY'
import json
from pathlib import Path
import sys
print(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")).get("execution", {}).get("gazebo_cpu_list", ""))
PY
)"
  core_cpu_list="$(python3 - "$runtime_dir/prepared_preset.json" <<'PY'
import json
from pathlib import Path
import sys
value = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")).get("execution", {}).get("core_cpu_list", [])
print(",".join(str(cpu) for cpu in value))
PY
)"
  recorder_cpu_list="$(python3 - "$runtime_dir/prepared_preset.json" <<'PY'
import json
from pathlib import Path
import sys
print(json.loads(Path(sys.argv[1]).read_text(encoding="utf-8")).get("execution", {}).get("recorder_cpu_list", ""))
PY
)"
  if [[ -n "$core_cpu_list" ]]; then
    export AUTOKART_CORE_CPU_LIST="$core_cpu_list"
  fi
  export GAZEBO_MODEL_PATH="$generated_model_path:$project_root/simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
else
  export GAZEBO_MODEL_PATH="$project_root/simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
  preset_software="1"
  gazebo_cpu_list=""
  recorder_cpu_list=""
fi
export GAZEBO_RESOURCE_PATH="$project_root/simulation${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
export GAZEBO_PLUGIN_PATH="/tmp/autokart-ros-install/autokart_gazebo/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-$preset_software}"

gazebo_command=(gzserver --verbose --lockstep "$world")
if [[ -n "$gazebo_cpu_list" ]] && command -v taskset >/dev/null 2>&1; then
  gazebo_command=(taskset -c "$gazebo_cpu_list" "${gazebo_command[@]}")
fi
"${gazebo_command[@]}" >"$runtime_dir/gzserver.log" 2>&1 &
server_pid=$!
bag_pid=""
cleanup() {
  if [[ -n "$bag_pid" ]]; then
    kill -INT "$bag_pid" >/dev/null 2>&1 || true
    wait "$bag_pid" >/dev/null 2>&1 || true
  fi
  kill "$server_pid" >/dev/null 2>&1 || true
  for _ in $(seq 1 20); do
    if ! kill -0 "$server_pid" >/dev/null 2>&1; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "$server_pid" >/dev/null 2>&1; then
    kill -KILL "$server_pid" >/dev/null 2>&1 || true
  fi
  wait "$server_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT

for _ in $(seq 1 100); do
  if ! kill -0 "$server_pid" >/dev/null 2>&1; then
    sed -n '1,240p' "$runtime_dir/gzserver.log" >&2
    exit 1
  fi
  if rg -q "AutoKart follower loaded" "$runtime_dir/gzserver.log"; then
    break
  fi
  sleep 0.1
done

if [[ "${AUTOKART_RECORD_ROSBAG:-0}" == "1" ]]; then
  ros2 bag record -a -o "$output_dir/rosbag/full_lap" \
    >"$runtime_dir/rosbag.log" 2>&1 &
  bag_pid=$!
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,utilization.gpu,memory.used \
    --format=csv,noheader >"$output_dir/gpu_before.txt" 2>&1 || true
else
  printf 'NOT_EXECUTED_EXTERNAL_DEPENDENCY: nvidia-smi unavailable\n' \
    >"$output_dir/gpu_before.txt"
fi

recorder_args=(
  --scenario "$scenario"
  --output-dir "$output_dir"
  --follower-hz "$follower_hz"
)
if [[ -n "$preset" ]]; then
  recorder_args+=(--preset "$preset")
fi
recorder_command=(ros2 run autokart_sim_bridge gazebo_full_lap "${recorder_args[@]}")
if [[ -n "$recorder_cpu_list" ]] && command -v taskset >/dev/null 2>&1; then
  recorder_command=(taskset -c "$recorder_cpu_list" "${recorder_command[@]}")
fi
/usr/bin/time -v -o "$output_dir/process_performance.txt" \
  "${recorder_command[@]}"

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,utilization.gpu,memory.used \
    --format=csv,noheader >"$output_dir/gpu_after.txt" 2>&1 || true
fi

printf 'Gazebo follower artifacts: %s\n' "$output_dir"
