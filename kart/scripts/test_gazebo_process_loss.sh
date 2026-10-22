#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
runtime_dir="/tmp/autokart-gazebo-process-loss"
mkdir -p "$runtime_dir" artifacts/gazebo_follower/failure

if [[ "${1:-}" == "--verify-log" ]]; then
  if rg -q "\\[ros2run\\]: Process exited with failure 4" \
    artifacts/gazebo_follower/failure/process_loss.log; then
    echo "SY6_GAZEBO_PROCESS_LOSS: PASS (recorded native run rejected on coverage)"
    exit 0
  fi
  exit 4
fi

set +e
./scripts/run_gazebo_follower.sh \
  --scenario configs/scenarios/gazebo_follower_full_lap.yaml \
  --output-dir "$runtime_dir/artifacts" \
  >"$runtime_dir/run.log" 2>&1 &
runner_pid=$!
set -e

server_pid=""
for _ in $(seq 1 200); do
  server_pid="$(pgrep -P "$runner_pid" -x gzserver | head -n 1 || true)"
  if [[ -n "$server_pid" ]] && rg -q "AutoKart follower loaded" \
    /tmp/autokart-gazebo-follower/gzserver.log; then
    break
  fi
  sleep 0.1
done
if [[ -z "$server_pid" ]]; then
  kill "$runner_pid" >/dev/null 2>&1 || true
  wait "$runner_pid" >/dev/null 2>&1 || true
  echo "Gazebo process-loss test could not find the server" >&2
  exit 2
fi

sleep 1
kill "$server_pid"
set +e
wait "$runner_pid"
status=$?
set -e
cp "$runtime_dir/run.log" artifacts/gazebo_follower/failure/process_loss.log
if [[ "$status" -eq 0 ]]; then
  echo "Gazebo process loss was not rejected" >&2
  exit 3
fi
if ! rg -q "Gazebo process/follower subscription lost; native run invalid|\\[ros2run\\]: Process exited with failure 4" \
  "$runtime_dir/run.log"; then
  sed -n '1,200p' "$runtime_dir/run.log" >&2
  exit 4
fi
echo "SY6_GAZEBO_PROCESS_LOSS: PASS (native run rejected, internal command path not bypassed)"
