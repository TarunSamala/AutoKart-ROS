#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"

set +u
source /opt/ros/humble/setup.bash
source /tmp/autokart-ros-install/setup.bash
source /usr/share/gazebo/setup.sh
set -u

runtime_dir="/tmp/autokart-gazebo-sync-probe"
mkdir -p "$runtime_dir"
export GAZEBO_MODEL_PATH="$project_root/simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="$project_root/simulation${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
export GAZEBO_PLUGIN_PATH="/tmp/autokart-ros-install/autokart_gazebo/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

gzserver --verbose --lockstep simulation/worlds/autokart_gazebo_follower_world.sdf \
  >"$runtime_dir/gzserver.log" 2>&1 &
server_pid=$!
cleanup() {
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
  if rg -q "AutoKart follower loaded" "$runtime_dir/gzserver.log"; then
    break
  fi
  sleep 0.1
done

ros2 run autokart_sim_bridge gazebo_sync_probe \
  --output artifacts/gazebo_follower/synchronization/sync_probe.json
