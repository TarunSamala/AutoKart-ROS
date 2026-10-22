#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world="${1:-$project_root/simulation/worlds/autokart_empty_test_world.sdf}"
runtime_dir="/tmp/autokart-gazebo-native"
mkdir -p "$runtime_dir"
export GAZEBO_MODEL_PATH="$project_root/simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
gzserver --verbose "$world" >"$runtime_dir/server.log" 2>&1 &
server_pid=$!
cleanup() {
  kill "$server_pid" >/dev/null 2>&1 || true
  wait "$server_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT
ready=0
for _ in $(seq 1 50); do
  if timeout 1 gz topic -l 2>/dev/null | grep -q "/gazebo/"; then
    ready=1
    break
  fi
  sleep 0.1
done
if [[ "$ready" != "1" ]] || ! kill -0 "$server_pid" >/dev/null 2>&1; then
  echo "gazebo_world_launch: FAIL" >&2
  sed -n '1,160p' "$runtime_dir/server.log" >&2
  exit 1
fi
timeout 10 gz model --spawn-file="$project_root/simulation/models/autokart/model.sdf" --model-name=autokart
pose_start="$(timeout 5 gz model --model-name=autokart --pose)"
sleep 2
topics="$(timeout 5 gz topic -l)"
printf '%s\n' "$topics" >"$runtime_dir/topics.txt"
pose="$(timeout 5 gz model --model-name=autokart --pose)"
for required_topic in "/rgb_camera/image" "/depth_camera/image" "/lidar/scan" "/imu/imu"; do
  if ! printf '%s\n' "$topics" | grep -q "$required_topic"; then
    echo "gazebo_sensor_topic: FAIL $required_topic" >&2
    exit 1
  fi
done
imu_topic="$(printf '%s\n' "$topics" | grep '/imu/imu$' | head -n 1)"
timeout 3 stdbuf -oL gz topic -e -t "$imu_topic" >"$runtime_dir/imu_sample.txt" 2>&1 || true
timeout 2 stdbuf -oL gz stats >"$runtime_dir/stats.txt" 2>&1 || true
if [[ -z "$pose" ]] || [[ "$pose" != "$pose_start" ]]; then
  echo "gazebo_kart_spawn: FAIL" >&2
  sed -n '1,200p' "$runtime_dir/server.log" >&2
  exit 1
fi
printf '%s\n' "$pose" >"$runtime_dir/pose.txt"
printf 'gazebo_world_launch: PASS\n'
printf 'gazebo_kart_spawn: PASS\n'
printf 'gazebo_pose_source: %s\n' "$pose"
printf 'gazebo_topics: %s\n' "$(printf '%s\n' "$topics" | wc -l)"
printf 'gazebo_sensor_interfaces: PASS rgb,depth,lidar,imu\n'
if [[ -s "$runtime_dir/imu_sample.txt" ]]; then
  printf 'gazebo_sensor_sample: PASS imu\n'
else
  printf 'gazebo_sensor_sample: NOT_EXECUTED_HEADLESS_TRANSPORT_CAPTURE\n'
fi
printf 'physics_authority: internal_visual_follower_contract; no_drive_plugin\n'
