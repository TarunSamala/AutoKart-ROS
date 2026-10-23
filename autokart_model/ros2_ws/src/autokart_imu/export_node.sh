#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 output_directory [autokart_msgs_source]" >&2
  exit 2
fi
node_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
node_name=$(basename "$node_dir")
msgs_src=${2:-"$node_dir/../../kart/ros2_ws/src/autokart_msgs"}
[[ -f "$msgs_src/package.xml" ]] || { echo "autokart_msgs_source unavailable" >&2; exit 1; }
mkdir -p "$1"
output_dir=$(cd -- "$1" && pwd)
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
mkdir -p "$stage/src/$node_name" "$stage/src/autokart_msgs"
tar --exclude='__pycache__' --exclude='.pytest_cache' --exclude='*.pyc' --exclude='*.tar.gz' --exclude='build' --exclude='install' --exclude='log' -cf - -C "$node_dir" . | tar -xf - -C "$stage/src/$node_name"
tar --exclude='__pycache__' --exclude='build' --exclude='install' --exclude='log' -cf - -C "$msgs_src" . | tar -xf - -C "$stage/src/autokart_msgs"
tar -czf "$output_dir/${node_name}_export.tar.gz" -C "$stage" src
printf '%s\n' "$output_dir/${node_name}_export.tar.gz"
