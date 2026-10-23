#!/usr/bin/env bash
set -euo pipefail

bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
runtime="$bundle_root/runtime"

source "$runtime/platform.sh"
target=$(get_sdk_lib_target)
sdk="$runtime/vendor/linux_ros/linux"
build_dir="$runtime/phase_01_perception/native/build-$target"

cmake -S "$runtime/phase_01_perception/native" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DSDK_ROOT="$sdk" -DSDK_LIB_TARGET="$target"
cmake --build "$build_dir" --parallel

echo "Built: $build_dir/libhp60c_bridge.so"

