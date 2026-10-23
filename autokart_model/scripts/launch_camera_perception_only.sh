#!/usr/bin/env bash
set -eo pipefail

if [[ "${AUTOKART_NO_ACTUATION_ACK:-}" != "YES" ]]; then
    echo "Refusing to start." >&2
    echo "Physically disconnect motor/traction power and all actuator authority, then run:" >&2
    echo "AUTOKART_NO_ACTUATION_ACK=YES bash scripts/launch_camera_perception_only.sh" >&2
    exit 3
fi

bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ws="$bundle_root/ros2_ws"
runtime="$bundle_root/runtime"
source "$runtime/platform.sh"
target=$(get_sdk_lib_target)
bridge="$runtime/phase_01_perception/native/build-$target/libhp60c_bridge.so"
camera_config="$runtime/vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json"
algo="$runtime/phase_01_perception/src"
model="$bundle_root/models/yolo26s/best.onnx"
model_manifest="$bundle_root/models/yolo26s/best.json"

for required in /opt/ros/humble/setup.bash "$ws/install/local_setup.bash" "$bridge" "$camera_config" "$model" "$model_manifest"; do
    if [[ ! -e "$required" ]]; then
        echo "Missing required file: $required" >&2
        exit 2
    fi
done

source /opt/ros/humble/setup.bash
source "$ws/install/local_setup.bash"

for forbidden in autokart_vcu_bridge autokart_controller autokart_safety steering brake bldc; do
    if ros2 node list 2>/dev/null | grep -qi "$forbidden"; then
        echo "Refusing to start: existing ROS node matches forbidden pattern '$forbidden'." >&2
        exit 4
    fi
done

cleanup() {
    kill "${camera_pid:-}" "${perception_pid:-}" 2>/dev/null || true
    wait "${camera_pid:-}" "${perception_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting stationary camera + perception only. No VCU/control/actuation node is launched."
ros2 run autokart_hp60c hp60c_node --ros-args \
  -p backend:=hp60c \
  -p algorithm_package_path:="$algo" \
  -p hp60c_library_path:="$bridge" \
  -p hp60c_configuration_path:="$camera_config" \
  -p max_age_ms:=250 &
camera_pid=$!

ros2 run autokart_perception_adapter perception_adapter_node --ros-args \
  -p backend:=algorithm \
  -p input_mode:=topics \
  -p source_label:=LIVE \
  -p algorithm_package_path:="$algo" \
  -p model_path:="$model" \
  -p model_manifest:="$model_manifest" &
perception_pid=$!

wait -n "$camera_pid" "$perception_pid"

