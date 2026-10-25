#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
FIRMWARE_ROOT=${AUTOKART_FIRMWARE_ROOT:-"$ROOT/../../AutoKart-firmware"}
OUTPUT_DIR=${1:-"$ROOT/dist"}
PACKAGE_NAME=AutoKart-Brake-Test-Ubuntu22.04-Pi4B
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
STAGE="$WORK/$PACKAGE_NAME"
mkdir -p "$STAGE" "$OUTPUT_DIR"

copy_tree() {
    local source=$1 destination=$2
    mkdir -p "$(dirname "$STAGE/$destination")"
    cp -a "$ROOT/$source" "$STAGE/$destination"
}

copy_tree tools/bldc_encoder_camera_ui.py tools/bldc_encoder_camera_ui.py
copy_tree models/yolo26s models/yolo26s
copy_tree autonomy_stack_phase1/platform.sh autonomy_stack_phase1/platform.sh
copy_tree autonomy_stack_phase1/setup.sh autonomy_stack_phase1/setup.sh
copy_tree autonomy_stack_phase1/phase_01_perception/build_native.sh autonomy_stack_phase1/phase_01_perception/build_native.sh
copy_tree autonomy_stack_phase1/phase_01_perception/native autonomy_stack_phase1/phase_01_perception/native
copy_tree autonomy_stack_phase1/phase_01_perception/src autonomy_stack_phase1/phase_01_perception/src
copy_tree autonomy_stack_phase1/vendor/linux_ros/linux/libs autonomy_stack_phase1/vendor/linux_ros/linux/libs
copy_tree autonomy_stack_phase1/vendor/linux_ros/linux/configurationfiles autonomy_stack_phase1/vendor/linux_ros/linux/configurationfiles
copy_tree autonomy_stack_phase1/vendor/linux_ros/linux/scripts autonomy_stack_phase1/vendor/linux_ros/linux/scripts
copy_tree kart/ros2_ws/src/autokart_vehicle_geometry kart/ros2_ws/src/autokart_vehicle_geometry
copy_tree kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml
copy_tree brake_test/portable brake_test/portable

[[ -f "$FIRMWARE_ROOT/bench/full_bldc_encoder_test.py" ]] || {
    printf 'Matching AutoKart firmware not found at %s\n' "$FIRMWARE_ROOT" >&2
    exit 1
}
mkdir -p "$STAGE/esp32"
cp "$FIRMWARE_ROOT/bench/full_bldc_encoder_test.py" "$STAGE/esp32/main.py"
cp "$FIRMWARE_ROOT/bench/bldc_blue_cone_no_heartbeat_config.py" \
    "$STAGE/esp32/bldc_bench_config.py"
cp "$ROOT/brake_test/portable/ESP32_FLASH_README.md" "$STAGE/esp32/README.md"

find "$STAGE" -type d \( -name __pycache__ -o -name '.pytest_cache' -o -name 'build-*' \) -prune -exec rm -rf {} +
chmod +x "$STAGE/brake_test/portable/"*.sh

cp "$ROOT/brake_test/portable/AutoKart Brake Test.portable.desktop" \
    "$STAGE/AutoKart Brake Test.desktop"
chmod +x "$STAGE/AutoKart Brake Test.desktop"

cp "$ROOT/brake_test/portable/PACKAGE_README.md" "$STAGE/README.md"
cp "$ROOT/brake_test/RASPBERRY_PI4B_BRAKE_TEST.md" \
    "$STAGE/RASPBERRY_PI4B_BRAKE_TEST.md"

ARCHIVE="$OUTPUT_DIR/$PACKAGE_NAME.tar.gz"
tar -C "$WORK" -czf "$ARCHIVE" "$PACKAGE_NAME"
(
    cd "$OUTPUT_DIR"
    sha256sum "$PACKAGE_NAME.tar.gz" > "$PACKAGE_NAME.tar.gz.sha256"
)
printf '%s\n' "Built: $ARCHIVE" "Checksum: $ARCHIVE.sha256"
