# External dependencies — autokart_hp60c

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `sensor_msgs` — ROS 2 Humble core, assumed present on any Humble install.
- `autokart_msgs` — this repository's shared interface package
  (`kart/ros2_ws/src/autokart_msgs`). Only `SensorDeliveryMetadata.msg` is used. To reuse this node
  in another workspace, copy `autokart_msgs` alongside it (it is interface-only: no Python/C++ logic,
  no further dependency chain beyond `std_msgs`, `builtin_interfaces`, `geometry_msgs`).

## Hardware/SDK dependency (`backend: hp60c` only)

This node does **not** vendor or reimplement HP60C SDK access. With `backend: synthetic` (the
default), there is no SDK dependency at all — the package builds and all tests pass on a plain
Ubuntu 22.04 + ROS 2 Humble machine with nothing else installed.

With `backend: hp60c`, at runtime (not build time) this node imports and reuses:

- `autokart_perception.camera.HP60C` (Python, ctypes wrapper), from
  `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py` in the main
  `autonomous_kart` repository.
- which in turn loads the native `libhp60c_bridge.so` built from
  `autonomy_stack_phase1/phase_01_perception/native/hp60c_bridge.cpp` via
  `autonomy_stack_phase1/phase_01_perception/build_native.sh`, against the YDLIDAR vendor SDK under
  `autonomy_stack_phase1/vendor/linux_ros/`.
- and a vendor camera configuration JSON (e.g.
  `vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json`).

**To reuse this node in a clean workspace with real hardware:**
1. Copy (or otherwise make available) the `autokart_perception` algorithm package's `src/` directory
   and the built `libhp60c_bridge.so` (plus its vendor configuration file) onto the target machine.
2. Set the `algorithm_package_path`, `hp60c_library_path` and `hp60c_configuration_path` parameters
   in `config/hp60c_live.yaml` to their actual paths on that machine — never hardcode a
   `/home/mark1/...` path into source; the launch file takes the config path as an argument.
3. Set `backend: hp60c`.

No path in this package's source code is hardcoded to this repository's location. If the dependency
path is wrong or missing, the node fails closed at startup (`DEPENDENCY_PATH_NOT_FOUND` /
`SDK_ADAPTER_IMPORT_FAILED` in `/hp60c/status`), not silently.

## What was NOT verified in this session

No physical HP60C camera was available. `backend: hp60c` is implemented and unit-tested for its
failure paths (missing dependency, import failure) but **the live HP60C read path itself has not
been exercised against real hardware by this change** — that inherits the same "not physically
qualified" status already recorded for the existing embedded camera code in
`docs/decisions/CANONICAL_COMPONENTS.md`. This node reuses that exact, already-existing `HP60C`
class rather than re-implementing SDK access, so it carries the same hardware-verification gap, not
a new one.
