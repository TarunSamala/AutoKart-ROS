# autokart_vcu_bridge

Sole USB/serial VCU endpoint owner. Relays an already-authorized `autokart_msgs/MotionCommand` candidate
into the AutoKart laptop<->VCU wire protocol (`docs/VCU_PROTOCOL.md`) and publishes `ActuatorFeedback` /
`VcuState` from VCU telemetry. It **does not decide motion authority**: nonzero throttle requires an
external `safety_armed` input. That is a placeholder: node 10 (`autokart_safety`, built concurrently) is not yet wired to it, so throttle stays gated off. See
`CONTRACT.md` for the interface and `DEPENDENCIES.md` for what it reuses.

Reuses, lazily from the `algorithm_package_path` parameter (never hardcoded):
`autokart_vehicle_bridge.protocol` (framing/CRC/encode/decode) and `autokart_vehicle_bridge.core`
(`ThrottleController`, `CommandGate`, `raw_to_rad`, `brake_pct_for_candidate`) from
`kart/ros2_ws/src/autokart_vehicle_bridge`. Nothing in those modules is reimplemented here.

## Quick start (no hardware required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to autokart_vcu_bridge
source install/setup.bash
# Fully portable: synthetic VCU, no reused control law -> CORE_LOGIC_UNAVAILABLE, throttle forced 0.
ros2 launch autokart_vcu_bridge vcu_bridge.launch.py
# Synthetic VCU + the real reused core.py control law:
ros2 run autokart_vcu_bridge vcu_bridge_node --ros-args \
  -p algorithm_package_path:=/path/to/kart/ros2_ws/src/autokart_vehicle_bridge
ros2 topic echo /vcu_bridge/status
ros2 topic echo /vcu/state
```

## Running against a real ESP32-S3 VCU

Set `backend: serial`, `serial_port`, and `algorithm_package_path` in `config/vcu_bridge_live.yaml`.
**This path is NOT RUN in this session — no ESP32-S3/VCU is attached.** Only its fail-closed behaviour
(missing dependency path, nonexistent serial port against the real `pyserial`) and its real use of the
reused `protocol.py` are tested.

## Tests

```bash
source /opt/ros/humble/setup.bash
source <workspace with a CURRENT autokart_msgs build>/install/setup.bash
cd autokart_ros_nodes/autokart_vcu_bridge
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

Note: `kart/ros2_ws/install` in this repository is stale relative to its own `autokart_msgs` source
(the installed `VcuState` lacks `estop_input_fitted`). Build `autokart_msgs` fresh (e.g. into a scratch
workspace) before running the ROS tests; the export bundle does this automatically.

- `test/test_backends.py` — 13 unit tests, no ROS needed (synthetic VCU state machine; real import and
  use of `autokart_vehicle_bridge.protocol`; fail-closed paths).
- `test/test_node_integration.py` — 2 rclpy tests with the real `core.py`: throttle flows only with
  ARMED + `safety_armed` + a fresh candidate; never without `safety_armed`.
- `test/test_fault_injection.py` — 7 tests: persistent read failure, unexpected exception, missing
  dependency keeps reporting, missing `core.py` reports `CORE_LOGIC_UNAVAILABLE` with safe output,
  best-effort DIS on shutdown (and survives a failing DIS), real-process SIGINT exits cleanly.

22 tests total. Set `AUTOKART_VEHICLE_BRIDGE_PATH` to point the 4 real-algorithm tests at a non-default
checkout (e.g. an export bundle's `third_party/autokart_vehicle_bridge`).

Tests needing the real `autokart_vehicle_bridge` checkout skip (not fail) when it is absent; see
`reports/VALIDATOR_REPORT.md` for exactly which ran where.

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir                    # node + autokart_msgs
./export_node.sh --with-algorithm /path/to/export/dir   # also ships the reused protocol/core source
```

## Status

IMPLEMENTED and TESTED (software-only, synthetic VCU + real reused protocol/core + fault injection).
Live serial/USB path to a physical VCU: **NOT RUN**. Integration with node 10 (`autokart_safety`):
**NOT RUN**. `safety_armed` is a placeholder that fails closed (CONTRACT.md "Relationship to node 10"). **Not a vehicle-readiness claim.**
