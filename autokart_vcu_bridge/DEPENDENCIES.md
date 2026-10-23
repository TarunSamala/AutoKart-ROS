# External dependencies — autokart_vcu_bridge

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `std_msgs`, `builtin_interfaces` — ROS 2 Humble core.
- `autokart_msgs` — this repository's shared interface package (`kart/ros2_ws/src/autokart_msgs`).
  `ActuatorFeedback`, `VcuState`, `MotionCommand`, `SensorDeliveryMetadata` are used — no new message
  type was introduced (see CONTRACT.md "Known, documented interface gaps" for what doesn't fit and
  why nothing was silently added).

## Reused algorithm dependency (`autokart_vehicle_bridge`)

This node does **not** vendor or reimplement the VCU wire protocol or the throttle-control/safety-gate
logic. At runtime (not build time), it lazily imports, from the `algorithm_package_path` ROS parameter
(never hardcoded):

- `autokart_vehicle_bridge.protocol` (`kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/protocol.py`)
  — CRC16-CCITT framing, `CMD`/`ARM`/`DIS`/`RST` encoding, `TEL`/`HELLO`/`NAK` decoding. Used inside
  `backends.py`'s `SerialVcuBackend` (the real, hardware-facing adapter).
- `autokart_vehicle_bridge.core` (same directory's `core.py`) — `ThrottleController`, `ThrottleConfig`,
  `CommandGate`, `raw_to_rad`, `brake_pct_for_candidate`. Used directly in `node.py`, since this is pure
  control-law/gating logic with no hardware dependency of its own — it is reused identically regardless
  of which backend (`synthetic` or `serial`) is active.

With `backend: synthetic` (the default) and **no** `algorithm_package_path` configured, there is **no**
dependency beyond the four ROS packages above: the package builds, and the `synthetic`-only unit/
integration/fault-injection tests pass, on a plain Ubuntu 22.04 + ROS 2 Humble machine with no access to
the rest of this repository. In that configuration the node falls back to a trivially safe degenerate
policy (throttle always 0 mV, brake always 100%) and reports `CORE_LOGIC_UNAVAILABLE` on every status
message — see CONTRACT.md.

**To reuse this node with the real control law and/or real hardware:**
1. Copy (or otherwise make available) `kart/ros2_ws/src/autokart_vehicle_bridge`'s source directory
   onto the target machine.
2. Set `algorithm_package_path` in `config/vcu_bridge_live.yaml` (or any config) to that directory's
   path on the target machine — never hardcoded in source.
3. For real hardware, also set `backend: serial` and `serial_port`/`serial_baud`.

No path in this package's source code is hardcoded to this repository's location. If the dependency
path is wrong or missing, both the `core.py` control law and (for `backend: serial`) the serial
transport fail closed independently (`CORE_LOGIC_PATH_NOT_CONFIGURED`/`DEPENDENCY_PATH_NOT_FOUND`/
`SDK_ADAPTER_IMPORT_FAILED` in `/vcu_bridge/status`), never silently.

## `pyserial` (backend: serial only)

`pyserial` (`import serial`) is imported lazily inside `SerialVcuBackend`, not declared as a hard
`package.xml` dependency, so `backend: synthetic` has zero dependency on it. It happens to already be
installed in this development environment (`pyserial 3.5`), which let this session exercise the
"serial port does not exist" fail-closed path against the **real** `pyserial` library (not a fake), even
though no physical VCU is attached — see `reports/VALIDATOR_REPORT.md`.

## A general characteristic of this lazy sys.path-based import pattern (shared with autokart_hp60c)

Both `SerialVcuBackend` (for `protocol`) and `node.py` (for `core`) resolve the dependency by inserting
`algorithm_package_path` into `sys.path` and calling `importlib.import_module`, which caches the result
in `sys.modules` under the plain package name (`autokart_vehicle_bridge`). A single node process only
ever does this once, with one fixed `algorithm_package_path`, so this is never an issue in actual use.
It **is** something this session's own test suite had to account for: `test/test_backends.py` points
different tests at two different `autokart_vehicle_bridge`-named directories (one real, one a
deliberately-incomplete fake) within the same pytest process, and without explicit `sys.modules`
clean-up between tests, the second test would silently reuse whichever package the first test's import
cached, rather than genuinely re-resolving the new path. `test/test_backends.py` now has an `autouse`
fixture that clears this cache between tests — a test-hygiene fix, not a production code change.

## What was NOT verified in this session

No physical ESP32-S3/VCU is attached in this development environment. `backend: serial`'s actual
read/write-over-USB path against real hardware is **not exercised** — only its failure paths (missing
`algorithm_package_path`, missing/unopenable serial port) and its real use of `protocol.build_cmd`/
`parse_frame`/`parse_message` (against the genuine `autokart_vehicle_bridge.protocol` module, not a
fake) are tested. See `reports/VALIDATOR_REPORT.md`.
