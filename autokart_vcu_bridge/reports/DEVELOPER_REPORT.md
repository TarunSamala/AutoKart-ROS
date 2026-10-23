# Developer report — autokart_vcu_bridge

## Audit performed before writing any code

Read in full: `kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/{core.py, protocol.py,
node.py}`, its `test/test_core_and_protocol.py`, `docs/VCU_PROTOCOL.md`, `docs/NODE_CONTRACTS.md`, and
`autokart_msgs/msg/{VcuState, ActuatorFeedback, MotionCommand, SensorDeliveryMetadata}.msg`.
`run_logger.py`, `run_log.py`, `demo_drive.py` and their tests were skimmed for scope only: they implement
the live node's run logging and dashboard-driven DEMO DRIVE feature, which are out of scope for a
standalone relay node and are not reused here.

Findings that shaped the design:

- `protocol.py` (framing/CRC/encode/decode) and `core.py` (`ThrottleController`, `CommandGate`,
  `raw_to_rad`, `brake_pct_for_candidate`, `arm_request_approved`) are pure Python with no ROS dependency
  and are already covered by `test_core_and_protocol.py`. They are reused, not rewritten.
- The real `node.py` is timer-driven (20 Hz `_tick`), not thread-driven. Serial reads are non-blocking
  (`timeout=0`). This node keeps that model. It differs from `autokart_hp60c`/`autokart_imu`, which need a
  capture thread for blocking reads.
- `arm_request_approved` implements a two-phase ARMING/ARMED handshake fed by `autokart_vcu_safety`'s
  JSON status. No safety supervisor existed in this library when this node was designed. Node 10 (`autokart_safety`) was
  built concurrently but is not wired to this node (see CONTRACT.md "Relationship to node 10"), so that function is
  **deliberately not reused**. Reusing it would mean fabricating the upstream handshake. ARM/DIS/RST are
  relayed directly, and nonzero throttle remains gated by a `safety_armed` input that fails closed (see
  CONTRACT.md).
- **Environment finding:** `kart/ros2_ws/install` is stale relative to its own `src/autokart_msgs`. The
  installed `VcuState` (Jul 29) lacks `estop_input_fitted`, which the source `.msg` (Oct 19) and the
  real vehicle-bridge node both use. The first integration run failed with
  `AttributeError: 'VcuState' object has no attribute 'estop_input_fitted'`. `kart/ros2_ws` was not
  touched. All ROS tests here run against a fresh `autokart_msgs` built from source into a /tmp scratch
  workspace, and the export bundle builds it fresh too.

## Field-by-field: what the architecture asks for vs. what exists (no fields invented)

| Architecture field | Source in the real protocol | Published where |
|---|---|---|
| sequence | `TEL.fwseq` | not in any typed message (see gap); our own outgoing `seq` is in `/vcu_bridge/status.sequence` |
| sample_time_us | `TEL.t_ms` (milliseconds since VCU boot, not µs) | not published as a typed field (gap) |
| axle_count_total | `TEL.counts` (`-1` = unavailable) | not in `ActuatorFeedback`/`VcuState` (gap); speed is in `ActuatorFeedback.motor_speed_radps` (m/s, same documented field reuse as the real node) |
| steering_actual_rad | `TEL.steer_raw` → `core.raw_to_rad` | `ActuatorFeedback.road_wheel_angle_rad` |
| brake status | `TEL.brk`, `TEL.brake_raw` | `ActuatorFeedback.brake_actuator_position_m` (raw AS5600 angle, MEASURE) |
| validity/fault bitmaps | `TEL.fault`, `TEL.flags` | `VcuState.active_faults`/`limiting_reason`/`estop_input_fitted`/`emergency_stop_active`, `ActuatorFeedback.valid` |
| command ACK | `TEL.ack_seq`, `TEL.link_age_ms` | **genuine gap**: no field in any existing message; logged at debug level only |
| command expiry | `MotionCommand.expiry` | honoured (0 = no expiry, documented convention) |
| boot_id | **does not exist** in the current wire protocol (`HELLO` = `fw_version,proto_version` only) | **genuine gap**: cannot be published without fabricating it |

Per the brief, these gaps are reported here rather than closed by silently adding message fields. Closing
them is a separate, reviewed change to `kart/ros2_ws/src/autokart_msgs` and, for `boot_id`, to the
firmware protocol itself.

## What was implemented

- `backends.py`: transport only. `SyntheticBackend` simulates the firmware's SAFE/ARMED/FAULT state
  machine (CMD_RANGE latch, RST, NAK on wrong state, monotonic axle counts) with no serial port and no
  import from `autokart_vehicle_bridge`. `SerialVcuBackend` lazily imports `autokart_vehicle_bridge.protocol`
  from `algorithm_package_path` and uses pyserial. A local `VcuTelemetry` dataclass mirrors the data shape
  only; the CRC/framing logic stays in `protocol.py`.
- `node.py`: lazily imports `autokart_vehicle_bridge.core` from the same parameter and runs the real
  `ThrottleController` and `CommandGate` every tick. If `core` cannot be imported, it falls back to
  throttle 0 / brake 100 and reports `CORE_LOGIC_UNAVAILABLE` on every status message; the control law is
  never reimplemented. It publishes `ActuatorFeedback`, `VcuState` and `/vcu_bridge/status`
  (`SensorDeliveryMetadata`), and sends a best-effort DIS on shutdown.
- Package skeleton, launch file, two config profiles, `export_node.sh` (with an optional
  `--with-algorithm` that ships the reused source under `third_party/`, outside `src/`), CONTRACT,
  DEPENDENCIES, README.
- 22 tests (13 unit, 2 rclpy integration, 7 fault injection), listed in VALIDATOR_REPORT.md.

## Defects found during development (beyond the separate review pass)

- Test isolation: `sys.modules` caching made a negative import test poison a later real-import test in
  the same process. Fixed with an autouse fixture. This is test hygiene, not a production defect; it is
  documented in DEPENDENCIES.md.
- First integration-test draft was invalid: it asserted on `motor_enable`, which follows the VCU's ARM
  state rather than the gate, and it published a one-shot candidate that went stale within 0.25 s.
  Rewritten to assert on measured speed with a continuously republished candidate.

## Known gaps (explicit)

- **Not integrated with node 10.** `autokart_safety` (node 10) was built concurrently in this library during this session. It publishes its arming decision as `MissionDebug` JSON on `/autokart/safety/status` and forwards ARM/DIS/RST on `/autokart/vcu/request`. This node does **not** consume that JSON status. Nothing publishes the `safety_armed` Bool, so throttle stays gated off when the two run together (fail closed). `request_topic` can be set to `/autokart/vcu/request` (same String ARM/DIS/RST semantics). Wiring `safety_armed` to node 10's status is an identified integration gap, NOT RUN.

- Live serial/USB path to a physical ESP32-S3: NOT RUN (no hardware).
- No upstream safety supervisor in this library: `safety_armed` is a placeholder that fails closed.
- `SerialVcuBackend` retries forever on read/write errors. The reference node instead drops the port
  (`self.ser = None`) after the first error. This is a deliberate difference in recovery behaviour,
  untested against a real USB unplug.
- Single-serial-owner hazard if run alongside the real `autokart_vehicle_bridge` against the same VCU
  (CONTRACT.md).
