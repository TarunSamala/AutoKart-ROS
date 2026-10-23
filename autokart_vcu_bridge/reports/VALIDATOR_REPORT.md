# Independent validator report — autokart_vcu_bridge

## Environment

ROS 2 Humble, colcon, and pyserial 3.5. **No ESP32-S3/VCU attached** and no serial device.
`kart/ros2_ws/install` is stale (its installed `VcuState` lacks `estop_input_fitted`, which caused the
first integration run to fail with `AttributeError`). It was not used and not touched. In-repo ROS runs
used a fresh `autokart_msgs` built from source into `/tmp/ak_nodes_scratch/msgs_ws`.

## Runs and actual output

1. **Unit tests** (`test/test_backends.py`, no ROS needed) → `13 passed`. In verbose mode, both
   real-`protocol.py` tests show `PASSED` (they ran, not skipped), i.e. a genuine import of
   `kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/protocol.py`, `build_cmd`/`parse_frame`
   round trip, and `HELLO` parse.
2. **Full suite, in repo** → `22 passed` (13 unit + 2 integration + 7 fault injection).
   **Soak:** 10 consecutive runs → 10 × `22 passed`, 0 failures, 0 teardown warnings after the
   spin-thread-join fix. Before that fix, the `Destroyable` exit message appeared in about 2 of ~20 runs
   and never failed an assertion.
3. **Reviewer fixes reproduced pre-fix:**
   - DIS-on-shutdown line removed → `test_shutdown_sends_best_effort_disarm` FAILED; restored → passes.
   - `main()` guard reverted → `test_sigint_shutdown_exits_cleanly_without_rclerror` FAILED; restored →
     passes. This defect was first observed live: `RCLError: rcl_shutdown already called` on Ctrl-C of
     the node launched from the export bundle.
4. **Integration behaviour (real `core.py`, synthetic VCU):**
   - ARM + `safety_armed=True` (republished at 20 Hz) + a fresh valid 1 m/s candidate (republished at
     20 Hz) → `ActuatorFeedback.valid=true`, `motor_speed_radps > 0` (throttle actually flowed),
     `active_faults=[]`.
   - Same, but **no** `safety_armed` → the VCU reports `ARMED` (ARM is relayed directly), yet over 1 s of
     fresh 5 m/s candidates the measured speed stays 0. The gate held.
5. **Clean-workspace portability** (final code, `portability.sh` under `env -i`, only `/opt/ros/humble`;
   `AMENT_PREFIX_PATH=/opt/ros/humble` printed before the build):
   - `./export_node.sh --with-algorithm` → `src/{autokart_msgs,autokart_vcu_bridge}` plus
     `third_party/autokart_vehicle_bridge`. Verified the install tree has only `autokart_msgs` and
     `autokart_vcu_bridge`, so no second VCU bridge is built.
   - `colcon build --packages-up-to autokart_vcu_bridge` → `Summary: 2 packages finished`.
   - pytest in the bundle **without** the algorithm env var → `17 passed, 4 skipped` (the 4 skips are the
     real-algorithm tests, as designed). **With** `AUTOKART_VEHICLE_BRIDGE_PATH=<bundle>/third_party/...`
     → `22 passed`.
   - Live, portable profile (`ros2 launch autokart_vcu_bridge vcu_bridge.launch.py`, no algorithm path),
     from an earlier bundle of the same node: `/vcu_bridge/status` showed `quality: INVALID`,
     `fault_state: CORE_LOGIC_UNAVAILABLE:CORE_LOGIC_PATH_NOT_CONFIGURED`; `/vcu/state` showed
     `state: SAFE`, `motor_enable: false`. This is the documented safe fallback.
   - Live, with the bundled algorithm (final code): `ros2 topic list` showed all 6 topics.
     `/vcu_bridge/status`: `quality: OK`, `fault_state: NONE`. `/vcu/state`: `state: SAFE`,
     `estop_input_fitted: false`, `limiting_reason: NONE`. `/vehicle/actuator_feedback`:
     `road_wheel_angle_rad: -0.0027`, `motor_speed_radps: 0.0`, `valid: true`.
   - SIGINT to the process group: empty log, no traceback. Afterwards `pgrep` found no `vcu_bridge_node`.
     An earlier manual run had left a `ros2 run` child process alive; it was found with `pgrep` and
     killed.

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds standalone (ament_python) | VALIDATED | in-repo and clean `env -i` workspace |
| Reuses `protocol.py` (framing/CRC/codec) without reimplementation | VALIDATED | real-import unit tests; no CRC/framing code in package |
| Reuses `core.py` (`ThrottleController`, `CommandGate`, `raw_to_rad`, `brake_pct_for_candidate`) | VALIDATED | integration tests run with the real module |
| Never passes throttle without `safety_armed` | VALIDATED | integration test with the VCU ARMED and fresh candidates |
| Steering never actuated (`steer_cdeg=0`) | VALIDATED | code review (`send_command` signature) |
| Safe fallback + explicit report when `core.py` is missing | VALIDATED | fault-injection test plus live portable run |
| Fault reporting: read failure, unexpected exception, missing dependency (repeated, not one-shot) | VALIDATED | fault-injection tests |
| Serial port missing → `SERIAL_PORT_UNAVAILABLE` (real pyserial) | VALIDATED | unit test against the real pyserial library |
| Best-effort DIS on shutdown | VALIDATED | regression tests, pre-fix failure reproduced |
| Clean SIGINT shutdown | VALIDATED | subprocess test plus live run |
| Portability (export → clean build → test → launch) | VALIDATED | run 5 above |
| No hardcoded paths in source | VALIDATED | grep |
| `colcon test` hook | BLOCKED | setuptools `tests_require` tooling gap (same as `autokart_hp60c`); pytest run directly |
| Live USB-serial I/O with an ESP32-S3 (`backend: serial` read/write, TEL at 50 Hz, reconnects) | NOT RUN | no hardware |
| `ack_seq`/`link_age_ms`/`boot_id` in typed messages | NOT RUN | genuine interface gap (no fields; `boot_id` absent from the protocol); see DEVELOPER_REPORT |
| Two-phase ARMING handshake (`arm_request_approved`) | NOT RUN | deliberately not reused (no wired upstream supervisor) |
| Integration with node 10 `autokart_safety` (`/autokart/safety/status` JSON → `safety_armed`) | NOT RUN | built concurrently; not wired; throttle stays gated off together (fail closed) |
| AS5600 zero/scale, throttle feed-forward calibration | NOT RUN | MEASURE placeholders |
| Single-serial-owner coordination with the production bridge | NOT RUN | operator responsibility; documented |

**Not a vehicle-readiness claim.** `vehicle_authorized`, `calibration_verified` and `braking_verified` are
untouched.
