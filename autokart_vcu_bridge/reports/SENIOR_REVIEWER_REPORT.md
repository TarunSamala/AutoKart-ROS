# Senior reviewer report — autokart_vcu_bridge

A separate, adversarial re-read of `node.py`/`backends.py`, done line by line against the reference
`kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/node.py`. No multi-agent runtime
exists here; this was a genuine sequential pass.

## Defects found and fixed

1. **Dead, misleading conditional in `SyntheticBackend.poll()`.** The code read
   `flags = 16 if armed or True else 0`. The `or True` made it always 16, so it looked conditional on ARM
   state but never was. This is the same class of defect as `autokart_hp60c`'s reviewer finding #1. No
   behaviour change resulted. **Fixed:** replaced with the unconditional `flags = 16 | 2 | 4`, with a
   comment.

2. **No disarm on shutdown.** The reference node's `main()` sends a best-effort `DIS` before closing the
   port. This node only closed the port. The firmware's 300 ms LINK_TIMEOUT is a backstop, but dropping
   an explicit disarm on a VCU-facing node is a real regression from the reference. **Fixed:**
   `destroy_node()` now sends `DIS` (best effort; a failure is logged and the port is still closed).
   **Regression tests:** `test_shutdown_sends_best_effort_disarm` and
   `test_shutdown_disarm_failure_does_not_raise_and_still_closes`. **Reproduced pre-fix:** with the DIS
   line removed, `test_shutdown_sends_best_effort_disarm` failed; with it restored, it passes.

3. **Ctrl-C ended in an `RCLError` traceback.** This was found during the live clean-workspace launch,
   not by reading code. Humble's SIGINT handler shuts the context down, `spin()` raises
   `ExternalShutdownException` (which was not caught), and `main()` then called `rclpy.shutdown()`
   unguarded: `RCLError: rcl_shutdown already called`. The reference node already guards this. **Fixed:**
   `main()` now catches `ExternalShutdownException` and calls `rclpy.shutdown()` only `if rclpy.ok()`.
   **Regression test:** `test_sigint_shutdown_exits_cleanly_without_rclerror` runs the real entry point as
   a subprocess, sends SIGINT, and asserts exit 0 with no traceback. **Reproduced pre-fix:** with the
   guard reverted, that test failed; restored, it passes. The same bug was present in `autokart_imu`
   (fixed there too) and is still present in `autokart_hp60c` (not in this batch's scope, so not touched;
   flagged for its owner).

## Checked and found correct

- **Signature order of every reused `core.py` call** was checked against the source:
  `CommandGate.compute(now, safety_armed, vcu_armed, throttle_v, brake_pct, cmd_stamp)`,
  `ThrottleController.step(now, target, measured)`, `brake_pct_for_candidate(valid, target)`, and
  `raw_to_rad(raw, zero, scale)`. All match. Swapping `safety_armed` and `vcu_armed` would have been
  silent, so this was checked explicitly.
- **Gate fail-closed by default.** With nothing publishing `/vcu_bridge/safety_armed`, the gate returns
  `"safety not armed"` every tick. `test_vcu_bridge_without_safety_armed_never_sends_nonzero_throttle`
  proves this end to end with the real `core.py`: the VCU reports ARMED and fresh 5 m/s candidates are
  arriving, yet measured speed stays 0.
- **Steering is never actuated.** `send_command` always passes `steer_cdeg=0` to `protocol.build_cmd`, as
  the wire protocol requires.
- **No fabricated telemetry.** Stale telemetry gives `VcuState.state="UNKNOWN"` and
  `ActuatorFeedback.valid=false`. When `core` is missing, `road_wheel_angle_rad` is not computed by a
  local reimplementation of `raw_to_rad`; `valid=false` is reported instead.
- **Freshness clocks.** Candidate, safety and telemetry freshness all use `time.monotonic()`, consistent
  with `CommandGate`'s own use. Only `MotionCommand.expiry` uses the ROS clock, because it is a ROS
  timestamp.
- **No hardcoded paths** in package source; `/home/mark1` appears only in YAML comments and report prose.

## Accepted, documented (not defects)

- Callback state is not locked. Production `main()` uses a single-threaded executor, matching the
  reference node. The tests use `MultiThreadedExecutor` only to combine two nodes.
- An intermittent `cannot use Destroyable because destruction was requested` message appears at test
  interpreter exit in about 2 of 20 full runs. It never causes an assertion failure. It arises during
  multi-node `MultiThreadedExecutor` teardown in tests; production `main()` destroys the node only after
  single-threaded `spin()` returns.
- `SerialVcuBackend` retries forever on I/O errors rather than abandoning the port (DEVELOPER_REPORT).

## Not independently verified

- Real USB-CDC behaviour of the ESP32-S3, including partial frames, reconnects and timing at 20 Hz
  against 50 Hz TEL. No hardware is present.
- Correctness of the AS5600 zero/scale defaults. These are MEASURE placeholders carried over from the
  reference config.
