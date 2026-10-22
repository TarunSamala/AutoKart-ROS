# Senior reviewer report — autokart_imu

Performed as a distinct, adversarial second pass over the developer's actual source (same discipline
as `autokart_hp60c`'s reviewer pass: no multi-agent runtime is wired up in this repository, so this is
a genuine sequential re-read, not a simulated second agent).

## Defects found and their resolution

### 1. Missing backoff in the drop branches

**The `NON_MONOTONIC_TIMESTAMP` and `SATURATED_SAMPLE` drop branches in `_capture_loop` had no backoff
wait**, unlike every other fault path in the same loop (the `ImuBackendError`/unexpected-exception
branches both call `self._stop.wait(...)`). A backend that persistently emits bad timestamps or
saturated samples — plausible for a real sensor glitching, and exactly what the fault-injection tests
for this node do on purpose — turns this into a hot, un-yielding Python loop. Because the GIL is shared
with the `MultiThreadedExecutor` thread that dispatches this same node's own publisher calls and the
test process's subscription callbacks, the hot loop can starve that thread badly enough that a
status message published during the fault window is never actually delivered to a subscriber before
the loop races on to a different state — this node looking alive and "fine" from the outside while
actually losing status transitions.

**Reproduced before confirming the fix**, the same way the hp60c validator did: the backoff calls were
reverted, and `test/test_fault_injection.py`'s own fake-backend sleeps were also reverted (to reproduce
the exact original symptom, not a weaker version of it) — both the `NON_MONOTONIC_TIMESTAMP` and
`SATURATED_SAMPLE` tests then failed reliably, with the node's own log confirming the fault *was*
detected (`WARN ... autokart_imu fault: SATURATED_SAMPLE`) even though the subscriber never saw the
corresponding status message — i.e. this is a genuine delivery/starvation defect, not a test artifact.
Restoring the `self._stop.wait(0.01)` backoff on both branches (and keeping the fake backends' small
sleeps, which better model a real sensor's sample period) made the full suite pass again, 19/19,
stably across four consecutive runs.

**Fixed**: both branches now call `self._stop.wait(0.01)` before `continue`, matching the backoff
pattern used everywhere else in this loop.

## Further defects found after the first pass (live validation and the vcu_bridge review)

2. **Ctrl-C produced an `RCLError` traceback in `main()`.** Humble's SIGINT handler shuts the context down,
   `spin()` raises `ExternalShutdownException` (not caught), and `rclpy.shutdown()` was then called
   unguarded. This was inherited from `autokart_hp60c`'s `main()`, which still has it; it is outside this
   batch's scope and was flagged, not touched. **Fixed:** catch `ExternalShutdownException`, and call
   `rclpy.shutdown()` only `if rclpy.ok()`.

3. **The capture thread died with an uncaught `RCLError` on SIGINT.** The new subprocess regression test
   `test_sigint_shutdown_exits_cleanly_without_rclerror` exposed this deeper problem once the `main()` fix
   was in. The context is invalidated by the signal handler **before** `destroy_node()` sets
   `self._stop`, so the thread's next `publish()` raised `publisher's context is invalid` and the thread
   exited with a traceback. A publish failure for any other reason would have killed the thread silently in
   the same way. **Fixed:** `_safe_publish()` wraps every capture-thread publish. If the context is gone it
   stops the loop quietly; otherwise it logs the error and keeps running. **Reproduced pre-fix:** with
   `_safe_publish` reverted (and the `main()` fix kept), the SIGINT test failed with the `RCLError` above;
   restored, it passes. Live SIGINT from the clean export bundle now logs
   `process has finished cleanly`.

4. **A flaky integration test, which was a test defect rather than a node defect.**
   `test_imu_node_publishes_data_and_status_with_orientation_sentinel` failed about 1 in 15 runs with
   `'CALIBRATING' == 'OK'`. It asserted the status the instant the first `Imu` arrived, but `/imu/status` is a
   separate topic whose matching `OK` callback may not have run yet. **Fixed:** wait for the condition.
   Afterwards: 10/10 consecutive suite runs passed.

5. **Test teardown raced the spin thread (all suites in this batch).** Tests did not join the spin thread
   after `executor.shutdown()`, causing sporadic `cannot use Destroyable` messages at exit. **Fixed:** join
   before destroying. One cosmetic occurrence was still seen in 10 IMU soak runs, with no assertion
   failure. It is recorded rather than hidden.

## Checked and found correct

- **No fabricated orientation**: `orientation_covariance[0] = -1.0` is set unconditionally on every
  published `Imu` message, and the quaternion fields are left at an explicit all-zero (non-unit,
  deliberately invalid) placeholder rather than a plausible-looking identity rotation that a careless
  consumer might mistake for "measured zero heading". Verified by reading `sample_to_imu_msg` and by
  the live `ros2 topic echo` run in `VALIDATOR_REPORT.md`.
- **No fabricated data during calibration**: `/imu/data_raw` is never published while `self.bias is
  None` (the calibration-window branch always `continue`s before reaching the publish call) — verified
  both by reading the code path and by `test_imu_node_reports_calibrating_before_first_data`, which
  asserts `imu_count == 0` while status reports `CALIBRATING`.
- **Dropped samples never reach `/imu/data_raw`**: both the non-monotonic-timestamp and
  saturated-sample branches `continue` before the publish call; `cumulative_dropped` is incremented
  and `cumulative_delivered` is not, so the two counters stay honestly divergent whenever rejections
  occur (checked against `test_saturated_sample_is_dropped_never_published_as_data`'s explicit
  assertion that every `SATURATED_SAMPLE` status also carries `payload_present=False`).
- **MPU-6500 register math**: `ACCEL_XOUT_H_REG = 0x3B` and the 14-byte contiguous read
  (accel×3, temp, gyro×3) matches the public MPU-60x0/9250-family register map, which the MPU-6500
  shares; `WHO_AM_I_EXPECTED = 0x70` matches the MPU-6500's published WHOAMI value (distinguishing it
  from MPU-6050's `0x68` and MPU-9250's `0x71`/`0x73` depending on revision) -- but this reviewer notes
  explicitly that WHO_AM_I values can vary across silicon revisions/counterfeits in the field, and this
  has **not** been checked against a physical chip (none is available). This is flagged in
  `DEPENDENCIES.md`, not silently assumed universal.
- **Saturation heuristic is a documented simplification**: `MPU6500I2CBackend.read()` flags
  `saturated=True` only when a raw axis equals exactly `INT16_MAX`/`INT16_MIN` (the ADC's rail values).
  A real sensor under heavy vibration/clipping might report values very close to, but not exactly at,
  those rails due to noise; this exact-rail heuristic is the standard approach for this sensor family
  but has not been validated against a real saturating chip. Documented here rather than silently
  assumed perfect.
- **`SENSOR_NOT_FOUND` covers two distinct failure shapes under one reason label** (bus-open failure
  vs. unexpected `WHO_AM_I`): both surface as `SENSOR_NOT_FOUND:...` with different detail text after
  the colon. This is a deliberate simplification (both mean "no correctly-identified MPU-6500 at this
  configured bus/address"), not a bug, and the detail text still distinguishes them for a human reading
  `/imu/status` or the logs.
- **No hidden/hardcoded path**: grepped the package for `/home/mark1` and absolute paths; none in
  source, only in this report's and `DEPENDENCIES.md`'s prose.
- **Bias correction never applied to acceleration**: confirmed `sample_to_imu_msg` only subtracts
  `bias` from `gyro_radps`, never from `accel_mps2` — correct per `CONTRACT.md`'s explicit statement
  that a naive startup-average accel correction would silently remove real gravity data.
- **Test independence**: `test/test_backends.py`'s `smbus2` fault-injection tests use `monkeypatch` to
  inject a fake module into `sys.modules` and restore it automatically at teardown (pytest's own
  `monkeypatch` fixture) — confirmed no cross-test leakage by running the full suite repeatedly (see
  `VALIDATOR_REPORT.md`).

## Not independently re-verified by this reviewer

- The actual achievable I2C read rate / timing budget on a real Raspberry Pi against the ~200 Hz
  architecture aspiration — no hardware to measure this against.
- Whether the stationary-bias-calibration window is long enough (100 samples) to average out realistic
  MEMS gyro noise on a genuine MPU-6500 — this is a reasonable default, not a value derived from a real
  noise characterization (none exists in this environment).
