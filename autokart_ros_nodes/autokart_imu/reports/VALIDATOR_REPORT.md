# Independent validator report — autokart_imu

Performed as a distinct, sequential validation pass: built the package independently, ran its tests
(including deliberately reproducing the reviewer's fault pre-fix to confirm the fault-injection tests
actually catch it), then exercised the export/clean-workspace path end to end.

## Environment

- ROS 2 Humble, `colcon` present. No MPU-6500, no I2C bus, no Raspberry Pi attached to this machine.
  `python3 -m pip show smbus2` → "Package(s) not found" (confirmed before writing any code and again
  before this report). This is a dev-environment-only validation; see "BLOCKED / NOT RUN" below.
- `kart/ros2_ws` was already colcon-built in this repository (used as the `autokart_msgs` source for the
  in-place build).

## Runs performed and their actual output

1. **In-place colcon build** (symlinked scratch workspace, `kart` layout untouched):
   `colcon build --packages-select autokart_msgs autokart_imu` →
   `Summary: 2 packages finished`, no errors (the same harmless `tests_require` setuptools-deprecation
   warning seen in `autokart_hp60c`'s build, not a failure).

2. **Unit tests, no ROS sourced**: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_backends.py -q`
   → `12 passed`.

3. **Full suite with ROS Humble + autokart_msgs sourced**: `python3 -m pytest test/ -q` → `19 passed`
   (12 unit + 2 integration + 5 fault-injection). Repeated 4 times consecutively to check for flakiness
   (the kind the reviewer's fix specifically addresses) → `19 passed` every time, 5.0-5.5s each.

4. **Live launch, synthetic backend**:
   `ros2 launch autokart_imu imu.launch.py config_file:=.../imu_synthetic.yaml` →
   `ros2 topic list` showed `/imu/data_raw` and `/imu/status`. `ros2 topic echo /imu/status --once`
   showed `quality: OK`, `fault_state: NONE`, `sensor_id: mpu6500`, growing cumulative counters.
   `ros2 topic echo /imu/data_raw --once` showed `orientation_covariance[0] == -1.0`, an all-zero
   orientation quaternion, bias-corrected angular velocity near zero (stationary synthetic backend),
   and `linear_acceleration.z ≈ 9.80665` (gravity). No leftover process after `pkill` (checked with
   `pgrep -af`).

5. **Reviewer's fix, reproduced pre-fix and post-fix** (not just taken on trust): reverted the
   `self._stop.wait(0.01)` backoff on the non-monotonic/saturated drop branches AND the fault-injection
   tests' own fake-backend sleeps (to reproduce the original symptom exactly), re-ran
   `test_nonmonotonic_timestamp_is_dropped_and_node_recovers` and
   `test_saturated_sample_is_dropped_never_published_as_data` → both **failed** (node's own log showed
   the fault WAS detected, `WARN ... autokart_imu fault: SATURATED_SAMPLE`, but the subscriber never
   received the corresponding status — confirming a genuine delivery/starvation defect, not a test
   artifact). Restored both fixes → full suite green again, 19/19, repeated runs stable. See
   `SENIOR_REVIEWER_REPORT.md` for the defect description.

6. **Fault injection, actually executed** (current, fixed code):
   - Persistent `I2C_READ_FAILED` backend → `/imu/status` observed `quality=INVALID`,
     `fault_state=I2C_READ_FAILED`, `payload_present=false`. PASS.
   - Backend raising a bare `RuntimeError` → node kept running, status showed
     `UNEXPECTED_BACKEND_ERROR:RuntimeError:...`, confirmed still publishing 0.2s later. PASS.
   - Non-monotonic timestamp injected on sample 2 of a 4-sample run → observed
     `fault_state=NON_MONOTONIC_TIMESTAMP`, then recovered to `quality=OK`. PASS.
   - Saturated sample injected on samples 1-2 → observed `fault_state=SATURATED_SAMPLE` with
     `quality=INVALID`/`payload_present=false` on every such status, then recovered to `OK`. PASS.
   - `backend=mpu6500` with no `smbus2` installed (true in this environment, not simulated) → node
     fails closed at construction (`node.backend is None`,
     `"DEPENDENCY_UNAVAILABLE" in node.startup_error`), and `/imu/status` repeats
     `quality=INVALID`/`DEPENDENCY_UNAVAILABLE:...` (≥2 messages observed within 5s, not a one-shot).
     PASS.
   - `smbus2` module faked via `monkeypatch` to simulate "library present, no physical bus" →
     `SENSOR_NOT_FOUND:...` raised from `start()`. PASS.
   - `smbus2` faked further to simulate "bus opens, but WHO_AM_I doesn't match" →
     `SENSOR_NOT_FOUND:unexpected_who_am_i=...` raised. PASS.
   - `smbus2` faked with a well-behaved fake bus returning a known raw gyro-Z register value (655,
     FS=500dps) → real conversion math exercised end to end, result `10.0 deg/s` exactly as expected.
     PASS — this is the actual register/conversion code under real test, against a fake transport
     (no real MPU-6500 exists in this environment to test against).

7. **Clean-workspace portability acceptance test**, performed after the reviewer's fix:
   - Ran `export_node.sh` → produced a tarball containing only `autokart_imu/` and `autokart_msgs/`.
   - Extracted into a brand-new scratch directory, with the repository's own `kart/ros2_ws` install
     **not sourced** (only `/opt/ros/humble/setup.bash`).
   - `colcon build --packages-up-to autokart_imu` → succeeded, 2 packages finished, no errors.
   - `python3 -m pytest test/ -q` from inside the extracted bundle, workspace's own
     `install/setup.bash` sourced → `19 passed`.
   - `ros2 launch autokart_imu imu.launch.py config_file:=.../imu_synthetic.yaml` → process started;
     `ros2 topic list` showed both expected topics; `ros2 topic echo` on `/imu/status` and
     `/imu/data_raw` returned well-formed messages (`quality: OK`, `orientation_covariance[0] == -1.0`,
     gravity on Z). No leftover `ros2`/`imu_node` process after `pkill` (checked with `pgrep -af`).
   This demonstrates the package satisfies "copy into another workspace, build, launch, verify outputs"
   independent of the specific machine this was developed on, with no undisclosed dependency on the
   original repository's paths (the `smbus2` dependency, when needed, is documented, not vendored).

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package | VALIDATED | colcon build, twice, independent workspaces |
| Publishes `/imu/data_raw`, `/imu/status` | VALIDATED | `ros2 topic list` + `ros2 topic echo`, twice (in-place and clean workspace) |
| Synthetic backend (no hardware) | VALIDATED | integration tests + live launch |
| Orientation sentinel (`orientation_covariance[0]=-1`, no fabricated heading) | VALIDATED | integration test + live `ros2 topic echo` |
| Stationary bias correction, no data published before calibration completes | VALIDATED | dedicated integration test + code read |
| Fault reporting: read failure / unexpected exception | VALIDATED | fault-injection tests, live status observed |
| Fault reporting: non-monotonic timestamp / saturated sample rejected, not published | VALIDATED | fault-injection tests, regression confirmed against pre-fix code |
| Fault reporting: misconfigured/missing dependency keeps reporting (not one-shot) | VALIDATED | fault-injection test |
| MPU-6500 register/conversion math (against a fake transport) | VALIDATED | unit test with a known raw value and exact expected physical result |
| Portability: clean workspace, export bundle, build, launch | VALIDATED | full run recorded above |
| No hardcoded project-home paths in source | VALIDATED | grep check, defaults are empty/documented placeholders |
| `colcon test` standard pytest hook | BLOCKED | same setuptools/pytest-runner incompatibility already recorded for `autokart_hp60c` in this environment — worked around by running pytest directly |
| Live MPU-6500 I2C read path (`backend: mpu6500`, actual hardware) | NOT RUN | no sensor, no I2C bus, no Raspberry Pi, `smbus2` itself not installed. Only the backend's *failure* paths and its conversion math (via a fake transport) are tested |
| Achievable sample rate on real I2C hardware (~200 Hz architecture target) | NOT RUN | requires physical rig |
| Static `base_link -> imu_link` TF | NOT RUN | mount geometry unmeasured; explicitly out of scope per the task |
| Stationary-bias-window validity on a real, possibly-not-perfectly-still vehicle | NOT RUN | requires physical rig; this node cannot self-verify its own stillness assumption |

## Re-validation after the post-resume fixes (final code)

The `main()` SIGINT guard, `_safe_publish()` in the capture thread, the 20th test (real-process SIGINT),
the cross-topic flake fix and the spin-thread-join teardown fix all changed code after the original
portability run, so everything below was re-run on the final code.

- **Pre-fix reproduction:** with `_safe_publish` reverted, `test_sigint_shutdown_exits_cleanly_without_rclerror`
  → `1 failed` (`RCLError: Failed to publish: publisher's context is invalid`); restored → passes.
- **Environment:** `kart/ros2_ws/install` is stale relative to its own `autokart_msgs` source (discovered
  while building `autokart_vcu_bridge`; it does not affect the message types this node uses, but for
  consistency all final runs used a fresh `autokart_msgs` built into `/tmp/ak_nodes_scratch/msgs_ws`).
- **In repo:** `python3 -m pytest test/ -q` → `20 passed`. **Soak:** 10 consecutive runs → 10 × `20 passed`,
  0 failures. One run printed a cosmetic `cannot use Destroyable` exit message, with no assertion failure.
- **Clean workspace** (`portability.sh` under `env -i`, `AMENT_PREFIX_PATH=/opt/ros/humble` printed before
  the build): `export_node.sh` → `src/{autokart_imu,autokart_msgs}`; `colcon build` →
  `Summary: 2 packages finished`; pytest in the bundle → `20 passed`;
  `ros2 launch autokart_imu imu.launch.py` → topics `/imu/data_raw`, `/imu/status`.
  `/imu/status`: `quality: OK`, `fault_state: NONE`. `/imu/data_raw`: `orientation_covariance[0] = -1.0`,
  all-zero quaternion, `linear_acceleration.z = 9.80665`. SIGINT → launch log
  `process has finished cleanly`, no traceback. `pgrep` afterwards found no `imu_node`.

Updated matrix rows: **Clean SIGINT shutdown: VALIDATED** (subprocess test plus live run). **Test count: 20**
(12 unit + 2 integration + 6 fault injection). Every NOT RUN item above is unchanged.

## Explicit statement

**Software-only acceptance above does not authorize or imply vehicle motion, physical sensor deployment
readiness, or calibration correctness.** No flag in this change sets `vehicle_authorized`,
`calibration_verified`, or any other safety gate to true. This node's gyro/accel output is not fused
into any heading or pose estimate by this node or by `autokart_odometry` in this same batch — see both
nodes' `CONTRACT.md`.
