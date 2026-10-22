# Developer report — autokart_imu

## Audit performed before writing any code

- Grepped the entire repository (not just `kart/ros2_ws`) for `mpu6500`, `mpu-6500`, and `imu` as a
  driver/topic concept. Found **no existing MPU-6500 driver, no IMU ROS node, no IMU topic** anywhere.
  `docs/decisions/CANONICAL_COMPONENTS.md` records "MPU-6500 identity, connection, driver and
  calibration: UNKNOWN". `kart/ros2_ws/src/autokart_localization/autokart_localization/odometry.py`
  states in its own module docstring: "There is no IMU in this configuration ... so yaw is a
  model-based estimate, never a direct measurement."
- Confirmed `autokart_msgs/SensorDeliveryMetadata.msg` fits this node's status contract without a new
  message type (same message `autokart_hp60c` already uses for `/hp60c/status`).
- Confirmed this dev environment has no I2C hardware and no `smbus2` installed
  (`python3 -m pip show smbus2` → not found), matching "no Raspberry Pi, no camera, no IMU, no
  ESP32-S3/VCU, no encoders" in the mission statement.

## Decision: genuinely new code, not a wrapper

Unlike the other two nodes in this batch, there is nothing to reuse here. `backends.py` is structured
exactly like the proven `autokart_hp60c`/`autokart_vcu_bridge` pattern (a dependency-free `SyntheticBackend`
plus a real-hardware adapter that lazily imports its one external dependency and fails closed), but the
MPU-6500 register map and conversion math in `MPU6500I2CBackend` are drawn from the public MPU-6500
datasheet (WHO_AM_I register, FS sensitivity tables, two's-complement 16-bit samples), not copied from
any existing file in this repository, because no such file exists.

## What was implemented

- `autokart_imu/backends.py`: `ImuBackend` protocol, `RawSample` dataclass (physical units, NOT
  bias-corrected — bias correction is node-level policy, not a hardware concern), `SyntheticBackend`
  (deterministic, stationary by default, with test-only hooks for rotation/saturation/non-monotonic
  timestamps), `MPU6500I2CBackend` (structurally correct, lazy `smbus2` import, fails closed with
  `DEPENDENCY_UNAVAILABLE`/`SENSOR_NOT_FOUND`/`SENSOR_INIT_FAILED`/`I2C_READ_FAILED`).
- `autokart_imu/node.py`: `ImuNode`. One background capture thread (same "exactly one caller of the
  backend" pattern as `autokart_hp60c`). Runs a startup stationary-bias-calibration window
  (`calibration_samples`, default 100) before publishing any `/imu/data_raw`; rejects non-monotonic
  timestamps and saturated raw samples (dropped, reported, never published as valid); sets
  `orientation_covariance[0] = -1.0` and leaves the quaternion as an explicit all-zero placeholder,
  per the master architecture doc's instruction not to fabricate a heading. Publishes
  `autokart_msgs/SensorDeliveryMetadata` on `/imu/status` with the same cumulative-counter convention
  as `autokart_hp60c`.
- Full ament_python package (`package.xml`, `setup.py`, `setup.cfg`, `resource/`), launch file, two
  config profiles (`synthetic`, `live` — the latter with hardware parameters left at documented,
  unverified defaults, not fabricated calibration).
- 20 tests: 12 unit (backend logic incl. real MPU-6500 register/conversion math exercised against a
  fake-but-well-behaved `smbus2` module, no ROS needed), 2 ROS integration (real rclpy node + real
  subscriber, checks message contents and the no-data-during-calibration invariant), 6 fault-injection
  (persistent read failure, unexpected exception, non-monotonic timestamp, saturated sample, missing
  dependency, real-process SIGINT shutdown -- the last added after two more defects were found, see
  SENIOR_REVIEWER_REPORT.md items 2-3).
- `export_node.sh`, `CONTRACT.md`, `DEPENDENCIES.md`, `README.md` (this directory).

## A genuine defect found and fixed during the node's own fault-injection tests (not a separate review pass)

While writing `test_fault_injection.py`, the `NON_MONOTONIC_TIMESTAMP` and `SATURATED_SAMPLE` drop
branches in `_capture_loop` turned out to have **no backoff wait**, unlike every other fault path in
the loop (`ImuBackendError`/unexpected-exception paths call `self._stop.wait(...)`). With a
fast/un-throttled backend (as in the fault-injection tests, and potentially a real sensor emitting a
burst of bad samples), this becomes a hot, un-yielding Python loop that can starve the ROS executor
thread responsible for dispatching this very node's own publishers/subscriptions — observed directly
as test flakiness (a status message sent during the fault window was never delivered to the test's
subscriber before the loop raced ahead to a later state). Fixed by adding the same `self._stop.wait(0.01)`
backoff used elsewhere. This is recorded here rather than silently rewritten, since it was caught by
the tests doing their job, not a separate reviewer pass — the senior-reviewer pass below additionally
checked for other instances of the same class of bug.

## Known gaps (explicit, not hidden)

- `backend: mpu6500`'s actual register-read path against real hardware is **not exercised** — no I2C
  bus, no sensor, no Raspberry Pi in this environment, and `smbus2` itself is not installed. Only its
  failure paths and its conversion math (via a fake transport) are tested. See `VALIDATOR_REPORT.md`.
- No static `base_link -> imu_link` TF publication — mount geometry is unmeasured. Out of scope per the
  mission statement; not guessed.
- `angular_velocity_covariance`/`linear_acceleration_covariance` are placeholder diagonal values, not
  independently characterized against a real sensor (documented in `CONTRACT.md`, not silently
  presented as a measured noise spec).
- The stationary bias-calibration window assumes the sensor is actually stationary at startup; this
  node has no independent way to verify that assumption (documented in `CONTRACT.md`).
- Rate target (~200 Hz) is the architecture's aspiration; the synthetic backend's default period
  (5 ms) is a documented target, not a measured achievable I2C throughput.
