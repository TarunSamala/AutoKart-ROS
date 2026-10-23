# autokart_imu

MPU-6500 IMU driver. Publishes `sensor_msgs/Imu` (bias-corrected angular rate, raw linear acceleration,
orientation deliberately marked unavailable) and a delivery-status message. See `CONTRACT.md` for the
exact interface and `DEPENDENCIES.md` for what this package needs from elsewhere.

There is no existing MPU-6500 driver anywhere in this repository — this node is new code, not a wrapper
around something already working (unlike `autokart_hp60c`, `autokart_vcu_bridge`, `autokart_odometry` in
this same library). See `reports/DEVELOPER_REPORT.md` for the audit that established this.

## Quick start (no hardware required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select autokart_imu
source install/setup.bash
ros2 launch autokart_imu imu.launch.py config_file:=$(ros2 pkg prefix autokart_imu)/share/autokart_imu/config/imu_synthetic.yaml
# in another shell:
ros2 topic echo /imu/status
ros2 topic echo /imu/data_raw
```

This runs the built-in `SyntheticBackend` — deterministic fake gyro/accel samples, `quality=CALIBRATING`
for the first `calibration_samples` readings then `quality=OK`. It exists so this package is buildable,
testable and runnable with zero hardware dependency.

## Running against real MPU-6500 hardware

See `DEPENDENCIES.md`. Set `backend: mpu6500` and the I2C parameters in `config/imu_live.yaml`, then:

```bash
ros2 launch autokart_imu imu.launch.py config_file:=/path/to/imu_live.yaml
```

Sensor acquisition errors surface on `/imu/status` (`quality=INVALID`, `fault_state=<reason>`); the node
keeps retrying and never fabricates a sample. **This path is NOT RUN in this session — no I2C hardware,
no Raspberry Pi, no MPU-6500, and `smbus2` itself is not installed in this environment.** Only its
fail-closed behavior is tested (see `reports/VALIDATOR_REPORT.md`).

## Tests

```bash
source /opt/ros/humble/setup.bash
source <this_or_any_workspace>/install/setup.bash   # needs autokart_msgs built
cd autokart_ros_nodes/autokart_imu
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_backends.py` — unit tests, no ROS runtime needed (12 tests, including the MPU-6500 register
  math and fail-closed paths exercised against a fake-but-well-behaved `smbus2` module).
- `test/test_node_integration.py` — real rclpy node + real subscriber, checks actual message contents,
  including that no data is published during the calibration window (2 tests).
- `test/test_fault_injection.py` — read failure, unexpected exception, non-monotonic timestamp, saturated
  sample, missing dependency, real-process SIGINT exits cleanly (6 tests).

All 20 pass as of this writing (10/10 consecutive soak runs); see `reports/VALIDATOR_REPORT.md` for the
exact run log and the clean-workspace portability check (`export_node.sh`).

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

packages this node plus the `autokart_msgs` interface package it depends on into a self-contained
tarball you can drop into any ROS 2 Humble workspace's `src/` and `colcon build`.

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md`
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only, synthetic + fault-injection). The `mpu6500` backend's
dependency-failure paths are tested; the live I2C hardware read path is **NOT RUN** (no sensor, no I2C
bus, no `smbus2` in this environment) — see DEPENDENCIES.md and the validator report. No static
`base_link -> imu_link` TF is published (mount geometry unmeasured) — **NOT RUN**, not guessed.
**Software-only acceptance is not a vehicle-readiness claim.**
