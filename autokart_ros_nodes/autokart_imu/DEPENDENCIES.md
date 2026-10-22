# External dependencies — autokart_imu

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `sensor_msgs` — ROS 2 Humble core, assumed present on any Humble install.
- `autokart_msgs` — this repository's shared interface package (`kart/ros2_ws/src/autokart_msgs`). Only
  `SensorDeliveryMetadata.msg` is used. To reuse this node in another workspace, copy `autokart_msgs`
  alongside it (interface-only, no further dependency chain beyond `std_msgs`, `builtin_interfaces`,
  `geometry_msgs`).

## Hardware/library dependency (`backend: mpu6500` only)

With `backend: synthetic` (the default), there is **no** dependency beyond the two ROS packages above —
the package builds and all tests pass on a plain Ubuntu 22.04 + ROS 2 Humble machine with nothing else
installed. Confirmed in this session: this dev environment itself has no `smbus2` installed and no I2C
hardware, and the full test suite still passes.

With `backend: mpu6500`, at runtime (not build time, not import time) this node imports the third-party
`smbus2` Python package to talk to the sensor over I2C. `smbus2` is **not installed in this
environment** (`python3 -m pip show smbus2` → "Package(s) not found", confirmed during this session's
audit) and is not vendored, copied, or reimplemented by this node — same "lazy import, no undisclosed
substitution" discipline as `autokart_hp60c`'s `HP60CBackend`. If missing, the backend fails closed with
`DEPENDENCY_UNAVAILABLE:smbus2:...`, not a build failure and not a fabricated reading.

**To reuse this node on a real Raspberry Pi with an MPU-6500 wired up:**
1. `pip3 install smbus2` (or the distro's `python3-smbus2` package, if available) on the target machine.
2. Enable I2C (`raspi-config` or `/boot/config.txt` on a Pi) and confirm the sensor answers at the
   configured address (`i2cdetect -y <bus>`).
3. Set `i2c_bus_num` and `i2c_address` in `config/imu_live.yaml` to the actual values for that machine —
   never hardcoded in source; defaults are a Raspberry Pi convention (`bus 1`, address `0x68`), not a
   verified value for any specific build.
4. Set `backend: mpu6500`.

No path in this package's source code is hardcoded to this repository's location, and nothing in it
assumes a specific machine. If the dependency is missing or the sensor does not answer, the node fails
closed (`DEPENDENCY_UNAVAILABLE` / `SENSOR_NOT_FOUND` in `/imu/status`), not silently.

## Unlike the other two nodes in this batch: nothing is reused from elsewhere in this repository

`autokart_hp60c` reuses `autokart_perception.camera.HP60C`. `autokart_vcu_bridge` reuses
`autokart_vehicle_bridge.{protocol,core}`. `autokart_odometry` reuses `autokart_localization.odometry.WheelOdometry`.
This node has nothing to reuse: there is no MPU-6500 driver, no IMU integration, anywhere else in this
repository (confirmed by a repository-wide grep for `mpu6500`/`mpu-6500`/`imu` during Phase 0 audit; see
`../AUDIT_PHASE0.md` and `reports/DEVELOPER_REPORT.md`). The register map and conversion constants in
`backends.py` (`WHO_AM_I_REG`, sensitivity tables, etc.) are public MPU-6500 datasheet facts, not
proprietary logic copied from this codebase.

## What was NOT verified in this session

No physical MPU-6500, no I2C bus, no Raspberry Pi is present in this development environment. The
`mpu6500` backend's actual register-read path (`MPU6500I2CBackend.start()`/`.read()` against a real
chip) is **implemented but NOT exercised against real hardware**. Its failure paths (missing `smbus2`,
bus-open failure, unexpected `WHO_AM_I`) ARE exercised, using a fake-but-well-behaved `smbus2` module
injected via `monkeypatch` to reproduce exactly the failure shapes a real deployment would hit — see
`test/test_backends.py` and `reports/VALIDATOR_REPORT.md`.
