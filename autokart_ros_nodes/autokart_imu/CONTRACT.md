# autokart_imu — interface contract

## Topics published

| Topic | Type | QoS | Rate |
|---|---|---|---|
| `/imu/data_raw` | `sensor_msgs/Imu` | sensor data (best-effort, volatile) | one per accepted sample (synthetic default ~200 Hz target, see "Rate" below) |
| `/imu/status` | `autokart_msgs/SensorDeliveryMetadata` | reliable, depth 10 | one per sample attempt, including rejected/calibrating samples |

No topics are subscribed. This node has no actuation path and no dependency on any other node in this
library.

## Orientation — read this before consuming `/imu/data_raw`

This node **never publishes a fabricated or integrated absolute heading**. `orientation` is left as an
all-zero (non-unit, deliberately invalid) quaternion and `orientation_covariance[0] = -1.0`, the
`sensor_msgs/Imu` convention meaning "orientation data is unavailable, ignore the orientation fields and
covariance matrix" (see http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html). This matches the
master architecture doc's explicit instruction: "if yaw is not directly observed, set IMU orientation
covariance sentinel -1 ... rather than publish a fictitious absolute heading." Any yaw estimate that
fuses this gyro with other sensors is out of scope for this node — see `autokart_odometry`
(`autokart_ros_nodes/autokart_odometry/`), which does its own model-based yaw and explicitly does NOT
fuse this node's gyro either (see that node's CONTRACT.md "no IMU fusion" section). Fusing the two is
future work for a dedicated estimator node, not either of these two.

## Bias correction

`angular_velocity` is the raw gyro **minus a per-axis bias estimated once at startup**, over a
stationary window (`calibration_samples` parameter, default 100 samples). No `/imu/data_raw` message is
published until that window completes; `/imu/status` reports `quality=CALIBRATING` meanwhile. This
assumes the sensor is physically stationary and level during that window — **this node cannot itself
verify that assumption** (it has no independent stillness signal, e.g. wheel speed); if the vehicle is
moving or being handled during startup, the estimated bias will be wrong for the rest of the run. No
re-calibration is performed after startup.

`linear_acceleration` is NOT bias-corrected (no analogous "zero-g at startup" assumption is made, since
gravity is always present on at least one axis depending on mount orientation — subtracting a naive
startup average would silently remove real gravity component data).

## Fault behavior

On any backend read failure, non-monotonic timestamp, or saturated raw sample, this node:
- does **not** publish `/imu/data_raw` for that sample (no stale/zero-filled/saturated reading is ever
  marked valid),
- publishes `/imu/status` with `quality=INVALID`, `fault_state=<reason>`, `payload_present=false`,
- keeps retrying (bounded backoff) and recovers to `quality=OK` on the next good sample.

Fault reasons: `BACKEND_NOT_STARTED`, `DEPENDENCY_UNAVAILABLE:...` (library missing), `SENSOR_NOT_FOUND:...`
(no device / wrong device at the configured I2C address), `SENSOR_INIT_FAILED:...`, `I2C_READ_FAILED:...`,
`NON_MONOTONIC_TIMESTAMP`, `SATURATED_SAMPLE`, `UNEXPECTED_BACKEND_ERROR:<type>:<message>` (an
unanticipated backend exception, caught rather than allowed to kill the capture thread).

This node never decides to stop the vehicle; it only reports. A future safety/system-monitor node is
the authority that must treat `INVALID`/stale `/imu/status` as a blocking input.

## Parameters

See `config/imu_synthetic.yaml` and `config/imu_live.yaml`. `backend` selects `synthetic` (default, no
external dependency) or `mpu6500` (I2C, requires the optional `smbus2` Python library — see
DEPENDENCIES.md).

## Frame and mount

`frame_id` (param, default `imu_link`). This node does **not** publish a static `base_link -> imu_link`
transform (no measured mount offset/orientation is available in this environment) — **NOT RUN**, not
guessed. A consumer needing that transform must obtain it from a separately measured and published
static TF.

## Rate

The master architecture doc's aspiration for this sensor class is ~200 Hz. The synthetic backend's
default `synthetic_period_s` (0.005 s, i.e. nominally 200 Hz) is a documented target, not a
hardware-verified rate — no physical MPU-6500 has been read in this environment to confirm achievable
throughput over I2C at this sample interval alongside calibration and bias-correction overhead.

## What this node is not

It does not fuse gyro into a heading, does not publish a static TF, does not perform anything beyond a
single-pass stationary bias correction, and does not talk to any other node in this library. It is not
a wrapper around an existing algorithm — there is no pre-existing MPU-6500 driver anywhere in this
repository (confirmed by audit, see `../AUDIT_PHASE0.md` and this node's `reports/DEVELOPER_REPORT.md`),
so this is new code, held to the same honesty standard (fail closed, no fabricated data) as the reused
nodes in this library.
