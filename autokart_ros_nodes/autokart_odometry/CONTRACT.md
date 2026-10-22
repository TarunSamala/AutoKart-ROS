# autokart_odometry — interface contract

## Role

Wheel-encoder + **observed**-steering odometry. It wraps the existing, already-tested
`autokart_localization.odometry.WheelOdometry` (bicycle model, from `kart/ros2_ws/src/autokart_localization`).
The class is imported lazily from `algorithm_package_path` and never reimplemented. The node publishes
`/odom` and the `odom -> base_link` TF.

## NO IMU FUSION

Yaw is a bicycle-model **estimate** (`yaw_rate = v / wheelbase * tan(steering)`), never a measurement.
This node does not subscribe to `autokart_imu`'s `/imu/data_raw` and does not claim IMU-fused yaw. That
matches the reused module's own docstring ("There is no IMU in this configuration … yaw is a model-based
estimate, never a direct measurement"). Fusing gyro with this estimate is future work for a dedicated
estimator node.

## Topics and TF

| Output | Type | QoS | Notes |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | reliable, depth 10 | `header.frame_id = odom_frame_id` (default `odom`), `child_frame_id = base_frame_id` (default `base_link`). Published **only** for samples the reused algorithm accepted. |
| TF `odom -> base_link` | `tf2_msgs/TFMessage` via `tf2_ros.TransformBroadcaster` | — | Same pose and stamp as `/odom`; disable with `publish_tf: false`. |
| `/odom/status` | `autokart_msgs/SensorDeliveryMetadata` | reliable, depth 10 | Reused message type, same pattern as the other nodes in this library. |

Stamps on `/odom` and TF come from the node's ROS clock at publish time. Input sample stamps are used
**only** to compute `dt` between samples, because node 04 stamps `ActuatorFeedback.measurement_time` from
a monotonic clock rather than ROS time.

| Input (`source`) | Type | Notes |
|---|---|---|
| `synthetic` (default) | internal generator | Deterministic; for portability and tests. Never claims LIVE. |
| `actuator_feedback` | `autokart_msgs/ActuatorFeedback` on `feedback_topic` (default `/vehicle/actuator_feedback`, node 04's output) | `motor_speed_radps` carries rear-axle **linear speed in m/s** (a documented field reuse by the VCU bridge); `road_wheel_angle_rad` is the observed steering angle; `valid` is required. |

## Single TF writer — operator hazard

This node is the only publisher of `odom -> base_link` **in this package**. Audit of `kart/ros2_ws/src`
(grep for `TransformBroadcaster`/`sendTransform`/static publishers) found:

- `autokart_localization` (production wheel odometry) publishes `nav_msgs/Odometry` on `/autokart/odom`
  and `/vehicle/odometry` but does **not** broadcast TF. Running both is not a TF conflict, but it does
  give two independent odometry estimates of the same vehicle on different topics, which consumers must
  not mix.
- `autokart_gazebo`'s follower plugin (`src/gazebo_follower_plugin.cpp`) **does** broadcast
  `odom -> base_link`. Running this node with `publish_tf: true` alongside the Gazebo simulation is a
  **duplicate-TF-writer hazard**.

The master architecture's single-writer rule forbids two writers of one transform. The operator must avoid
it, either by running only one writer or by setting `publish_tf: false` here. This node cannot detect or
resolve that system-level conflict.

## Status mapping (lossless)

| `/odom/status.quality` | `WheelOdometry.status` | `/odom` published? |
|---|---|---|
| `OK` (`fault_state=NONE`) | `ENCODER_STEER_MODEL` | yes |
| `DEGRADED` (`fault_state=STEERING_UNAVAILABLE_YAW_FROZEN`) | `ENCODER_ONLY_NO_YAW` | yes; yaw frozen, translation continues along the frozen heading |
| `INVALID` (`fault_state=<reason>`) | `INVALID` | **no** odometry, **no** TF |

`calibration_version` is always `"UNVERIFIED"`. Wheelbase, `counts_per_m` and encoder sign have not been
physically verified: no lifted-axle or push test was possible in this environment.

INVALID reasons: `ALGORITHM_PATH_NOT_CONFIGURED`, `DEPENDENCY_PATH_NOT_FOUND:…`, `ALGORITHM_IMPORT_FAILED:…`,
`ALGORITHM_API_MISMATCH:…`, `INVALID_CONFIG:…` (including the reused class's own
`wheelbase_m must be positive` check, and any variance that is not finite and > 0),
`COUNTS_MODE_UNSUPPORTED:…`, `UNKNOWN_SOURCE:…`, `INPUT_INVALID`, `AWAITING_SECOND_SAMPLE`,
`NON_MONOTONIC_TIMESTAMP`, `AXLE_COUNTS_UNAVAILABLE`, `TIMING_OR_SPEED_INVALID` (the reused class rejects
`dt <= 0` or `dt > 0.2 s`, and non-finite speed), `INPUT_STALE_OR_ABSENT`.

After any rejection, the next sample only re-anchors. Travel across a rejected interval is never
integrated (the pose stays frozen), so a gap can never be integrated as one large jump.

## Displacement semantics — read before trusting `/odom` while reversing

- `WheelOdometry` integrates **signed** speed into `x_m`/`y_m`. It has **no** separate unsigned path-length
  field (`distance_travelled_total` or similar does not exist in `odometry.py`, and no `autokart_msgs`
  message has such a field). This node therefore publishes no such field. See DEVELOPER_REPORT.md.
- **Speed mode (`counts_per_m = 0`), including all `actuator_feedback` input, has no reliable direction
  signal.** The VCU's `TEL.speed_mmps` is treated as unsigned (`< 0` means "unavailable" in both the
  firmware protocol and the reused `core.py`), and `WheelOdometry.update()` does not apply
  `encoder_forward_sign` (only `update_counts()` does). Reversing the kart in speed mode is therefore
  either integrated as forward motion or rejected; it is never correctly signed. Counts mode handles
  sign via `encoder_forward_sign` but needs raw axle counts, which `ActuatorFeedback` does not carry.
  That combination is refused (`COUNTS_MODE_UNSUPPORTED`), never silently downgraded. Push tests in both
  directions are **NOT RUN**.
- Because node 04 sets `ActuatorFeedback.valid = false` whenever steering is unreadable, the reused
  algorithm's `ENCODER_ONLY_NO_YAW` degraded mode is reachable via the `synthetic` source but **not** via
  `actuator_feedback` (there, steering loss means the whole sample is INVALID). This is conservative and
  inherited from the single `valid` flag in `ActuatorFeedback`.

## Covariance

`position_variance_m2`, `yaw_variance_rad2`, `speed_variance_m2ps2` and `yaw_rate_variance_rad2ps2` are
**placeholders, not measured**. They must be finite and > 0, otherwise the node fails closed. Unobserved
planar axes (z, roll, pitch; lateral/vertical twist) get `1e6`, so they never claim certainty.

## Parameters with no safe default

`wheelbase_m` defaults to `0.0`, so the node fails closed until a measured value is supplied; no nominal
constant is substituted. The synthetic profile sets `wheelbase_m: 1.0` and `counts_per_m: 100.0`,
explicitly labelled as synthetic test values.
