# Developer report — autokart_odometry

## Audit performed before writing any code

Read in full: `kart/ros2_ws/src/autokart_localization/autokart_localization/{odometry.py, node.py, __init__.py}`
and `test/test_odometry.py`. Grepped `kart/ros2_ws/src` for every TF broadcaster.

Findings:

1. **The brief's premise about field names does not match the code.** The brief asked to "maintain the
   existing distinction between `distance_travelled_total` (unsigned path length) and signed displacement,
   as the existing `odometry.py` already does — read its code before assuming the field names." Having
   read it: `WheelOdometry` holds only `x_m`, `y_m`, `yaw_rad`, `yaw_rate_radps`, `speed_mps`, `status`
   (plus `cfg`). There is **no** unsigned path-length / `distance_travelled_total` field, and grepping
   `autokart_localization`, `autokart_vehicle_bridge` and `autokart_msgs` finds nothing of that name.
   Following the brief's own instruction to use what is actually there, this node does not add, publish or
   claim such a field. `test_real_algorithm_signed_reverse_motion_reduces_x` pins this down: reversal
   reduces `x_m`, and `hasattr(odo, "distance_travelled_total")` is `False`. Adding an unsigned odometer
   would be new logic plus a new message field, which is a separate, reviewed change.
2. `odometry.py` is pure Python. The package `__init__.py` is empty, and `node.py` imports
   `autokart_mapping` only inside `main()`. So `autokart_localization.odometry` can be imported lazily with
   no ROS dependency, which was verified with `python3 -I`.
3. **TF writers in `kart/ros2_ws`:** the production `autokart_localization` node publishes `Odometry`
   messages but **no TF**. The only existing `odom -> base_link` broadcaster is
   `autokart_gazebo/src/gazebo_follower_plugin.cpp:626`. My first draft of CONTRACT.md wrongly said
   `autokart_localization` was the conflicting writer; this was corrected after the grep.
4. Observation about the existing node, which was not modified: `LocalizationNode.on_feedback` computes
   `dt = max(1e-3, (stamp - last)/1e9)`, so a duplicate or backwards feedback timestamp is integrated as a
   1 ms step instead of being rejected. This node rejects it (`NON_MONOTONIC_TIMESTAMP`) and re-anchors.
5. `WheelOdometry.update()` (speed mode) ignores `encoder_forward_sign`; only `update_counts()` applies it.
   Combined with the VCU's effectively unsigned `speed_mmps`, speed mode cannot represent reversing (see
   CONTRACT.md "Displacement semantics").

## What was implemented

- `backends.py`: `OdomSample`; a deterministic `SyntheticSampleSource` (counts generated consistently with
  speed, plus test hooks for steering loss, speed loss, gap and non-monotonic stamps); and
  `load_wheel_odometry()`, a lazy import from `algorithm_package_path` with explicit failure reasons and an
  API-shape check.
- `node.py`: `OdometryNode`, timer-driven, with two sources (`synthetic` or `actuator_feedback` from node
  04). It runs the reused `update_counts()` (counts mode) or `update()` (speed mode) and publishes `/odom`,
  the `odom -> base_link` TF (`publish_tf`), and `/odom/status`. Status maps losslessly to the reused
  `STATUS_*` values, and `calibration_version` is always `"UNVERIFIED"`. After any rejection the node
  re-anchors, so nothing is integrated across a gap. A stale-input watchdog is included. It fails closed,
  with no `/odom` and no TF, when the algorithm is missing, the wheelbase is unconfigured (the reused
  class's own validation), variances are invalid, or counts mode is paired with `ActuatorFeedback` (which
  has no counts). There is **no IMU fusion**.
- Package skeleton, launch file, two configs (synthetic values explicitly labelled; live config has
  `wheelbase_m: 0.0`, so it refuses to run until a measured value is entered), `export_node.sh
  [--with-algorithm]`, CONTRACT, DEPENDENCIES, README.
- 23 tests (9 unit, 2 rclpy integration including a real TF lookup, 12 fault injection including a
  real-process SIGINT).

## Known gaps (explicit)

- No IMU fusion (by design and by brief).
- No unsigned path-length output (the field does not exist upstream; finding 1).
- Speed mode has no direction information (finding 5). Counts mode needs a raw-counts ROS input that
  neither node 04 here nor `ActuatorFeedback` provides. The production bridge's `/autokart/vcu/axle_counts`
  JSON-over-`String` topic was not adopted, because it is a stringly-typed contract.
- `ENCODER_ONLY_NO_YAW` is unreachable through `actuator_feedback`, because node 04 marks the whole sample
  invalid on steering loss.
- Covariances are placeholders. Wheelbase, `counts_per_m`, encoder sign, and lifted-axle/push tests are all
  NOT RUN.
- Duplicate-TF hazard with the Gazebo follower plugin when both run (CONTRACT.md).
