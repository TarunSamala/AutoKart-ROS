# autokart_odometry

Wheel-encoder + observed-steering odometry. It wraps the existing, already-tested
`autokart_localization.odometry.WheelOdometry` (bicycle model, imported lazily from `algorithm_package_path`,
not reimplemented) and publishes `nav_msgs/Odometry` on `/odom`, the `odom -> base_link` TF, and
`/odom/status`. **No IMU fusion**: yaw is a model estimate, never a measurement. See `CONTRACT.md`
(interface, status mapping, the duplicate-TF-writer hazard, speed-mode direction caveat) and
`DEPENDENCIES.md`.

## Quick start (no hardware required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to autokart_odometry
source install/setup.bash
ros2 run autokart_odometry odometry_node --ros-args \
  --params-file $(ros2 pkg prefix autokart_odometry)/share/autokart_odometry/config/odometry_synthetic.yaml \
  -p algorithm_package_path:=/path/to/kart/ros2_ws/src/autokart_localization
ros2 topic echo /odom/status
ros2 run tf2_ros tf2_echo odom base_link
```

Without `algorithm_package_path`, the node fails closed: `/odom/status` repeats
`ALGORITHM_PATH_NOT_CONFIGURED` and no `/odom` or TF is published.

## Consuming node 04's feedback

`config/odometry_live.yaml` uses `source: actuator_feedback`. Its `wheelbase_m` is deliberately `0.0`, so
the node refuses to run (`INVALID_CONFIG`) until a measured wheelbase is entered. **NOT RUN in this
session**: no VCU, no encoders, no lifted-axle or push test.

## Tests

```bash
source /opt/ros/humble/setup.bash
source <workspace with a CURRENT autokart_msgs build>/install/setup.bash
cd autokart_ros_nodes/autokart_odometry
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_backends.py`: 9 unit tests, no ROS. Synthetic source determinism and hooks; loader fail-closed
  paths (unconfigured, missing, import failure, API mismatch); the real `WheelOdometry` driven by synthetic
  counts (1 m/s × 1 s → x ≈ 1 m); signed reversal.
- `test/test_node_integration.py`: 2 rclpy tests. Synthetic counts mode publishes `/odom`, status `OK` and
  a real `odom -> base_link` TF (looked up via `tf2_ros.Buffer`); the `actuator_feedback` source integrates
  and refuses invalid feedback.
- `test/test_fault_injection.py`: 12 tests. Missing algorithm, unconfigured wheelbase, bad covariance
  (×3), counts mode with feedback refused, stale feedback, steering loss → DEGRADED with frozen yaw, speed
  loss → no odometry, timing gap and non-monotonic stamp rejected then recovered, and a real-process SIGINT
  exits cleanly.

23 tests total. Set `AUTOKART_LOCALIZATION_PATH` to use a non-default `autokart_localization` checkout. The
tests that need it skip, not fail, when it is absent.

## Reusing this node elsewhere

```bash
./export_node.sh --with-algorithm /path/to/export/dir   # node + autokart_msgs + third_party/autokart_localization
```

## Status

IMPLEMENTED and TESTED (software-only: synthetic and topic-fed inputs, real reused algorithm, fault
injection). Lifted-axle and push-test validation, the wheelbase measurement, encoder calibration and sign
checks are all **NOT RUN**. No IMU fusion. **Not a vehicle-readiness claim.**
