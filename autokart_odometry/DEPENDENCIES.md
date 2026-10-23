# External dependencies — autokart_odometry

## ROS package dependencies

- `rclpy`, `nav_msgs`, `geometry_msgs`, `tf2_ros`: ROS 2 Humble core.
- `autokart_msgs` (`kart/ros2_ws/src/autokart_msgs`): `SensorDeliveryMetadata` for status and
  `ActuatorFeedback` for the `actuator_feedback` source. No new message type was introduced.

## Reused algorithm (required at runtime, not build time)

`autokart_localization.odometry` (`WheelOdometry`, `OdometryConfig`, `STATUS_*`) from
`kart/ros2_ws/src/autokart_localization/autokart_localization/odometry.py`. It is pure Python, and
importing it pulls in no ROS dependency. That module's ROS node (`node.py`, which imports
`autokart_mapping`) is **not** imported.

It is imported lazily from the `algorithm_package_path` ROS parameter, which must point at the directory
that **contains** the `autokart_localization/` Python package (e.g. `.../kart/ros2_ws/src/autokart_localization`).
The path is never hardcoded. The loader also checks that the expected API names exist
(`ALGORITHM_API_MISMATCH` otherwise), so an incompatible future change to `odometry.py` fails loudly rather
than being half-used.

**Unlike `autokart_vcu_bridge`, there is no safe degenerate fallback.** Without the reused algorithm this
node publishes no `/odom` and no TF. It reports `ALGORITHM_PATH_NOT_CONFIGURED` /
`DEPENDENCY_PATH_NOT_FOUND` / `ALGORITHM_IMPORT_FAILED` on `/odom/status` repeatedly. The package still
builds and its algorithm-independent tests still pass without it; the tests that need it skip rather
than fail.

**To reuse in another workspace:** `./export_node.sh --with-algorithm <dir>` ships a verbatim copy of
`autokart_localization` under `third_party/` (outside `src/`, so colcon never builds a second
localization node). Point `algorithm_package_path` at that copy. The tests honour
`AUTOKART_LOCALIZATION_PATH` for the same purpose.

## Upstream data dependency

For `source: actuator_feedback`, this node depends on node 04 (`autokart_vcu_bridge`, or the production
`autokart_vehicle_bridge`) publishing `/vehicle/actuator_feedback`. If nothing publishes it, the node
reports `INPUT_STALE_OR_ABSENT` and publishes no odometry.

## The lazy sys.path import caveat

This node uses the same lazy `sys.path` import pattern as the other nodes, with the same `sys.modules`
caching characteristic documented in `autokart_vcu_bridge/DEPENDENCIES.md`. It is harmless in a real
process, which imports exactly once. The unit tests clear the cache between tests because they deliberately
load both real and fake `autokart_localization` directories.

## Not verified in this session

No VCU, no AS5047P/AS5600 encoders, no kart. Lifted-axle and push tests, the wheelbase measurement,
`counts_per_m` calibration and encoder sign are all **NOT RUN**.
