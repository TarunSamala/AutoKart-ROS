# Phase 1 — interface freeze (for autokart_ros_nodes)

Per the master architecture doc §15 ("reuse autokart_msgs message definitions... add or evolve
compatible messages only after inspected interfaces").

## Reused as-is, unmodified

- `autokart_msgs/msg/SensorDeliveryMetadata.msg` — used by `autokart_hp60c` for `/hp60c/status`.
  Inspected fields match the camera-delivery status contract needed (sequence, quality, fault_state,
  payload_present, cumulative counters) with no gaps requiring a new field. No change made to this
  message definition.
- Standard `sensor_msgs/Image`, `sensor_msgs/CameraInfo` — used for `/hp60c/{rgb,depth}/image_raw`
  and `/hp60c/{rgb,depth}/camera_info`, per the master doc's "standard messages first" rule.

## Not yet needed / deferred

- `autokart_msgs/msg/Cone.msg`, `ConeArray.msg` — will matter once node 02 (`autokart_perception`) is
  built in this library; not touched by node 01.
- `autokart_msgs/msg/VcuState.msg`, `VehicleState.msg`, `MotionCommand.msg`,
  `AuthoritativeVehicleSnapshot.msg`, `ActuatorFeedback.msg`, `WheelState(s).msg`,
  `BenchVcuStatus.msg`, `SafetyEnvelope.msg`, `MissionDebug.msg`, `GazeboFollowerState.msg` — all
  reviewed (see `AUDIT_PHASE0.md`) but not exercised by any node built this session.

## No new message types were introduced this session.

If a future node in this library needs a field `autokart_msgs` doesn't have, the correct move (per
the master doc) is to evolve the existing message compatibly in `kart/ros2_ws/src/autokart_msgs` —
not to fork a parallel message definition inside this library — and that is a decision for the
specific node's own development pass, reviewed on its own, not pre-decided here.
