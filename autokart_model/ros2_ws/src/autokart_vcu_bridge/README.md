# autokart_vcu_bridge

Read-only adapter to the existing `autokart_vehicle_bridge.protocol` v2 serial
parser in `kart/ros2_ws/src/autokart_vehicle_bridge`. Firmware is unchanged.
The inspected protocol sends raw axle counts, t_ms, raw angle readings and ACK;
it lacks a qualified boot/session ID, clock conversion, calibration identity,
measured road-wheel steering and independent ground velocity.

This adapter consequently publishes diagnostic telemetry with validity false.
It never sends CMD, ARM, DIS or RST, never maps a ROS speed into DAC voltage, and
has no command subscriptions. Even direct calls to legacy SerialVcuBackend's
send APIs fail disabled. SyntheticBackend retains a local firmware simulator
for existing backend unit fixtures; the ROS node never commands it.

Default synthetic mode opens no device. `hardware_enabled:false` also applies
to the shipped live YAML. Deliberate serial access uses pyserial exclusive
ownership; launch exactly one serial owner and never run a micro-ROS agent
against that endpoint. An unrelated process that ignores advisory locking
still requires deployment ownership audit.

Outputs: `/vehicle/wheel_encoder`, `/vehicle/stop_evidence`,
`/vehicle/telemetry` String JSON v1; `/vehicle/status` and compatible
`/vcu_bridge/status` SensorDeliveryMetadata; invalid ActuatorFeedback and
observed VcuState. Unknown actuator values are never marked valid. `/vcu/state`
legacy fields cannot express contactor validity; contactor_open=false is UNKNOWN,
not proof of a closed or open contactor; use validity in canonical envelopes.

Launch `ros2 launch autokart_vcu_bridge vcu_bridge.launch.py`. Hardware runtime,
firmware identity, physical stop/brake and clock alignment remain BLOCKED.
