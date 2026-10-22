# Replay

Internal telemetry and `SensorEnvelope` preserve capture, measurement, and
delivery timestamps. Deterministic replay reuses recorded payloads/timestamps,
can pause or step via `SimulationClock`, and may add an explicit fault overlay.
It must not regenerate measurement noise during exact replay. Rosbag import and
native ROS bag output remain optional adapter work.
