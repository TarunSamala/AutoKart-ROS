# ROS 2 simulated time

`SimulationClock` maps `/clock` to the authoritative fixed-step simulation time.
Pause leaves time unchanged, step advances exactly `N * dt`, and reset emits a
new epoch at time zero while paused. Sensor capture, measurement, and delivery
times remain separate in `SensorEnvelope`; delivery time is never substituted
for measurement time. Exact replay uses recorded timestamps and does not
regenerate sensor noise.
