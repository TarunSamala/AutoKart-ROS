# Sensor Timing

Sensors schedule from `next_capture += 1/rate`, never from physics-time divisibility. Message delivery is capture end plus processing, transport, and seeded non-negative jitter. `SensorHeader` keeps capture start/end, measurement, delivery, and sensor-clock timestamps. Queue overflow, out-of-order delivery, power reboot, and history-buffer interpolation are not yet implemented.
