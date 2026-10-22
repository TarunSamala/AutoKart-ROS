# Software in the loop

Supported deterministic CI mode uses `DeterministicRosBridge` with a fixed-step
clock and production debug-truth isolation. Intended launch modes are baseline
controllers, external ROS autonomy, sensor replay, headless CI, and faulted
SIL. Native ROS launch is optional; no real actuator interface is enabled by
this adapter.
