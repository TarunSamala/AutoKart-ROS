# ROS 2 architecture

Purpose: expose a deterministic headless simulator to autonomy software without
moving the VCU safety boundary into ROS. The core uses transport-neutral
contracts in `kart_sim.autonomy`; `kart_sim.ros2_sil` is the deterministic CI
bridge, and `adapters/ros2/` is the optional native ROS 2 boundary.

`sensor model -> bridge publisher -> autonomy -> high-level MotionCommand ->
VCU -> independent safety override -> actuator model -> vehicle` is the only
normal command path. ROS, replay, and dashboard adapters cannot publish a
`VehicleCommand` or write an actuator directly.

ROS CLI and `rclpy` were detected in this environment. Native graph publication
is intentionally optional and requires a separate launch/integration run.
