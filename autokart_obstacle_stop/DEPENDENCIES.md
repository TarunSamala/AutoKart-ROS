# External dependencies — autokart_obstacle_stop

## ROS package dependencies

- `rclpy`, `builtin_interfaces`, `std_msgs` — ROS 2 Humble core.
- `autokart_msgs` — `ConeArray` and `MotionCommand`.

No detector, camera SDK, motor driver, serial library or GPIO library is
owned by this package. It consumes qualified metric cones and filters an
existing command candidate. The existing VCU bridge remains the only serial
owner.

## Runtime producers and consumers

- Input cones: `autokart_perception_adapter` on `/perception/cones`.
- Input command: `autokart_controller` on
  `/autokart_controller/motion_command_candidate`.
- Output consumer: configure `autokart_vcu_bridge.candidate_topic` as
  `/autokart/obstacle_stop/motion_command_candidate`.

The live perception backend additionally needs its trained model, camera
calibration and HP60C data. Those are dependencies of the perception package,
not this guard.
