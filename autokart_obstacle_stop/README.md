# AutoKart forward-cone obstacle stop

First ROS obstacle-avoidance safety test: a qualified cone in the forward
collision corridor forces the command sent toward the VCU to zero speed.

## Build and test

Place this package beside `autokart_msgs` in a ROS 2 Humble workspace:

```bash
colcon build --packages-up-to autokart_obstacle_stop
source install/setup.bash
pytest -q src/autokart_obstacle_stop/test/test_core.py
ros2 launch autokart_obstacle_stop obstacle_stop.launch.py
```

Observe decisions:

```bash
ros2 topic echo /autokart/obstacle_stop/status
ros2 topic echo /autokart/obstacle_stop/motion_command_candidate
```

For VCU integration, point `autokart_vcu_bridge.candidate_topic` at the
filtered output. Never run two serial-owning VCU bridges simultaneously.

## Current validation boundary

The pure stop geometry, latch and stale-data policy are unit-testable without
ROS. Physical camera detection, metric HP60C calibration, ROS-to-VCU transport,
braking distance and actual BLDC stopping remain live hardware tests.

