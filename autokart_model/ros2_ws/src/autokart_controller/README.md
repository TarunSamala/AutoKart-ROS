# autokart_controller

ROS 2 Pure Pursuit candidate adapter. See CONTRACT.md for canonical topics,
expiry, diagnostics and gates. It publishes speed/road-wheel-steering requests
with output_inhibited=true; it has no actuator transport.

Build in a workspace containing autokart_msgs:

```bash
colcon build --packages-up-to autokart_controller
source install/setup.bash
ros2 launch autokart_controller controller.launch.py
```

Default UNVERIFIED configuration remains disarmed and requires fresh planner,
pose and measured steering feedback. Set the optional reused backend dependency
path explicitly. Physical testing is not part of this package validation.
