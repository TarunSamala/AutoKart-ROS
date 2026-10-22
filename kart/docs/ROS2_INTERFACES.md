# ROS 2 Interfaces

The workspace is `ros2_ws/src`. Twelve packages build under ROS 2 Humble:
messages, description, bringup, Gazebo, simulation bridge, control,
localization, perception, planning, safety bridge, diagnostics and scenarios.

Standard messages should carry clock, TF, joint state, odometry, image,
camera-info, point-cloud, laser-scan, IMU, path and diagnostics semantics.
`autokart_msgs` supplies domain-specific motion, vehicle/wheel, actuator, VCU,
safety-envelope and cone types.

The command ingress is `/control/motion_command`. It includes source, sequence,
measurement timestamp, expiry, mode, target speed, steering/curvature,
acceleration/deceleration limits, validity and integrity. It is converted to
the transport-neutral VCU command. `/vcu/validated_command` is an observable
output, not an actuator input.

Topic groups and exact names are in
`ros2_ws/src/autokart_bringup/config/topics.yaml`. QoS contracts live in
`kart_sim.ros2_sil.TOPIC_QOS`. Sensors use bounded best effort; commands, VCU,
safety and diagnostics use reliable delivery with explicit deadlines where
appropriate.

TF:

```text
map -> odom -> base_link -> chassis_link
                              +-> wheel/axle links
                              +-> camera_link -> camera_optical_frame
                              +-> depth_camera_link / lidar_link / imu_link / gps_link
```

Static and dynamic transforms are separated. `FrameTree` rejects cycles and
duplicate children and normalizes quaternions. Multi-kart frames are prefixed
with `kart_01/`, `kart_02/`, and so on.

Build and smoke:

```bash
./scripts/build_ros2.sh
./scripts/test_ros2_native.sh
ros2 launch autokart_bringup full_stack.launch.py
```

The native custom-message round trip passed using local shared memory. UDP
transport reported sandbox permission warnings. The current native
`vcu_bridge` subscribes only to the high-level request; complete simulator
state publishers and sensor bridges remain integration work.

