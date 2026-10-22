# Optional ROS 2 adapter

The deterministic simulator core does not import ROS 2.  `kart_sim.autonomy`
contains transport-neutral contracts and `kart_sim.ros2_sil` supplies the
deterministic CI bridge.  This package is the integration boundary for an
installed ROS 2 distribution: it must convert messages explicitly, use the
topic QoS table, publish `/clock`, and pass high-level commands only to the
VCU adapter.  It must never send a ROS command directly to a steering, brake,
or motor model.

The current repository validates the bridge contracts without a running ROS
graph.  Native publisher/subscriber launch testing remains an external
integration test.

The `.msg` files and `CMakeLists.txt` provide a ROSIDL package boundary.
`kart_sim.ros2_wire` is the runtime-independent conversion reference. When a
complete ROS environment is available, build with:

```bash
colcon build --base-paths adapters/ros2
colcon test --packages-select kart_sim_ros2_adapter
```

These commands were not executed in the current environment because `rclpy`
is unavailable.
