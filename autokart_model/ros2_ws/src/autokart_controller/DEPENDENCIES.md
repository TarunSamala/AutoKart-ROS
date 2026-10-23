# Dependencies

ROS 2 Humble: rclpy, nav_msgs, geometry_msgs, std_msgs, builtin_interfaces,
autokart_msgs including MissionDebug. Source a freshly built interface overlay;
the historical kart/ros2_ws install lacks MissionDebug and is insufficient.

Optional reused backend: kart_sim directory under configured kart_sim_package_path.
The adapter checks the selected source files exist and rejects a cached import
from another tree. The portable builtin backend requires only Python math.

Export includes the sibling source autokart_msgs package. No symlink dependencies
or original algorithm library are edited. Physical qualification is external.
