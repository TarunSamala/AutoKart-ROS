cmake_minimum_required(VERSION 3.8)
project(kart_sim_ros2_adapter)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommand.msg"
  "msg/SensorTiming.msg"
  "msg/VcuState.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
