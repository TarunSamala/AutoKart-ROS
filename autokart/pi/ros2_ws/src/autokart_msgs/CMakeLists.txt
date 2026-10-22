cmake_minimum_required(VERSION 3.8)
project(autokart_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConeDetection.msg"
  "msg/ConeDetectionArray.msg"
  "msg/LocalMap.msg"
  "msg/PlannedPath.msg"
  "msg/ControllerState.msg"
  "msg/VehicleCommand.msg"
  "msg/VehicleTelemetry.msg"
  "msg/VehicleOdom.msg"
  "msg/StartupStage.msg"
  "msg/SafetyState.msg"
  DEPENDENCIES std_msgs geometry_msgs)
ament_package()
