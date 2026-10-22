cmake_minimum_required(VERSION 3.8)
project(autokart_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(autokart_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(autokart_gazebo_follower SHARED
  src/gazebo_follower_plugin.cpp
)
target_include_directories(autokart_gazebo_follower PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(autokart_gazebo_follower ${GAZEBO_LIBRARIES})
ament_target_dependencies(autokart_gazebo_follower
  autokart_msgs diagnostic_msgs gazebo_ros geometry_msgs nav_msgs rclcpp
  rosgraph_msgs sensor_msgs tf2_ros
)

add_library(autokart_gazebo_sensor_delivery SHARED
  src/gazebo_sensor_delivery_plugin.cpp
)
target_include_directories(autokart_gazebo_sensor_delivery PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(autokart_gazebo_sensor_delivery ${GAZEBO_LIBRARIES})
ament_target_dependencies(autokart_gazebo_sensor_delivery
  autokart_msgs gazebo_ros rclcpp rosgraph_msgs sensor_msgs
)

install(TARGETS
  autokart_gazebo_follower
  autokart_gazebo_sensor_delivery
  LIBRARY DESTINATION lib
)
install(DIRECTORY worlds models config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    gazebo_follower_contract
    test/test_gazebo_follower_contract.py
  )
endif()

ament_package()
