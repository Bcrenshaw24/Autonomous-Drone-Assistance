cmake_minimum_required(VERSION 3.5)
project(drone_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(drone_mover src/drone_mover.cpp)
ament_target_dependencies(drone_mover rclcpp geometry_msgs)

install(TARGETS
  drone_mover
  DESTINATION lib/${PROJECT_NAME})

ament_package()
