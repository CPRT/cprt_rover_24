
cmake_minimum_required(VERSION 3.5)
project(description)

find_package(ament_cmake REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(position_controllers REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY config launch robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
