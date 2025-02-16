cmake_minimum_required(VERSION 3.5)
project(joystick_control)

# Find the required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(joy REQUIRED)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Include directories for ROS 2
include_directories(include/${PROJECT_NAME})

# Declare the executables for the nodes
add_executable(thrustmaster_control src/ThrustmasterControl.cpp src/DriveMode.cpp )

# Link the dependencies to the executable
ament_target_dependencies(thrustmaster_control
  rclcpp
  geometry_msgs
  nav_msgs
  joy
)

# Install the executable
install(TARGETS
thrustmaster_control
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files (if any)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files (if any)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()
