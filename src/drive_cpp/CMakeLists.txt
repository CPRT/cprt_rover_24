cmake_minimum_required(VERSION 3.5)
project(drive_cpp)

# Find the required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_phoenix REQUIRED)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Include directories for ROS 2
include_directories(include/${PROJECT_NAME})

# Declare the executables for the nodes
add_executable(drive_cpp src/TalonDriveController.cpp src/WheelControl.cpp)

# Link the dependencies to the executable
ament_target_dependencies(drive_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  ros_phoenix
)

# Install the executable
install(TARGETS
  drive_cpp
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files (if any)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Install configuration files (if any)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
# )

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()
