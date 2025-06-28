cmake_minimum_required(VERSION 3.5)
project(ros2_control_rover_arm LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ros_phoenix REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  rcutils
  std_srvs
  std_msgs
  rosidl_default_generators
  interfaces
  ros_phoenix
)

add_library(ros2_control_rover_arm SHARED
  src/rover_arm.cpp
)

target_compile_features(ros2_control_rover_arm PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(ros2_control_rover_arm PUBLIC Phoenix_No_WPI)

target_include_directories(ros2_control_rover_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ros_phoenix_INCLUDE_DIRS}
)

target_link_libraries(ros2_control_rover_arm
  PUBLIC
    CTRE_Phoenix
    CTRE_PhoenixCCI
)

ament_target_dependencies(
  ros2_control_rover_arm
  PUBLIC
    ${THIS_PACKAGE_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  ros2_control_rover_arm.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ros2_control_rover_arm
  EXPORT  export_ros2_control_rover_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_ros2_control_rover_arm HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})

ament_package()
