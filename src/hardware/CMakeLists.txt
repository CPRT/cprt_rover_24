cmake_minimum_required(VERSION 3.16)
project(ros2_control_rover_arm LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  interfaces
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros2_control_rover_arm
  SHARED
  src/rover_arm.cpp
)
target_compile_features(ros2_control_rover_arm PUBLIC c_std_99 cxx_std_17)
target_include_directories(ros2_control_rover_arm PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/src>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ros2_control_rover_arm PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_rover_arm.xml)

# INSTALL
install(
  DIRECTORY src/
  DESTINATION src/ros2_control_rover_arm
)

install(TARGETS ros2_control_rover_arm
  EXPORT export_ros2_control_rover_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_ros2_control_rover_arm HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
