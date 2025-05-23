cmake_minimum_required(VERSION 3.5)
project(interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#for msgs
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SixFloats.msg"
  "msg/LiveTune.msg"
  "msg/PIDmsg.msg"
  "msg/ArucoMarkers.msg"
  "msg/PointArray.msg"
  "msg/ArmCmd.msg"
  "msg/VideoSource.msg"
  "msg/SystemTelemetry.msg"
  "srv/ArmPos.srv"
  "srv/ArmCmd.srv"
  "msg/GPIOmsg.msg" #ik its goofy the actual gpio package is GPIO
  "msg/GasSensorReading.msg"
  "srv/NavToGPSGeopose.srv"
  "srv/VideoOut.srv"
  "srv/MoveServo.srv"
  "srv/VideoCapture.srv"
  "srv/GetCameras.srv"
  "srv/Raman.srv"
  DEPENDENCIES 
  builtin_interfaces
  geometry_msgs 
  geographic_msgs # Add packages that above messages depend on
  sensor_msgs
 )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
