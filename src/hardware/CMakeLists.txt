cmake_minimum_required(VERSION 3.5)
project(ros2_control_rover_arm LANGUAGES CXX)

# Detect architecture
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
    set(CTRE_ARCH "raspbian")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
    set(CTRE_ARCH "aarch64bionic")
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "x86_64-linux-gnu")
    set(CTRE_ARCH "x86-64")
else()
    message(FATAL_ERROR "Unsupported architecture for CTRE Phoenix: ${CMAKE_LIBRARY_ARCHITECTURE}")
endif()

set(CTRE_VERSION "5.21.2")

# Macro for downloading and extracting CTRE Phoenix
macro(add_ctre_library LIB_NAME URL_NAME)
    set(LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/ctre/${CTRE_VERSION}/${URL_NAME}")
    set(LIB_DEST "${LIB_DIR}/linux/${CTRE_ARCH}/shared/lib${LIB_NAME}.so")
    file(MAKE_DIRECTORY ${LIB_DIR})
    file(MAKE_DIRECTORY ${LIB_DIR}/ctre)

    set(FILE_NAME "${URL_NAME}-${CTRE_VERSION}-linux${CTRE_ARCH}.zip")
    set(FILE_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${FILE_NAME}")
    set(FILE_DEST "${LIB_DIR}/${FILE_NAME}")
    if(NOT EXISTS ${FILE_DEST})
        message(STATUS "Downloading ${LIB_NAME} from ${FILE_URL}")
        file(DOWNLOAD ${FILE_URL} ${FILE_DEST})
    endif()

    set(HEADER_NAME "${URL_NAME}-${CTRE_VERSION}-headers.zip")
    set(HEADER_URL "http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/${URL_NAME}/${CTRE_VERSION}/${HEADER_NAME}")
    set(HEADER_DEST "${LIB_DIR}/${HEADER_NAME}")
    if(NOT EXISTS ${HEADER_DEST})
        message(STATUS "Downloading headers for ${LIB_NAME} from ${HEADER_URL}")
        file(DOWNLOAD ${HEADER_URL} ${HEADER_DEST})
    endif()

    add_custom_target(${LIB_NAME}-unzip DEPENDS ${LIB_DEST})
    add_custom_command(OUTPUT ${LIB_DEST} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${FILE_DEST}
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${HEADER_DEST}
        WORKING_DIRECTORY ${LIB_DIR}
        COMMENT "Unpacking ${LIB_NAME}"
        VERBATIM
    )

    add_library(${LIB_NAME} SHARED IMPORTED)
    add_dependencies(${LIB_NAME} ${LIB_NAME}-unzip)
    set_target_properties(${LIB_NAME} PROPERTIES
        IMPORTED_LOCATION ${LIB_DEST}
        INTERFACE_INCLUDE_DIRECTORIES ${LIB_DIR}/ctre
    )

    # Install library and headers
    install(FILES ${LIB_DEST}
        DESTINATION lib
    )
    install(DIRECTORY ${LIB_DIR}/ctre/
        DESTINATION include/ctre
    )
endmacro()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Download and prepare CTRE Phoenix (both api-cpp and cci)
add_ctre_library(CTRE_Phoenix api-cpp)
add_ctre_library(CTRE_PhoenixCCI cci)

# --- ROS2 dependencies ---
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  interfaces
  ros_phoenix
  std_srvs
  std_msgs
  Threads
  rclcpp_components
  rcutils
  rosidl_default_generators
)

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
add_dependencies(ros2_control_rover_arm CTRE_Phoenix-unzip CTRE_PhoenixCCI-unzip)
target_compile_features(ros2_control_rover_arm PUBLIC c_std_99 cxx_std_17)
target_include_directories(ros2_control_rover_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ctre/${CTRE_VERSION}/api-cpp/ctre>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/ctre/${CTRE_VERSION}/cci/ctre>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ros2_control_rover_arm
  PUBLIC
    CTRE_Phoenix
    CTRE_PhoenixCCI
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
