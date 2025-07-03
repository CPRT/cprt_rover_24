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

# --- dependencies ---
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp_lifecycle
  interfaces
  ros_phoenix
  std_srvs
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorControl.msg"
  "msg/MotorStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# List of Phoenix shared libraries
set(PHOENIX_LIBRARIES CTRE_Phoenix
                      CTRE_PhoenixCCI
)

# Download and prepare CTRE Phoenix (both api-cpp and cci)
add_ctre_library(CTRE_Phoenix api-cpp)
add_ctre_library(CTRE_PhoenixCCI cci)

# Create shared library with components and CTRE libraries
add_library(${PROJECT_NAME}_components SHARED
  "src/rover_arm.cpp")
ament_target_dependencies(${PROJECT_NAME}_components
  "rclcpp"
  "rclcpp_components"
  "rcutils")
target_link_libraries(${PROJECT_NAME}_components
  ${PHOENIX_LIBRARIES}
  Threads::Threads
  ros_phoenix__rosidl_typesupport_cpp)

# Register components
rclcpp_components_register_nodes(${PROJECT_NAME}_components "ros_phoenix::TalonSRX")

# Create shared libary with ros2_control PhoenixBridge and PhoenixSystem
add_library(${PROJECT_NAME}_control
  "src/rover_arm.cpp")
ament_target_dependencies(${PROJECT_NAME}_control
  "rclcpp"
  "hardware_interface"
  "pluginlib"
  "rcutils")
target_link_libraries(${PROJECT_NAME}_control
  ${PROJECT_NAME}_components
  ros_phoenix__rosidl_typesupport_cpp)

# Register plugins for control
pluginlib_export_plugin_description_file(hardware_interface ros_phoenix_hardware.xml)

# Install Components
install(TARGETS ${PROJECT_NAME}_components ${PROJECT_NAME}_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include/
)
ament_export_libraries(
  ${PROJECT_NAME}_components ${PROJECT_NAME}_control
)

ament_package()