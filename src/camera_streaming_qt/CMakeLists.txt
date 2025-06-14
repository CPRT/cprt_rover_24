cmake_minimum_required(VERSION 3.16)

project(camera_streaming_qt VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

include_directories(
    include/${PROJECT_NAME}
)

set(SOURCES
    src/camera_client.cpp
    src/mainwindow.cpp
)

set(HEADERS
    include/camera_streaming_qt/camera_client.h
    include/camera_streaming_qt/mainwindow.h
)

add_executable(camera_client_node ${SOURCES} ${HEADERS})

target_link_libraries(
    camera_client_node 
    rclcpp::rclcpp
    Qt${QT_VERSION_MAJOR}::Widgets
)

ament_target_dependencies(camera_client_node rclcpp std_msgs)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(TARGETS 
    camera_client_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_package()
