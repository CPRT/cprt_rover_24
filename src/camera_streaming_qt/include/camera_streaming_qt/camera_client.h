#ifndef CAMERA_CLIENT_H
#define CAMERA_CLIENT_H

#include <rclcpp/rclcpp.hpp>

class CameraClient : public rclcpp::Node {
    CameraClient();

    ~CameraClient();
};

#endif
