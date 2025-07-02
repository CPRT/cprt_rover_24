#ifndef CAMERA_CLIENT_H
#define CAMERA_CLIENT_H

#include <interfaces/srv/get_cameras.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class CameraClient : public rclcpp::Node {
 public:
  CameraClient();
  ~CameraClient();

  std::vector<std::string> get_cameras();
};

#endif
