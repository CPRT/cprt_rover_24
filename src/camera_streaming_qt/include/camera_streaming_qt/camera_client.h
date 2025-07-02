#ifndef CAMERA_CLIENT_H
#define CAMERA_CLIENT_H

#include <interfaces/srv/get_cameras.hpp>
#include <rclcpp/rclcpp.hpp>

class CameraClient : public rclcpp::Node {
 public:
  CameraClient();
  ~CameraClient();

  void get_cameras();

 private:
  void on_cameras_received(
      rclcpp::Client<interfaces::srv::GetCameras>::SharedFuture future);
};

#endif
