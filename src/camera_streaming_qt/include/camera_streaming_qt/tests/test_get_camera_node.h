#ifndef TEST_GET_CAMERA_NODE_H
#define TEST_GET_CAMERA_NODE_H

#include <interfaces/srv/get_cameras.hpp>
#include <rclcpp/rclcpp.hpp>

class GetCameraNode : public rclcpp::Node {
 public:
  GetCameraNode();
  ~GetCameraNode();

  void get_cameras(
      const std::shared_ptr<interfaces::srv::GetCameras::Request> request,
      std::shared_ptr<interfaces::srv::GetCameras::Response> response);
  rclcpp::Service<interfaces::srv::GetCameras>::SharedPtr get_cameras_service_;
};

#endif