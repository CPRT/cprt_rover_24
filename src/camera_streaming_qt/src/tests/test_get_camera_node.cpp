#include <rclcpp/rclcpp.hpp>
#include <interfaces/srv/get_cameras.hpp>
#include <string>
#include <iostream>

#include "tests/test_get_camera_node.h"

GetCameraNode::GetCameraNode() : Node("get_camera_node") {
 get_cameras_service_ = this->create_service<interfaces::srv::GetCameras>("get_cameras", std::bind(&GetCameraNode::get_cameras, this, std::placeholders::_1, std::placeholders::_2));
 RCLCPP_INFO(this->get_logger(), "Service ready: get_cameras");
}

GetCameraNode::~GetCameraNode() {}

void GetCameraNode::get_cameras(const std::shared_ptr<interfaces::srv::GetCameras::Request> request, std::shared_ptr<interfaces::srv::GetCameras::Response> response) {
 std::vector<std::string> names{"test source 1", "test source 2", "test source 3"};
 
 for(int i = 0; i < names.size(); i++) {
  response->sources.push_back(names[i]);
 }

 RCLCPP_INFO(this->get_logger(), "Sending response with sources: ");

 for(int i = 0; i < response->sources.size(); i++) {
  RCLCPP_INFO(this->get_logger(), response->sources[i].c_str());
 }
}

int main(int argc, char** argv) {
 rclcpp::init(argc, argv);

 GetCameraNode::SharedPtr get_camera_node = std::make_shared<GetCameraNode>();

 rclcpp::spin(get_camera_node);
 rclcpp::shutdown();
 return 0;
}