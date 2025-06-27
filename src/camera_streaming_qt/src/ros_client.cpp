#include <QDebug>
#include "ros_client.h"

ROSClient::ROSClient() {
}

void ROSClient::get_cameras() {
 std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("/get_cameras");
 rclcpp::Client<interfaces::srv::GetCameras>::SharedPtr client = 
  node->create_client(interfaces::srv::GetCameras)("/get_cameras");

 auto request = std::make_shared<interfadces::srv::GetCameras::Request>();

 while(!client->wait_for_service(1s)) {
  if(!rclpp::ok()) {
   qDebug() << rcl::get_logger("rcl")
  }
 }
}