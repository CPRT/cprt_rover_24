#include "ros_client.h"

#include <QDebug>
#include <string>

ROSClient::ROSClient() {}

void ROSClient::get_cameras() {
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("get_camera_client");
  rclcpp::Client<interfaces::srv::GetCameras>::SharedPtr client =
      node->create_client<interfaces::srv::GetCameras>("get_cameras");

  auto request = std::make_shared<interfaces::srv::GetCameras::Request>();

  /*
   std::chrono::seconds wait_interval(1);
   while (!client->wait_for_service(wait_interval)) {
     if (!rclcpp::ok()) {
       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the service. Exiting.");
       return;
     }

     RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                 "Service not available, waiting again...");
   }
  */

  auto result_future = client->async_send_request(
      request,
      std::bind(&ROSClient::on_cameras_received, this, std::placeholders::_1));

  std::chrono::seconds wait_interval(5);
  auto status = result_future.wait_for(wait_interval);

  if (status != std::future_status::ready) {
    qDebug() << "Timeout exceeded: did not receive camera sources within given "
                "time.";
  }
}

void ROSClient::on_cameras_received(
    rclcpp::Client<interfaces::srv::GetCameras>::SharedFuture future) {
  std::cout << "received" << std::endl;
}