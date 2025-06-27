#include "ros_client.h"

#include <QDebug>
#include <string>

ROSClient::ROSClient() {}

void ROSClient::get_cameras() {
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_cameras");
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

  // Send the request
  auto result = client->async_send_request(
      request,
      [](rclcpp::Client<interfaces::srv::GetCameras>::SharedFuture response) {
       // If the response is valid, return the sources
       if(response.valid()) {
        std::vector<std::string> sources = response.get()->sources;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received sources:");

        for (int i = 0; i < sources.size(); i++) {
          qDebug() << QString::fromStdString(sources[i]);
        }
       }
       else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not receive sources.");
       }
      });
}