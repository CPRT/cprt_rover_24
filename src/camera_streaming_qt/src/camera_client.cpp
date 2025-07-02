#include "../include/camera_streaming_qt/camera_client.h"

#include <QApplication>
#include <QDebug>
#include <vector>

#include "mainwindow.h"
#include "ros_client.h"

CameraClient::CameraClient() : Node("camera_client_node") {}

CameraClient::~CameraClient() {}

void CameraClient::get_cameras() {
  rclcpp::Client<interfaces::srv::GetCameras>::SharedPtr client =
      this->create_client<interfaces::srv::GetCameras>("get_cameras");

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
      request, std::bind(&CameraClient::on_cameras_received, this,
                         std::placeholders::_1));

  /*
  std::chrono::seconds wait_interval(5);
  auto status = result_future.wait_for(wait_interval);

  if (status != std::future_status::ready) {
    qDebug() << "Timeout exceeded: did not receive camera sources within given "
                "time.";
  }
  */
}

void CameraClient::on_cameras_received(
    rclcpp::Client<interfaces::srv::GetCameras>::SharedFuture future) {
  std::vector<std::string> sources = future.get()->sources;

  RCLCPP_INFO(this->get_logger(), "Received sources: ");

  for (int i = 0; i < sources.size(); i++) {
    RCLCPP_INFO(this->get_logger(), sources[i].c_str());
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto camera_client = std::make_shared<CameraClient>();

  QApplication a(argc, argv);
  MainWindow* w = new MainWindow(camera_client.get(), nullptr);
  w->resize(1280, 720);
  w->show();
  a.exec();

  rclcpp::spin(camera_client);
  rclcpp::shutdown();

  return 0;
}