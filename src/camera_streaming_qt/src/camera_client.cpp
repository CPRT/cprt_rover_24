#include "../include/camera_streaming_qt/camera_client.h"

#include <QApplication>
#include <QDebug>
#include <vector>

#include "mainwindow.h"
#include "ros_client.h"

CameraClient::CameraClient() : Node("camera_client_node") {}

CameraClient::~CameraClient() {}

std::vector<std::string> CameraClient::get_cameras() {
  rclcpp::Client<interfaces::srv::GetCameras>::SharedPtr client =
      this->create_client<interfaces::srv::GetCameras>("get_cameras");

  auto request = std::make_shared<interfaces::srv::GetCameras::Request>();

  std::chrono::seconds wait_interval(1);
  int counter_ = 0;
  while (!client->wait_for_service(wait_interval)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return std::vector<std::string>();
    }

    counter_++;
    // 5 second timeout
    if (counter_ >= 5) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Get camera service timed out.");
      return std::vector<std::string>();
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Service not available, waiting again...");
  }

  auto future = client->async_send_request(request);

  // Wait for result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Get Cameras service call failed.");
    return std::vector<std::string>();
  }

  std::vector<std::string> sources = future.get()->sources;

  RCLCPP_INFO(this->get_logger(), "Received sources: ");

  for (int i = 0; i < sources.size(); i++) {
    RCLCPP_INFO(this->get_logger(), sources[i].c_str());
  }

  return sources;

  /*
  std::chrono::seconds wait_interval(5);
  auto status = result_future.wait_for(wait_interval);

  if (status != std::future_status::ready) {
    qDebug() << "Timeout exceeded: did not receive camera sources within given "
                "time.";
  }
  */
}

void CameraClient::start_video(
    int num_sources, std::vector<interfaces::msg::VideoSource> sources) {
  rclcpp::Client<interfaces::srv::VideoOut>::SharedPtr client =
      this->create_client<interfaces::srv::VideoOut>("start_video");

  auto request = std::make_shared<interfaces::srv::VideoOut::Request>();
  request->num_sources = num_sources;
  request->sources = sources;

  std::chrono::seconds wait_interval(1);
  int counter_ = 0;
  while (!client->wait_for_service(wait_interval)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
    }

    counter_++;
    // 5 second timeout
    if (counter_ >= 5) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Start video service timed out.");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Service not available, waiting again...");
  }

  auto future = client->async_send_request(request);

  // Wait for result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Start video service call failed.");
  }

  RCLCPP_INFO(get_logger(), "Start video service call succeeded.");
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