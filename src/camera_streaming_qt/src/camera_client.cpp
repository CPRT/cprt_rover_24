#include "../include/camera_streaming_qt/camera_client.h"

#include <QApplication>

#include "mainwindow.h"
#include "ros_client.h"

CameraClient::CameraClient() : Node("camera_client_node") {}

CameraClient::~CameraClient() {}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  QApplication a(argc, argv);
  MainWindow w;
  w.resize(1280, 720);
  w.show();
  a.exec();

  rclcpp::spin(std::make_shared<CameraClient>());
  rclcpp::shutdown();

  return 0;
}