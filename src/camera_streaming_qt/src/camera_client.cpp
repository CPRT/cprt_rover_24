#include "../include/camera_streaming_qt/camera_client.h"

#include <QApplication>

#include "mainwindow.h"

CameraClient::CameraClient() : Node("camera_client_node") {}

CameraClient::~CameraClient() {}

int main(int argc, char** argv) {
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraClient>());
  rclcpp::shutdown();

  return a.exec();
}