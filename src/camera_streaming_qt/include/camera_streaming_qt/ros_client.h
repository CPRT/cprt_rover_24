#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include <QObject>
#include <interfaces/srv/get_cameras.hpp>
#include <rclcpp/rclcpp.hpp>

class ROSClient : public QObject {
  Q_OBJECT

 public:
  ROSClient();
  ROSClient(const ROSClient&);

 public slots:
  void get_cameras();

 private:
  void on_cameras_received(
      rclcpp::Client<interfaces::srv::GetCameras>::SharedFuture future);
};

#endif