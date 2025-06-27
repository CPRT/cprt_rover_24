#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>

#include <interfaces/srv/get_cameras.hpp>

class ROSClient : public QObject {
  Q_OBJECT

 public:
  ROSClient();
  ROSClient(const ROSClient&);

 public slots:
  void get_cameras();

};

#endif