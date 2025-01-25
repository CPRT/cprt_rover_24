#ifndef TALON_DRIVE_CONTROLLER_HPP
#define TALON_DRIVE_CONTROLLER_HPP

#include "WheelControl.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class TalonDriveController : public rclcpp::Node {
 public:
  TalonDriveController();

 private:
  void odom_pub_callback();
  void control_timer_callback();
  void twist_callback(const Twist::SharedPtr msg);

  double maxSpeed_;
  double baseWidth_;
  bool pubOdom_;
  bool pubElec_;

  double lastTimestamp_;
  double lastVel_;
  std::vector<WheelControl> wheels_;
  std::vector<rclcpp::Subscription<MotorStatus>::SharedPtr> statusSubs_;
  rclcpp::Subscription<Twist>::SharedPtr twistSub_;
  rclcpp::Publisher<Odometry>::SharedPtr odomPub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr odomTimer_;
};

#endif  // TALON_DRIVE_CONTROLLER_HPP
