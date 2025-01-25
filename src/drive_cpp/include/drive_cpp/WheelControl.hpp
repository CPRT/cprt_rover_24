#ifndef WHEEL_CONTROL_HPP
#define WHEEL_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

using namespace ros_phoenix::msg;

enum class WheelSide { LEFT, RIGHT };

class WheelControl {
 public:
  struct Status {
    double velocity;
    double temperature;
    double current;
    double voltageIn;
    double voltageOut;
  };
  WheelControl(std::string wheel_name, rclcpp::Node* node);

  void setVelocity(double value);
  void send() const;

  WheelSide getWheelSide() const { return side_; }
  double getVelocity() const { return status_.velocity; }
  void setStatus(const MotorStatus::SharedPtr msg);

 private:
  std::string name_;
  WheelSide side_;
  MotorControl control_;
  Status status_;
  rclcpp::Node* node_;
  rclcpp::Publisher<MotorControl>::SharedPtr pub_;
};

#endif  // WHEEL_CONTROL_HPP
