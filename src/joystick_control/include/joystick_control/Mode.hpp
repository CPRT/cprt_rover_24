#ifndef MODE_HPP
#define MODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class Mode {
 public:
  Mode(std::string name, rclcpp::Node* node) : name_(name), node_(node) {}
  virtual ~Mode() = default;
  virtual void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) = 0;
  std::string getName() { return name_; }

 protected:
  std::string name_;
  rclcpp::Node* node_;
};

#endif  // MODE_HPP