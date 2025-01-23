#ifndef KEYBOARD_PUBLISHER_H
#define KEYBOARD_PUBLISHER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "interfaces/msg/arm_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class KeyboardPublisher : public rclcpp::Node {
public:
  KeyboardPublisher();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
  size_t count_;
  double defSpeed = 10;
  bool isOpen = true;
};

#endif
