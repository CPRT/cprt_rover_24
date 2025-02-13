#ifndef JOYSTICK_CONTROL__DRIVEMODE_HPP_
#define JOYSTICK_CONTROL__DRIVEMODE_HPP_

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class DriveMode : public Mode {
 public:
  DriveMode(rclcpp::Node& node);
  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

 private:
  void handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void handleCam(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void handleVideo(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void declare_parameters();

  uint8_t kThrottleAxis;
  uint8_t kForwardAxis;
  uint8_t kYawAxis;
  uint8_t kCamTiltAxis;
  uint8_t kCamPanAxis;
  uint8_t kCamReset;
  uint8_t kCamNext;
  uint8_t kCamPrev;
  uint8_t kCruiseControl;
  double kMaxLinear;
  double kMaxAngular;
  double kMaxIncrement;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

#endif  // JOYSTICK_CONTROL__DRIVEMODE_HPP_