#ifndef JOYSTICK_CONTROL__DRIVEMODE_HPP_
#define JOYSTICK_CONTROL__DRIVEMODE_HPP_

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class DriveMode : public Mode {
 public:
  DriveMode(rclcpp::Node* node);
  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

 private:
  void handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
  void handleCam(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
  void handleVideo(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
  void declare_parameters();
  double getThrottleValue(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

  int8_t kForwardAxis;
  int8_t kYawAxis;
  int8_t kCamTiltAxis;
  int8_t kCamPanAxis;
  int8_t kCamReset;
  int8_t kCamNext;
  int8_t kCamPrev;
  int8_t kCruiseControl;
  int8_t kThrottleAxis;

  double kThrottleMax;
  double kThrottleMin;
  double kMaxLinear;
  double kMaxAngular;
  double kMaxIncrement;
  double kMinSpeed;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

#endif  // JOYSTICK_CONTROL__DRIVEMODE_HPP_