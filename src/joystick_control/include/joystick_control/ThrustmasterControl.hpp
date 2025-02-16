#ifndef THRUSTMASTER_CONTROL_HPP
#define THRUSTMASTER_CONTROL_HPP

#include "DriveMode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class ThrustmasterControl : public rclcpp::Node {
 public:
  ThrustmasterControl();
  void processJoystick(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

 private:
  void loadParameters();
  enum class ModeType { NONE, DRIVE, ARM_IK, ARM_MANUAL, NAV, SCIENCE };
  uint8_t kDriveModeButton;
  uint8_t kArmIKModeButton;
  uint8_t kArmManualModeButton;
  uint8_t kNavModeButton;
  uint8_t kScienceModeButton;

  ModeType currentMode_;

  bool checkForModeChange(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  bool changeMode(ModeType mode);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::unique_ptr<Mode> mode_;
};

#endif  // THRUSTMASTER_CONTROL_HPP