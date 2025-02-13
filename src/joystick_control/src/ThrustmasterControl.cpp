#include "ThrustmasterControl.hpp"

ThrustmasterControl::ThrustmasterControl() : Node("thrustmaster_control") {
  currentMode_ = ModeType::DRIVE;
  changeMode(currentMode_);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ThrustmasterControl::processJoystick, this,
                            std::placeholders::_1));
}

void ThrustmasterControl::processJoystick(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (checkForModeChange(joystickMsg)) {
    return;
  };
  mode_->processJoystickInput(joystickMsg);
}

bool ThrustmasterControl::checkForModeChange(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (joystickMsg->buttons[kDriveModeButton]) {
    return changeMode(ModeType::DRIVE);
  } else if (joystickMsg->buttons[kArmIKModeButton]) {
    return changeMode(ModeType::ARM_IK);
  } else if (joystickMsg->buttons[kArmManualModeButton]) {
    return changeMode(ModeType::ARM_MANUAL);
  } else if (joystickMsg->buttons[kNavModeButton]) {
    return changeMode(ModeType::NAV);
  } else if (joystickMsg->buttons[kScienceModeButton]) {
    return changeMode(ModeType::SCIENCE);
  }
  return false;
}

bool ThrustmasterControl::changeMode(ModeType mode) {
  if (currentMode_ == mode) {
    return false;
  }
  switch (mode) {
    case ModeType::DRIVE:
      mode_ = std::make_unique<DriveMode>(this);
      return true;
    default:
      RCLCPP_ERROR(this->get_logger(), "Mode not implemented");
      return false;
  }
  return false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrustmasterControl>());
  rclcpp::shutdown();
  return 0;
}