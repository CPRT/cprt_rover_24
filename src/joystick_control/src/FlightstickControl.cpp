#include "FlightstickControl.hpp"

FlightstickControl::FlightstickControl()
    : Node("flightstick_control"),
      mode_(nullptr),
      currentMode_(ModeType::NONE) {
  declareParameters();
  loadParameters();
  changeMode(currentMode_);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&FlightstickControl::processJoystick, this,
                std::placeholders::_1));

  status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/flightstick/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  light_pub_ = this->create_publisher<std_msgs::msg::Int8>("/lights", qos);
}

void FlightstickControl::processJoystick(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (checkForModeChange(joystickMsg)) {
    return;
  }
  if (mode_ != nullptr && currentMode_ != ModeType::NONE) {
    mode_->processJoystickInput(joystickMsg);
  }
}

bool FlightstickControl::checkForModeChange(
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

bool FlightstickControl::changeMode(ModeType mode) {
  if (currentMode_ == mode) {
    return false;
  }
  currentMode_ = mode;
  auto message = std_msgs::msg::String();
  switch (mode) {
    case ModeType::NONE:
      mode_ = nullptr;
      message.data = "Idle";
      status_pub_->publish(message);
      break;
    case ModeType::DRIVE:
      RCLCPP_INFO(this->get_logger(), "Entering Drive Mode");
      mode_ = std::make_unique<DriveMode>(this);
      message.data = "Drive";
      status_pub_->publish(message);
      break;
    case ModeType::ARM_MANUAL:
      RCLCPP_INFO(this->get_logger(), "Entering Manual Mode");
      mode_ = std::make_unique<ArmManualMode>(this);
      message.data = "Manual";
      status_pub_->publish(message);
      break;
    case ModeType::SCIENCE:
      RCLCPP_INFO(this->get_logger(), "Entering Science Mode");
      mode_ = std::make_unique<ScienceMode>(this);
      message.data = "Science";
      status_pub_->publish(message);
      break;
    case ModeType::ARM_IK:
      RCLCPP_INFO(this->get_logger(), "Entering IK Mode");
      mode_ = std::make_unique<ArmIKMode>(this);
      message.data = "IK";
      status_pub_->publish(message);
      break;
    default:
      RCLCPP_WARN(this->get_logger(),
                  "Mode not implemented, returning to NONE");
      currentMode_ = ModeType::NONE;
      mode_ = nullptr;
      message.data = "Idle";
      status_pub_->publish(message);
      return false;
  }
  auto msg = std_msgs::msg::Int8();
  msg.data = kTeleopLightMode;
  light_pub_->publish(msg);
  return true;
}

void FlightstickControl::declareParameters() {
  this->declare_parameter("drive_mode_button", 12);
  this->declare_parameter("arm_ik_mode_button", 11);
  this->declare_parameter("arm_manual_mode_button", 10);
  this->declare_parameter("nav_mode_button", 13);
  this->declare_parameter("science_mode_button", 14);
  this->declare_parameter("teleop_light_mode", 1);
  DriveMode::declareParameters(this);
  ArmManualMode::declareParameters(this);
  ArmIKMode::declareParameters(this);
  ScienceMode::declareParameters(this);
}

void FlightstickControl::loadParameters() {
  this->get_parameter("drive_mode_button", kDriveModeButton);
  this->get_parameter("arm_ik_mode_button", kArmIKModeButton);
  this->get_parameter("arm_manual_mode_button", kArmManualModeButton);
  this->get_parameter("nav_mode_button", kNavModeButton);
  this->get_parameter("science_mode_button", kScienceModeButton);
  this->get_parameter("teleop_light_mode", kTeleopLightMode);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightstickControl>());
  rclcpp::shutdown();
  return 0;
}
