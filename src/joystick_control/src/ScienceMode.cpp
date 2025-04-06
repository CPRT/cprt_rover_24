#include "ScienceMode.hpp"

ScienceMode::ScienceMode(rclcpp::Node* node) : Mode("Science", node) {
  RCLCPP_INFO(node_->get_logger(), "Science Mode");
  loadParameters();
  platform_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/platform/set", 10);
  drill_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/drill/set", 10);  // TODO: create drill launch file
  microscope_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/microscope/set", 10);
  // TODO:
  // panoramic_pub_ = node_->create_publisher<Bool?>("/science_panoramic", ?);
  // soil_collection_pub_ = node_->create_publisher<???>("/science_soil", ?);
}

void ScienceMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handlePlatform(joystickMsg);
  handleDrill(joystickMsg);
  handleMicroscope(joystickMsg);
  // handlePanoramic(joystickMsg);
  // handleSoilCollection(joystickMsg);
}

void ScienceMode::handlePlatform(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Process input and output linear component
  double value = joystickMsg->axes[kPlatformAxis];
  ros_phoenix::msg::MotorControl platform_;
  platform_.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  platform_.value = value;
  platform_pub_->publish(platform_);
}

void ScienceMode::handleDrill(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Turn it on and off
  int drill_value = joystickMsg->buttons[kDrillButton];
  int value;
  ros_phoenix::msg::MotorControl drill_;
  drill_.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  if (drill_value == true) {
    value = 1;
  } else {
    value = 0;
  }
  drill_.value = value;
  drill_pub_->publish(drill_);
}

void ScienceMode::handleMicroscope(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Process input and output linear component
  double value = joystickMsg->axes[kMicroscopeAxis];
  ros_phoenix::msg::MotorControl microscope_;
  microscope_.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  microscope_.value = value;
  microscope_pub_->publish(microscope_);
}

void ScienceMode::handlePanoramic(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // TODO
}

void ScienceMode::handleSoilCollection(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // TODO
}

void ScienceMode::declareParameters(rclcpp::Node* node) {
  node->declare_parameter("science_mode.platform_axis", 1);
  node->declare_parameter("science_mode.drill_button", 0);
  node->declare_parameter("science_mode.microscope_axis", 5);
  node->declare_parameter("science_mode.drill_state", false);
  // node->declare_parameter("science_mode.panoramic_button", ?);
  // node->declare_parameter("science_mode.soil_collection_button, ?);
}

void ScienceMode::loadParameters() {
  node_->get_parameter("science_mode.platform_axis", kPlatformAxis);
  node_->get_parameter("science_mode.drill_button", kDrillButton);
  node_->get_parameter("science_mode.microscope_axis", kMicroscopeAxis);
  node_->get_parameter("science_mode.drill_state", kDrillState);
}
