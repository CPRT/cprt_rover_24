#include "ScienceMode.hpp"

ScienceMode::ScienceMode(rclcpp::Node* node) : Mode("Science", node) {
  RCLCPP_INFO(node_->get_logger(), "Science Mode");
  loadParameters();
  platform_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/platform/set", 10);
  drill_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/drill/set", 10);
  drill_sub_ = node_->create_subscription<ros_phoenix::msg::MotorStatus>(
      "/drill/status", 10,
      std::bind(&ScienceMode::drill_callback, this, std::placeholders::_1));
  platform_sub_ = node_->create_subscription<ros_phoenix::msg::MotorStatus>(
      "/platform/status", 10,
      std::bind(&ScienceMode::platform_callback, this, std::placeholders::_1));
  this->autoDrill = false;
  this->drillHeight = 0;
  // microscope_pub_ =
  // node_->create_publisher<geometry_msgs::msg::Twist>("/microscope/set", 10);
  // TODO:
  // panoramic_pub_ = node_->create_publisher<Bool?>("/science_panoramic", ?);
  // soil_collection_pub_ = node_->create_publisher<???>("/science_soil", ?);
  const double frequency = 10.0;
  const int periodMs = 1000 / frequency;
  autoDrillTimer_ = node->create_wall_timer(
      std::chrono::milliseconds(periodMs),
      std::bind(&ScienceMode::auto_drill_callback, this));
}

void ScienceMode::auto_drill_callback() {
  if (this->autoDrill) {
    if (this->drillHeight < 100) {  // 100mm /10cm
      ros_phoenix::msg::MotorControl drill_control;
      drill_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      drill_control.value = 1.0;
      drill_pub_->publish(drill_control);
      ros_phoenix::msg::MotorControl platform_control;
      platform_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      platform_control.value = 0.5;
      platform_pub_->publish(platform_control);

    } else {
      // handle the servo grabbing soil, and then stop the drill once it reaches
      // 11 ish cm
      this->autoDrill = false;
      ros_phoenix::msg::MotorControl drill_control;
      drill_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      drill_control.value = 0.0;
      drill_pub_->publish(drill_control);
      ros_phoenix::msg::MotorControl platform_control;
      platform_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      platform_control.value = 0.0;
      platform_pub_->publish(platform_control);
    }
  }
}

void ScienceMode::drill_callback(
    const ros_phoenix::msg::MotorStatus::SharedPtr msg) {}
void ScienceMode::platform_callback(
    const ros_phoenix::msg::MotorStatus::SharedPtr msg) {
  // convert from msg position to movement in the platform for drill height
  int distTraveled =
      msg->position - this->drillHeight;  // do better conversion math for the
                                          // difference betwen now and before
  if (distTraveled > 0) {
    this->drillHeight -= distTraveled;
  }
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
  if (!this->autoDrill) {
    ros_phoenix::msg::MotorControl platform_control;
    platform_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    platform_control.value = value;
    platform_pub_->publish(platform_control);
  }
}

void ScienceMode::handleDrill(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Turn it on and off
  int drill_value = joystickMsg->buttons[kDrillToggle];
  int backward_value = joystickMsg->buttons[kDrillBackwardButton];
  if (!this->autoDrill) {
    ros_phoenix::msg::MotorControl drill_control;
    drill_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    if (drill_value) {
      drill_control.value = 1.0;
    } else if (backward_value) {
      drill_control.value = -1.0;
    } else {
      drill_control.value = 0.0;
    }
    drill_pub_->publish(drill_control);
  }
}

void ScienceMode::handleMicroscope(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Process input and output linear component
  double value = joystickMsg->axes[kMicroscopeAxis];
}

void ScienceMode::handlePanoramic(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // TODO
}

void ScienceMode::handleSoilCollection(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  // This is to handle automatically drilling down to 10cm and collecting soil
  this->drillHeight = 0;
  this->autoDrill = true;
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
