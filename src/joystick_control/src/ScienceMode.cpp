#include "ScienceMode.hpp"

ScienceMode::ScienceMode(rclcpp::Node* node) : Mode("Science", node) {
  RCLCPP_INFO(node_->get_logger(), "Science Mode");
  loadParameters();
  platform_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/platform/set", 10);
  drill_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/drill/set", 10);
  platform_sub_ = node_->create_subscription<ros_phoenix::msg::MotorStatus>(
      "/platform/status", 10,
      std::bind(&ScienceMode::platform_callback, this, std::placeholders::_1));
  servo_client_ = node_->create_client<interfaces::srv::MoveServo>(
      "/science_servo_service");
  this->autoDrill = false;
  this->drillHeight = 0;
  // TODO:
  // panoramic_pub_ = node_->create_publisher<Bool?>("/science_panoramic", ?);
}

void ScienceMode::auto_drill_callback() {
  if (this->autoDrill) {
    if (this->drillHeight < 110) {
      if (this->drillHeight < 100) {
        setServoPosition(kCollectionServo, kCollectionClose);
      } else {
        setServoPosition(kCollectionServo, kCollectionOpen);
      }
      ros_phoenix::msg::MotorControl drill_control;
      drill_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      drill_control.value = 1.0;
      drill_pub_->publish(drill_control);
      ros_phoenix::msg::MotorControl platform_control;
      platform_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      platform_control.value = 1.0;
      platform_pub_->publish(platform_control);
    } else {
      // Stop the drill and platform
      this->autoDrill = false;
      ros_phoenix::msg::MotorControl drill_control;
      drill_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      drill_control.value = 0.0;
      drill_pub_->publish(drill_control);
      ros_phoenix::msg::MotorControl platform_control;
      platform_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      platform_control.value = 0.0;
      platform_pub_->publish(platform_control);
      autoDrillTimer_.reset();
      setServoPosition(kCollectionServo, kCollectionClose);
    }
  }
}

void ScienceMode::drill_callback(
    const ros_phoenix::msg::MotorStatus::SharedPtr msg) {}
void ScienceMode::platform_callback(
    const ros_phoenix::msg::MotorStatus::SharedPtr msg) {
  static double last_position;
  last_position = msg->position;
  this->drillHeight += (msg->position - last_position);
}

void ScienceMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handlePlatform(joystickMsg);
  handleDrill(joystickMsg);
  handleMicroscope(joystickMsg);
  handleSoilCollection(joystickMsg);
  // handlePanoramic(joystickMsg);
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
  bool drill_value = joystickMsg->buttons[kDrillButton];
  if (!this->autoDrill) {
    ros_phoenix::msg::MotorControl drill_control;
    drill_control.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    if (drill_value) {
      drill_control.value = 1.0;
    } else {
      drill_control.value = 0.0;
    }
    drill_pub_->publish(drill_control);
  }
}

void ScienceMode::handleMicroscope(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // Process input and output linear component
  static double position;
  double value = joystickMsg->axes[kMicroscopeAxis];
  if (value != 0) {
    position += value;
    setServoPosition(kMicroscopeServo, position);
  }
}

void ScienceMode::handlePanoramic(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // TODO
}

void ScienceMode::handleSoilCollection(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  // This is to handle automatically drilling down to 10cm and collecting soil
  if (joystickMsg->buttons[kAutoDrillButton] && !this->autoDrill) {
    this->drillHeight = 0;
    this->autoDrill = true;
    autoDrillTimer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ScienceMode::auto_drill_callback, this));
  } else if (joystickMsg->buttons[kCancelCollectionButton] && this->autoDrill) {
    this->autoDrill = false;
    autoDrillTimer_.reset();
  }
}

void ScienceMode::setServoPosition(int port, int position) const {
  auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
  request->port = port;
  request->pos = position;

  // Wait for the service to be available
  if (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available after waiting");
    return;
  }

  servo_client_->async_send_request(request);
}

void ScienceMode::declareParameters(rclcpp::Node* node) {
  node->declare_parameter("science_mode.platform_axis", 1);
  node->declare_parameter("science_mode.drill_button", 2);
  node->declare_parameter("science_mode.microscope_axis", 3);
  node->declare_parameter("science_mode.soil_collection_button", 4);
  node->declare_parameter("science_mode.cancel_collection_button", 5);
  node->declare_parameter("science_mode.panoramic_button", 6);
  node->declare_parameter("science_mode.collection_servo", 0);
  node->declare_parameter("science_mode.microscope_servo", 1);
  node->declare_parameter("science_mode.collection_open", 0);
  node->declare_parameter("science_mode.collection_close", 90);
}

void ScienceMode::loadParameters() {
  node_->get_parameter("science_mode.platform_axis", kPlatformAxis);
  node_->get_parameter("science_mode.drill_button", kDrillButton);
  node_->get_parameter("science_mode.microscope_axis", kMicroscopeAxis);
  node_->get_parameter("science_mode.soil_collection_button", kAutoDrillButton);
  node_->get_parameter("science_mode.cancel_collection_button",
                       kCancelCollectionButton);
  node_->get_parameter("science_mode.panoramic_button", kPanoramicButton);
  node_->get_parameter("science_mode.collection_servo", kCollectionServo);
  node_->get_parameter("science_mode.microscope_servo", kMicroscopeServo);
  node_->get_parameter("science_mode.collection_open", kCollectionOpen);
  node_->get_parameter("science_mode.collection_close", kCollectionClose);
}
