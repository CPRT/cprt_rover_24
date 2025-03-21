#include "DriveMode.hpp"

bool DriveMode::initalized_ = false;

DriveMode::DriveMode(rclcpp::Node* node) : Mode("Drive", node) {
  RCLCPP_INFO(node_->get_logger(), "Drive Mode");
  declare_parameters();
  twist_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  initalized_ = true;
}

void DriveMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
  handleCam(joystickMsg);
  handleVideo(joystickMsg);
}

double DriveMode::getThrottleValue(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  if (kThrottleAxis != -1) {
    double throttle = joystickMsg->axes[kThrottleAxis];
    throttle = std::max(kThrottleMin, std::min(kThrottleMax, throttle));
    // Normalize the throttle value to be between 0 and 1
    return (throttle - kThrottleMin) / (kThrottleMax - kThrottleMin);
  }
  return 1.0;
}

void DriveMode::handleTwist(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  static double last_forward = 0;
  double forward, yaw, throttle;
  throttle = getThrottleValue(joystickMsg);
  if (joystickMsg->buttons[kCruiseControl] == 1) {
    forward = last_forward;
  } else {
    forward = joystickMsg->axes[kForwardAxis] * throttle * kMaxLinear;
  }
  yaw = joystickMsg->axes[kYawAxis] * throttle * kMaxAngular;
  if (std::abs(forward) < kMinSpeed) {
    forward = 0;
  }
  if (std::abs(yaw) < kMinSpeed) {
    yaw = 0;
  }
  if (forward >= last_forward) {
    forward = std::min(forward, last_forward + kMaxIncrement);
  } else {
    forward = std::max(forward, last_forward - kMaxIncrement);
  }
  last_forward = forward;
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = forward;
  twist.angular.z = yaw;
  twist_pub_->publish(twist);
}

void DriveMode::handleCam(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // TODO: Implement camera control
}

void DriveMode::handleVideo(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // TODO: Implement stream control
}

void DriveMode::declare_parameters() {
  if (!initalized_) {
    node_->declare_parameter("drive_mode.forward_axis", 1);
    node_->declare_parameter("drive_mode.yaw_axis", 2);
    node_->declare_parameter("drive_mode.cam_tilt_axis", 3);
    node_->declare_parameter("drive_mode.cam_pan_axis", 4);
    node_->declare_parameter("drive_mode.cam_reset", 5);
    node_->declare_parameter("drive_mode.cam_next", 6);
    node_->declare_parameter("drive_mode.cam_prev", 7);
    node_->declare_parameter("drive_mode.cruise_control", 8);
    node_->declare_parameter("drive_mode.max_linear", 2.0);
    node_->declare_parameter("drive_mode.max_angular", 2.0);
    node_->declare_parameter("drive_mode.max_increment", 0.1);
    node_->declare_parameter("drive_mode.min_speed", 0.1);
    node_->declare_parameter("drive_mode.throttle.axis", 0);
    node_->declare_parameter("drive_mode.throttle.max", 1.0);
    node_->declare_parameter("drive_mode.throttle.min", -1.0);
  }

  node_->get_parameter("drive_mode.forward_axis", kForwardAxis);
  node_->get_parameter("drive_mode.yaw_axis", kYawAxis);
  node_->get_parameter("drive_mode.cam_tilt_axis", kCamTiltAxis);
  node_->get_parameter("drive_mode.cam_pan_axis", kCamPanAxis);
  node_->get_parameter("drive_mode.cam_reset", kCamReset);
  node_->get_parameter("drive_mode.cam_next", kCamNext);
  node_->get_parameter("drive_mode.cam_prev", kCamPrev);
  node_->get_parameter("drive_mode.cruise_control", kCruiseControl);
  node_->get_parameter("drive_mode.max_linear", kMaxLinear);
  node_->get_parameter("drive_mode.max_angular", kMaxAngular);
  node_->get_parameter("drive_mode.max_increment", kMaxIncrement);
  node_->get_parameter("drive_mode.min_speed", kMinSpeed);
  node_->get_parameter("drive_mode.throttle.axis", kThrottleAxis);
  node_->get_parameter("drive_mode.throttle.max", kThrottleMax);
  node_->get_parameter("drive_mode.throttle.min", kThrottleMin);
}