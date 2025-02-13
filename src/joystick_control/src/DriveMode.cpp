#include "DriveMode.hpp"

DriveMode::DriveMode(rclcpp::Node* node) : Mode("Drive", node) {
  RCLCPP_INFO(node_->get_logger(), "Drive Mode");
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/drive/cmd_vel", 10);
}

void DriveMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
}

void DriveMode::handleTwist(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  static double last_forward = 0;
  double throttle = (joystickMsg->axes[kThrottleAxis] + 1.0) / 2.0;
  double forward, yaw;

  if (joystickMsg->buttons[kCruiseControl]) {
    forward = last_forward;
  } else {
    forward = joystickMsg->axes[kForwardAxis] * throttle * kMaxLinear;
  }
  yaw = joystickMsg->axes[kYawAxis] * throttle * kMaxAngular;
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

void DriveMode::handleCam(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  // TODO: Implement camera control
}

void DriveMode::handleVideo(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  // TODO: Implement stream control
}

void DriveMode::declare_parameters() {
  node_->declare_parameter("drive_mode.throttle_axis", 0);
  node_->declare_parameter("drive_mode.forward_axis", 1);
  node_->declare_parameter("drive_mode.yaw_axis", 2);
  node_->declare_parameter("drive_mode.cam_tilt_axis", 3);
  node_->declare_parameter("drive_mode.cam_pan_axis", 4);
  node_->declare_parameter("drive_mode.cam_reset", 5);
  node_->declare_parameter("drive_mode.cam_next", 6);
  node_->declare_parameter("drive_mode.cam_prev", 7);
  node_->declare_parameter("drive_mode.cruise_control", 8);
  node_->declare_parameter("drive_mode.max_linear", 2);
  node_->declare_parameter("drive_mode.max_angular", 2);
  node_->declare_parameter("drive_mode.max_increment", 0.1);

  node_->get_parameter("drive_mode.throttle_axis", kThrottleAxis);
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
}