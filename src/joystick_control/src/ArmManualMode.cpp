#include "ArmManualMode.hpp"

ArmManualMode::ArmManualMode(rclcpp::Node* node) : Mode("Manual Arm", node) {
  RCLCPP_INFO(node_->get_logger(), "Arm Manual Mode");
  loadParameters();
  twist_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  base_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/base/set", 10);
  diff1_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/diff1/set", 10);
  diff2_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/diff2/set", 10);
  elbow_pub_ =
      node_->create_publisher<ros_phoenix::msg::MotorControl>("/elbow/set", 10);
  wristTilt_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/wristTilt/set", 10);
  wristTurn_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>(
      "/wristTurn/set", 10);
  servo_client_ =
      node_->create_client<interfaces::srv::MoveServo>("servo_service");

  kServoPort = 0;
  kServoMin = 0;
  kServoMax = 180;
  kClawMax = 62;
  kClawMin = 8;
  servoPos = kClawMax;
  servoRequest(kServoPort, servoPos, kServoMin, kServoMax);
  buttonPressed = false;
}

double ArmManualMode::getThrottleValue(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  if (kThrottleAxis != -1) {
    double throttle = joystickMsg->axes[kThrottleAxis];
    throttle = std::max(kThrottleMin, std::min(kThrottleMax, throttle));
    // Normalize the throttle value to be between 0.2 and 1.0
    return 0.2 +
           ((throttle - kThrottleMin) / (kThrottleMax - kThrottleMin) * 0.80);
  }
  return 1.0;
}

void ArmManualMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
}

void ArmManualMode::handleTwist(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  double throttle = getThrottleValue(joystickMsg);

  base_.mode = 0;
  diff1_.mode = 0;
  diff2_.mode = 0;
  elbow_.mode = 0;
  wristTilt_.mode = 0;
  wristTurn_.mode = 0;

  // Base (might want a deadzone. TBD)
  base_.value = -joystickMsg->axes[kBaseAxis] * throttle;

  // Simple straight movement (NOT inverse kin)
  if (joystickMsg->buttons[kSimpleForward] == 1) {
    diff1_.value = -joystickMsg->buttons[kSimpleForward] * 0.75 * throttle; // some scaling to move in a straight line 
    diff2_.value = -joystickMsg->buttons[kSimpleForward] * throttle;
  } else if (joystickMsg->buttons[kSimpleBackward] == 1) {
    diff1_.value = joystickMsg->buttons[kSimpleBackward] * 0.75 * throttle; 
    diff2_.value = joystickMsg->buttons[kSimpleBackward] * 0.80 * throttle;
  } else {
    // Diff1
    diff1_.value = -joystickMsg->axes[kDiff1Axis] * throttle;

    // Diff2
    diff2_.value = -joystickMsg->axes[kDiff2Axis] * throttle;
  }

  // Elbow
  if (joystickMsg->axes[kElbowYaw] < 0.2 &&
      joystickMsg->axes[kElbowYaw] >
          -0.2) {  // Deadzone, easy to turn this one by accident.
    elbow_.value = 0;
  } else {
    elbow_.value = joystickMsg->axes[kElbowYaw] * throttle;
  }

  // Wrist Tilt
  wristTilt_.value = (joystickMsg->buttons[kWristYawPositive] -
                      joystickMsg->buttons[kWristYawNegative]) *
                     throttle;

  // Wrist Turn
  wristTurn_.value = -joystickMsg->axes[kWristRoll] * throttle;

  // Gripper. Will cycle between open, half open, and close on button release.
  if (joystickMsg->buttons[kClawOpen] == 1 && !buttonPressed) {
    if (servoPos + ((kClawMax - kClawMin) / 2) < kClawMax + 1) {
      buttonPressed = true;
      servoPos = servoPos + ((kClawMax - kClawMin) / 2);
      servoRequest(kServoPort, servoPos, kClawMin, kClawMax);
    } else {
      buttonPressed = true;
      RCLCPP_INFO(node_->get_logger(), "Max Open");
      RCLCPP_INFO(node_->get_logger(), "%d", servoPos);
    }
  } else if (joystickMsg->buttons[kClawClose] == 1 && !buttonPressed) {
    if (servoPos - ((kClawMax - kClawMin) / 2) > kClawMin - 1) {
      buttonPressed = true;
      servoPos = servoPos - ((kClawMax - kClawMin) / 2);
      servoRequest(kServoPort, servoPos, kClawMin, kClawMax);
    } else {
      buttonPressed = true;
      RCLCPP_INFO(node_->get_logger(), "Max Close");
      RCLCPP_INFO(node_->get_logger(), "%d", servoPos);
    }
  } else if ((joystickMsg->buttons[kClawClose] == 0) &&
             (joystickMsg->buttons[kClawOpen] == 0)) {
    buttonPressed = false;
  }

  base_pub_->publish(base_);
  diff1_pub_->publish(diff1_);
  diff2_pub_->publish(diff2_);
  elbow_pub_->publish(elbow_);
  wristTilt_pub_->publish(wristTilt_);
  wristTurn_pub_->publish(wristTurn_);
}

void ArmManualMode::declareParameters(rclcpp::Node* node) {
  node->declare_parameter("arm_manual_mode.base_axis", 0);
  node->declare_parameter("arm_manual_mode.wrist_roll", 1);
  node->declare_parameter("arm_manual_mode.wrist_yaw_positive", 2);
  node->declare_parameter("arm_manual_mode.wrist_yaw_negative", 3);
  node->declare_parameter("arm_manual_mode.diff1_axis", 4);
  node->declare_parameter("arm_manual_mode.diff2_axis", 5);
  node->declare_parameter("arm_manual_mode.elbow_yaw", 6);
  node->declare_parameter("arm_manual_mode.claw_open", 8);
  node->declare_parameter("arm_manual_mode.claw_close", 9);
  node->declare_parameter("arm_manual_mode.simple_forward", 10);
  node->declare_parameter("arm_manual_mode.simple_backward", 11);
  node->declare_parameter("arm_manual_mode.throttle.axis", 7);
  node->declare_parameter("arm_manual_mode.throttle.min", -1.0);
  node->declare_parameter("arm_manual_mode.throttle.max", 1.0);
}

void ArmManualMode::loadParameters() {
  node_->get_parameter("arm_manual_mode.base_axis", kBaseAxis);
  node_->get_parameter("arm_manual_mode.wrist_roll", kWristRoll);
  node_->get_parameter("arm_manual_mode.wrist_yaw_positive", kWristYawPositive);
  node_->get_parameter("arm_manual_mode.wrist_yaw_negative", kWristYawNegative);
  node_->get_parameter("arm_manual_mode.diff1_axis", kDiff1Axis);
  node_->get_parameter("arm_manual_mode.diff2_axis", kDiff2Axis);
  node_->get_parameter("arm_manual_mode.elbow_yaw", kElbowYaw);
  node_->get_parameter("arm_manual_mode.claw_open", kClawOpen);
  node_->get_parameter("arm_manual_mode.claw_close", kClawClose);
  node_->get_parameter("arm_manual_mode.simple_forward", kSimpleForward);
  node_->get_parameter("arm_manual_mode.simple_backward", kSimpleBackward);
  node_->get_parameter("arm_manual_mode.throttle.axis", kThrottleAxis);
  node_->get_parameter("arm_manual_mode.throttle.max", kThrottleMax);
  node_->get_parameter("arm_manual_mode.throttle.min", kThrottleMin);
}

interfaces::srv::MoveServo::Response ArmManualMode::sendRequest(int port,
                                                                int pos,
                                                                int min,
                                                                int max) const {
  auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
  request->port = port;
  request->pos = pos;
  request->min = min;
  request->max = max;

  // Wait for the service to be available
  if (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available after waiting");
    return interfaces::srv::MoveServo::Response();
  }

  auto future = servo_client_->async_send_request(request);

  // Wait for the result (with timeout)
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed");
    return interfaces::srv::MoveServo::Response();
  }

  return *future.get();
}

void ArmManualMode::servoRequest(int req_port, int req_pos, int req_min,
                                 int req_max) const {
  auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
  request->port = req_port;
  request->pos = req_pos;
  request->min = req_min;
  request->max = req_max;

  if (!servo_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Service not available");
    return;
  }

  // Simple callback that just logs errors
  auto callback =
      [this](rclcpp::Client<interfaces::srv::MoveServo>::SharedFuture future) {
        try {
          auto response = future.get();
          if (!response->status) {
            RCLCPP_ERROR(node_->get_logger(), "Servo move failed");
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s",
                       e.what());
        }
      };

  servo_client_->async_send_request(request, callback);
}