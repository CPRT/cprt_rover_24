#include "ArmManualMode.hpp"

ArmManualMode::ArmManualMode(rclcpp::Node* node) : Mode("Manual Arm", node){
    RCLCPP_INFO(node_->get_logger(), "Drive Mode");
    loadParameters();
    twist_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    base_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>("/base/set", 10);
    diff1_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>("/diff1/set", 10);
    diff2_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>("/diff2/set", 10);
    elbow_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>("/elbow/set", 10);
    wristTilt_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>("/wristTilt/set", 10);
    wristTurn_pub_ = node_->create_publisher<ros_phoenix::msg::MotorControl>("/wristTurn/set", 10);

    base_ = std::make_shared<ros_phoenix::msg::MotorControl>();
    diff1_ = std::make_shared<ros_phoenix::msg::MotorControl>();
    diff2_ = std::make_shared<ros_phoenix::msg::MotorControl>();
    elbow_ = std::make_shared<ros_phoenix::msg::MotorControl>();
    wristTilt_ = std::make_shared<ros_phoenix::msg::MotorControl>();
    wristTurn_ = std::make_shared<ros_phoenix::msg::MotorControl>();
}

double ArmManualMode::getThrottleValue(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
    if (kThrottleAxis != -1) {
        double throttle = joystickMsg->axes[kThrottleAxis];
        throttle = std::max(kThrottleMin, std::min(kThrottleMax, throttle));
        // Normalize the throttle value to be between 0.25 and 1 
        return 0.25 + ((throttle - kThrottleMin) / (kThrottleMax - kThrottleMin) * 0.75);
    }
    return 1.0;
}

void ArmManualMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
}

void ArmManualMode::handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
    double throttle = getThrottleValue(joystickMsg);

    base_->mode = 0;
    diff1_->mode = 0;
    diff2_->mode = 0;
    elbow_->mode = 0;
    wristTilt_->mode = 0;
    wristTurn_->mode = 0;

    if (joystickMsg->axes[kBaseAxis] > 0.1){
        base_->value = -joystickMsg->axes[kBaseAxis] * throttle;
    }
    else if (joystickMsg->axes[kBaseAxis] < -0.1){
        base_->value = -joystickMsg->axes[kBaseAxis] * throttle;
    }
    
    base_pub_->publish(*base_);
}

void ArmManualMode::declareParameters(rclcpp::Node* node){
    node->declare_parameter("arm_manual_mode.base_axis", 0);
    node->declare_parameter("arm_manual_mode.wrist_roll", 1);
    node->declare_parameter("arm_manual_mode.wrist_yaw_positive", 2);
    node->declare_parameter("arm_manual_mode.wrist_yaw_negative", 3);
    node->declare_parameter("arm_manual_mode.diff1_axis", 4);
    node->declare_parameter("arm_manual_mode.diff2_axis", 5);
    node->declare_parameter("arm_manual_mode.elbow_axis", 6);
    node->declare_parameter("arm_manual_mode.throttle.axis", 7);
    node->declare_parameter("arm_manual_mode.throttle.min", -1.0);
    node->declare_parameter("arm_manual_mode.throttle.max", 1.0);
}

void ArmManualMode::loadParameters(){
    node_->get_parameter("arm_manual_mode.base_axis", kBaseAxis);
    node_->get_parameter("arm_manual_mode.wrist_roll", kWristRoll);
    node_->get_parameter("arm_manual_mode.wrist_yaw_positive", kWristYawPositive);
    node_->get_parameter("arm_manual_mode.wrist_yaw_negative", kWristYawNegative);
    node_->get_parameter("arm_manual_mode.diff1_axis", kDiff1Axis);
    node_->get_parameter("arm_manual_mode.diff2_axis", kDiff2Axis);
    node_->get_parameter("arm_manual_mode.elbow_axis", kElbowYaw);
    node_->get_parameter("arm_manual_mode.throttle.axis", kThrottleAxis);
    node_->get_parameter("arm_manual_mode.throttle.min", kThrottleMin);
    node_->get_parameter("arm_manual_mode.throttle.max", kThrottleMax);
}