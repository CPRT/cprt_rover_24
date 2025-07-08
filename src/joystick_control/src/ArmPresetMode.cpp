#include "ArmPresetMode.hpp"

ArmPresetMode::ArmPresetMode(rclcpp::Node* node, const std::string& planning_group) : Mode("ArmPresetMode", node), move_group_(node->shared_from_this(), planning_group)
{
  RCLCPP_INFO(node_->get_logger(), "Preset Arm Mode Initialized");

  declareParameters(node_);
  loadParameters();
}


void ArmPresetMode::processJoystickInput(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg)
{
  if (joystickMsg->buttons[preset_1_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_1");
    moveToPose(preset_1_pose_);
  } else if (joystickMsg->buttons[preset_2_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_2");
    moveToPose(preset_2_pose_);
  } else if (joystickMsg->buttons[preset_3_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_3");
    moveToPose(preset_3_pose_);
  } else if (joystickMsg->buttons[preset_4_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_4");
    moveToPose(preset_4_pose_);
  }
}

bool ArmPresetMode::moveToPose(const geometry_msgs::msg::Pose& target_pose)
{
  move_group_.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(node_->get_logger(), "Plan successful. Executing...");
    move_group_.move();
    return true;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Planning failed.");
    return false;
  }
}

void ArmPresetMode::declareParameters(rclcpp::Node* node)
{
  node->declare_parameter("preset_mode.preset_1_button", 0);
  node->declare_parameter("preset_mode.preset_2_button", 1);
  node->declare_parameter("preset_mode.preset_3_button", 2);
  node->declare_parameter("preset_mode.preset_4_button", 3);

  node->declare_parameter("preset_mode.preset_1.x", 0.0);
  node->declare_parameter("preset_mode.preset_1.y", 0.0);
  node->declare_parameter("preset_mode.preset_1.z", 0.0);

  node->declare_parameter("preset_mode.preset_2.x", 0.0);
  node->declare_parameter("preset_mode.preset_2.y", 0.0);
  node->declare_parameter("preset_mode.preset_2.z", 0.0);

  node->declare_parameter("preset_mode.preset_3.x", 0.0);
  node->declare_parameter("preset_mode.preset_3.y", 0.0);
  node->declare_parameter("preset_mode.preset_3.z", 0.0);

  node->declare_parameter("preset_mode.preset_4.x", 0.0);
  node->declare_parameter("preset_mode.preset_4.y", 0.0);
  node->declare_parameter("preset_mode.preset_4.z", 0.0);
}

void ArmPresetMode::loadParameters()
{
  node_->get_parameter("preset_mode.preset_1_button", preset_1_button_);
  node_->get_parameter("preset_mode.preset_2_button", preset_2_button_);
  node_->get_parameter("preset_mode.preset_3_button", preset_3_button_);
  node_->get_parameter("preset_mode.preset_4_button", preset_4_button_);

  node_->get_parameter("preset_mode.preset_1.x", preset_1_pose_.position.x);
  node_->get_parameter("preset_mode.preset_1.y", preset_1_pose_.position.y);
  node_->get_parameter("preset_mode.preset_1.z", preset_1_pose_.position.z);
  preset_1_pose_.orientation.w = 1.0;

  node_->get_parameter("preset_mode.preset_2.x", preset_2_pose_.position.x);
  node_->get_parameter("preset_mode.preset_2.y", preset_2_pose_.position.y);
  node_->get_parameter("preset_mode.preset_2.z", preset_2_pose_.position.z);
  preset_2_pose_.orientation.w = 1.0;

  node_->get_parameter("preset_mode.preset_3.x", preset_3_pose_.position.x);
  node_->get_parameter("preset_mode.preset_3.y", preset_3_pose_.position.y);
  node_->get_parameter("preset_mode.preset_3.z", preset_3_pose_.position.z);
  preset_3_pose_.orientation.w = 1.0;

  node_->get_parameter("preset_mode.preset_4.x", preset_4_pose_.position.x);
  node_->get_parameter("preset_mode.preset_4.y", preset_4_pose_.position.y);
  node_->get_parameter("preset_mode.preset_4.z", preset_4_pose_.position.z);
  preset_4_pose_.orientation.w = 1.0;
}