#include "ArmPresetMode.hpp"

ArmPresetMode::ArmPresetMode(rclcpp::Node* node,
                             const std::string& planning_group)
    : Mode("ArmPresetMode", node),
      move_group_(node->shared_from_this(), planning_group),
      node_(node) {
  RCLCPP_INFO(node_->get_logger(), "Preset Arm Mode Initialized");

  declareParameters(node_);
  loadParameters();
  move_group_.setMaxAccelerationScalingFactor(1.0);
  move_group_.setMaxVelocityScalingFactor(1.0);
  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose("Link_6");
  RCLCPP_INFO(node_->get_logger(), "Current link position: x=%f, y=%f, z=%f", 
            current_pose.pose.position.x, 
            current_pose.pose.position.y, 
            current_pose.pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "Current link orientation: x=%f, y=%f, z=%f, w=%f", 
            current_pose.pose.orientation.x, 
            current_pose.pose.orientation.y, 
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);

  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose("Joint_6");
  RCLCPP_INFO(node_->get_logger(), "Current joint position: x=%f, y=%f, z=%f", 
            current_pose.pose.position.x, 
            current_pose.pose.position.y, 
            current_pose.pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "Current joint orientation: x=%f, y=%f, z=%f, w=%f", 
            current_pose.pose.orientation.x, 
            current_pose.pose.orientation.y, 
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);

  geometry_msgs::msg::PoseStamped current_pose = move_group_.getCurrentPose("eef_link");
  RCLCPP_INFO(node_->get_logger(), "Current eef position: x=%f, y=%f, z=%f", 
            current_pose.pose.position.x, 
            current_pose.pose.position.y, 
            current_pose.pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "Current eef orientation: x=%f, y=%f, z=%f, w=%f", 
            current_pose.pose.orientation.x, 
            current_pose.pose.orientation.y, 
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
}

void ArmPresetMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (joystickMsg->buttons.size() == 0) return;

  geometry_msgs::msg::PoseStamped stamped_pose;
  stamped_pose.header.stamp = node_->get_clock()->now();
  stamped_pose.header.frame_id = "eef_link";

  if (joystickMsg->buttons[preset_1_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_1");
    stamped_pose.pose = preset_1_pose_;
    moveToPose(stamped_pose);
  } else if (joystickMsg->buttons[preset_2_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_2");
    stamped_pose.pose = preset_2_pose_;
    moveToPose(stamped_pose);
  } else if (joystickMsg->buttons[preset_3_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_3");
    stamped_pose.pose = preset_3_pose_;
    moveToPose(stamped_pose);
  } else if (joystickMsg->buttons[preset_4_button_]) {
    RCLCPP_INFO(node_->get_logger(), "Moving to preset_4");
    stamped_pose.pose = preset_4_pose_;
    moveToPose(stamped_pose);
  }
}

bool ArmPresetMode::moveToPose(const geometry_msgs::msg::PoseStamped& target_pose) {
  move_group_.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
      (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(node_->get_logger(), "Plan successful. Executing...");
    move_group_.move();
    return true;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Planning failed.");
    return false;
  }
}

void ArmPresetMode::declareParameters(rclcpp::Node* node) {
  auto declare_pose = [node](const std::string& name, int default_button) {
    std::string base = "preset_mode." + name;
    node->declare_parameter<int>(base + ".button", default_button);

    std::string pose_base = base + ".pose";
    node->declare_parameter<double>(pose_base + ".x", 0.0);
    node->declare_parameter<double>(pose_base + ".y", 0.0);
    node->declare_parameter<double>(pose_base + ".z", 0.0);
    node->declare_parameter<double>(pose_base + ".qx", 0.0);
    node->declare_parameter<double>(pose_base + ".qy", 0.0);
    node->declare_parameter<double>(pose_base + ".qz", 0.0);
    node->declare_parameter<double>(pose_base + ".qw", 1.0);
  };

  declare_pose("preset_1", 3);
  declare_pose("preset_2", 4);
  declare_pose("preset_3", 5);
  declare_pose("preset_4", 6);
}

void ArmPresetMode::loadParameters() {
  auto load_pose = [this](const std::string& name, int& button,
                          geometry_msgs::msg::Pose& pose) {
    std::string base = "preset_mode." + name;
    std::string pose_base = base + ".pose";

    node_->get_parameter(base + ".button", button);
    node_->get_parameter(pose_base + ".x", pose.position.x);
    node_->get_parameter(pose_base + ".y", pose.position.y);
    node_->get_parameter(pose_base + ".z", pose.position.z);
    node_->get_parameter(pose_base + ".qx", pose.orientation.x);
    node_->get_parameter(pose_base + ".qy", pose.orientation.y);
    node_->get_parameter(pose_base + ".qz", pose.orientation.z);
    node_->get_parameter(pose_base + ".qw", pose.orientation.w);
  };

  load_pose("preset_1", preset_1_button_, preset_1_pose_);
  load_pose("preset_2", preset_2_button_, preset_2_pose_);
  load_pose("preset_3", preset_3_button_, preset_3_pose_);
  load_pose("preset_4", preset_4_button_, preset_4_pose_);
}
