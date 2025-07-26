#include "ArmPresetMode.hpp"

ArmPresetMode::ArmPresetMode(rclcpp::Node* node)
    : Mode("ArmPresetMode", node), node_(node) {
  RCLCPP_INFO(node_->get_logger(), "Preset Arm Mode Initialized");

  declareParameters(node_);
  loadParameters();
  move_group_ = std::make_shared<MoveGroupAsyncWrapper>(node_, group_name_);
}

void ArmPresetMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  const size_t num_buttons = joystickMsg->buttons.size();
  if (joystickMsg->buttons[cancel_button_] == 1) {
    RCLCPP_INFO(node_->get_logger(), "Canceling current motion");
    move_group_->cancelGoal();
    return;
  }
  for (const auto& preset : presets_) {
    if (preset.button < 0 || preset.button >= num_buttons) {
      RCLCPP_WARN(node_->get_logger(),
                  "Preset button index %d out of range, skipping preset '%s'",
                  preset.button, preset.name.c_str());
      continue;
    }
    if (joystickMsg->buttons[preset.button] == 1) {
      RCLCPP_INFO(node_->get_logger(), "Moving to preset '%s'",
                  preset.name.c_str());
      if (move_group_->isRunning()) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Move group is already running. Ignoring input for preset '%s'",
            preset.name.c_str());
        return;
      }
      move_group_->moveToPose(preset.pose);
      return;
    }
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
  node->declare_parameter<std::vector<std::string>>("presets",
                                                    std::vector<std::string>());
  node->declare_parameter<std::string>("planning_group", "rover_arm");

  std::vector<std::string> preset_names =
      node->get_parameter("presets").as_string_array();
  if (preset_names.empty()) {
    RCLCPP_WARN(node->get_logger(),
                "No presets defined. No actions will be taken.");
    return;
  }
  for (const auto& name : preset_names) {
    declare_pose(name, 0);
  }
  node->declare_parameter<int>("cancel_button", 0);
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
  std::vector<std::string> preset_names =
      node_->get_parameter("presets").as_string_array();
  if (preset_names.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "No presets defined. No actions will be taken.");
    return;
  }
  for (const auto& name : preset_names) {
    PresetPose preset;
    preset.name = name;
    preset.button = 0;
    load_pose(name, preset.button, preset.pose);
    presets_.push_back(preset);
    RCLCPP_INFO(node_->get_logger(), "Loaded preset '%s' with button %d",
                name.c_str(), preset.button);
  }
  node_->get_parameter("cancel_button", cancel_button_);
  node_->get_parameter("planning_group", group_name_);
}
