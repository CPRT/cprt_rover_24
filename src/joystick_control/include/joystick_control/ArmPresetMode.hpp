#ifndef JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_
#define JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_

#include "Mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

/**
 * @class ArmPresetMode
 * @brief A mode that uses joystick input to move the robotic arm to named preset poses.
 */
class ArmPresetMode : public Mode {
 public:
  ArmPresetMode(rclcpp::Node* node, const std::string& planning_group);

  void processJoystickInput(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

  static void declareParameters(rclcpp::Node* node);

 private:
  moveit::planning_interface::MoveGroupInterface move_group_;

  // Button bindings
  int preset_1_button_;
  int preset_2_button_;
  int preset_3_button_;
  int preset_4_button_;

  // Poses
  geometry_msgs::msg::Pose preset_1_pose_;
  geometry_msgs::msg::Pose preset_2_pose_;
  geometry_msgs::msg::Pose preset_3_pose_;
  geometry_msgs::msg::Pose preset_4_pose_;

  void loadParameters();
  bool moveToPose(const geometry_msgs::msg::Pose& target_pose);
};

#endif  // JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_
