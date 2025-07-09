#ifndef JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_
#define JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_

#include "Mode.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

/**
 * @class ArmPresetMode
 * @brief A mode that uses joystick input to move the robotic arm to named
 * preset poses.
 */
class ArmPresetMode : public Mode {
 public:
  /**
   * @brief Construct the ArmPresetMode.
   * @param node A shared pointer to the ROS node.
   * @param planning_group The MoveIt planning group name (e.g., "arm").
   */
  ArmPresetMode(rclcpp::Node* node, const std::string& planning_group);

  /**
   * @brief Process joystick input to move to a preset pose if a button is pressed.
   */
  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

  /**
   * @brief Declare required parameters for preset poses.
   */
  static void declareParameters(rclcpp::Node* node);

 private:
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Node* node_;  // store for access in member functions

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

  /**
   * @brief Load preset parameters from the ROS parameter server.
   */
  void loadParameters();

  /**
   * @brief Plan and execute motion to the given pose.
   * @param target_pose The target pose to move to.
   * @return True if planning and execution succeeded.
   */
  bool moveToPose(const geometry_msgs::msg::Pose& target_pose);
};

#endif  // JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_
