#ifndef JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_
#define JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_

#include <action_msgs/msg/goal_status.hpp>

#include "Mode.hpp"
#include "MoveGroupAsyncWrapper.hpp"
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
  ArmPresetMode(rclcpp::Node* node);

  /**
   * @brief Process joystick input to move to a preset pose if a button is
   * pressed.
   */
  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

  /**
   * @brief Declare required parameters for preset poses.
   */
  static void declareParameters(rclcpp::Node* node);

 private:
  struct PresetPose {
    std::string name;               ///< Name of the preset.
    int button;                     ///< Button index for the preset.
    geometry_msgs::msg::Pose pose;  ///< The pose to move to.
  };
  std::vector<PresetPose> presets_;  ///< List of preset poses.
  rclcpp::Node* node_;
  std::shared_ptr<MoveGroupAsyncWrapper>
      move_group_;          ///< Move group interface for arm control.
  std::string group_name_;  ///< The planning group name.

  int preset_1_button_;
  int preset_2_button_;
  int preset_3_button_;
  int preset_4_button_;
  int cancel_button_;

  /**
   * @brief Load preset parameters from the ROS parameter server.
   */
  void loadParameters();
};

#endif  // JOYSTICK_CONTROL__ARMPRESET_MODE_HPP_
