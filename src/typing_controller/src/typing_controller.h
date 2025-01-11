#ifndef TYPING_CONTROLLER_H
#define TYPING_CONTROLLER

#include <memory>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/arm_cmd.hpp"
#include <moveit/robot_state/robot_state.h>
//#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

bool isEmpty(const geometry_msgs::msg::Pose &p);

class TypingNode : public rclcpp::Node
{
  public:
    TypingNode();

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<moveit_msgs::action::ExecuteTrajectory_FeedbackMessage>::SharedPtr subscription2_;
    void topic_callback(const std_msgs::msg::String &msg);
    void arm_callback(const moveit_msgs::action::ExecuteTrajectory_FeedbackMessage &msg);
    //moveit_rviz_plugin::MotionPlanningDisplay motionPlanningDisplay;
};

#endif
