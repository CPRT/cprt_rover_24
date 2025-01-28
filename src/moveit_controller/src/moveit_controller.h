#ifndef MOVEIT_CONTROLLER_H
#define MOVEIT_CONTROLLER_H

#define ARM_DEFAULT_X 0.636922
#define ARM_DEFAULT_Y 0.064768
#define ARM_DEFAULT_Z 0.678810

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "interfaces/msg/arm_cmd.hpp"
#include "std_msgs/msg/string.hpp"

void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj,
                       moveit::planning_interface::MoveGroupInterfacePtr mgi);

void executePlan(
    moveit::planning_interface::MoveGroupInterface::Plan &rotationPlan,
    moveit::planning_interface::MoveGroupInterfacePtr mgi);

bool isEmpty(const geometry_msgs::msg::Pose &p);

class MoveitController : public rclcpp::Node {
 public:
  MoveitController(const rclcpp::NodeOptions &options);

 private:
  static constexpr double JUMP_THRESHOLD = 0;
  static constexpr double EEF_STEP = 0.01;
  rclcpp::Subscription<interfaces::msg::ArmCmd>::SharedPtr subscription;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
  rclcpp::Node::SharedPtr node_ptr;
  rclcpp::Executor::SharedPtr executor_ptr;
  std::thread executor_thread;
  std::thread move_it_thread;
  moveit_msgs::msg::RobotTrajectory trajectory;
  geometry_msgs::msg::Pose default_pose;

  moveit::planning_interface::MoveGroupInterface::Plan rotationPlan;

  void topic_callback(const interfaces::msg::ArmCmd &armMsg);
};

#endif
