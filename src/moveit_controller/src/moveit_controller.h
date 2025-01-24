#ifndef MOVEIT_CONTROLLER_H
#define MOVEIT_CONTROLLER_H

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
  rclcpp::Subscription<interfaces::msg::ArmCmd>::SharedPtr subscription;

  std::string node_name;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
  rclcpp::Node::SharedPtr node_ptr;
  rclcpp::Executor::SharedPtr executor_ptr;
  std::thread executor_thread;
  std::thread th;
  moveit_msgs::msg::RobotTrajectory trajectory;
  geometry_msgs::msg::Pose default_pose;

  moveit::planning_interface::MoveGroupInterface::Plan rotationPlan;

  void topic_callback(const interfaces::msg::ArmCmd &armMsg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
};

#endif
