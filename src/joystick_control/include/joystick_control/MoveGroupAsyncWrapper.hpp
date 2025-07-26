#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

class MoveGroupAsyncWrapper {
 public:
  MoveGroupAsyncWrapper(rclcpp::Node* node,
                        const std::string& group_name = "rover_arm");
  ~MoveGroupAsyncWrapper();

  void moveToPose(const geometry_msgs::msg::Pose& pose);
  void cancelGoal();
  bool isRunning();

 private:
  void statusMonitorCallback();

  rclcpp::Node* node_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr client_;
  std::shared_ptr<
      rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>>
      goal_handle_;

  std::mutex mutex_;
  std::atomic<bool> running_;
  int8_t last_status_;
  std::string group_name_;

  rclcpp::TimerBase::SharedPtr timer_;
};
