#include "MoveGroupAsyncWrapper.hpp"

#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

MoveGroupAsyncWrapper::MoveGroupAsyncWrapper(rclcpp::Node* node,
                                             const std::string& group_name)
    : node_(node), running_(false), last_status_(-1), group_name_(group_name) {
  client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      node_, "move_group");
  while (!client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for move_group action server...");
  }

  timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&MoveGroupAsyncWrapper::statusMonitorCallback, this));
}

MoveGroupAsyncWrapper::~MoveGroupAsyncWrapper() { timer_->cancel(); }

void MoveGroupAsyncWrapper::moveToPose(const geometry_msgs::msg::Pose& pose) {
  moveit_msgs::action::MoveGroup::Goal goal;

  goal.request.group_name = group_name_;
  goal.request.allowed_planning_time = 5.0;
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "Joint_1";
  pose_stamped.header.stamp = node_->now();
  pose_stamped.pose = pose;

  goal.request.goal_constraints.push_back(
      kinematic_constraints::constructGoalConstraints("eef_link",
                                                      pose_stamped));

  auto send_goal_options =
      rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<
             moveit_msgs::action::MoveGroup>::SharedPtr goal_handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!goal_handle) {
          RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
          running_ = false;
        } else {
          RCLCPP_INFO(node_->get_logger(), "Goal accepted by server");
          goal_handle_ = goal_handle;
          running_ = true;
        }
      };

  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<
             moveit_msgs::action::MoveGroup>::WrappedResult& result) {
        RCLCPP_INFO(node_->get_logger(), "Goal finished with code: %d",
                    static_cast<int>(result.code));
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        goal_handle_.reset();
      };

  client_->async_send_goal(goal, send_goal_options);
}

void MoveGroupAsyncWrapper::cancelGoal() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    RCLCPP_WARN(node_->get_logger(), "Requested goal cancel");
  }
}

bool MoveGroupAsyncWrapper::isRunning() { return running_; }

void MoveGroupAsyncWrapper::statusMonitorCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_) {
    auto status = goal_handle_->get_status();
    if (status != last_status_) {
      RCLCPP_INFO(node_->get_logger(), "Goal status: %d", status);
      last_status_ = status;
    }
  }
}
