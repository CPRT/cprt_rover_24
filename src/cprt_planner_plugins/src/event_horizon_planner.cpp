#include "cprt_planner_plugins/event_horizon_planner.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"

namespace cprt_planner_plugins {

EventHorizonPlanner::EventHorizonPlanner()
    : lp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
      primary_planner_(nullptr) {}

void EventHorizonPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node_->get_logger();

  std::string primary_planner;

  // Parameter initialization

  // Interpolation resolution
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".interpolation_resolution",
      rclcpp::ParameterValue(DEFAULT_INTERPOLATION_RESOLUTION));
  node_->get_parameter(name_ + ".interpolation_resolution",
                       interpolation_resolution_);

  // Primary Planner
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".primary_planner", rclcpp::PARAMETER_STRING);
  primary_planner =
      node_->get_parameter(name_ + ".primary_planner").as_string();

  // Horizon Distance
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".horizon_distance",
      rclcpp::ParameterValue(DEFAULT_HORIZON_DISTANCE));
  node_->get_parameter(name_ + ".horizon_distance", horizon_distance_);

  try {
    primary_planner_ = lp_loader_.createUniqueInstance(primary_planner);
  } catch (const pluginlib::PluginlibException& ex) {
    RCLCPP_FATAL(logger_, "Failed to create planner instance. Exception: %s",
                 ex.what());
    return;
  }

  primary_planner_->configure(parent, name, tf, costmap_ros);

  new_goal_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          "intermediate_goal", 10);
}

// taken from ros-navigation/navigation2_tutorials
void EventHorizonPlanner::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type EventHorizonPlanner",
              name_.c_str());
  primary_planner_->cleanup();
}

// taken from ros-navigation/navigation2_tutorials
void EventHorizonPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating plugin %s of type EventHorizonPlanner",
              name_.c_str());
  if (!primary_planner_) {
    RCLCPP_FATAL(logger_, "primary_planner_ is null in activate(). Aborting.");
    std::terminate();
  }  
  primary_planner_->activate();
}

// taken from ros-navigation/navigation2_tutorials
void EventHorizonPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type EventHorizonPlanner",
              name_.c_str());
  primary_planner_->deactivate();
}

nav_msgs::msg::Path EventHorizonPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  RCLCPP_INFO(logger_, "Creating plan");
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(logger_,
                 "Planner will only accept start position from %s frame",
                 global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(logger_, "Planner will only accept goal position from %s frame",
                global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // get updated goal for primary planner
  const geometry_msgs::msg::PoseStamped new_goal = getNewGoal(start, goal);

  // get path from primary planner
  global_path = primary_planner_->createPlan(start, new_goal);

  if (goal != new_goal) {
    // create a straight line from new_goal on the horizon to the original goal

    int total_number_of_loop =
        std::hypot(goal.pose.position.x - new_goal.pose.position.x,
                   goal.pose.position.y - new_goal.pose.position.y) /
        interpolation_resolution_;

    if (total_number_of_loop > 0) {
      double x_increment = (goal.pose.position.x - new_goal.pose.position.x) /
                           total_number_of_loop;
      double y_increment = (goal.pose.position.y - new_goal.pose.position.y) /
                           total_number_of_loop;

      for (int i = 0; i < total_number_of_loop; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = new_goal.pose.position.x + x_increment * i;
        pose.pose.position.y = new_goal.pose.position.y + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = new_goal.pose.orientation;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
      }
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);
  }

  return global_path;
}

geometry_msgs::msg::PoseStamped EventHorizonPlanner::getNewGoal(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  geometry_msgs::msg::PoseStamped new_goal;

  if (std::sqrt(std::pow(goal.pose.position.x - start.pose.position.x, 2) +
                std::pow(goal.pose.position.y - start.pose.position.y, 2)) >
      horizon_distance_) {
    float angle = std::atan2(goal.pose.position.y - start.pose.position.y,
                             goal.pose.position.x - start.pose.position.x);
    new_goal.pose.position.x =
        start.pose.position.x + horizon_distance_ * std::cos(angle);
    RCLCPP_INFO(logger_, "New goal x: %f", new_goal.pose.position.x);
    new_goal.pose.position.y =
        start.pose.position.y + horizon_distance_ * std::sin(angle);
    RCLCPP_INFO(logger_, "New goal y: %f", new_goal.pose.position.y);
    new_goal.pose.position.z = 0.0;
    new_goal.pose.orientation = EulerToQuaternion(0.0, 0.0, angle);
    new_goal.header.stamp = node_->now();
    new_goal.header.frame_id = global_frame_;
  } else {
    new_goal = goal;
  }

  // Publish the new goal
  new_goal_publisher_->publish(new_goal);

  return new_goal;
}

geometry_msgs::msg::Quaternion EventHorizonPlanner::EulerToQuaternion(
    float roll, float pitch, float yaw) {
  float cy = std::cos(yaw * 0.5);
  float sy = std::sin(yaw * 0.5);
  float cp = std::cos(pitch * 0.5);
  float sp = std::sin(pitch * 0.5);
  float cr = std::cos(roll * 0.5);
  float sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}

}  // namespace cprt_planner_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cprt_planner_plugins::EventHorizonPlanner,
                       nav2_core::GlobalPlanner)
