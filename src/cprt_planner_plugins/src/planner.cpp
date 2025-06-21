
#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "cprt_planner_plugins/planner.hpp"

namespace cprt_planner_plugins
{

void Placeholder::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(DEFAULT_INTERPOLATION_RESOLUTION));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".horizon_distance", rclcpp::ParameterValue(DEFAULT_HORIZON_DISTANCE));
  node_->get_parameter(name_ + ".horizon_distance", horizon_distance_);
}

    //taken from ros-navigation/navigation2_tutorials
void Placeholder::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

    //taken from ros-navigation/navigation2_tutorials
void Placeholder::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

    //taken from ros-navigation/navigation2_tutorials
void Placeholder::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path Placeholder::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  return global_path;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cprt_planner_plugins::Placeholder, nav2_core::GlobalPlanner)