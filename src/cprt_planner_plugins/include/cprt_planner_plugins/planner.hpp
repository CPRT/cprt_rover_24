
#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace cprt_planner_plugins
{

class Placeholder : public nav2_core::GlobalPlanner
{
public:
  Placeholder() = default;
  ~Placeholder() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;


private:
  std::shared_ptr<tf2_ros::Buffer> tf_;

  nav2_util::LifecycleNode::SharedPtr node_;

  nav2_costmap_2d::Costmap2D * costmap_;

  std::string global_frame_, name_;

  double interpolation_resolution_;

  double const DEFAULT_INTERPOLATION_RESOLUTION = 0.1;

  double horizon_distance_;

  double const DEFAULT_HORIZON_DISTANCE = 100.0;

  std::string planner_type_;
};

}  

#endif