#include "cprt_costmap_plugins/footprint_clearing_layer.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cprt_costmap_plugins {

FootprintClearingLayer::FootprintClearingLayer() {}
FootprintClearingLayer::~FootprintClearingLayer() {}

void FootprintClearingLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("clear_cost", rclcpp::ParameterValue((int)FREE_SPACE));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled",
                      footprint_clearing_enabled_);
  int temp_clear_cost;
  node->get_parameter(name_ + "." + "clear_cost", temp_clear_cost);
  clear_cost_ = static_cast<unsigned char>(temp_clear_cost);

  current_ = true;
}

void FootprintClearingLayer::activate() { matchSize(); }
void FootprintClearingLayer::deactivate() {}
void FootprintClearingLayer::reset() { current_ = false; }
void FootprintClearingLayer::matchSize() {
}  // This layer doesn't have it's own costmap

void FootprintClearingLayer::updateBounds(double robot_x, double robot_y,
                                          double robot_yaw, double* min_x,
                                          double* min_y, double* max_x,
                                          double* max_y) {
  if (!enabled_ || !footprint_clearing_enabled_) {
    return;
  }

  // Get the robot's footprint from the layered costmap.
  // It returns std::vector<geometry_msgs::msg::Point> in ROS 2.
  const std::vector<geometry_msgs::msg::Point>& footprint =
      layered_costmap_->getFootprint();

  // Transform the footprint to the current robot pose.
  // The function takes 'const std::vector<geometry_msgs::msg::Point>&' as
  // input.
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, footprint,
                                      transformed_footprint_);

  // Update bounds to include the robot's footprint
  double min_by_x = std::numeric_limits<double>::infinity();
  double min_by_y = std::numeric_limits<double>::infinity();
  double max_by_x = -std::numeric_limits<double>::infinity();
  double max_by_y = -std::numeric_limits<double>::infinity();

  for (const auto& pt : transformed_footprint_) {
    min_by_x = std::min(min_by_x, pt.x);
    min_by_y = std::min(min_by_y, pt.y);
    max_by_x = std::max(max_by_x, pt.x);
    max_by_y = std::max(max_by_y, pt.y);
  }

  // Expand the overall update area to include the footprint's bounds
  *min_x = std::min(*min_x, min_by_x);
  *min_y = std::min(*min_y, min_by_y);
  *max_x = std::max(*max_x, max_by_x);
  *max_y = std::max(*max_y, max_by_y);

  current_ = true;
}

void FootprintClearingLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
    int max_j) {
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (!enabled_ || !footprint_clearing_enabled_ ||
      transformed_footprint_.empty()) {
    return;
  }

  master_grid.setConvexPolygonCost(transformed_footprint_, clear_cost_);
}

}  // namespace cprt_costmap_plugins

PLUGINLIB_EXPORT_CLASS(cprt_costmap_plugins::FootprintClearingLayer,
                       nav2_costmap_2d::Layer)