#include "cprt_costmap_plugins/footprint_clearing_layer.hpp"

#include <algorithm>  // Required for std::min
#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"  // Required for makeFootprintFromString

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::MapLocation;
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
  // Declare the new override_footprint parameter
  declareParameter("override_footprint",
                   rclcpp::ParameterValue(std::string("")));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "footprint_clearing_enabled",
                      footprint_clearing_enabled_);
  int temp_clear_cost;
  node->get_parameter(name_ + "." + "clear_cost", temp_clear_cost);
  clear_cost_ = static_cast<unsigned char>(temp_clear_cost);
  // Get the value of the new override_footprint parameter
  node->get_parameter(name_ + "." + "override_footprint",
                      override_footprint_str_);

  // Ensure clear_cost_ is not set to NO_INFORMATION or LETHAL_OBSTACLE
  // A good default for this new logic would be LETHAL_OBSTACLE - 10, or
  // similar. The parameter `clear_cost` from YAML will now act as the maximum
  // cost that a cell can be reduced to by this layer. It's recommended to set
  // this in the YAML to a value like 243 (LETHAL_OBSTACLE - 10) or a lower
  // value if you want the cleared path to be more traversable.
  if (clear_cost_ == NO_INFORMATION || clear_cost_ >= LETHAL_OBSTACLE) {
    RCLCPP_WARN(
        node->get_logger(),
        "FootprintClearingLayer: 'clear_cost' parameter should be "
        "less than LETHAL_OBSTACLE (253) and not NO_INFORMATION (255) "
        "for min-based clearing. Using LETHAL_OBSTACLE - 10 (243) as default.");
    clear_cost_ = LETHAL_OBSTACLE - 10;  // Default to a safe high cost
  }

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

  // Determine which footprint to use
  std::vector<geometry_msgs::msg::Point> active_footprint;
  if (override_footprint_str_.empty()) {
    // Use the default footprint from the layered costmap
    active_footprint = layered_costmap_->getFootprint();
  } else {
    // Parse and use the custom footprint from the parameter
    // makeFootprintFromString returns a bool and modifies the second argument
    // by reference
    if (!nav2_costmap_2d::makeFootprintFromString(override_footprint_str_,
                                                  active_footprint)) {
      RCLCPP_ERROR(node_.lock()->get_logger(),
                   "FootprintClearingLayer: Failed to parse override_footprint "
                   "string '%s'. "
                   "Falling back to default footprint.",
                   override_footprint_str_.c_str());
      active_footprint =
          layered_costmap_->getFootprint();  // Fallback to default
    }
  }

  // Transform the chosen footprint to the current robot pose.
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw,
                                      active_footprint, transformed_footprint_);

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

  unsigned char target_clear_cost = clear_cost_;

  // Convert the world-coordinate footprint to map-coordinate polygon
  std::vector<MapLocation> map_polygon;
  for (const auto& pt : transformed_footprint_) {
    MapLocation loc;
    if (master_grid.worldToMap(pt.x, pt.y, loc.x, loc.y)) {
      map_polygon.push_back(loc);
    } else {
      // If any point of the footprint is outside the map, we cannot clear it
      // reliably. This might happen if the robot is partially off the costmap.
      // For this layer, we'll just skip clearing if the footprint is out of
      // bounds.
      RCLCPP_WARN(node_.lock()->get_logger(),
                  "Footprint point (%.2f, %.2f) is outside costmap bounds. "
                  "Skipping footprint clearing.",
                  pt.x, pt.y);
      return;
    }
  }

  // Ensure the polygon has at least 3 points to be valid for filling
  if (map_polygon.size() < 3) {
    RCLCPP_WARN(
        node_.lock()->get_logger(),
        "Transformed footprint has less than 3 points, cannot fill polygon.");
    return;
  }

  // Get all cells that fill the polygon
  std::vector<MapLocation> polygon_cells;
  master_grid.convexFillCells(map_polygon, polygon_cells);

  // Set the cost of those cells using the min logic, preserving NO_INFORMATION
  for (const auto& cell_loc : polygon_cells) {
    unsigned char current_cost = master_grid.getCost(cell_loc.x, cell_loc.y);

    // If the cell is NO_INFORMATION, leave it as NO_INFORMATION.
    // Otherwise, apply the min logic.
    if (current_cost == NO_INFORMATION) {
      // Do nothing, leave it as NO_INFORMATION
    } else {
      unsigned char new_cost = std::min(current_cost, target_clear_cost);
      master_grid.setCost(cell_loc.x, cell_loc.y, new_cost);
    }
  }
}

}  // namespace cprt_costmap_plugins

PLUGINLIB_EXPORT_CLASS(cprt_costmap_plugins::FootprintClearingLayer,
                       nav2_costmap_2d::Layer)
