#ifndef NAV2_COSTMAP_FOOTPRINT_CLEARING_LAYER_HPP_
#define NAV2_COSTMAP_FOOTPRINT_CLEARING_LAYER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace cprt_costmap_plugins {

class FootprintClearingLayer : public nav2_costmap_2d::CostmapLayer {
 public:
  FootprintClearingLayer();
  virtual ~FootprintClearingLayer();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();
  virtual void matchSize();

  virtual bool isClearable() { return true; }

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);

  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);

 protected:
  void getParameters();

  bool footprint_clearing_enabled_;
  unsigned char clear_cost_;

  // Declare using the explicit ROS 2 message type
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;

  // New parameter to override the default footprint
  std::string override_footprint_str_;
};

}  // namespace cprt_costmap_plugins

#endif  // NAV2_COSTMAP_FOOTPRINT_CLEARING_LAYER_HPP_
