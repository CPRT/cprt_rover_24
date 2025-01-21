/**
 * PreserveCostInflationFilter.hpp
 *
 *    Created on: Jan 19, 2025
 *        Author: Erik Caldwell
 *  Organization: Carleton Planetary Robotics Team
 */

#include "PreserveCostInflationFilter.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map {

template <typename T>
PreserveCostInflationFilter<T>::PreserveCostInflationFilter()
    : method_(Method::RadialInflationSerial), inflationRadius_(0.0) {
  ///
}

template <typename T>
PreserveCostInflationFilter<T>::~PreserveCostInflationFilter() {
  ///
}

template <typename T>
bool PreserveCostInflationFilter<T>::configure() {
  grid_map::ParameterReader param_reader(this->param_prefix_,
                                         this->params_interface_);

  if (!param_reader.get(std::string("input_layer"), this->inputLayer_)) {
    RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "PreserveCostInflationFilter did not find parameter 'input_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("output_layer"), this->outputLayer_)) {
    RCLCPP_ERROR(
        this->logging_interface_->get_logger(),
        "PreserveCostInflationFilter did not find parameter 'output_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("inflation_radius"),
                        this->inflationRadius_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "PreserveCostInflationFilter did not find parameter "
                 "'inflation_radius'.");
    return false;
  }

  return true;
}

template <typename T>
bool PreserveCostInflationFilter<T>::update(const T &mapIn, T &mapOut) {
  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(this->inputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "Layer %s does not exist. Unable to apply preserve cost "
                 "inflation filter.",
                 this->inputLayer_.c_str());
    return false;
  }

  mapOut.add(this->outputLayer_, 1.0);

  this->computeWithSimpleSerialMethod(mapIn, mapOut);

  return true;
}

template <typename T>
void PreserveCostInflationFilter<T>::computeWithSimpleSerialMethod(
    const grid_map::GridMap &mapIn, grid_map::GridMap &mapOut) {
  rclcpp::Clock clock;
  const double start = clock.now().seconds();

  const Eigen::MatrixXf &layerIn = mapOut[this->inputLayer_];

  for (grid_map::GridMapIterator iterator(mapIn); !iterator.isPastEnd();
       ++iterator) {
    if (!mapIn.isValid(*iterator, this->inputLayer_)) {
      continue;
    }

    const grid_map::Index index = *iterator;
    grid_map::Position position;
    mapIn.getPosition(
        index, position);  // Get position of cell from grid_map::Index of cell

    this->radialInflateSerial(mapOut, position,
                              layerIn.coeff(index(0), index(1)));
  }

  const double end = clock.now().seconds();
  RCLCPP_DEBUG_THROTTLE(this->logging_interface_->get_logger(), clock, 2.0,
                        "NORMAL COMPUTATION TIME = %f", (end - start));
}

template <typename T>
void PreserveCostInflationFilter<T>::radialInflateSerial(
    grid_map::GridMap &mapOut, const grid_map::Position &position,
    const float value) {
  Eigen::MatrixXf &layerOut = mapOut[this->outputLayer_];

  for (grid_map::CircleIterator iterator(mapOut, position,
                                         this->inflationRadius_);
       !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, this->outputLayer_)) {
      continue;
    }

    layerOut.coeffRef((*iterator)(0), (*iterator)(1)) =
        std::min(layerOut.coeff((*iterator)(0), (*iterator)(1)), value);
  }
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::PreserveCostInflationFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
