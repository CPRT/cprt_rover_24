/**
 * TraversabilityFilter.hpp
 *
 *    Created on: Jan 19, 2025
 *        Author: Erik Caldwell
 *  Organization: Carleton Planetary Robotics Team
 */

#include "TraversabilityFilter.hpp"

#include <cmath>
#include <grid_map_core/grid_map_core.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>

#include "grid_map_cv/utilities.hpp"

namespace grid_map {

template <typename T>
TraversabilityFilter<T>::TraversabilityFilter()
    : slopeWindowSize_(7),
      smoothingWindowSize_(11),
      roughnessScalar_(0.0),
      slopeScalar_(0.0),
      numThreads_(1),
      isDebugLayersShown_(false) {
  ///
}

template <typename T>
TraversabilityFilter<T>::~TraversabilityFilter() {
  ///
}

template <typename T>
bool TraversabilityFilter<T>::configure() {
  grid_map::ParameterReader param_reader(this->param_prefix_,
                                         this->params_interface_);

  if (!param_reader.get(std::string("input_layer"), this->inputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter 'input_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("output_layer"), this->outputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter 'output_layer'.");
    return false;
  }

  if (!param_reader.get(std::string("slope_window_size"),
                        this->slopeWindowSize_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter "
                 "'slope_window_size'.");
    return false;
  } else if ((this->slopeWindowSize_ <= 0) ||
             (this->slopeWindowSize_ % 2 == 0)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter parameter 'slope_window_size' must be "
                 "an odd positive number.");
    return false;
  }

  if (!param_reader.get(std::string("smoothing_window_size"),
                        this->smoothingWindowSize_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter "
                 "'smoothing_window_size'.");
    return false;
  } else if ((this->smoothingWindowSize_ <= 0) ||
             (this->smoothingWindowSize_ % 2 == 0)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter parameter 'smoothing_window_size' must "
                 "be an odd positive number.");
    return false;
  }

  if (!param_reader.get(std::string("roughness_scalar"),
                        this->roughnessScalar_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter "
                 "'roughness_scalar'.");
    return false;
  }

  if (!param_reader.get(std::string("slope_scalar"), this->slopeScalar_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter "
                 "'slope_scalar'.");
    return false;
  }

  if (!param_reader.get(std::string("low_pass_value"), this->lowPassValue_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter "
                 "'low_pass_value'.");
    return false;
  }

  if (!param_reader.get(std::string("num_threads"), this->numThreads_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter "
                 "'num_threads'.");
    return false;
  } else if (this->numThreads_ < 1) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter parameter 'num_threads' must be "
                 "greater than 0.");
    return false;
  } else if (this->numThreads_ >
             static_cast<int>(std::thread::hardware_concurrency())) {
    RCLCPP_WARN(this->logging_interface_->get_logger(),
                "TraversabilityFilter parameter 'num_threads' is greater "
                "than the number of hardware threads. Setting to maximum "
                "hardware threads.");
    this->numThreads_ = std::thread::hardware_concurrency();
  }

  if (!param_reader.get(std::string("is_debug_layers_shown"),
                        this->isDebugLayersShown_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "TraversabilityFilter did not find parameter "
                 "'is_debug_layers_shown'.");
    return false;
  }

  return true;
}

template <typename T>
bool TraversabilityFilter<T>::update(const T &mapIn, T &mapOut) {
  mapOut = mapIn;

  // Check if layer exists.
  if (!mapOut.exists(this->inputLayer_)) {
    RCLCPP_ERROR(this->logging_interface_->get_logger(),
                 "Layer %s does not exist. Unable to apply preserve cost "
                 "inflation filter.",
                 this->inputLayer_.c_str());
    return false;
  }

  mapOut.add(this->outputLayer_, 0.0);

  if (this->isDebugLayersShown_) {
    mapOut.add("elevation_passthru", 0.0);
    mapOut.add("smoothed_elevation", 0.0);
    mapOut.add("slope", 0.0);
    mapOut.add("roughness", 0.0);
  }

  this->processMapSingleThreaded(mapIn, mapOut);

  return true;
}

template <typename T>
void TraversabilityFilter<T>::processRows(const grid_map::GridMap &mapIn,
                                         grid_map::GridMap &mapOut,
                                         int rowStart, int rowEnd) {
  const int rows = mapIn[this->inputLayer_].rows();
  const int cols = mapIn[this->inputLayer_].cols();

  int numNan = 0;
  int numProcessed = 0;
  for (int i = rowStart; i < rowEnd; ++i) {
    for (int j = 0; j < cols; ++j) {
      const auto ijIndex = grid_map::Index(i, j);
      const auto &ijElevation = mapIn.at(this->inputLayer_, ijIndex);
      if (std::isnan(ijElevation)) {
        mapOut.at(this->outputLayer_, ijIndex) = ijElevation;
        numNan++;
        continue;
      }

      // Calculate smoothedElevation
      float smoothSum = 0;
      int smoothCount = 0;
      int smoothHalfWindow = this->smoothingWindowSize_ / 2;

      for (int x = std::max(0, i - smoothHalfWindow);
           x <= std::min(rows - 1, i + smoothHalfWindow); ++x) {
        for (int y = std::max(0, j - smoothHalfWindow);
             y <= std::min(cols - 1, j + smoothHalfWindow); ++y) {
          const auto &xyElevation =
              mapIn.at(this->inputLayer_, grid_map::Index(x, y));
          if (std::isnan(xyElevation)) {
            continue;
          }

          smoothSum += xyElevation;
          smoothCount++;
        }
      }

      const float smoothedElevation = smoothSum / smoothCount;

      // Calculate slope using original elevation (nested loops)
      float dxSum = 0;
      float dySum = 0;
      int slopeCount = 0;
      int slopeHalfWindow = this->slopeWindowSize_ / 2;

      for (int x = std::max(0, i - slopeHalfWindow);
           x <= std::min(rows - 1, i + slopeHalfWindow); ++x) {
        for (int y = std::max(0, j - slopeHalfWindow);
             y <= std::min(cols - 1, j + slopeHalfWindow); ++y) {
          if (x != i && y != j) {  // Avoid using the center point itself
            const auto &xyElevation =
                mapIn.at(this->inputLayer_, grid_map::Index(x, y));
            if (std::isnan(xyElevation)) {
              continue;
            }
            dxSum += (xyElevation - ijElevation) * (y - j);
            dySum += (xyElevation - ijElevation) * (x - i);
            slopeCount++;
          }
        }
      }

      const float dx = (slopeCount > 0) ? dxSum / slopeCount : 0;
      const float dy = (slopeCount > 0) ? dySum / slopeCount : 0;
      const float slope = std::atan(std::sqrt(dx * dx + dy * dy));

      // Calculate roughness and traversability
      const float roughness = std::abs(ijElevation - smoothedElevation);
      float traversability =
          std::clamp(static_cast<float>(slope * this->slopeScalar_ +
                                        roughness * this->roughnessScalar_),
                     0.0f, 1.0f);

      // Low pass filter
      if (this->lowPassValue_ > 0.0001 &&
          traversability < this->lowPassValue_) {
        traversability = 0.0;
      }

      numProcessed++;
      mapOut.at(this->outputLayer_, ijIndex) = traversability;

      if (this->isDebugLayersShown_) {
        mapOut.at("elevation_passthru", ijIndex) = mapIn.at(this->inputLayer_, ijIndex);
        mapOut.at("smoothed_elevation", ijIndex) = smoothedElevation;
        mapOut.at("slope", ijIndex) = slope * this->slopeScalar_;
        mapOut.at("roughness", ijIndex) = roughness * this->roughnessScalar_;
      }
    }
  }
}

template <typename T>
void TraversabilityFilter<T>::processMapMultiThreaded(
    const grid_map::GridMap &mapIn, grid_map::GridMap &mapOut) {
  // int rowsPerThread = map.size() / this->numThreads_;
  // std::vector<std::future<void>> futures;

  // for (int i = 0; i < this->numThreads_; ++i) {
  //   int startRow = i * rowsPerThread;
  //   int endRow =
  //       (i == this->numThreads_ - 1) ? map.size() : (i + 1) *
  //       rowsPerThread;
  //   futures.push_back(std::async(std::launch::async, processRow,
  //                                std::ref(mapIn), std::ref(mapOut),
  //                                startRow, endRow));
  // }

  // for (auto &future : futures) {
  //   future.wait();
  // }
}

template <typename T>
void TraversabilityFilter<T>::processMapSingleThreaded(
    const grid_map::GridMap &mapIn, grid_map::GridMap &mapOut) {
  processRows(mapIn, mapOut, 0, mapIn[this->inputLayer_].rows());
}

}  // namespace grid_map

PLUGINLIB_EXPORT_CLASS(grid_map::TraversabilityFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
