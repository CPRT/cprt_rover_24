/**
 * TraversabilityFilter.hpp
 *
 *    Created on: Jan 19, 2025
 *        Author: Erik Caldwell
 *  Organization: Carleton Planetary Robotics Team
 */

#ifndef CPRTGRIDMAPFILTERS_TRAVERSABILITYFILTER_HPP_
#define CPRTGRIDMAPFILTERS_TRAVERSABILITYFILTER_HPP_

#include <Eigen/Core>
#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <string>
#include <vector>

namespace grid_map {

enum class Method {
  RadialInflationSerial,
};

/**
 * Preserve Cost Inflation filter will inflate the gridmap by a set size
 * to account for the size of the robot. Behaves very similarly to the Nav2
 * inflation layer except it will preserve the cost values of the cells
 * when it inflates.
 */
template <typename T>
class TraversabilityFilter : public filters::FilterBase<T> {
 public:
  /**
   * Constructor
   */
  TraversabilityFilter();

  /**
   * Destructor.
   */
  virtual ~TraversabilityFilter();

  /**
   * Configures the filter from parameters on the parameter server.
   */
  bool configure() override;

  /**
   * Inflates a gridmap by the predefined inflation radius.
   *
   * @param mapIn GridMap with the different layers to inflate.
   * @param mapOut GridMap with the inflation applied to the layers.
   */
  bool update(const T &mapIn, T &mapOut) override;

 private:
  void processRows(const grid_map::GridMap &mapIn, grid_map::GridMap &mapOut,
                  int rowStart, int rowEnd);

  void processMapSingleThreaded(const grid_map::GridMap &mapIn,
                                grid_map::GridMap &mapOut);

  void processMapSingleThreadedV2(const grid_map::GridMap &mapIn,
                                grid_map::GridMap &mapOut);

 private:
  // Input layer name to inflate.
  std::string inputLayer_;

  // Output layer name.
  std::string outputLayer_;

  // Whether to use the original elevation or smoothed elevation for slope
  bool useSmoothedElevationForSlope_;

  // Slope window size.
  int slopeWindowSize_;

  // Smoothing window size.
  int smoothingWindowSize_;

  // Roughness arbitrary scaler. To convert a roughness value to roughly [0, 1]
  double roughnessScalar_;

  // Slope arbitrary scaler. To convert a slope value to roughly [0, 1]
  double slopeScalar_;

  // Low pass value for treating small obstacles as flat ground
  double lowPassValue_;

  // Add additional debug layers
  bool isDebugLayersShown_;

};  // class TraversabilityFilter

}  // namespace grid_map

#endif  // CPRTGRIDMAPFILTERS_TRAVERSABILITYFILTER_HPP_
