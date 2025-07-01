#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <grid_map_core/GridMap.hpp>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

struct PointXYZRGBConfidenceDevice {
  float x, y, z;
  uint8_t r, g, b;
  float confidence_ratio;
};

namespace elevation_mapping {

class ElevationMappingGPU {
 public:
  static bool updateMapGPU(
      const PointCloudType::Ptr pointCloud, const Eigen::VectorXf& variances,
      float scanTimeSinceInitialization, float currentTimeSecondsPattern,
      const Eigen::Vector3f& sensorTranslation,
      const float minHorizontalVariance, const float multiHeightNoise,
      const float mahalanobisDistanceThreshold, const float scanningDuration,
      grid_map::GridMap& map);

 private:
  static void to_GPU(const PointCloudType::Ptr pointCloud,
                     PointXYZRGBConfidenceDevice*& d_points);
};

}  // namespace elevation_mapping