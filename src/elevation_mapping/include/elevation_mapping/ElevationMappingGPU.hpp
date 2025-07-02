#include <cuda_runtime.h>

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
  ElevationMappingGPU();
  ~ElevationMappingGPU();
  bool updateMapGPU(const PointCloudType::Ptr pointCloud,
                    const Eigen::VectorXf& variances,
                    float scanTimeSinceInitialization,
                    float currentTimeSecondsPattern,
                    const Eigen::Vector3f& sensorTranslation,
                    const float minHorizontalVariance,
                    const float multiHeightNoise,
                    const float mahalanobisDistanceThreshold,
                    const float scanningDuration, grid_map::GridMap& map);

 private:
  void to_GPU(const PointCloudType::Ptr pointCloud,
              PointXYZRGBConfidenceDevice*& d_points, cudaStream_t& stream);
  void allocate(size_t size);
  void deallocate();
  float *d_elevation, *d_variance, *d_horzVarX, *d_horzVarY, *d_horzVarXY,
      *d_time, *d_dynamicTime, *d_lowestScanPoint, *d_sensorXatLowest,
      *d_sensorYatLowest, *d_sensorZatLowest;
  size_t lastSize_;
  cudaStream_t stream_;
};

}  // namespace elevation_mapping