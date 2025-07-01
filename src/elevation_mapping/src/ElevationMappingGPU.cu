#include "elevation_mapping/ElevationMappingGPU.hpp"

#include <cuda_runtime.h>

__global__ void updateElevationFullKernel(
  const PointXYZRGBConfidenceDevice* points,
  const float* variances,
  int num_points,
  float* elevation,
  float* variance,
  float* horizontal_variance_x,
  float* horizontal_variance_y,
  float* horizontal_variance_xy,
  uint32_t* color,
  float* time,
  float* dynamic_time,
  float* lowest_scan_point,
  float* sensor_x_at_lowest_scan,
  float* sensor_y_at_lowest_scan,
  float* sensor_z_at_lowest_scan,
  int width, int height,
  float resolution, float originX, float originY,
  float minHorizontalVariance,
  float multiHeightNoise,
  float mahalanobisDistanceThreshold,
  float scanningDuration,
  float scanTimeSinceInitialization,
  float currentTimeSecondsPattern,
  float sensorX, float sensorY, float sensorZ)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  // Convert point to grid indices
  int grid_x = int((points[idx].x - originX) / resolution);
  int grid_y = int((points[idx].y - originY) / resolution);

  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height)
      return;

  int cell_index = grid_y * width + grid_x;

  float pointVariance = 1e-11f * variances[idx];

  // Check if all basic layers valid (NaN check)
  bool valid = !isnan(elevation[cell_index]) && !isnan(variance[cell_index]) &&
               !isnan(horizontal_variance_x[cell_index]) && !isnan(horizontal_variance_y[cell_index]) &&
               !isnan(horizontal_variance_xy[cell_index]) && color[cell_index] != 0 &&
               !isnan(time[cell_index]) && !isnan(dynamic_time[cell_index]) &&
               !isnan(lowest_scan_point[cell_index]) && !isnan(sensor_x_at_lowest_scan[cell_index]) &&
               !isnan(sensor_y_at_lowest_scan[cell_index]) && !isnan(sensor_z_at_lowest_scan[cell_index]);

  if (!valid) {
      // Initialize cell from point
      elevation[cell_index] = points[idx].z;
      variance[cell_index] = pointVariance;
      horizontal_variance_x[cell_index] = minHorizontalVariance;
      horizontal_variance_y[cell_index] = minHorizontalVariance;
      horizontal_variance_xy[cell_index] = 0.0f;

      color[cell_index] = ((points[idx].r & 0xFF) << 16) | ((points[idx].g & 0xFF) << 8) | (points[idx].b & 0xFF);

      time[cell_index] = scanTimeSinceInitialization;
      dynamic_time[cell_index] = currentTimeSecondsPattern;

      lowest_scan_point[cell_index] = points[idx].z + 3.0f * sqrtf(pointVariance);
      sensor_x_at_lowest_scan[cell_index] = sensorX;
      sensor_y_at_lowest_scan[cell_index] = sensorY;
      sensor_z_at_lowest_scan[cell_index] = sensorZ;

      return;
  }

  // Mahalanobis distance
  float mahalanobisDistance = fabsf(points[idx].z - elevation[cell_index]) / sqrtf(variance[cell_index]);

  if (mahalanobisDistance > mahalanobisDistanceThreshold) {
      if ((scanTimeSinceInitialization - time[cell_index]) <= scanningDuration && elevation[cell_index] > points[idx].z) {
          // Ignore lower point within scanning duration
          return;
      } else if ((scanTimeSinceInitialization - time[cell_index]) <= scanningDuration) {
          // Point higher, update elevation and variance
          elevation[cell_index] = points[idx].z;
          variance[cell_index] = pointVariance;
      } else {
          // Increase variance due to multi-height noise
          variance[cell_index] += multiHeightNoise;
      }
      return;
  }

  // Update lowest scan point and sensor pose
  float pointHeightPlusUncertainty = points[idx].z + 3.0f * sqrtf(pointVariance);
  if (isnan(lowest_scan_point[cell_index]) || pointHeightPlusUncertainty < lowest_scan_point[cell_index]) {
      lowest_scan_point[cell_index] = pointHeightPlusUncertainty;
      sensor_x_at_lowest_scan[cell_index] = sensorX;
      sensor_y_at_lowest_scan[cell_index] = sensorY;
      sensor_z_at_lowest_scan[cell_index] = sensorZ;
  }

  // Fuse elevation and variance (Kalman update)
  float combinedVariance = variance[cell_index] + pointVariance;
  elevation[cell_index] = (variance[cell_index] * points[idx].z + pointVariance * elevation[cell_index]) / combinedVariance;
  variance[cell_index] = (pointVariance * variance[cell_index]) / combinedVariance;

  // TODO: Fuse color (simple overwrite here)
  color[cell_index] = ((points[idx].r & 0xFF) << 16) | ((points[idx].g & 0xFF) << 8) | (points[idx].b & 0xFF);

  time[cell_index] = scanTimeSinceInitialization;
  dynamic_time[cell_index] = currentTimeSecondsPattern;

  horizontal_variance_x[cell_index] = minHorizontalVariance;
  horizontal_variance_y[cell_index] = minHorizontalVariance;
  horizontal_variance_xy[cell_index] = 0.0f;
}



namespace elevation_mapping {

void ElevationMappingGPU::to_GPU(const PointCloudType::Ptr pointCloud,
                                 PointXYZRGBConfidenceDevice*& d_points) {
  std::vector<PointXYZRGBConfidenceDevice> gpuPoints;
  gpuPoints.reserve(pointCloud->size());

  for (const auto& pt : pointCloud->points) {
    PointXYZRGBConfidenceDevice dpt;
    dpt.x = pt.x;
    dpt.y = pt.y;
    dpt.z = pt.z;
    dpt.r = (pt.rgba >> 16) & 0xFF;
    dpt.g = (pt.rgba >> 8) & 0xFF;
    dpt.b = pt.rgba & 0xFF;
    dpt.confidence_ratio = pt.confidence_ratio;
    gpuPoints.push_back(dpt);
  }

  cudaMalloc(&d_points, sizeof(PointXYZRGBConfidenceDevice) * gpuPoints.size());
  cudaMemcpy(d_points, gpuPoints.data(),
             sizeof(PointXYZRGBConfidenceDevice) * gpuPoints.size(),
             cudaMemcpyHostToDevice);
}

bool ElevationMappingGPU::updateMapGPU(
    const PointCloudType::Ptr pointCloud, const Eigen::VectorXf& variances,
    float scanTimeSinceInitialization, float currentTimeSecondsPattern,
    const Eigen::Vector3f& sensorTranslation, const float minHorizontalVariance,
    const float multiHeightNoise, const float mahalanobisDistanceThreshold,
    const float scanningDuration, grid_map::GridMap& map) {
  // Map size
  const int width = map.getSize()(0);
  const int height = map.getSize()(1);
  const int mapSize = width * height;
  const grid_map::Position origin = map.getPosition();

  // --- Prepare device memory pointers ---
  // Elevation and variance (float)
  float *d_elevation, *d_variance;
  float *d_horzVarX, *d_horzVarY, *d_horzVarXY;
  float *d_time, *d_dynamicTime, *d_lowestScanPoint;
  float *d_sensorXatLowest, *d_sensorYatLowest, *d_sensorZatLowest;

  // Color (uint32_t)
  uint32_t* d_color;

  // Points
  PointXYZRGBConfidenceDevice* d_points;

  // Allocate GPU memory for map layers
  cudaMalloc(&d_elevation, sizeof(float) * mapSize);
  cudaMalloc(&d_variance, sizeof(float) * mapSize);
  cudaMalloc(&d_horzVarX, sizeof(float) * mapSize);
  cudaMalloc(&d_horzVarY, sizeof(float) * mapSize);
  cudaMalloc(&d_horzVarXY, sizeof(float) * mapSize);
  cudaMalloc(&d_time, sizeof(float) * mapSize);
  cudaMalloc(&d_dynamicTime, sizeof(float) * mapSize);
  cudaMalloc(&d_lowestScanPoint, sizeof(float) * mapSize);
  cudaMalloc(&d_sensorXatLowest, sizeof(float) * mapSize);
  cudaMalloc(&d_sensorYatLowest, sizeof(float) * mapSize);
  cudaMalloc(&d_sensorZatLowest, sizeof(float) * mapSize);
  cudaMalloc(&d_color, sizeof(uint32_t) * mapSize);

  // Copy map data from host to device
  auto& elevationLayer = map.get("elevation");
  auto& varianceLayer = map.get("variance");
  auto& horzVarXLayer = map.get("horizontal_variance_x");
  auto& horzVarYLayer = map.get("horizontal_variance_y");
  auto& horzVarXYLayer = map.get("horizontal_variance_xy");
  auto& timeLayer = map.get("time");
  auto& dynamicTimeLayer = map.get("dynamic_time");
  auto& lowestScanLayer = map.get("lowest_scan_point");
  auto& sensorXLayer = map.get("sensor_x_at_lowest_scan");
  auto& sensorYLayer = map.get("sensor_y_at_lowest_scan");
  auto& sensorZLayer = map.get("sensor_z_at_lowest_scan");
  auto& colorLayer = map.get("color");  // Assuming uint32_t packed RGB

  cudaMemcpy(d_elevation, elevationLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_variance, varianceLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_horzVarX, horzVarXLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_horzVarY, horzVarYLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_horzVarXY, horzVarXYLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_time, timeLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_dynamicTime, dynamicTimeLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_lowestScanPoint, lowestScanLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_sensorXatLowest, sensorXLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_sensorYatLowest, sensorYLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_sensorZatLowest, sensorZLayer.data(), sizeof(float) * mapSize,
             cudaMemcpyHostToDevice);
  cudaMemcpy(d_color, colorLayer.data(), sizeof(uint32_t) * mapSize,
             cudaMemcpyHostToDevice);

  to_GPU(pointCloud, d_points);

  // --- Launch kernel ---
  int threadsPerBlock = 256;
  int blocks = (static_cast<int>(pointCloud->size()) + threadsPerBlock - 1) /
               threadsPerBlock;

  updateElevationFullKernel<<<blocks, threadsPerBlock>>>(
      d_points, variances.data(), static_cast<int>(pointCloud->size()),
      d_elevation, d_variance, d_horzVarX, d_horzVarY, d_horzVarXY, d_color,
      d_time, d_dynamicTime, d_lowestScanPoint, d_sensorXatLowest,
      d_sensorYatLowest, d_sensorZatLowest, width, height, map.getResolution(), origin[0],
      origin[1], minHorizontalVariance, multiHeightNoise,
      mahalanobisDistanceThreshold, scanningDuration,
      scanTimeSinceInitialization, currentTimeSecondsPattern,
      sensorTranslation.x(), sensorTranslation.y(), sensorTranslation.z());
  cudaDeviceSynchronize();

  // --- Copy updated layers back to host ---
  cudaMemcpy(elevationLayer.data(), d_elevation, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(varianceLayer.data(), d_variance, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(horzVarXLayer.data(), d_horzVarX, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(horzVarYLayer.data(), d_horzVarY, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(horzVarXYLayer.data(), d_horzVarXY, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(timeLayer.data(), d_time, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(dynamicTimeLayer.data(), d_dynamicTime, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(lowestScanLayer.data(), d_lowestScanPoint, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(sensorXLayer.data(), d_sensorXatLowest, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(sensorYLayer.data(), d_sensorYatLowest, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(sensorZLayer.data(), d_sensorZatLowest, sizeof(float) * mapSize,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(colorLayer.data(), d_color, sizeof(uint32_t) * mapSize,
             cudaMemcpyDeviceToHost);

  // --- Free device memory ---
  cudaFree(d_elevation);
  cudaFree(d_variance);
  cudaFree(d_horzVarX);
  cudaFree(d_horzVarY);
  cudaFree(d_horzVarXY);
  cudaFree(d_time);
  cudaFree(d_dynamicTime);
  cudaFree(d_lowestScanPoint);
  cudaFree(d_sensorXatLowest);
  cudaFree(d_sensorYatLowest);
  cudaFree(d_sensorZatLowest);
  cudaFree(d_color);
  cudaFree(d_points);
  return true;
}
}  // namespace elevation_mapping