#include "elevation_mapping/ElevationMappingGPU.hpp"

#include <cuda_runtime.h>

struct CellUpdate {
  int cell_index;
  float x, y, z;
  float variance;
  uint8_t r, g, b;
  float confidence_ratio;
  float scan_time;
  float current_pattern_time;
  float sensor_x, sensor_y, sensor_z;
  bool valid;
};
__device__ float atomicCASFloat(float* address, float expected, float desired) {
  unsigned int* address_as_ui = (unsigned int*)address;
  unsigned int expected_ui = __float_as_uint(expected);
  unsigned int desired_ui = __float_as_uint(desired);
  unsigned int old_ui = atomicCAS(address_as_ui, expected_ui, desired_ui);
  return __uint_as_float(old_ui);
}

// Atomic Kalman filter update on elevation and variance in a loop
__device__ void atomicKalmanUpdate(float* elevation, float* variance, float new_z, float new_var) {
  float old_elev, old_var;
  float fused_z, fused_var;
  while (true) {
    old_elev = *elevation;
    old_var = *variance;

    if (isnan(old_elev) || isnan(old_var)) {
      // Initialize if not yet initialized
      fused_z = new_z;
      fused_var = new_var;
    } else {
      float combinedVar = old_var + new_var;
      fused_z = (old_var * new_z + new_var * old_elev) / combinedVar;
      fused_var = (old_var * new_var) / combinedVar;
    }

    float prev_elev = atomicCASFloat(elevation, old_elev, fused_z);
    if (prev_elev != old_elev) {
      // elevation changed, try again
      continue;
    }

    float prev_var = atomicCASFloat(variance, old_var, fused_var);
    if (prev_var != old_var) {
      // variance changed, rollback elevation and retry
      atomicExch(elevation, old_elev);
      continue;
    }

    // Successful update
    break;
  }
}
__global__ void computeUpdateInfoKernel(
  const PointXYZRGBConfidenceDevice* points,
  const float* variances,
  int num_points,
  CellUpdate* updateBuffer,
  int width, int height,
  float resolution, float originX, float originY,
  float scan_time, float current_pattern_time,
  float sensor_x, float sensor_y, float sensor_z)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  PointXYZRGBConfidenceDevice pt = points[idx];

  int gx = int((pt.x - originX) / resolution);
  int gy = int((pt.y - originY) / resolution);
  bool valid = gx >= 0 && gx < width && gy >= 0 && gy < height;

  CellUpdate update;
  update.cell_index = valid ? (gy * width + gx) : -1;
  update.x = pt.x;
  update.y = pt.y;
  update.z = pt.z;
  update.variance = variances[idx] * 1e-11f;
  update.r = pt.r;
  update.g = pt.g;
  update.b = pt.b;
  update.confidence_ratio = pt.confidence_ratio;
  update.scan_time = scan_time;
  update.current_pattern_time = current_pattern_time;
  update.sensor_x = sensor_x;
  update.sensor_y = sensor_y;
  update.sensor_z = sensor_z;
  update.valid = valid;

  updateBuffer[idx] = update;
}

__global__ void applyUpdateKernel(
  const CellUpdate* updates,
  int num_points,
  float* elevation, float* variance,
  float* horz_var_x, float* horz_var_y, float* horz_var_xy,
  uint32_t* color,
  float* time, float* dynamic_time,
  float* lowest_scan_point,
  float* sensor_x_at_lowest, float* sensor_y_at_lowest, float* sensor_z_at_lowest,
  float minHorizontalVariance, float multiHeightNoise,
  float mahalanobisThreshold, float scanningDuration)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  const CellUpdate& u = updates[idx];
  if (!u.valid) return;

  int cell = u.cell_index;
  float old_elev = elevation[cell];
  float old_var  = variance[cell];
  bool initialized = !isnan(old_elev) && !isnan(old_var);

  if (!initialized) {
    elevation[cell] = u.z;
    variance[cell] = u.variance;

    horz_var_x[cell] = minHorizontalVariance;
    horz_var_y[cell] = minHorizontalVariance;
    horz_var_xy[cell] = 0.0f;

    color[cell] = ((u.r & 0xFF) << 16) | ((u.g & 0xFF) << 8) | (u.b & 0xFF);

    time[cell] = u.scan_time;
    dynamic_time[cell] = u.current_pattern_time;

    float height_plus_sigma = u.z + 3.0f * sqrtf(u.variance);
    lowest_scan_point[cell] = height_plus_sigma;
    sensor_x_at_lowest[cell] = u.sensor_x;
    sensor_y_at_lowest[cell] = u.sensor_y;
    sensor_z_at_lowest[cell] = u.sensor_z;
    return;
  }

  // Mahalanobis check
  if (old_var <= 0.0f || isnan(old_var)) return;
  float dist = fabsf(u.z - old_elev) / sqrtf(old_var);
  if (dist > mahalanobisThreshold) {
    if ((u.scan_time - time[cell]) <= scanningDuration && old_elev > u.z) {
      return; // Ignore lower point
    } else if ((u.scan_time - time[cell]) <= scanningDuration) {
      elevation[cell] = u.z;
      variance[cell] = u.variance;
    } else {
      atomicAdd(&variance[cell], multiHeightNoise);
    }
    return;
  }

// Update lowest scan point if necessary
  float uncertainty_z = u.z + 3.0f * sqrtf(u.variance);
  float current_lowest = lowest_scan_point[cell];
  if (isnan(current_lowest) || uncertainty_z < current_lowest) {
    lowest_scan_point[cell] = uncertainty_z;
    sensor_x_at_lowest[cell] = u.sensor_x;
    sensor_y_at_lowest[cell] = u.sensor_y;
    sensor_z_at_lowest[cell] = u.sensor_z;
  }

  // Use atomic Kalman update here
  atomicKalmanUpdate(&elevation[cell], &variance[cell], u.z, u.variance);

  color[cell] = ((u.r & 0xFF) << 16) | ((u.g & 0xFF) << 8) | (u.b & 0xFF);

  time[cell] = u.scan_time;
  dynamic_time[cell] = u.current_pattern_time;

  horz_var_x[cell] = minHorizontalVariance;
  horz_var_y[cell] = minHorizontalVariance;
  horz_var_xy[cell] = 0.0f;
}



namespace elevation_mapping {

  ElevationMappingGPU::ElevationMappingGPU(){
    d_elevation = nullptr;
    d_variance = nullptr; 
    d_horzVarX = nullptr;
    d_horzVarY = nullptr;
    d_horzVarXY = nullptr;
    d_time = nullptr;
    d_dynamicTime = nullptr;
    d_lowestScanPoint = nullptr;
    d_sensorXatLowest = nullptr;
    d_sensorYatLowest = nullptr;
    d_sensorZatLowest = nullptr;
    d_variances = nullptr;
    d_color = nullptr;
    lastSize_ = 0;
  }
  ElevationMappingGPU::~ElevationMappingGPU(){
    deallocate();
  }

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
const float mapWidthMeters = width * map.getResolution();
const float mapHeightMeters = height * map.getResolution();

const float mapOriginX = origin.x() - mapWidthMeters / 2.0f;
const float mapOriginY = origin.y() - mapHeightMeters / 2.0f;

// Allocate GPU memory
PointXYZRGBConfidenceDevice* d_points;
CellUpdate* d_updates;

// Copy map layers from host
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
auto& colorLayer = map.get("color");
if (lastSize_ != mapSize) {
  allocate(mapSize);
  lastSize_ = mapSize;
}
cudaMalloc(&d_variances, sizeof(float) * variances.size());

// Copy map layer data from host to device
cudaMemcpy(d_elevation, elevationLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_variance, varianceLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_horzVarX, horzVarXLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_horzVarY, horzVarYLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_horzVarXY, horzVarXYLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_time, timeLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_dynamicTime, dynamicTimeLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_lowestScanPoint, lowestScanLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_sensorXatLowest, sensorXLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_sensorYatLowest, sensorYLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_sensorZatLowest, sensorZLayer.data(), sizeof(float) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_color, colorLayer.data(), sizeof(uint32_t) * mapSize, cudaMemcpyHostToDevice);
cudaMemcpy(d_variances, variances.data(), sizeof(float) * variances.size(), cudaMemcpyHostToDevice);

// Copy point cloud to GPU
to_GPU(pointCloud, d_points);

// Allocate update buffer
int num_points = pointCloud->size();
cudaMalloc(&d_updates, sizeof(CellUpdate) * num_points);

// Launch kernel 1: compute per-point updates
int threadsPerBlock = 256;
int blocks = (num_points + threadsPerBlock - 1) / threadsPerBlock;
computeUpdateInfoKernel<<<blocks, threadsPerBlock>>>(
    d_points, d_variances, num_points, d_updates,
    width, height, map.getResolution(), mapOriginX, mapOriginY,
    scanTimeSinceInitialization, currentTimeSecondsPattern,
    sensorTranslation.x(), sensorTranslation.y(), sensorTranslation.z());
cudaDeviceSynchronize();

// Launch kernel 2: apply updates with atomics
applyUpdateKernel<<<blocks, threadsPerBlock>>>(
    d_updates, num_points, d_elevation, d_variance,
    d_horzVarX, d_horzVarY, d_horzVarXY, d_color, d_time, d_dynamicTime,
    d_lowestScanPoint, d_sensorXatLowest, d_sensorYatLowest, d_sensorZatLowest,
    minHorizontalVariance, multiHeightNoise,
    mahalanobisDistanceThreshold, scanningDuration);
cudaDeviceSynchronize();

// Copy map layers back to host
cudaMemcpy(elevationLayer.data(), d_elevation, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(varianceLayer.data(), d_variance, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(horzVarXLayer.data(), d_horzVarX, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(horzVarYLayer.data(), d_horzVarY, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(horzVarXYLayer.data(), d_horzVarXY, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(timeLayer.data(), d_time, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(dynamicTimeLayer.data(), d_dynamicTime, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(lowestScanLayer.data(), d_lowestScanPoint, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(sensorXLayer.data(), d_sensorXatLowest, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(sensorYLayer.data(), d_sensorYatLowest, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(sensorZLayer.data(), d_sensorZatLowest, sizeof(float) * mapSize, cudaMemcpyDeviceToHost);
cudaMemcpy(colorLayer.data(), d_color, sizeof(uint32_t) * mapSize, cudaMemcpyDeviceToHost);

// Free device memory
cudaFree(d_points);
cudaFree(d_variances);
cudaFree(d_updates);
return true;
}

void ElevationMappingGPU::allocate(size_t size) {
  deallocate();
  cudaMalloc(&d_elevation, sizeof(float) * size);
  cudaMalloc(&d_variance, sizeof(float) * size);
  cudaMalloc(&d_horzVarX, sizeof(float) * size);
  cudaMalloc(&d_horzVarY, sizeof(float) * size);
  cudaMalloc(&d_horzVarXY, sizeof(float) * size);
  cudaMalloc(&d_time, sizeof(float) * size);
  cudaMalloc(&d_dynamicTime, sizeof(float) * size);
  cudaMalloc(&d_lowestScanPoint, sizeof(float) * size);
  cudaMalloc(&d_sensorXatLowest, sizeof(float) * size);
  cudaMalloc(&d_sensorYatLowest, sizeof(float) * size);
  cudaMalloc(&d_sensorZatLowest, sizeof(float) * size);
  cudaMalloc(&d_color, sizeof(uint32_t) * size);
}

void ElevationMappingGPU::deallocate() {
  if (d_elevation)
    cudaFree(d_elevation);
  if (d_variance)
  cudaFree(d_variance);
  if (d_horzVarX)
  cudaFree(d_horzVarX);
  if (d_horzVarY)
  cudaFree(d_horzVarY);
  if (d_horzVarXY)
  cudaFree(d_horzVarXY);
  if (d_time)
  cudaFree(d_time);
  if (d_dynamicTime)
  cudaFree(d_dynamicTime);
  if (d_lowestScanPoint)
  cudaFree(d_lowestScanPoint);
  if (d_sensorXatLowest)
  cudaFree(d_sensorXatLowest);
  if (d_sensorYatLowest)
  cudaFree(d_sensorYatLowest);
  if (d_sensorZatLowest)
  cudaFree(d_sensorZatLowest);
  if (d_color)
  cudaFree(d_color);
}


}  // namespace elevation_mapping