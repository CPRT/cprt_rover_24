/*
 * StructuredLightSensorProcessor.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

/*!
 * Sensor processor for StructuredLight-type (PrimeSense) structured light
 * sensors.
 */
class StructuredLightSensorProcessor : public SensorProcessorBase {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  StructuredLightSensorProcessor(
      std::shared_ptr<rclcpp::Node>& nodeHandle,
      const SensorProcessorBase::GeneralParameters& generalParameters);

  /*!
   * Destructor.
   */
  ~StructuredLightSensorProcessor() override;

 private:
  /*!
   * Reads and verifies the parameters.
   * @return true if successful.
   */
  bool readParameters(std::string& inputSourceName) override;

  /*!
   * Computes the elevation map height variances for each point in a point cloud
   * with the sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  bool computeVariances(const PointCloudType::ConstPtr pointCloud,
                        const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;

  /*!
   * Cuts off points that are not within the cutoff interval
   * @param pointCloud the point cloud to filter.
   * @return true if successful.
   */
  bool filterPointCloudSensorType(
      const PointCloudType::Ptr pointCloud) override;
};
} /* namespace elevation_mapping */
