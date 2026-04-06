/**
 * @file PerfectSensorProcessor.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 理想传感器：固定极小测量方差，用于仿真或基准对比。
 */

#pragma once

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

class PerfectSensorProcessor : public SensorProcessorBase {
 public:
  PerfectSensorProcessor(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                         const SensorProcessorBase::GeneralParameters& generalParameters);

  ~PerfectSensorProcessor() override;

 private:
  bool readParameters() override;
  bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;
};

}  // namespace elevation_mapping
