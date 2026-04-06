/**
 * @file LaserSensorProcessor.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 二维激光 / 线扫类传感器：距离与角度噪声模型，输出逐点高程方差。
 */

#pragma once

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

class LaserSensorProcessor : public SensorProcessorBase {
 public:
  LaserSensorProcessor(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                       const SensorProcessorBase::GeneralParameters& generalParameters);

  ~LaserSensorProcessor() override;

 private:
  bool readParameters() override;
  bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;
};

}  // namespace elevation_mapping
