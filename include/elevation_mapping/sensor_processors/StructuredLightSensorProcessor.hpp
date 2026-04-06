/**
 * @file StructuredLightSensorProcessor.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 结构光 / RGB-D（如 PrimeSense 类）传感器的点云滤波与测量方差模型。
 */
#pragma once

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

class StructuredLightSensorProcessor : public SensorProcessorBase {
 public:
  StructuredLightSensorProcessor(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                 const SensorProcessorBase::GeneralParameters& generalParameters);

  ~StructuredLightSensorProcessor() override;

 private:
  bool readParameters() override;

  /** 按深度噪声、基线几何与机器人位姿协方差推导每个点在高程上的方差。 */
  bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;

  /** 深度裁剪、无效点剔除等传感器专用滤波。 */
  bool filterPointCloudSensorType(PointCloudType::Ptr pointCloud) override;
};
}  // namespace elevation_mapping
