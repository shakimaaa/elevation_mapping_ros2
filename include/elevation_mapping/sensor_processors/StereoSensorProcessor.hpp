/**
 * @file StereoSensorProcessor.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 双目立体视觉点云：视差/基线相关的不确定性模型与点云清理。
 *
 * 算法背景见 Keller, ETH 学期项目（腐蚀监测攀爬机器人定位与规划）。
 */

#pragma once

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

class StereoSensorProcessor : public SensorProcessorBase {
 public:
  StereoSensorProcessor(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                        const SensorProcessorBase::GeneralParameters& generalParameters);

  ~StereoSensorProcessor() override;

 private:
  bool readParameters() override;
  bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;
  bool filterPointCloudSensorType(PointCloudType::Ptr pointCloud) override;

  /** 将线性索引还原为有序点云中的 (i,j)。 */
  int getI(int index);
  int getJ(int index);

  /** 清洗后点云保留的原始点索引，用于方差与几何对应。 */
  std::vector<int> indices_;
  int originalWidth_;
};

}  // namespace elevation_mapping
