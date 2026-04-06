/**
 * @file SensorProcessorBase.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 传感器点云预处理与测量方差估计的抽象基类（ROS 2 + TF2 + YAML/参数双源配置）。
 *
 * 流程：`process()` 内在给定时间戳下更新 TF → 转传感器系 → 滤波 → 转地图系 → 按高度裁剪 → `computeVariances()`。
 * 参数来源：若 `setSensorProcessorYaml()` 提供了 YAML 子树（来自 `input_sources` 文件），则优先读其中键；
 * 否则回退到节点参数 `sensor_processor.*`（与单路兼容模式一致）。
 */
#pragma once

#include <Eigen/Core>
#include <kindr/Core>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include <tf2_ros/buffer.h>

#include <yaml-cpp/yaml.h>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

namespace elevation_mapping {

class SensorProcessorBase {
 public:
  using Ptr = std::unique_ptr<SensorProcessorBase>;
  friend class ElevationMapping;
  friend class Input;

  /** 机器人底盘帧与高程地图帧，用于多步 TF 查询。 */
  struct GeneralParameters {
    std::string robotBaseFrameId_;
    std::string mapFrameId_;

    explicit GeneralParameters(std::string robotBaseFrameId = "robot", std::string mapFrameId = "map")
        : robotBaseFrameId_(std::move(robotBaseFrameId)), mapFrameId_(std::move(mapFrameId)) {}
  };

  SensorProcessorBase(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                      const GeneralParameters& generalConfig);
  virtual ~SensorProcessorBase();

  /** 由 `Input::configureSensorProcessor` 注入，供 `readParameters()` 解析每路独立 YAML。 */
  void setSensorProcessorYaml(const YAML::Node& yaml) { sensor_processor_yaml_ = yaml; }

  /**
   * @brief 完整处理链；成功时 `pointCloudMapFrame` 在地图系，`variances` 与点一一对应。
   * @param sensorFrame 点云消息头中的 frame_id（传感器光学/雷达帧）。
   */
  bool process(PointCloudType::ConstPtr pointCloudInput, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
               PointCloudType::Ptr pointCloudMapFrame, Eigen::VectorXf& variances, std::string sensorFrame);

  /** 是否至少成功查询过一次 TF（用于区分「从未收到 TF」与「临时 TF 失败」）。 */
  bool isTfAvailableInBuffer() const { return firstTfAvailable_; }

 protected:
  virtual bool readParameters();

  bool filterPointCloud(PointCloudType::Ptr pointCloud);
  virtual bool filterPointCloudSensorType(PointCloudType::Ptr pointCloud);
  virtual bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                Eigen::VectorXf& variances) = 0;

  bool updateTransformations(const rclcpp::Time& timeStamp);
  bool transformPointCloud(PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr pointCloudTransformed, const std::string& targetFrame);
  /** 在地图系下按相对底盘高度阈值裁剪点云。 */
  void removePointsOutsideLimits(PointCloudType::ConstPtr reference, std::vector<PointCloudType::Ptr>& pointClouds);

  /** 优先 YAML，否则 `declareOrGetParameter(node, "sensor_processor."+key, def)`。 */
  double getSensorDouble(const std::string& key, double default_value);
  bool getSensorBool(const std::string& key, bool default_value);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  kindr::RotationMatrixD rotationBaseToSensor_;
  kindr::Position3D translationBaseToSensorInBaseFrame_;
  kindr::RotationMatrixD rotationMapToBase_;
  kindr::Position3D translationMapToBaseInMapFrame_;
  /** 最近一帧 successful TF 下的 传感器→地图 变换，供 `ElevationMap::add` 使用。 */
  Eigen::Affine3d transformationSensorToMap_;

  GeneralParameters generalParameters_;

  struct Parameters {
    double ignorePointsUpperThreshold_{std::numeric_limits<double>::infinity()};
    double ignorePointsLowerThreshold_{-std::numeric_limits<double>::infinity()};
    bool applyVoxelGridFilter_{false};
    std::unordered_map<std::string, double> sensorParameters_;
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  std::string sensorFrameId_;
  bool firstTfAvailable_{false};
  YAML::Node sensor_processor_yaml_;
};

}  // namespace elevation_mapping
