/**
 * @file SensorProcessorBase.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 传感器基类：TF 链更新、点云坐标变换、体素/高度裁剪与方差接口默认实现。
 */
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/ParameterHelpers.hpp"

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                         const GeneralParameters& generalConfig)
    : node_(std::move(node)), tf_buffer_(std::move(tf_buffer)), generalParameters_(generalConfig) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  transformationSensorToMap_.setIdentity();
  RCLCPP_DEBUG(node_->get_logger(),
               "Sensor processor: robot_base_frame_id=%s map_frame_id=%s", generalConfig.robotBaseFrameId_.c_str(),
               generalConfig.mapFrameId_.c_str());
}

SensorProcessorBase::~SensorProcessorBase() = default;

double SensorProcessorBase::getSensorDouble(const std::string& key, double default_value) {
  if (sensor_processor_yaml_ && sensor_processor_yaml_[key]) {
    return sensor_processor_yaml_[key].as<double>();
  }
  return declareOrGetParameter(node_.get(), "sensor_processor." + key, default_value);
}

bool SensorProcessorBase::getSensorBool(const std::string& key, bool default_value) {
  if (sensor_processor_yaml_ && sensor_processor_yaml_[key]) {
    return sensor_processor_yaml_[key].as<bool>();
  }
  return declareOrGetParameter(node_.get(), "sensor_processor." + key, default_value);
}

bool SensorProcessorBase::readParameters() {
  Parameters parameters;
  parameters.ignorePointsUpperThreshold_ = getSensorDouble("ignore_points_above", std::numeric_limits<double>::infinity());
  parameters.ignorePointsLowerThreshold_ = getSensorDouble("ignore_points_below", -std::numeric_limits<double>::infinity());
  parameters.applyVoxelGridFilter_ = getSensorBool("apply_voxelgrid_filter", false);
  parameters.sensorParameters_["voxelgrid_filter_size"] = getSensorDouble("voxelgrid_filter_size", 0.0);
  parameters_.setData(parameters);
  return true;
}

bool SensorProcessorBase::process(const PointCloudType::ConstPtr pointCloudInput, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                  const PointCloudType::Ptr pointCloudMapFrame, Eigen::VectorXf& variances, std::string sensorFrame) {
  const Parameters parameters{parameters_.getData()};
  sensorFrameId_ = std::move(sensorFrame);
  RCLCPP_DEBUG(node_->get_logger(), "Sensor processor frame %s", sensorFrameId_.c_str());

  rclcpp::Time timeStamp(static_cast<int64_t>(pointCloudInput->header.stamp) * 1000LL, node_->get_clock()->get_clock_type());
  if (!updateTransformations(timeStamp)) {
    return false;
  }

  PointCloudType::Ptr pointCloudSensorFrame(new PointCloudType);
  transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);

  filterPointCloud(pointCloudSensorFrame);
  filterPointCloudSensorType(pointCloudSensorFrame);

  if (!transformPointCloud(pointCloudSensorFrame, pointCloudMapFrame, generalParameters_.mapFrameId_)) {
    return false;
  }
  std::vector<PointCloudType::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);

  return computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances);
}

bool SensorProcessorBase::updateTransformations(const rclcpp::Time& timeStamp) {
  try {
    const auto timeout = rclcpp::Duration::from_seconds(1.0);
    auto tf_map_sensor =
        tf_buffer_->lookupTransform(generalParameters_.mapFrameId_, sensorFrameId_, timeStamp, timeout);
    transformationSensorToMap_ = Eigen::Affine3d(tf2::transformToEigen(tf_map_sensor.transform));

    auto tf_base_sensor =
        tf_buffer_->lookupTransform(generalParameters_.robotBaseFrameId_, sensorFrameId_, timeStamp, timeout);
    Eigen::Isometry3d transform_base_sensor = tf2::transformToEigen(tf_base_sensor.transform);
    rotationBaseToSensor_.setMatrix(transform_base_sensor.rotation().matrix());
    translationBaseToSensorInBaseFrame_.toImplementation() = transform_base_sensor.translation();

    auto tf_map_base =
        tf_buffer_->lookupTransform(generalParameters_.mapFrameId_, generalParameters_.robotBaseFrameId_, timeStamp, timeout);
    Eigen::Isometry3d transform_map_base = tf2::transformToEigen(tf_map_base.transform);
    rotationMapToBase_.setMatrix(transform_map_base.rotation().matrix());
    translationMapToBaseInMapFrame_.toImplementation() = transform_map_base.translation();

    if (!firstTfAvailable_) {
      firstTfAvailable_ = true;
    }
    return true;
  } catch (const tf2::TransformException& ex) {
    if (!firstTfAvailable_) {
      return false;
    }
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return false;
  }
}

bool SensorProcessorBase::transformPointCloud(PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr pointCloudTransformed,
                                              const std::string& targetFrame) {
  rclcpp::Time timeStamp(static_cast<int64_t>(pointCloud->header.stamp) * 1000LL, node_->get_clock()->get_clock_type());
  const std::string inputFrameId(pointCloud->header.frame_id);

  try {
    const auto timeout = rclcpp::Duration::from_seconds(1.0);
    auto tf_msg = tf_buffer_->lookupTransform(targetFrame, inputFrameId, timeStamp, timeout);
    Eigen::Isometry3d transform = tf2::transformToEigen(tf_msg.transform);
    pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
    pointCloudTransformed->header.frame_id = targetFrame;
    RCLCPP_DEBUG(node_->get_logger(), "Point cloud transformed to frame %s", targetFrame.c_str());
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return false;
  }
}

void SensorProcessorBase::removePointsOutsideLimits(PointCloudType::ConstPtr reference, std::vector<PointCloudType::Ptr>& pointClouds) {
  const Parameters parameters{parameters_.getData()};
  if (!std::isfinite(parameters.ignorePointsLowerThreshold_) && !std::isfinite(parameters.ignorePointsUpperThreshold_)) {
    return;
  }

  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z");
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + parameters.ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + parameters.ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndeces(new std::vector<int>);
  passThroughFilter.filter(*insideIndeces);

  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndeces);
    PointCloudType tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }
}

bool SensorProcessorBase::filterPointCloud(const PointCloudType::Ptr pointCloud) {
  const Parameters parameters{parameters_.getData()};
  PointCloudType tempPointCloud;

  std::vector<int> indices;
  if (!pointCloud->is_dense) {
    pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices);
    tempPointCloud.is_dense = true;
    pointCloud->swap(tempPointCloud);
  }

  if (parameters.applyVoxelGridFilter_) {
    pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio> voxelGridFilter;
    voxelGridFilter.setInputCloud(pointCloud);
    double filter_size = parameters.sensorParameters_.at("voxelgrid_filter_size");
    voxelGridFilter.setLeafSize(filter_size, filter_size, filter_size);
    voxelGridFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }
  RCLCPP_DEBUG(node_->get_logger(), "cleanPointCloud size %zu", pointCloud->size());
  return true;
}

bool SensorProcessorBase::filterPointCloudSensorType(const PointCloudType::Ptr /*pointCloud*/) {
  return true;
}

}  // namespace elevation_mapping
