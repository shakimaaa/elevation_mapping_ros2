/**
 * @file ElevationMapRobotFrameSampler.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 高程图栅格采样并在机器人基座系发布点云。
 */
#include "elevation_mapping/ElevationMapRobotFrameSampler.hpp"
#include "elevation_mapping/ParameterHelpers.hpp"

#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>
#include <boost/thread/lock_guard.hpp>
#include <cmath>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace elevation_mapping {

ElevationMapRobotFrameSampler::ElevationMapRobotFrameSampler(rclcpp::Node::SharedPtr node, ElevationMap& elevation_map,
                                                             std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(std::move(node)), elevation_map_(elevation_map), tf_buffer_(std::move(tf_buffer)) {
  readParameters();
  if (!params_.enable_) {
    return;
  }
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(params_.topic_, rclcpp::SystemDefaultsQoS());
  double rate = params_.publish_rate_hz_;
  if (rate <= 0.0) {
    rate = 1.0;
  }
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate));
  timer_ = node_->create_wall_timer(period, [this]() { timerCallback(); });
  RCLCPP_INFO(node_->get_logger(),
              "ElevationMapRobotFrameSampler: enabled, topic=%s rate=%.2f Hz stride=(%d,%d) fused=%s layer=%s",
              params_.topic_.c_str(), rate, params_.stride_x_, params_.stride_y_, params_.use_fused_map_ ? "true" : "false",
              params_.layer_name_.c_str());
}

void ElevationMapRobotFrameSampler::readParameters() {
  params_.enable_ = declareOrGetParameter(node_.get(), "elevation_sampling.enable", false);
  params_.publish_rate_hz_ = declareOrGetParameter(node_.get(), "elevation_sampling.publish_rate", 2.0);
  params_.topic_ = declareOrGetParameter(node_.get(), "elevation_sampling.topic", std::string("elevation_sampled_cloud"));
  params_.stride_x_ = declareOrGetParameter(node_.get(), "elevation_sampling.stride_x", 1);
  params_.stride_y_ = declareOrGetParameter(node_.get(), "elevation_sampling.stride_y", 1);
  params_.use_fused_map_ = declareOrGetParameter(node_.get(), "elevation_sampling.use_fused_map", true);
  params_.layer_name_ = declareOrGetParameter(node_.get(), "elevation_sampling.layer_name", std::string("elevation"));
  params_.robot_base_frame_id_ = declareOrGetParameter(node_.get(), "robot_base_frame_id", std::string("robot"));
  if (params_.stride_x_ < 1) {
    params_.stride_x_ = 1;
  }
  if (params_.stride_y_ < 1) {
    params_.stride_y_ = 1;
  }
}

void ElevationMapRobotFrameSampler::timerCallback() {
  const std::string& map_frame = elevation_map_.getFrameId();
  if (map_frame.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ElevationMapRobotFrameSampler: map frame id empty, skip");
    return;
  }

  Eigen::Isometry3d T_robot_from_map;
  try {
    const auto timeout = rclcpp::Duration::from_seconds(1.0);
    rclcpp::Time tf_time = elevation_map_.getTimeOfLastFusion();
    if (tf_time.nanoseconds() == 0) {
      tf_time = node_->now();
    }
    auto tf_msg = tf_buffer_->lookupTransform(params_.robot_base_frame_id_, map_frame, tf_time, timeout);
    T_robot_from_map = tf2::transformToEigen(tf_msg.transform);
  } catch (const tf2::TransformException&) {
    try {
      const tf2::Duration tf_timeout = tf2::durationFromSec(1.0);
      auto tf_msg =
          tf_buffer_->lookupTransform(params_.robot_base_frame_id_, map_frame, tf2::TimePointZero, tf_timeout);
      T_robot_from_map = tf2::transformToEigen(tf_msg.transform);
    } catch (const tf2::TransformException& ex2) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ElevationMapRobotFrameSampler: TF failed: %s",
                           ex2.what());
      return;
    }
  }

  boost::recursive_mutex* mutex = params_.use_fused_map_ ? &elevation_map_.getFusedDataMutex() : &elevation_map_.getRawDataMutex();
  boost::lock_guard<boost::recursive_mutex> lock(*mutex);
  grid_map::GridMap& gm = params_.use_fused_map_ ? elevation_map_.getFusedGridMap() : elevation_map_.getRawGridMap();

  if (!gm.exists(params_.layer_name_)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ElevationMapRobotFrameSampler: layer '%s' missing",
                         params_.layer_name_.c_str());
    return;
  }

  const Eigen::Array2i size = gm.getSize();
  std::vector<Eigen::Vector3f> points_robot;
  if (size(0) <= 0 || size(1) <= 0) {
    sensor_msgs::msg::PointCloud2 empty_cloud;
    empty_cloud.header.stamp = node_->now();
    empty_cloud.header.frame_id = params_.robot_base_frame_id_;
    empty_cloud.height = 1;
    empty_cloud.width = 0;
    empty_cloud.is_dense = true;
    empty_cloud.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier mod(empty_cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(0);
    publisher_->publish(empty_cloud);
    return;
  }

  points_robot.reserve(static_cast<size_t>((size(0) / params_.stride_x_ + 1) * (size(1) / params_.stride_y_ + 1)));

  // 点序：与 grid_map 中 getSize()(0)=行、getSize()(1)=列 一致——右 = j 最大，下 = i 最大。
  // 外层 j 从右向左，内层 i 从下向上（同列内 i 递减），故第 0 个点为 (i_max, j_max)。
  const int i_max = ((size(0) - 1) / params_.stride_x_) * params_.stride_x_;
  const int j_max = ((size(1) - 1) / params_.stride_y_) * params_.stride_y_;
  for (int j = j_max; j >= 0; j -= params_.stride_y_) {
    for (int i = i_max; i >= 0; i -= params_.stride_x_) {
      const Eigen::Array2i index(i, j);
      if (!gm.isValid(index)) {
        continue;
      }
      const float h = gm.at(params_.layer_name_, index);
      if (!std::isfinite(h)) {
        continue;
      }
      Eigen::Vector3d p_map;
      if (!gm.getPosition3(params_.layer_name_, index, p_map)) {
        continue;
      }
      const Eigen::Vector3d p_robot = T_robot_from_map * p_map;
      points_robot.emplace_back(p_robot.cast<float>());
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = node_->now();
  cloud.header.frame_id = params_.robot_base_frame_id_;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points_robot.size());
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points_robot.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto& p : points_robot) {
    *iter_x = p.x();
    *iter_y = p.y();
    *iter_z = p.z();
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  publisher_->publish(cloud);
}

}  // namespace elevation_mapping
