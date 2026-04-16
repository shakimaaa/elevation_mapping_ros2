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
#include <visualization_msgs/msg/marker.hpp>

namespace elevation_mapping {

ElevationMapRobotFrameSampler::ElevationMapRobotFrameSampler(rclcpp::Node::SharedPtr node, ElevationMap& elevation_map,
                                                             std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(std::move(node)), elevation_map_(elevation_map), tf_buffer_(std::move(tf_buffer)) {
  readParameters();
  if (!params_.enable_) {
    return;
  }
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(params_.topic_, rclcpp::SystemDefaultsQoS());
  if (params_.publish_index_markers_) {
    marker_publisher_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(params_.marker_topic_, rclcpp::SystemDefaultsQoS());
  }
  double rate = params_.publish_rate_hz_;
  if (rate <= 0.0) {
    rate = 1.0;
  }
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate));
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = node_->create_wall_timer(period, [this]() { timerCallback(); }, callback_group_);
  RCLCPP_INFO(node_->get_logger(),
              "ElevationMapRobotFrameSampler: enabled, topic=%s rate=%.2f Hz samples=(lat:%d,long:%d) window=(%.2f,%.2f) fused=%s layer=%s",
              params_.topic_.c_str(), rate, params_.lateral_samples_, params_.longitudinal_samples_, params_.lateral_length_,
              params_.longitudinal_length_, params_.use_fused_map_ ? "true" : "false", params_.layer_name_.c_str());
}

void ElevationMapRobotFrameSampler::readParameters() {
  params_.enable_ = declareOrGetParameter(node_.get(), "elevation_sampling.enable", false);
  params_.publish_rate_hz_ = declareOrGetParameter(node_.get(), "elevation_sampling.publish_rate", 2.0);
  params_.topic_ = declareOrGetParameter(node_.get(), "elevation_sampling.topic", std::string("elevation_sampled_cloud"));
  params_.use_fused_map_ = declareOrGetParameter(node_.get(), "elevation_sampling.use_fused_map", true);
  params_.layer_name_ = declareOrGetParameter(node_.get(), "elevation_sampling.layer_name", std::string("elevation"));
  params_.robot_base_frame_id_ = declareOrGetParameter(node_.get(), "robot_base_frame_id", std::string("robot"));
  params_.invalid_height_fill_mode_ =
      declareOrGetParameter(node_.get(), "elevation_sampling.invalid_height_fill_mode", std::string("none"));
  params_.invalid_height_body_offset_ =
      declareOrGetParameter(node_.get(), "elevation_sampling.invalid_height_body_offset", 0.0);
  params_.rotate_output_with_robot_attitude_ =
      declareOrGetParameter(node_.get(), "elevation_sampling.rotate_output_with_robot_attitude", true);
  params_.publish_index_markers_ =
      declareOrGetParameter(node_.get(), "elevation_sampling.publish_index_markers", false);
  params_.marker_topic_ = declareOrGetParameter(node_.get(), "elevation_sampling.marker_topic",
                                                std::string("elevation_sampled_cloud_indices"));
  params_.marker_scale_ = declareOrGetParameter(node_.get(), "elevation_sampling.marker_scale", 0.06);
  params_.longitudinal_length_ = declareOrGetParameter(node_.get(), "elevation_sampling.longitudinal_length", 2.5);
  params_.lateral_length_ = declareOrGetParameter(node_.get(), "elevation_sampling.lateral_length", 1.5);
  params_.longitudinal_samples_ = declareOrGetParameter(node_.get(), "elevation_sampling.longitudinal_samples", 26);
  params_.lateral_samples_ = declareOrGetParameter(node_.get(), "elevation_sampling.lateral_samples", 16);
  if (params_.longitudinal_length_ < 0.0) {
    params_.longitudinal_length_ = 0.0;
  }
  if (params_.lateral_length_ < 0.0) {
    params_.lateral_length_ = 0.0;
  }
  if (params_.longitudinal_samples_ < 1) {
    params_.longitudinal_samples_ = 1;
  }
  if (params_.lateral_samples_ < 1) {
    params_.lateral_samples_ = 1;
  }
  if (params_.marker_scale_ <= 0.0) {
    params_.marker_scale_ = 0.06;
  }
}

void ElevationMapRobotFrameSampler::timerCallback() {
  if (!publisher_ || publisher_->get_subscription_count() == 0) {
    return;
  }

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

  std::vector<Eigen::Vector3f> points_robot;
  points_robot.reserve(static_cast<size_t>(params_.lateral_samples_ * params_.longitudinal_samples_));

  const double lateral_step =
      params_.lateral_samples_ > 1 ? params_.lateral_length_ / static_cast<double>(params_.lateral_samples_ - 1) : 0.0;
  const double longitudinal_step = params_.longitudinal_samples_ > 1
                                       ? params_.longitudinal_length_ / static_cast<double>(params_.longitudinal_samples_ - 1)
                                       : 0.0;
  const double half_lateral = 0.5 * params_.lateral_length_;
  const double half_longitudinal = 0.5 * params_.longitudinal_length_;
  const Eigen::Isometry3d T_map_from_robot = T_robot_from_map.inverse();
  bool has_last_valid_height = false;
  float last_valid_height = 0.0f;

  {
    boost::recursive_mutex* mutex = params_.use_fused_map_ ? &elevation_map_.getFusedDataMutex() : &elevation_map_.getRawDataMutex();
    boost::lock_guard<boost::recursive_mutex> lock(*mutex);
    grid_map::GridMap& gm = params_.use_fused_map_ ? elevation_map_.getFusedGridMap() : elevation_map_.getRawGridMap();

    if (!gm.exists(params_.layer_name_)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ElevationMapRobotFrameSampler: layer '%s' missing",
                           params_.layer_name_.c_str());
      return;
    }
    if (gm.getSize()(0) <= 0 || gm.getSize()(1) <= 0) {
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

    // 点序固定为 right-back 开始；列方向从右到左，行方向从后到前。
    for (int lateral_idx = 0; lateral_idx < params_.lateral_samples_; ++lateral_idx) {
      const double y_robot = -half_lateral + static_cast<double>(lateral_idx) * lateral_step;
      for (int longitudinal_idx = 0; longitudinal_idx < params_.longitudinal_samples_; ++longitudinal_idx) {
        const double x_robot = -half_longitudinal + static_cast<double>(longitudinal_idx) * longitudinal_step;
        const Eigen::Vector3d p_robot(x_robot, y_robot, 0.0);
        const Eigen::Vector3d p_map_xy = T_map_from_robot * p_robot;
        const grid_map::Position sample_position(p_map_xy.x(), p_map_xy.y());
        const auto append_filled_point = [&]() {
          if (params_.invalid_height_fill_mode_ == "last_valid" && has_last_valid_height) {
            points_robot.emplace_back(Eigen::Vector3f(static_cast<float>(x_robot), static_cast<float>(y_robot), last_valid_height));
            return true;
          }
          if (params_.invalid_height_fill_mode_ == "body") {
            const Eigen::Vector3d body_height_map(sample_position.x(), sample_position.y(), 0.0);
            const Eigen::Vector3d filled_point_robot = T_robot_from_map * body_height_map;
            if (params_.rotate_output_with_robot_attitude_) {
              Eigen::Vector3f filled_point = filled_point_robot.cast<float>();
              filled_point.z() += static_cast<float>(params_.invalid_height_body_offset_);
              points_robot.emplace_back(filled_point);
            } else {
              points_robot.emplace_back(Eigen::Vector3f(static_cast<float>(x_robot), static_cast<float>(y_robot),
                                                        static_cast<float>(filled_point_robot.z() + params_.invalid_height_body_offset_)));
            }
            return true;
          }
          return false;
        };
        if (!gm.isInside(sample_position)) {
          append_filled_point();
          continue;
        }
        grid_map::Index index;
        if (!gm.getIndex(sample_position, index)) {
          append_filled_point();
          continue;
        }
        if (!gm.isValid(index)) {
          append_filled_point();
          continue;
        }
        const float h = gm.at(params_.layer_name_, index);
        if (!std::isfinite(h)) {
          append_filled_point();
          continue;
        }
        const Eigen::Vector3d p_map(sample_position.x(), sample_position.y(), static_cast<double>(h));
        const Eigen::Vector3d sampled_point_robot = T_robot_from_map * p_map;
        if (params_.rotate_output_with_robot_attitude_) {
          points_robot.emplace_back(sampled_point_robot.cast<float>());
          last_valid_height = static_cast<float>(sampled_point_robot.z());
        } else {
          // 保持输出点严格落在 body 局部规则网格上，以保证 x/y 间距固定，仅使用姿态变换后的 z。
          points_robot.emplace_back(Eigen::Vector3f(static_cast<float>(x_robot), static_cast<float>(y_robot),
                                                    static_cast<float>(sampled_point_robot.z())));
          last_valid_height = static_cast<float>(sampled_point_robot.z());
        }
        has_last_valid_height = true;
      }
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

  if (marker_publisher_) {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(points_robot.size() + 1);

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = cloud.header;
    clear_marker.ns = "elevation_sample_indices";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    for (size_t i = 0; i < points_robot.size(); ++i) {
      const auto& p = points_robot[i];
      visualization_msgs::msg::Marker marker;
      marker.header = cloud.header;
      marker.ns = "elevation_sample_indices";
      marker.id = static_cast<int32_t>(i + 1);
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = p.x();
      marker.pose.position.y = p.y();
      marker.pose.position.z = p.z() + params_.marker_scale_;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = params_.marker_scale_;
      marker.color.a = 1.0f;
      marker.color.r = 1.0f;
      marker.color.g = 0.2f;
      marker.color.b = 0.2f;
      marker.text = std::to_string(i);
      marker_array.markers.push_back(std::move(marker));
    }

    marker_publisher_->publish(marker_array);
  }
}

}  // namespace elevation_mapping
