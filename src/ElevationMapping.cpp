/**
 * @file ElevationMapping.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief ElevationMapping 类实现：ROS 2 参数、订阅/服务/定时器、点云主循环与地图服务。
 *
 * 数据流简述：
 * 1) 点云到达 → 与位姿缓存时间对齐 → SensorProcessor 做 TF 与方差 → updateMapLocation 平移网格 →
 *    updatePrediction 增大不确定性 → map_.add 写入原始层 → 可选后处理发布与融合。
 * 2) 无点云超过 max_no_update_duration 时，定时器触发仅运动预测 + 发布（保持地图随机器人漂移更新）。
 * 3) 子图服务在持锁下 fuseArea/getSubmap，避免与正在写入的原始图严重竞态（融合图单独 mutex）。
 */
#include "elevation_mapping/ElevationMapping.hpp"

#include <functional>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "elevation_mapping/KindrRosCompat.hpp"
#include "elevation_mapping/ParameterHelpers.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

/** 构造：创建 TF、输入管理器、地图与运动更新器，顺序读参、建通信、initialize。 */
ElevationMapping::ElevationMapping(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false)),
      inputSources_(node_, tf_buffer_),
      map_(node_),
      robot_motion_map_updater_(node_) {
  last_point_cloud_update_time_ = node_->now();
#ifndef NDEBUG
  RCLCPP_WARN(node_->get_logger(), "Debug build: use Release for better performance.");
#endif
  RCLCPP_INFO(node_->get_logger(), "Elevation mapping (ROS 2) started.");
  if (!readParameters()) {
    throw std::runtime_error("elevation_mapping: readParameters() failed.");
  }
  setupSubscribers();
  setupServices();
  setupTimers();
  initialize();
  RCLCPP_INFO(node_->get_logger(), "Elevation mapping ready.");
}

/** 析构前取消各 wall timer，避免 spin 结束后仍触发回调。 */
ElevationMapping::~ElevationMapping() {
  if (map_update_timer_) {
    map_update_timer_->cancel();
  }
  if (fused_map_publish_timer_) {
    fused_map_publish_timer_->cancel();
  }
  if (visibility_cleanup_timer_) {
    visibility_cleanup_timer_->cancel();
  }
}

/**
 * 从节点读取全部标量参数并写入 ThreadSafe 的 Parameters；同时配置默认 sensor_processor_（单路兼容用）。
 * reload==true 时不改地图几何（length/resolution），便于热重载噪声与发布频率等。
 */
bool ElevationMapping::readParameters(bool reload) {
  auto [parameters, parametersGuard] = parameters_.getDataToWrite();
  auto [mapParameters, mapParametersGuard] = map_.parameters_.getDataToWrite();

  parameters.pointCloudTopic_ = declareOrGetParameter(node_.get(), "point_cloud_topic", std::string("/points"));
  parameters.robotPoseTopic_ = declareOrGetParameter(node_.get(), "robot_pose_with_covariance_topic", std::string("/pose"));
  parameters.trackPointFrameId_ = declareOrGetParameter(node_.get(), "track_point_frame_id", std::string("robot"));
  parameters.trackPoint_.x() = declareOrGetParameter(node_.get(), "track_point_x", 0.0);
  parameters.trackPoint_.y() = declareOrGetParameter(node_.get(), "track_point_y", 0.0);
  parameters.trackPoint_.z() = declareOrGetParameter(node_.get(), "track_point_z", 0.0);
  parameters.robotPoseCacheSize_ = declareOrGetParameter(node_.get(), "robot_pose_cache_size", 200);

  double minUpdateRate = declareOrGetParameter(node_.get(), "min_update_rate", 2.0);
  if (minUpdateRate == 0.0) {
    parameters.maxNoUpdateDuration_ = rclcpp::Duration(0, 0);
    RCLCPP_WARN(node_->get_logger(), "min_update_rate is zero.");
  } else {
    parameters.maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(1.0 / minUpdateRate);
  }

  double timeTolerance = declareOrGetParameter(node_.get(), "time_tolerance", 0.0);
  parameters.timeTolerance_ = rclcpp::Duration::from_seconds(timeTolerance);

  double fusedMapPublishingRate = declareOrGetParameter(node_.get(), "fused_map_publishing_rate", 1.0);
  if (fusedMapPublishingRate == 0.0) {
    parameters.fusedMapPublishTimerDuration_ = rclcpp::Duration(0, 0);
    RCLCPP_WARN(node_->get_logger(), "fused_map_publishing_rate is zero; fused map only via service.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    parameters.isContinuouslyFusing_ = true;
    parameters.fusedMapPublishTimerDuration_ = rclcpp::Duration(0, 0);
  } else {
    parameters.fusedMapPublishTimerDuration_ = rclcpp::Duration::from_seconds(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate = declareOrGetParameter(node_.get(), "visibility_cleanup_rate", 1.0);
  if (visibilityCleanupRate == 0.0) {
    parameters.visibilityCleanupTimerDuration_ = rclcpp::Duration(0, 0);
  } else {
    parameters.visibilityCleanupTimerDuration_ = rclcpp::Duration::from_seconds(1.0 / visibilityCleanupRate);
    mapParameters.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  parameters.mapFrameId_ = declareOrGetParameter(node_.get(), "map_frame_id", std::string("map"));

  grid_map::Length length;
  grid_map::Position position;
  double resolution = declareOrGetParameter(node_.get(), "resolution", 0.01);
  length(0) = declareOrGetParameter(node_.get(), "length_in_x", 1.5);
  length(1) = declareOrGetParameter(node_.get(), "length_in_y", 1.5);
  position.x() = declareOrGetParameter(node_.get(), "position_x", 0.0);
  position.y() = declareOrGetParameter(node_.get(), "position_y", 0.0);

  if (!reload) {
    map_.setFrameId(parameters.mapFrameId_);
    map_.setGeometry(length, resolution, position);
  }

  mapParameters.minVariance_ = declareOrGetParameter(node_.get(), "min_variance", std::pow(0.003, 2));
  mapParameters.maxVariance_ = declareOrGetParameter(node_.get(), "max_variance", std::pow(0.03, 2));
  mapParameters.mahalanobisDistanceThreshold_ = declareOrGetParameter(node_.get(), "mahalanobis_distance_threshold", 2.5);
  mapParameters.multiHeightNoise_ = declareOrGetParameter(node_.get(), "multi_height_noise", std::pow(0.003, 2));
  mapParameters.minHorizontalVariance_ = declareOrGetParameter(node_.get(), "min_horizontal_variance", std::pow(resolution / 2.0, 2));
  mapParameters.maxHorizontalVariance_ = declareOrGetParameter(node_.get(), "max_horizontal_variance", 0.5);
  mapParameters.underlyingMapTopic_ = declareOrGetParameter(node_.get(), "underlying_map_topic", std::string());
  mapParameters.enableVisibilityCleanup_ = declareOrGetParameter(node_.get(), "enable_visibility_cleanup", true);
  mapParameters.enableContinuousCleanup_ = declareOrGetParameter(node_.get(), "enable_continuous_cleanup", false);
  mapParameters.scanningDuration_ = declareOrGetParameter(node_.get(), "scanning_duration", 1.0);
  mapParameters.increaseHeightAlpha_ = declareOrGetParameter(node_.get(), "increase_height_alpha", 0.0);
  parameters.maskedReplaceServiceMaskLayerName_ =
      declareOrGetParameter(node_.get(), "masked_replace_service_mask_layer_name", std::string("mask"));

  parameters.initializeElevationMap_ = declareOrGetParameter(node_.get(), "initialize_elevation_map", false);
  parameters.initializationMethod_ = declareOrGetParameter(node_.get(), "initialization_method", 0);
  parameters.lengthInXInitSubmap_ = declareOrGetParameter(node_.get(), "length_in_x_init_submap", 1.2);
  parameters.lengthInYInitSubmap_ = declareOrGetParameter(node_.get(), "length_in_y_init_submap", 1.8);
  parameters.initSubmapHeightOffset_ = declareOrGetParameter(node_.get(), "init_submap_height_offset", 0.0);
  parameters.initSubmapVariance_ = declareOrGetParameter(node_.get(), "init_submap_variance", 0.01);
  parameters.targetFrameInitSubmap_ = declareOrGetParameter(node_.get(), "target_frame_init_submap", std::string("footprint"));

  std::string sensorType = declareOrGetParameter(node_.get(), "sensor_processor.type", std::string("structured_light"));
  SensorProcessorBase::GeneralParameters gen{declareOrGetParameter(node_.get(), "robot_base_frame_id", std::string("robot")),
                                           parameters.mapFrameId_};
  if (sensorType == "structured_light") {
    sensor_processor_ = std::make_unique<StructuredLightSensorProcessor>(node_, tf_buffer_, gen);
  } else if (sensorType == "stereo") {
    sensor_processor_ = std::make_unique<StereoSensorProcessor>(node_, tf_buffer_, gen);
  } else if (sensorType == "laser") {
    sensor_processor_ = std::make_unique<LaserSensorProcessor>(node_, tf_buffer_, gen);
  } else if (sensorType == "perfect") {
    sensor_processor_ = std::make_unique<PerfectSensorProcessor>(node_, tf_buffer_, gen);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown sensor_processor.type: %s", sensorType.c_str());
    return false;
  }
  if (!sensor_processor_->readParameters()) {
    return false;
  }
  if (!robot_motion_map_updater_.readParameters()) {
    return false;
  }
  return true;
}

/** 多路 YAML 优先；否则若仍存在 point_cloud_topic 则单订阅；另建位姿 Cache 供时间同步。 */
void ElevationMapping::setupSubscribers() {
  auto [parameters, parameterGuard] = parameters_.getDataToWrite();
  const bool configured_input_sources = inputSources_.configureFromRos();
  const bool has_deprecated_pc = node_->has_parameter("point_cloud_topic");
  if (has_deprecated_pc) {
    RCLCPP_WARN(node_->get_logger(), "point_cloud_topic is deprecated; use input_sources_file + YAML.");
  }
  if (!configured_input_sources && has_deprecated_pc) {
    point_cloud_subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        parameters.pointCloudTopic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { pointCloudCallback(msg, true, sensor_processor_); });
  }
  if (configured_input_sources) {
    inputSources_.registerWith(*this);
  }

  if (!parameters.robotPoseTopic_.empty()) {
    robot_pose_subscriber_ =
        std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>>(
            node_, parameters.robotPoseTopic_, rclcpp::QoS(10).get_rmw_qos_profile());
    robot_pose_cache_ = std::make_unique<message_filters::Cache<geometry_msgs::msg::PoseWithCovarianceStamped>>(
        *robot_pose_subscriber_, static_cast<unsigned int>(parameters.robotPoseCacheSize_));
  } else {
    parameters.ignoreRobotMotionUpdates_ = true;
  }
}

/** 注册与 ROS1 同名服务（trigger_fusion、get_submap、masked_replace 等）。 */
void ElevationMapping::setupServices() {
  fusion_trigger_service_ = node_->create_service<std_srvs::srv::Empty>(
      "trigger_fusion", std::bind(&ElevationMapping::fuseEntireMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  fused_submap_service_ = node_->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_submap",
      std::bind(&ElevationMapping::getFusedSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  raw_submap_service_ = node_->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_raw_submap",
      std::bind(&ElevationMapping::getRawSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  clear_map_service_ = node_->create_service<std_srvs::srv::Empty>(
      "clear_map", std::bind(&ElevationMapping::clearMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  enable_updates_service_ = node_->create_service<std_srvs::srv::Empty>(
      "enable_updates", std::bind(&ElevationMapping::enableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  disable_updates_service_ = node_->create_service<std_srvs::srv::Empty>(
      "disable_updates", std::bind(&ElevationMapping::disableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  masked_replace_service_ = node_->create_service<grid_map_msgs::srv::SetGridMap>(
      "masked_replace", std::bind(&ElevationMapping::maskedReplaceServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  save_map_service_ = node_->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_map", std::bind(&ElevationMapping::saveMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  load_map_service_ = node_->create_service<grid_map_msgs::srv::ProcessFile>(
      "load_map", std::bind(&ElevationMapping::loadMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  reload_parameters_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "reload_parameters",
      std::bind(&ElevationMapping::reloadParametersServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}

/** 地图强制更新定时器 + 可选融合发布周期定时器 + 可见性清理定时器。 */
void ElevationMapping::setupTimers() {
  const Parameters parameters{parameters_.getData()};
  const ElevationMap::Parameters mapParameters{map_.parameters_.getData()};
  resetMapUpdateTimer();

  if (parameters.fusedMapPublishTimerDuration_.nanoseconds() > 0) {
    fused_map_publish_timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(parameters.fusedMapPublishTimerDuration_.seconds())),
        std::bind(&ElevationMapping::publishFusedMapCallback, this));
  }

  if (mapParameters.enableVisibilityCleanup_ && parameters.visibilityCleanupTimerDuration_.nanoseconds() > 0 &&
      !mapParameters.enableContinuousCleanup_) {
    visibility_cleanup_timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(parameters.visibilityCleanupTimerDuration_.seconds())),
        std::bind(&ElevationMapping::visibilityCleanupCallback, this));
  }
}

/** 启动时短暂等待 TF 稳定，再重置定时器并可选执行高程初始化。 */
bool ElevationMapping::initialize() {
  RCLCPP_INFO(node_->get_logger(), "Initializing...");
  rclcpp::sleep_for(std::chrono::seconds(1));
  resetMapUpdateTimer();
  initializeElevationMap();
  return true;
}

/**
 * 点云主路径：禁用时仅刷新时间戳并可能发布；否则转换 PCL、查位姿协方差、处理点云、
 * 加锁更新地图位置与预测、写入高度、按需融合与发布。
 */
void ElevationMapping::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg, bool publishPointCloud,
                                          const SensorProcessorBase::Ptr& sensor_proc) {
  const Parameters parameters{parameters_.getData()};
  if (!parameters.updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Updates disabled.");
    if (publishPointCloud) {
      map_.setTimestamp(node_->now());
      map_.postprocessAndPublishRawElevationMap();
    }
    return;
  }

  if (!robot_pose_cache_) {
    received_first_matching_pointcloud_and_pose_ = true;
  }
  if (robot_pose_cache_ && !received_first_matching_pointcloud_and_pose_) {
    const double oldest = robot_pose_cache_->getOldestTime().seconds();
    const double pc_time = rclcpp::Time(pointCloudMsg->header.stamp, node_->get_clock()->get_clock_type()).seconds();
    if (pc_time < oldest) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Waiting for first pose/point cloud match.");
      return;
    }
    received_first_matching_pointcloud_and_pose_ = true;
    RCLCPP_INFO(node_->get_logger(), "First pose/point cloud match; mapping started.");
  }

  stopMapUpdateTimer();

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);
  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  last_point_cloud_update_time_ = rclcpp::Time(pointCloudMsg->header.stamp, node_->get_clock()->get_clock_type());

  Eigen::Matrix<double, 6, 6> robotPoseCovariance = Eigen::Matrix<double, 6, 6>::Zero();
  if (!parameters.ignoreRobotMotionUpdates_ && robot_pose_cache_) {
    auto poseMessage = robot_pose_cache_->getElemBeforeTime(last_point_cloud_update_time_);
    if (!poseMessage) {
      RCLCPP_ERROR(node_->get_logger(), "No pose for point cloud time.");
      resetMapUpdateTimer();
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);
  }

  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
  if (!sensor_proc->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances, pointCloudMsg->header.frame_id)) {
    if (!sensor_proc->isTfAvailableInBuffer()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Waiting for TF...");
      return;
    }
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000, "Point cloud processing failed.");
    resetMapUpdateTimer();
    return;
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());
  if (!updateMapLocation()) {
    resetMapUpdateTimer();
    return;
  }
  if (!updatePrediction(last_point_cloud_update_time_)) {
    RCLCPP_ERROR(node_->get_logger(), "updatePrediction failed.");
    resetMapUpdateTimer();
    return;
  }

  const ElevationMap::Parameters mapParameters{map_.parameters_.getData()};
  if (mapParameters.enableContinuousCleanup_) {
    map_.clear();
  }

  if (!map_.add(pointCloudProcessed, measurementVariances, last_point_cloud_update_time_,
                Eigen::Affine3d(sensor_proc->transformationSensorToMap_))) {
    RCLCPP_ERROR(node_->get_logger(), "map_.add failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    map_.postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      boost::recursive_mutex::scoped_lock fl(map_.getFusedDataMutex());
      map_.fuseAll();
      map_.publishFusedElevationMap();
    }
  }
  resetMapUpdateTimer();
}

/** 超时无传感器时：只做运动预测并发布（与点云路径尾部逻辑对称）。 */
void ElevationMapping::mapUpdateTimerCallback() {
  const Parameters parameters{parameters_.getData()};
  if (!parameters.updatesEnabled_) {
    map_.setTimestamp(node_->now());
    map_.postprocessAndPublishRawElevationMap();
    return;
  }
  rclcpp::Time time = node_->now();
  if ((time - last_point_cloud_update_time_) <= parameters.maxNoUpdateDuration_) {
    return;
  }
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Map update without sensor data.");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());
  stopMapUpdateTimer();
  if (!updatePrediction(time)) {
    resetMapUpdateTimer();
    return;
  }
  map_.postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    boost::recursive_mutex::scoped_lock fl(map_.getFusedDataMutex());
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }
  resetMapUpdateTimer();
}

/** 周期性全图融合并发布（仅在有融合图订阅者时才有意义）。 */
void ElevationMapping::publishFusedMapCallback() {
  if (!map_.hasFusedMapSubscribers()) {
    return;
  }
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

/** 委托 ElevationMap 基于可见性模型清理过时栅格。 */
void ElevationMapping::visibilityCleanupCallback() {
  map_.visibilityCleanup(last_point_cloud_update_time_);
}

/** 服务触发：立即 fuseAll + 发布融合图。 */
void ElevationMapping::fuseEntireMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

/** 无穷融合发布率 + 有订阅者时在点云回调内连续融合。 */
bool ElevationMapping::isFusingEnabled() {
  const Parameters parameters{parameters_.getData()};
  return parameters.isContinuouslyFusing_ && map_.hasFusedMapSubscribers();
}

/** 用 robot_pose_cache_ 在 time 时刻的位姿调用 RobotMotionMapUpdater 更新方差相关层。 */
bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  const Parameters parameters{parameters_.getData()};
  if (parameters.ignoreRobotMotionUpdates_) {
    return true;
  }
  if (!robot_pose_cache_) {
    return true;
  }

  if (time + parameters.timeTolerance_ < map_.getTimeOfLastUpdate()) {
    RCLCPP_ERROR(node_->get_logger(), "updatePrediction: time before last update.");
    return false;
  }
  if (time < map_.getTimeOfLastUpdate()) {
    return true;
  }

  auto poseMessage = robot_pose_cache_->getElemBeforeTime(time);
  if (!poseMessage) {
    RCLCPP_ERROR(node_->get_logger(), "No pose for prediction time.");
    return false;
  }
  kindr::HomTransformQuatD robotPose;
  kindr_compat::poseMsgToHomTransform(poseMessage->pose.pose, robotPose);
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);
  robot_motion_map_updater_.update(map_, robotPose, robotPoseCovariance, time);
  return true;
}

/** 将跟踪点从 track 帧变换到地图帧，得到平面位移并 map_.move。 */
bool ElevationMapping::updateMapLocation() {
  const Parameters parameters{parameters_.getData()};
  geometry_msgs::msg::PointStamped in;
  in.header.frame_id = parameters.trackPointFrameId_;
  in.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  kindr_compat::position3dToPointMsg(parameters.trackPoint_, in.point);
  try {
    geometry_msgs::msg::PointStamped out = tf_buffer_->transform(in, map_.getFrameId(), tf2::durationFromSec(1.0));
    kindr::Position3D position3d;
    kindr_compat::pointMsgToPosition3d(out.point, position3d);
    map_.move(position3d.vector().head(2));
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return false;
  }
}

/** 先 fuse 请求矩形再截取子图填入响应（图层列表可筛选）。 */
void ElevationMapping::getFusedSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);
  bool isSuccess{false};
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();
  if (request->layers.empty()) {
    auto msg = grid_map::GridMapRosConverter::toMessage(subMap);
    response->map = *msg;
  } else {
    auto msg = grid_map::GridMapRosConverter::toMessage(subMap, request->layers);
    response->map = *msg;
  }
  (void)isSuccess;
}

/** 直接从原始图截取子图，不做 fuse。 */
void ElevationMapping::getRawSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());
  bool isSuccess{false};
  grid_map::GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  scopedLock.unlock();
  if (request->layers.empty()) {
    auto msg = grid_map::GridMapRosConverter::toMessage(subMap);
    response->map = *msg;
  } else {
    auto msg = grid_map::GridMapRosConverter::toMessage(subMap, request->layers);
    response->map = *msg;
  }
  (void)isSuccess;
}

/** 关闭点云写入（定时器路径仍会发布占位/空更新行为由 mapUpdateTimerCallback 处理）。 */
void ElevationMapping::disableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                                     std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  auto [p, g] = parameters_.getDataToWrite();
  p.updatesEnabled_ = false;
}

/** 恢复点云对地图的写入。 */
void ElevationMapping::enableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  auto [p, g] = parameters_.getDataToWrite();
  p.updatesEnabled_ = true;
}

/** 平面初始化：查 map→target 的 TF，在机器人附近子矩形写入近似地面高度。 */
bool ElevationMapping::initializeElevationMap() {
  const Parameters parameters{parameters_.getData()};
  if (!parameters.initializeElevationMap_) {
    return true;
  }
  if (static_cast<InitializationMethods>(parameters.initializationMethod_) != InitializationMethods::PlanarFloorInitializer) {
    return true;
  }
  try {
    auto tf_msg = tf_buffer_->lookupTransform(parameters.mapFrameId_, parameters.targetFrameInitSubmap_, tf2::TimePointZero,
                                              tf2::durationFromSec(5.0));
    const grid_map::Position positionRobot(tf_msg.transform.translation.x, tf_msg.transform.translation.y);
    map_.move(positionRobot);
    map_.setRawSubmapHeight(positionRobot, static_cast<float>(tf_msg.transform.translation.z + parameters.initSubmapHeightOffset_),
                            static_cast<float>(parameters.initSubmapVariance_), parameters.lengthInXInitSubmap_,
                            parameters.lengthInYInitSubmap_);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Elevation init skipped: %s", ex.what());
    return false;
  }
}

/** 清空原始/融合数据并重新尝试高程初始化。 */
void ElevationMapping::clearMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                               std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  map_.clear();
  initializeElevationMap();
}

/** 按 mask 图层将请求 GridMap 各层拷贝到当前 raw 图重叠区域。 */
void ElevationMapping::maskedReplaceServiceCallback(const std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                                    std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> /*response*/) {
  const Parameters parameters{parameters_.getData()};
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request->map, sourceMap);
  grid_map::Matrix mask;
  if (sourceMap.exists(parameters.maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[parameters.maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }
  boost::recursive_mutex::scoped_lock scopedLockRawData(map_.getRawDataMutex());
  for (const auto& layerName : sourceMap.getLayers()) {
    if (layerName == parameters.maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    if (!map_.getRawGridMap().exists(layerName)) {
      RCLCPP_ERROR(node_->get_logger(), "Layer %s missing.", layerName.c_str());
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[layerName];
    grid_map::Matrix& destinationLayer = map_.getRawGridMap()[layerName];
    for (grid_map::GridMapIterator it(map_.getRawGridMap()); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map_.getRawGridMap().getPosition(*it, position);
      if (!sourceMap.isInside(position)) {
        continue;
      }
      grid_map::Index srcIdx;
      sourceMap.getIndex(position, srcIdx);
      grid_map::Index dstIdx(*it);
      if (!std::isnan(mask(srcIdx(0), srcIdx(1)))) {
        destinationLayer(dstIdx(0), dstIdx(1)) = sourceLayer(srcIdx(0), srcIdx(1));
      }
    }
  }
}

/** ROS2 占位：未接 rosbag2 持久化。 */
void ElevationMapping::saveMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> /*request*/,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_WARN(node_->get_logger(), "save_map (rosbag) not implemented for ROS 2 in this build.");
  response->success = false;
}

/** ROS2 占位：未接 rosbag2 加载。 */
void ElevationMapping::loadMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> /*request*/,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_WARN(node_->get_logger(), "load_map (rosbag) not implemented for ROS 2 in this build.");
  response->success = false;
}

/** 调用 readParameters(true)；失败则恢复调用前参数快照。 */
void ElevationMapping::reloadParametersServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  Parameters fallback{parameters_.getData()};
  const bool ok = readParameters(true);
  response->success = ok;
  response->message = ok ? "Reloaded parameters." : "Reload failed; reverted.";
  if (!ok) {
    parameters_.setData(fallback);
  }
}

/** 根据「距上次地图更新时间」计算单次 wall timer，在 max_no_update_duration 边界触发无传感器更新。 */
void ElevationMapping::resetMapUpdateTimer() {
  const Parameters parameters{parameters_.getData()};
  if (map_update_timer_) {
    map_update_timer_->cancel();
  }
  rclcpp::Duration periodSinceLastUpdate = node_->now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > parameters.maxNoUpdateDuration_) {
    periodSinceLastUpdate = rclcpp::Duration(0, 0);
  }
  rclcpp::Duration fireIn = parameters.maxNoUpdateDuration_ - periodSinceLastUpdate;
  if (fireIn.nanoseconds() < 0) {
    fireIn = rclcpp::Duration(0, 0);
  }
  const int64_t ns = std::max<int64_t>(fireIn.nanoseconds(), 1LL);
  map_update_timer_ = node_->create_wall_timer(std::chrono::nanoseconds(ns),
                                                 std::bind(&ElevationMapping::mapUpdateTimerCallback, this));
}

/** 点云处理进行中取消定时器，避免与回调重入。 */
void ElevationMapping::stopMapUpdateTimer() {
  if (map_update_timer_) {
    map_update_timer_->cancel();
  }
}

}  // namespace elevation_mapping
