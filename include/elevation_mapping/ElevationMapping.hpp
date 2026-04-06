/**
 * @file ElevationMapping.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief ROS 2 版高程地图主节点类：订阅点云与位姿、融合进网格图、提供服务与定时器。
 *
 * 职责概览：
 * - 从参数服务器 / YAML 读取配置，管理 `ElevationMap` 几何与图层参数；
 * - 通过 `InputSourceManager` 从 YAML（`input_sources_file`）加载多路输入，或兼容旧的单话题 `point_cloud_topic`；
 * - 使用 `tf2_ros::Buffer` 做坐标变换与地图随动（跟踪点）；
 * - 使用 `message_filters::Cache` 按时间对齐点云与 `PoseWithCovarianceStamped`；
 * - 将融合、子图查询等暴露为 ROS 2 服务，定时发布融合图与可选的可见性清理。
 *
 * 线程模型：由可执行文件中的 `MultiThreadedExecutor` 驱动本节点上的订阅、服务与定时器回调。
 */
#pragma once

#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_msgs/srv/process_file.hpp>
#include <grid_map_msgs/srv/set_grid_map.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/cache.hpp>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/thread.hpp>
#include <memory>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

/** 地图初始化策略枚举（当前实现平面高度初始化）。 */
enum class InitializationMethods { PlanarFloorInitializer };

class ElevationMapping {
 public:
  /**
   * @brief 构造并启动完整管线：读参、建订阅与服务、定时器、初始化地图。
   * @param node 本功能包使用的 ROS 2 节点（建议节点名 `elevation_mapping`）。
   * @throws std::runtime_error 当 `readParameters()` 失败时（例如传感器类型未知）。
   */
  explicit ElevationMapping(rclcpp::Node::SharedPtr node);
  virtual ~ElevationMapping();

  /**
   * @brief 点云回调：坐标变换、方差计算、运动预测、写入原始高程图并可选择发布。
   * @param pointCloudMsg 输入点云（传感器坐标系由消息 frame_id 指定）。
   * @param publishPointCloud 若为 true，在更新后走原始图后处理/发布逻辑（与各路 Input 的 `publish_on_update` 一致）。
   * @param sensorProcessor 该路输入对应的传感器处理器（内含 TF 与噪声模型参数）。
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg, bool publishPointCloud,
                          const SensorProcessorBase::Ptr& sensorProcessor);

  /** @brief 定时器：在长时间无点云时仍按机器人运动预测更新并发布地图。 */
  void mapUpdateTimerCallback();
  /** @brief 定时器：对融合图做 fuse 并发布（供外部仅订阅融合结果时使用）。 */
  void publishFusedMapCallback();
  /** @brief 定时器：基于可见性射线清理原始图中过时单元。 */
  void visibilityCleanupCallback();

  /** @brief 服务 `trigger_fusion`：立即融合整图并发布。 */
  void fuseEntireMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/);

  /** @brief 服务 `get_submap`：请求融合子图并填入响应中的 GridMap 消息。 */
  void getFusedSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response);

  /** @brief 服务 `get_raw_submap`：请求原始（未融合）子图。 */
  void getRawSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response);

  void enableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/);

  void disableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/);

  void clearMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                               std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/);

  /** @brief 服务 `masked_replace`：按掩码图层将外部 GridMap 写入原始地图重叠区域。 */
  void maskedReplaceServiceCallback(const std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                    std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> /*response*/);

  /** @brief 服务 `save_map`：当前 ROS 2 实现中为占位（rosbag 读写未接好）。 */
  void saveMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  /** @brief 服务 `load_map`：当前 ROS 2 实现中为占位。 */
  void loadMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  /** @brief 服务 `reload_parameters`：热重载参数（几何在 reload 时一般不重建，与原版行为一致）。 */
  void reloadParametersServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /** @brief 供节点入口侧组件（如 `ElevationMapRobotFrameSampler`）访问内部地图。 */
  ElevationMap& getElevationMap() { return map_; }
  const ElevationMap& getElevationMap() const { return map_; }
  /** @brief 与地图更新共用的 TF 缓冲（须与 `getElevationMap()` 同步使用）。 */
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() { return tf_buffer_; }

 private:
  /** @brief 从节点参数读取配置；`reload==true` 时跳过改变地图几何（仅更新标量参数）。 */
  bool readParameters(bool reload = false);
  /** @brief 启动阶段：短暂休眠以利 TF 填充，并重置/初始化地图。 */
  bool initialize();
  void setupSubscribers();
  void setupServices();
  void setupTimers();

  /** @brief 根据机器人位姿协方差推进地图方差（过程噪声），时间应对齐点云时刻。 */
  bool updatePrediction(const rclcpp::Time& time);
  /** @brief 将跟踪点从 `track_point_frame_id` 变换到地图系并 `move` 网格。 */
  bool updateMapLocation();

  /** @brief 根据「距上次更新时间」计算下一次强制更新的单次定时器间隔。 */
  void resetMapUpdateTimer();
  void stopMapUpdateTimer();

  /** @brief 可选：用 TF 在目标帧高度上初始化一块子图高度（平面地面假设）。 */
  bool initializeElevationMap();
  /** @brief 是否在点云回调里连续融合（需有订阅者且参数打开无穷发布率）。 */
  bool isFusingEnabled();

  rclcpp::Node::SharedPtr node_;
  /** TF2 缓冲与监听器：替代 ROS1 的 `tf::TransformListener`。 */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /** 多路 YAML 输入管理；与 `tf_buffer_` 一同传给各 `Input` 构建传感器处理器。 */
  InputSourceManager inputSources_;
  /** 兼容模式：未配置 `input_sources_file` 时单独订阅 `point_cloud_topic`。 */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_{nullptr};

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr fusion_trigger_service_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr fused_submap_service_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr raw_submap_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_updates_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disable_updates_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_map_service_;
  rclcpp::Service<grid_map_msgs::srv::SetGridMap>::SharedPtr masked_replace_service_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr save_map_service_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr load_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_parameters_service_;

  /**
   * @struct Parameters
   * @brief 与 ROS 参数一一对应的运行时配置（由 `ThreadSafeDataWrapper` 保护并发读写）。
   */
  struct Parameters {
    int robotPoseCacheSize_{200};
    std::string mapFrameId_;
    kindr::Position3D trackPoint_;
    std::string trackPointFrameId_;
    std::string pointCloudTopic_;
    std::string robotPoseTopic_;
    bool ignoreRobotMotionUpdates_{false};
    bool updatesEnabled_{true};
    rclcpp::Duration maxNoUpdateDuration_{0, 0};
    rclcpp::Duration timeTolerance_{0, 0};
    rclcpp::Duration fusedMapPublishTimerDuration_{0, 0};
    bool isContinuouslyFusing_{false};
    rclcpp::Duration visibilityCleanupTimerDuration_{0, 0};
    std::string maskedReplaceServiceMaskLayerName_;
    bool initializeElevationMap_{false};
    int initializationMethod_{0};
    double lengthInXInitSubmap_{1.2};
    double lengthInYInitSubmap_{1.8};
    std::string targetFrameInitSubmap_;
    double initSubmapHeightOffset_{0.0};
    double initSubmapVariance_{0.01};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  ElevationMap map_;
  /** 单路兼容模式下的默认传感器处理器；多路时由各个 `Input` 持有独立实例。 */
  SensorProcessorBase::Ptr sensor_processor_;
  RobotMotionMapUpdater robot_motion_map_updater_;

  /** 位姿订阅 + 时间缓存：按点云时间戳查找最近位姿。 */
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>> robot_pose_subscriber_;
  std::unique_ptr<message_filters::Cache<geometry_msgs::msg::PoseWithCovarianceStamped>> robot_pose_cache_;

  rclcpp::TimerBase::SharedPtr map_update_timer_;
  rclcpp::TimerBase::SharedPtr fused_map_publish_timer_;
  rclcpp::TimerBase::SharedPtr visibility_cleanup_timer_;

  /** 上一帧用于融合/可见性清理对齐的点云时间戳。 */
  rclcpp::Time last_point_cloud_update_time_;
  /** 首帧对齐标志：避免在 pose 缓存尚未覆盖点云时间前就开始建图。 */
  bool received_first_matching_pointcloud_and_pose_{false};
};

}  // namespace elevation_mapping
