/**
 * @file ElevationMap.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 以 grid_map 存储的机器人中心高程图：原始层、融合层、可见性调试层及发布逻辑。
 *
 * 主要图层（原始）：elevation / variance / 水平方差 / color / time / 扫描辅助层等；
 * 融合层：elevation / upper_bound / lower_bound / color。
 * 原始图经 `PostprocessorPool` 可选滤波链后发布到 `elevation_map_raw`（由后处理池配置）；
 * 融合图发布到 `elevation_map`。
 */
#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/thread/recursive_mutex.hpp>
#include <kindr/Core>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

namespace elevation_mapping {

class ElevationMap {
 public:
  explicit ElevationMap(rclcpp::Node::SharedPtr node);
  virtual ~ElevationMap();

  /** @brief 设置地图范围与分辨率，并清空数据。 */
  void setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position);

  /**
   * @brief 将一帧已变换到地图系的点云融合进原始高程层。
   * @param timeStamp 该帧时间，用于与动态层、方差更新一致。
   * @param transformationSensorToMap 传感器到地图的刚体变换（用于可见性等几何量）。
   */
  bool add(PointCloudType::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, const rclcpp::Time& timeStamp,
           const Eigen::Affine3d& transformationSensorToMap);

  /** @brief 应用运动预测带来的方差增量（由 RobotMotionMapUpdater 计算）。 */
  bool update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
              const grid_map::Matrix& horizontalVarianceUpdateY, const grid_map::Matrix& horizontalVarianceUpdateXY,
              const rclcpp::Time& time);

  bool fuseAll();
  bool fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length);
  bool clear();

  /** @brief 基于射线与扫描信息清理不可靠单元（调试图可选发布）。 */
  void visibilityCleanup(const rclcpp::Time& updatedTime);

  /** @brief 在地图平面内平移网格窗口（机器人中心地图经典操作）。 */
  void move(const Eigen::Vector2d& position);

  /** @brief 若有原始图订阅者，将当前 raw 图副本交给后处理线程池异步发布。 */
  bool postprocessAndPublishRawElevationMap();
  bool publishFusedElevationMap();
  bool publishVisibilityCleanupMap();

  grid_map::GridMap& getRawGridMap();
  void setRawGridMap(const grid_map::GridMap& map);
  grid_map::GridMap& getFusedGridMap();
  void setFusedGridMap(const grid_map::GridMap& map);

  rclcpp::Time getTimeOfLastUpdate();
  rclcpp::Time getTimeOfLastFusion();
  const kindr::HomTransformQuatD& getPose();
  bool getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position);

  boost::recursive_mutex& getFusedDataMutex();
  boost::recursive_mutex& getRawDataMutex();

  void setFrameId(const std::string& frameId);
  const std::string& getFrameId();
  void setTimestamp(rclcpp::Time timestamp);

  bool hasRawMapSubscribers() const;
  bool hasFusedMapSubscribers() const;

  /** @brief 底层参考地图回调（如真值/多机），用于填充或校正原始层。 */
  void underlyingMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr underlyingMap);

  /** @brief 在指定矩形区域写入常值高度与方差（初始化用）。 */
  void setRawSubmapHeight(const grid_map::Position& initPosition, float mapHeight, float variance, double lengthInXSubmap,
                          double lengthInYSubmap);

  friend class ElevationMapping;

 private:
  bool fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size);
  bool clean();
  void resetFusedData();
  static float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

  rclcpp::Node::SharedPtr node_;
  grid_map::GridMap rawMap_;
  grid_map::GridMap fusedMap_;
  grid_map::GridMap visibilityCleanupMap_;
  grid_map::GridMap underlyingMap_;
  PostprocessorPool postprocessorPool_;
  bool hasUnderlyingMap_;
  kindr::HomTransformQuatD pose_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevationMapFusedPublisher_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr visibilityCleanupMapPublisher_;

  boost::recursive_mutex fusedMapMutex_;
  boost::recursive_mutex rawMapMutex_;
  boost::recursive_mutex visibilityCleanupMapMutex_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr underlyingMapSubscriber_;

  /** 用于将动态层中的时间编码与「扫描周期」逻辑对齐的参考时间。 */
  rclcpp::Time initialTime_;

  struct Parameters {
    double minVariance_{0.000009};
    double maxVariance_{0.0009};
    double mahalanobisDistanceThreshold_{2.5};
    double multiHeightNoise_{0.000009};
    double minHorizontalVariance_{0.0001};
    double maxHorizontalVariance_{0.05};
    std::string underlyingMapTopic_;
    bool enableVisibilityCleanup_{true};
    bool enableContinuousCleanup_{false};
    double visibilityCleanupDuration_{0.0};
    double scanningDuration_{1.0};
    double increaseHeightAlpha_{1.0};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;
};

}  // namespace elevation_mapping
