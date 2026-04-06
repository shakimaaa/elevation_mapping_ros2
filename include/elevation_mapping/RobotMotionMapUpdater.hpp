/**
 * @file RobotMotionMapUpdater.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 根据机器人位姿（及协方差）推导高程图各单元的过程噪声与水平方差增量。
 *
 * 算法与原版论文/ANYbotics 实现一致：将 6×6 位姿协方差压缩到与平面运动相关的子空间，
 * 再按网格单元几何关系叠加到 `ElevationMap::update()` 所接收的四个矩阵中。
 */
#pragma once

#include "elevation_mapping/ElevationMap.hpp"

#include <Eigen/Core>
#include <kindr/Core>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {

class RobotMotionMapUpdater {
 public:
  using Pose = kindr::HomogeneousTransformationPosition3RotationQuaternionD;
  using Covariance = Eigen::Matrix<double, 3, 3>;
  using PoseCovariance = Eigen::Matrix<double, 6, 6>;
  using ReducedCovariance = Eigen::Matrix<double, 4, 4>;
  using Jacobian = Eigen::Matrix<double, 4, 4>;

  explicit RobotMotionMapUpdater(rclcpp::Node::SharedPtr node);
  virtual ~RobotMotionMapUpdater();

  bool readParameters();

  /**
   * @brief 对给定时刻位姿做一次预测更新；若与上次时间戳相同则视为无新运动（直接返回 true）。
   */
  bool update(ElevationMap& map, const Pose& robotPose, const PoseCovariance& robotPoseCovariance, const rclcpp::Time& time);

 private:
  static bool computeReducedCovariance(const Pose& robotPose, const PoseCovariance& robotPoseCovariance,
                                       ReducedCovariance& reducedCovariance);
  bool computeRelativeCovariance(const Pose& robotPose, const ReducedCovariance& reducedCovariance,
                                 ReducedCovariance& relativeCovariance);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time previousUpdateTime_;
  Pose previousRobotPose_;
  ReducedCovariance previousReducedCovariance_;
  double covarianceScale_{1.0};
};

}  // namespace elevation_mapping
