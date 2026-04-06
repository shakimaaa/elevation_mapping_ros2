/**
 * @file KindrRosCompat.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief ROS 2 下 kindr 与 geometry_msgs 之间的轻量转换（替代 ROS1 的 kindr_ros）。
 *
 * 说明：
 * - 原 elevation_mapping 大量依赖 kindr 做李群位姿与 3D 位置运算；
 * - ROS1 中由 kindr_ros 提供与 geometry_msgs 的互转；
 * - ROS2 若未单独安装 kindr_ros 的移植版，可在此集中维护最小必要的转换，避免业务代码散落重复实现。
 *
 * 约定：
 * - geometry_msgs 中四元数顺序为 (x, y, z, w)，而 kindr::RotationQuaternionD 构造使用 (w, x, y, z)，与 kindr 文档一致。
 */
#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <kindr/Core>

namespace elevation_mapping {
/**
 * @namespace kindr_compat
 * @brief 与 kindr 相关的 ROS 消息兼容层，命名空间隔离，避免污染全局。
 */
namespace kindr_compat {

/**
 * @brief 将 ROS 2 的 Pose 消息转为 kindr 齐次变换（位置 + 单位四元数旋转）。
 * @param pose  输入：geometry_msgs 位姿（位置 + 朝向四元数）。
 * @param out   输出：kindr::HomTransformQuatD，供后续运动预测、地图更新等使用。
 */
inline void poseMsgToHomTransform(const geometry_msgs::msg::Pose& pose, kindr::HomTransformQuatD& out) {
  out = kindr::HomTransformQuatD(
      kindr::Position3D(pose.position.x, pose.position.y, pose.position.z),
      kindr::RotationQuaternionD(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
}

/**
 * @brief 将 kindr 三维位置写入 ROS Point 消息（例如用于 TF 变换后的几何点）。
 * @param p      输入：kindr 位置向量。
 * @param point  输出：geometry_msgs::msg::Point。
 */
inline void position3dToPointMsg(const kindr::Position3D& p, geometry_msgs::msg::Point& point) {
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
}

/**
 * @brief 将 ROS Point 消息读入 kindr 三维位置。
 * @param point  输入：geometry_msgs::msg::Point。
 * @param p      输出：kindr::Position3D。
 */
inline void pointMsgToPosition3d(const geometry_msgs::msg::Point& point, kindr::Position3D& p) {
  p.x() = point.x;
  p.y() = point.y;
  p.z() = point.z;
}

}  // namespace kindr_compat
}  // namespace elevation_mapping
