/**
 * @file InputSourceManager.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 从 ROS 参数 `input_sources_file` 指向的 YAML 加载多路 `Input`，并注册到 `ElevationMapping`。
 *
 * YAML 顶层需包含键 `input_sources`，其下为若干命名子节点，结构与原版 ROS1 的 `input_sources` 参数块相同。
 * `configureFromRos()` 在文件路径为空时返回 false（非错误），此时可回退到单话题 `point_cloud_topic`。
 */
#pragma once

#include "elevation_mapping/input_sources/Input.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace elevation_mapping {

class ElevationMapping;

class InputSourceManager {
 public:
  InputSourceManager(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /** @brief 读取参数 `input_sources_file` 并解析其中 `input_sources` 映射。 */
  bool configureFromRos();

  /** @brief 直接对给定 YAML 节点（应为 map）配置多路输入；`sourceConfigurationName` 用于日志前缀。 */
  bool configure(const YAML::Node& config, const std::string& sourceConfigurationName);

  /** @brief 对已成功配置的每路 `Input` 调用 `registerCallback`（当前仅支持 type==`pointcloud`）。 */
  void registerWith(ElevationMapping& map);

  int getNumberOfSources();

 protected:
  std::vector<Input> sources_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace elevation_mapping
