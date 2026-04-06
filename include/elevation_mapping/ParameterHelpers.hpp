/**
 * @file ParameterHelpers.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief ROS 2 节点参数的声明与读取辅助工具。
 *
 * ROS 2 与 ROS1 的差异要点：
 * - 参数需先 declare_parameter，再 get_parameter；未声明直接 get 可能失败或行为依赖配置；
 * - 动态重载、多节点同名参数时要注意「已声明」异常（ParameterAlreadyDeclaredException）。
 *
 * 本工具用于迁移原 Catkin 代码中大量 nodeHandle.param("key", var, default) 的模式：
 * 在首次访问时自动完成声明（若尚未声明），再返回当前值，减少样板代码。
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace elevation_mapping {

/**
 * @brief 若参数尚未在节点上声明，则使用默认值声明；然后返回该参数的当前值。
 *
 * @tparam T            参数类型，须与 rclcpp::Parameter 支持的类型一致（如 double、int、bool、string 等）。
 * @param node          目标 ROS 2 节点指针（非空）。
 * @param name          参数全名（建议使用 ROS2 惯例，如 "map_frame_id" 或带命名空间前缀）。
 * @param default_value 首次声明时使用的默认值；若参数已由 launch/YAML 覆盖，则返回的是覆盖后的值。
 * @return 参数的当前取值。
 *
 * @note 若参数已被其他代码声明但类型与 T 不一致，get_value<T>() 可能抛异常，调用方需保证类型一致。
 * @note 对已声明参数重复 declare 会抛 ParameterAlreadyDeclaredException，此处已捕获并忽略，随后直接 get。
 */
template <typename T>
T declareOrGetParameter(rclcpp::Node* node, const std::string& name, const T& default_value) {
  if (!node->has_parameter(name)) {
    try {
      node->declare_parameter<T>(name, default_value);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
      // 并发或其它模块已声明同名参数时走此处，直接读取即可。
    }
  }
  return node->get_parameter(name).get_value<T>();
}

}  // namespace elevation_mapping
