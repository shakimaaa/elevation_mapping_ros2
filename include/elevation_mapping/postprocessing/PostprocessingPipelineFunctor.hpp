/**
 * @file PostprocessingPipelineFunctor.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 对单帧 GridMap 运行 `filters::FilterChain` 并发布到配置的话题（如 elevation_map_raw 的滤波版）。
 *
 * 参数 `filter_chain_parameters_name` 指向 ROS 参数命名空间下的滤波链配置（与 grid_map_filters 惯例一致）。
 * `ThreadSafeDataWrapper<Parameters>` 允许在服务重载参数时与正在执行的 operator() 并发但安全替换快照。
 */
#pragma once

#include <filters/filter_chain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

namespace elevation_mapping {

class PostprocessingPipelineFunctor {
 public:
  using GridMap = grid_map::GridMap;

  explicit PostprocessingPipelineFunctor(rclcpp::Node::SharedPtr node);
  ~PostprocessingPipelineFunctor();

  /** 就地滤波并返回处理后的图（可能修改 inputMap）。 */
  GridMap operator()(GridMap& inputMap);
  void publish(const GridMap& gridMap) const;
  bool hasSubscribers() const;

 private:
  void readParameters();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
  filters::FilterChain<grid_map::GridMap> filterChain_;

  struct Parameters {
    std::string outputTopic_;
    std::string filterChainParametersName_;
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  bool filterChainConfigured_{false};
};

}  // namespace elevation_mapping
