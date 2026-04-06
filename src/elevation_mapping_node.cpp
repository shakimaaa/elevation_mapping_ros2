/**
 * @file elevation_mapping_node.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief ROS 2 可执行入口：初始化 rclcpp、构造 ElevationMapping、使用多线程执行器 spin。
 *
 * 参数 `num_callback_threads`（默认 4）控制 `MultiThreadedExecutor` 线程数，使点云、服务、
 * 多个 wall timer 可并行调度；注意 `ElevationMap` 内部仍用 recursive_mutex 保护 raw/fused 数据。
 */
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping/ElevationMapRobotFrameSampler.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("elevation_mapping");

  elevation_mapping::ElevationMapping elevation_map(node);
  elevation_mapping::ElevationMapRobotFrameSampler elevation_sampler(node, elevation_map.getElevationMap(), elevation_map.getTfBuffer());

  const int num_threads = node->declare_parameter("num_callback_threads", 4);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, static_cast<size_t>(std::max(1, num_threads)));
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
