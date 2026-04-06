/**
 * @file PostprocessorPool.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 后处理线程池：并行执行多路 GridMap 滤波链，避免阻塞点云主线程。
 *
 * 设计要点：
 * - 固定 `poolSize` 条工作线程；任务到来时若有空闲 worker 则拷贝 raw 图异步处理，否则丢弃该帧（有意为之，防止积压）。
 * - `availableServices_` 队列记录空闲 worker 索引，由互斥锁保护，临界区极小。
 * - 每个 worker 内持有 `PostprocessingPipelineFunctor`（filters::FilterChain）与独立 io_service 线程。
 */
#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <grid_map_core/GridMap.hpp>
#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "elevation_mapping/postprocessing/PostprocessingWorker.hpp"

namespace elevation_mapping {

class PostprocessorPool {
 public:
  using GridMap = grid_map::GridMap;

  /**
   * @param poolSize 工作线程数量（通常与 CPU 核数或期望并行度一致）。
   * @param node 用于创建 functor 内发布者与读取参数。
   */
  PostprocessorPool(std::size_t poolSize, rclcpp::Node::SharedPtr node);

  /** 停止 io_service、join 各线程。 */
  ~PostprocessorPool();

  /**
   * @brief 若有空闲 worker，提交一份 raw 图拷贝进行后处理；否则返回 false。
   */
  bool runTask(const GridMap& gridMap);

  /** @brief 当前是否有节点订阅后处理输出话题（用于决定是否值得拷贝/入队）。 */
  bool pipelineHasSubscribers() const;

 private:
  /** 在 worker 完成时把其索引归还 `availableServices_`。 */
  void wrapTask(size_t serviceIndex);

  std::vector<std::unique_ptr<PostprocessingWorker>> workers_;

  boost::mutex availableServicesMutex_;
  std::deque<size_t> availableServices_;
};

}  // namespace elevation_mapping
