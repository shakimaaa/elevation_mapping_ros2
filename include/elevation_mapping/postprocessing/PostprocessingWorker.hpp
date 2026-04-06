/**
 * @file PostprocessingWorker.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 单条后处理流水线：functor + 异步 io_service 线程 + 待处理 GridMap 缓冲。
 *
 * 线程安全约定：本类的 `dataBuffer_` 与内部任务投递由 `PostprocessorPool` 在持锁或单线程语义下使用，
 * 避免多线程同时 `setDataBuffer`/`processBuffer`。
 */
#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <thread>

#include <grid_map_core/GridMap.hpp>
#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

class PostprocessingWorker {
 public:
  using GridMap = grid_map::GridMap;

  explicit PostprocessingWorker(rclcpp::Node::SharedPtr node);

  boost::asio::io_service& ioService() { return ioService_; }
  std::thread& thread() { return thread_; }
  const GridMap& dataBuffer() { return dataBuffer_; }
  void setDataBuffer(GridMap data) { dataBuffer_ = std::move(data); }

  /** 对 `dataBuffer_` 执行 functor 滤波链。 */
  GridMap processBuffer();
  void publish(const GridMap& gridMap) const;
  bool hasSubscribers() const;

 protected:
  PostprocessingPipelineFunctor functor_;

  boost::asio::io_service ioService_;
  boost::asio::io_service::work work_;
  std::thread thread_;

  GridMap dataBuffer_;
};

}  // namespace elevation_mapping
