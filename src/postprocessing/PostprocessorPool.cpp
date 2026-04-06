/**
 * @file PostprocessorPool.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 线程池任务投递：post 到 worker 的 io_service，完成时 wrapTask 归还空闲槽位。
 */

#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {

PostprocessorPool::PostprocessorPool(std::size_t poolSize, rclcpp::Node::SharedPtr node) {
  for (std::size_t i = 0; i < poolSize; ++i) {
    workers_.emplace_back(std::make_unique<PostprocessingWorker>(node));
    availableServices_.push_back(i);  // 初始时所有 worker 空闲
  }
}

PostprocessorPool::~PostprocessorPool() {
  for (auto& worker : workers_) {
    worker->ioService().stop();  // 使各 io_service::run() 返回
  }

  for (auto& worker : workers_) {
    try {
      if (worker->thread().joinable()) {
        worker->thread().join();
      }
    } catch (const std::exception&) {
    }
  }
}

bool PostprocessorPool::runTask(const GridMap& gridMap) {
  size_t serviceIndex{0};
  {
    boost::lock_guard<boost::mutex> lock(availableServicesMutex_);
    if (availableServices_.empty()) {
      return false;
    }
    serviceIndex = availableServices_.back();
    availableServices_.pop_back();
  }

  workers_.at(serviceIndex)->setDataBuffer(gridMap);  // 拷贝待处理图到该 worker 缓冲

  auto task = [this, serviceIndex] { wrapTask(serviceIndex); };
  workers_.at(serviceIndex)->ioService().post(task);
  return true;
}

void PostprocessorPool::wrapTask(size_t serviceIndex) {
  try {
    GridMap postprocessedMap = workers_.at(serviceIndex)->processBuffer();
    workers_.at(serviceIndex)->publish(postprocessedMap);
  }
  catch (const std::exception& exception) {  // 单任务失败不影响池内其它线程
    RCLCPP_ERROR(rclcpp::get_logger("elevation_mapping"), "Postprocessor pipeline thread %zu error: %s", serviceIndex,
                 exception.what());
  }

  boost::unique_lock<boost::mutex> lock(availableServicesMutex_);
  availableServices_.push_back(serviceIndex);  // 归还空闲 worker 索引
}

bool PostprocessorPool::pipelineHasSubscribers() const {
  return std::all_of(workers_.cbegin(), workers_.cend(),
                     [](const std::unique_ptr<PostprocessingWorker>& worker) { return worker->hasSubscribers(); });
}

}  // namespace elevation_mapping