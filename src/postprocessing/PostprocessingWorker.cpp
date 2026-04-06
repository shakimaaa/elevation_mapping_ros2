/**
 * @file PostprocessingWorker.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 启动独立线程跑 boost::asio io_service，functor 在 processBuffer 中同步调用。
 */
#include "elevation_mapping/postprocessing/PostprocessingWorker.hpp"

namespace elevation_mapping {

PostprocessingWorker::PostprocessingWorker(rclcpp::Node::SharedPtr node)
    : functor_(node), work_(ioService_), thread_([this] { this->ioService_.run(); }) {}

PostprocessingWorker::GridMap PostprocessingWorker::processBuffer() {
  return functor_(dataBuffer_);
}

void PostprocessingWorker::publish(const GridMap& gridMap) const {
  functor_.publish(gridMap);
}

bool PostprocessingWorker::hasSubscribers() const {
  return functor_.hasSubscribers();
}

}  // namespace elevation_mapping
