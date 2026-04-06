/**
 * @file ThreadSafeDataWrapper.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 线程安全的参数/配置包装器：多线程读与单写者模式下的互斥保护。
 *
 * ROS 2 回调可能并行执行（MultiThreadedExecutor），而 `Parameters` 结构在 `readParameters` 与
 * 传感器回调间共享；使用 `getData()` 得快照，`getDataToWrite()` 在持锁期间修改整块配置。
 *
 * @tparam Data 被包装的值类型，建议为可拷贝的结构体。
 */
#pragma once

#include <mutex>
#include <thread>

namespace elevation_mapping {

template <typename Data>
class ThreadSafeDataWrapper {
 public:
  ThreadSafeDataWrapper() = default;

  ThreadSafeDataWrapper(const ThreadSafeDataWrapper<Data>& other) : data_(other.getData()) {}

  /** 整体替换配置（会短暂加锁）。 */
  void setData(Data data) {
    std::lock_guard<std::mutex> _{dataMutex_};
    data_ = data;
  }

  /** @return 当前配置的拷贝（避免外部持有引用穿透锁）。 */
  Data getData() const {
    std::lock_guard<std::mutex> _{dataMutex_};
    return data_;
  }

  /**
   * @return 可写引用 + 已上锁的 unique_lock；锁在 guard 析构前一直有效，用于多字段原子更新。
   */
  std::pair<Data&, std::unique_lock<std::mutex>> getDataToWrite() {
    std::unique_lock<std::mutex> dataWriteGuard{dataMutex_};
    return {data_, std::move(dataWriteGuard)};
  }

 private:
  Data data_;
  mutable std::mutex dataMutex_;
};

}  // namespace elevation_mapping
