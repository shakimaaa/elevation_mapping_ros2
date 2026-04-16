/**
 * @file Input.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 单路数据源描述：从 YAML 配置话题、队列、是否回调后发布地图，并持有专属 `SensorProcessorBase`。
 *
 * 当前模板实现仅实例化 `sensor_msgs::msg::PointCloud2`；其它消息类型需在模板中扩展。
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

namespace elevation_mapping {

class ElevationMapping;

class Input {
 public:
  /** 指向 `ElevationMapping` 成员函数的回调类型（点云指针、是否发布、处理器引用）。 */
  template <typename MsgT>
  using CallbackT = void (ElevationMapping::*)(const typename MsgT::ConstSharedPtr&, bool, const SensorProcessorBase::Ptr&);

  Input(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  bool isEnabled() const {
    const Parameters parameters{parameters_.getData()};
    return parameters.isEnabled_;
  }

  /**
   * @brief 解析 YAML 中该路输入的 `type/topic/queue_size/publish_on_update/sensor_processor` 等字段。
   * @param name YAML 中的键名，仅用于日志。
   */
  bool configure(const std::string& name, const YAML::Node& configuration,
                 const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  /** @brief 创建订阅并将消息转发到 `ElevationMapping` 的指定成员回调。 */
  template <typename MsgT>
  void registerCallback(ElevationMapping& map, CallbackT<MsgT> callback,
                        const rclcpp::CallbackGroup::SharedPtr& callback_group = nullptr);

  /** @brief 返回实际订阅话题（绝对路径或带命名空间解析）。 */
  std::string getSubscribedTopic() const;

  std::string getType() {
    const Parameters parameters{parameters_.getData()};
    return parameters.type_;
  }

 private:
  bool configureSensorProcessor(const std::string& name, const YAML::Node& parameters,
                                const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  SensorProcessorBase::Ptr sensorProcessor_;

  struct Parameters {
    std::string name_;
    std::string type_;
    bool isEnabled_{true};
    uint32_t queueSize_{10};
    std::string topic_;
    bool publishOnUpdate_{true};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;
};

template <typename MsgT>
void Input::registerCallback(ElevationMapping& map, CallbackT<MsgT> callback,
                             const rclcpp::CallbackGroup::SharedPtr& callback_group) {
  static_assert(std::is_same_v<MsgT, sensor_msgs::msg::PointCloud2>, "Only sensor_msgs::msg::PointCloud2 is supported.");
  const Parameters parameters{parameters_.getData()};
  const size_t depth = std::max<size_t>(1U, parameters.queueSize_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;
  subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      parameters.topic_, qos,
      [this, &map, callback, publishOnUpdate = parameters.publishOnUpdate_](
          sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { (map.*callback)(msg, publishOnUpdate, sensorProcessor_); },
      options);
  RCLCPP_INFO(node_->get_logger(), "Subscribing %s: %s queue=%u", parameters.type_.c_str(), parameters.topic_.c_str(),
              parameters.queueSize_);
}

}  // namespace elevation_mapping
