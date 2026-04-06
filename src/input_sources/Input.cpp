/**
 * @file Input.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 解析单路 YAML 配置并实例化对应 SensorProcessor；话题名在 registerCallback 时创建订阅。
 */
#include "elevation_mapping/input_sources/Input.hpp"

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

Input::Input(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(std::move(node)), tf_buffer_(std::move(tf_buffer)) {}

bool Input::configure(const std::string& name, const YAML::Node& configuration,
                      const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  if (!configuration.IsMap()) {
    RCLCPP_ERROR(node_->get_logger(), "Input source %s: configuration must be a map.", name.c_str());
    return false;
  }

  Parameters parameters;
  if (configuration["enabled"]) {
    if (!configuration["enabled"].IsScalar()) {
      RCLCPP_ERROR(node_->get_logger(), "Input %s: 'enabled' must be boolean.", name.c_str());
      return false;
    }
    parameters.isEnabled_ = configuration["enabled"].as<bool>();
  }

  const char* required[] = {"type", "topic", "queue_size", "publish_on_update", "sensor_processor"};
  for (const char* key : required) {
    if (!configuration[key]) {
      RCLCPP_ERROR(node_->get_logger(), "Input %s: missing key '%s'.", name.c_str(), key);
      return false;
    }
  }

  parameters.name_ = name;
  parameters.type_ = configuration["type"].as<std::string>();
  parameters.topic_ = configuration["topic"].as<std::string>();
  const int queueSize = configuration["queue_size"].as<int>();
  if (queueSize < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Input %s: queue_size must be non-negative.", name.c_str());
    return false;
  }
  parameters.queueSize_ = static_cast<uint32_t>(queueSize);
  parameters.publishOnUpdate_ = configuration["publish_on_update"].as<bool>();

  parameters_.setData(parameters);

  if (!configureSensorProcessor(name, configuration["sensor_processor"], generalSensorProcessorParameters)) {
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Configured input %s type=%s topic=%s", name.c_str(), parameters.type_.c_str(),
               parameters.topic_.c_str());
  return true;
}

std::string Input::getSubscribedTopic() const {
  const Parameters parameters{parameters_.getData()};
  try {
    return rclcpp::expand_topic_or_service_name(parameters.topic_, node_->get_name(), node_->get_namespace(), false);
  } catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
    RCLCPP_WARN(node_->get_logger(), "Invalid topic name '%s': %s", parameters.topic_.c_str(), e.what());
    return parameters.topic_;
  }
}

bool Input::configureSensorProcessor(const std::string& name, const YAML::Node& parameters,
                                     const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  if (!parameters["type"] || !parameters["type"].IsScalar()) {
    RCLCPP_ERROR(node_->get_logger(), "Input %s: sensor_processor.type missing.", name.c_str());
    return false;
  }
  const std::string sensorType = parameters["type"].as<std::string>();
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(node_, tf_buffer_, generalSensorProcessorParameters);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(node_, tf_buffer_, generalSensorProcessorParameters);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(node_, tf_buffer_, generalSensorProcessorParameters);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(node_, tf_buffer_, generalSensorProcessorParameters);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Sensor type %s not available.", sensorType.c_str());
    return false;
  }
  sensorProcessor_->setSensorProcessorYaml(parameters);
  return sensorProcessor_->readParameters();
}

}  // namespace elevation_mapping
