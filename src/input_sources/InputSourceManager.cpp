/**
 * @file InputSourceManager.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 从 `input_sources_file` 加载 YAML，遍历 `input_sources` 映射并填充 sources_ 向量。
 */
#include "elevation_mapping/input_sources/InputSourceManager.hpp"

#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/ParameterHelpers.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace elevation_mapping {

InputSourceManager::InputSourceManager(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(std::move(node)), tf_buffer_(std::move(tf_buffer)) {}

bool InputSourceManager::configureFromRos() {
  const std::string path = declareOrGetParameter(node_.get(), "input_sources_file", std::string());
  if (path.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "input_sources_file empty; not loading YAML input sources.");
    return false;
  }
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root["input_sources"]) {
      RCLCPP_ERROR(node_->get_logger(), "YAML file must contain top-level key 'input_sources'.");
      return false;
    }
    return configure(root["input_sources"], "input_sources");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load input_sources_file '%s': %s", path.c_str(), e.what());
    return false;
  }
}

bool InputSourceManager::configure(const YAML::Node& config, const std::string& sourceConfigurationName) {
  if (config.IsSequence() && config.size() == 0) {
    return true;
  }
  if (!config.IsMap()) {
    RCLCPP_ERROR(node_->get_logger(), "%s: input_sources must be a map.", sourceConfigurationName.c_str());
    return false;
  }

  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{
      declareOrGetParameter(node_.get(), "robot_base_frame_id", std::string("robot")),
      declareOrGetParameter(node_.get(), "map_frame_id", std::string("map"))};

  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    Input source(node_, tf_buffer_);
    if (!source.configure(name, it->second, generalSensorProcessorConfig)) {
      successfulConfiguration = false;
      continue;
    }
    if (!source.isEnabled()) {
      continue;
    }
    const std::string subscribedTopic = source.getSubscribedTopic();
    if (!subscribedTopics.insert(subscribedTopic).second) {
      RCLCPP_WARN(node_->get_logger(), "Duplicate subscription topic %s ignored.", subscribedTopic.c_str());
      successfulConfiguration = false;
      continue;
    }
    sources_.push_back(std::move(source));
  }
  return successfulConfiguration;
}

void InputSourceManager::registerWith(ElevationMapping& map) {
  for (Input& source : sources_) {
    if (source.getType() == "pointcloud") {
      source.registerCallback<sensor_msgs::msg::PointCloud2>(map, &ElevationMapping::pointCloudCallback);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unsupported input type '%s' (only 'pointcloud').", source.getType().c_str());
    }
  }
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping
