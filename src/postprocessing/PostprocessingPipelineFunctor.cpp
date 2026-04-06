/**
 * @file PostprocessingPipelineFunctor.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 读取 output_topic 与滤波链参数命名空间，构造 GridMap 发布者与 FilterChain。
 */
#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "elevation_mapping/ParameterHelpers.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

namespace elevation_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), filterChain_("grid_map::GridMap") {
  readParameters();
  const Parameters parameters{parameters_.getData()};
  publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(parameters.outputTopic_, rclcpp::QoS(1).transient_local());

  if (!filterChain_.configure(parameters.filterChainParametersName_, node_->get_node_logging_interface(),
                              node_->get_node_parameters_interface())) {
    RCLCPP_WARN(node_->get_logger(), "Could not configure filter chain; publishing raw elevation map without postprocessing.");
    return;
  }
  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  Parameters parameters;
  parameters.outputTopic_ = declareOrGetParameter(node_.get(), "output_topic", std::string("elevation_map_raw"));
  parameters.filterChainParametersName_ =
      declareOrGetParameter(node_.get(), "postprocessor_pipeline_name", std::string("postprocessor_pipeline"));
  parameters_.setData(parameters);
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  if (!filterChainConfigured_) {
    RCLCPP_WARN(node_->get_logger(), "No postprocessing pipeline configured; forwarding raw map.");
    return inputMap;
  }
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(node_->get_logger(), "Filter chain update failed; forwarding raw map.");
    return inputMap;
  }
  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  auto outputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
  publisher_->publish(*outputMessage);
  RCLCPP_DEBUG(node_->get_logger(), "Published postprocessed raw elevation map.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0;
}

}  // namespace elevation_mapping
