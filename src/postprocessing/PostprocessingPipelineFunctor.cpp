/**
 * @file PostprocessingPipelineFunctor.cpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 读取 output_topic 与滤波链参数命名空间，构造 GridMap 发布者与 FilterChain。
 */
#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "elevation_mapping/ParameterHelpers.hpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace elevation_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)), filterChain_("grid_map::GridMap") {
  readParameters();
  const Parameters parameters{parameters_.getData()};
  publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(parameters.outputTopic_, rclcpp::QoS(1).transient_local());

  if (!filterChain_.configure(parameters.filterChainParametersName_, node_->get_node_logging_interface(),
                              node_->get_node_parameters_interface())) {
    RCLCPP_WARN(node_->get_logger(),
                "Could not configure filter chain; fallback to built-in elevation postprocessing hierarchy.");
  } else {
    filterChainConfigured_ = true;
  }
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  Parameters parameters;
  parameters.outputTopic_ = declareOrGetParameter(node_.get(), "output_topic", std::string("elevation_map_raw"));
  parameters.filterChainParametersName_ =
      declareOrGetParameter(node_.get(), "postprocessor_pipeline_name", std::string("postprocessor_pipeline"));
  parameters.enableBuiltinFilterHierarchy_ = declareOrGetParameter(node_.get(), "postprocessor_enable_builtin_filter_hierarchy", true);
  parameters.replaceSourceWithDenoised_ = declareOrGetParameter(node_.get(), "postprocessor_replace_source_with_denoised", true);
  parameters.sourceLayerName_ = declareOrGetParameter(node_.get(), "postprocessor_source_layer", std::string("elevation"));
  parameters.smoothLayerName1_ = declareOrGetParameter(node_.get(), "postprocessor_hs1_layer", std::string("elevation_hs1"));
  parameters.smoothLayerName2_ = declareOrGetParameter(node_.get(), "postprocessor_hs2_layer", std::string("elevation_hs2"));
  parameters.inpaintIterations_ = declareOrGetParameter(node_.get(), "postprocessor_inpaint_iterations", 4);
  parameters.outlierMedianKernelSize_ = declareOrGetParameter(node_.get(), "postprocessor_outlier_median_kernel", 3);
  parameters.outlierMedianPasses_ = declareOrGetParameter(node_.get(), "postprocessor_outlier_median_passes", 2);
  parameters.aggressiveMedianKernelSize_ = declareOrGetParameter(node_.get(), "postprocessor_aggressive_median_kernel", 7);
  parameters.dilationKernelSize_ = declareOrGetParameter(node_.get(), "postprocessor_dilation_kernel", 5);
  parameters.deltaHeightMaskThreshold_ = declareOrGetParameter(node_.get(), "postprocessor_delta_height_mask_threshold", 0.01);
  parameters.hs1Sigma_ = declareOrGetParameter(node_.get(), "postprocessor_hs1_sigma", 0.08);
  parameters.hs2Sigma_ = declareOrGetParameter(node_.get(), "postprocessor_hs2_sigma", 0.24);
  parameters_.setData(parameters);
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  const Parameters parameters{parameters_.getData()};
  if (parameters.enableBuiltinFilterHierarchy_) {
    return runBuiltinFilterHierarchy(inputMap);
  }
  if (filterChainConfigured_) {
    grid_map::GridMap outputMap;
    if (!filterChain_.update(inputMap, outputMap)) {
      RCLCPP_ERROR(node_->get_logger(), "Filter chain update failed; forwarding raw map.");
      return inputMap;
    }
    return outputMap;
  }
  RCLCPP_WARN(node_->get_logger(), "No available postprocessing pipeline; forwarding raw map.");
  return inputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  auto outputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
  publisher_->publish(*outputMessage);
  RCLCPP_DEBUG(node_->get_logger(), "Published postprocessed raw elevation map.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0;
}

int PostprocessingPipelineFunctor::ensureOddKernel(int kernelSize) {
  const int bounded = std::max(1, kernelSize);
  return (bounded % 2 == 0) ? bounded + 1 : bounded;
}

namespace {
cv::Mat medianFilterFloat32(const cv::Mat& src, int kernelSize) {
  const int k = std::max(1, (kernelSize % 2 == 0) ? kernelSize + 1 : kernelSize);
  if (k == 1) {
    return src.clone();
  }
  const int radius = k / 2;
  cv::Mat dst = src.clone();
  std::vector<float> window;
  window.reserve(static_cast<size_t>(k * k));

  for (int r = 0; r < src.rows; ++r) {
    for (int c = 0; c < src.cols; ++c) {
      window.clear();
      for (int dr = -radius; dr <= radius; ++dr) {
        const int rr = std::clamp(r + dr, 0, src.rows - 1);
        for (int dc = -radius; dc <= radius; ++dc) {
          const int cc = std::clamp(c + dc, 0, src.cols - 1);
          window.push_back(src.at<float>(rr, cc));
        }
      }
      const auto midIt = window.begin() + static_cast<std::ptrdiff_t>(window.size() / 2);
      std::nth_element(window.begin(), midIt, window.end());
      dst.at<float>(r, c) = *midIt;
    }
  }
  return dst;
}
}  // namespace

cv::Mat PostprocessingPipelineFunctor::gridMapLayerToCvMat(const GridMap& map, const std::string& layerName) const {
  if (!map.exists(layerName)) {
    return {};
  }
  const auto& matrix = map.get(layerName);
  cv::Mat cvLayer(matrix.rows(), matrix.cols(), CV_32FC1);
  for (int r = 0; r < matrix.rows(); ++r) {
    float* rowPtr = cvLayer.ptr<float>(r);
    for (int c = 0; c < matrix.cols(); ++c) {
      rowPtr[c] = matrix(r, c);
    }
  }
  return cvLayer;
}

cv::Mat PostprocessingPipelineFunctor::inpaintWithBorderMin(const cv::Mat& raw, int maxIterations) const {
  cv::Mat filled = raw.clone();
  const int rows = filled.rows;
  const int cols = filled.cols;
  const int iterationCount = std::max(1, maxIterations);

  for (int it = 0; it < iterationCount; ++it) {
    bool changed = false;
    cv::Mat next = filled.clone();
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if (std::isfinite(filled.at<float>(r, c))) {
          continue;
        }
        float minNeighbor = std::numeric_limits<float>::infinity();
        bool hasNeighbor = false;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) {
              continue;
            }
            const int rr = r + dr;
            const int cc = c + dc;
            if (rr < 0 || rr >= rows || cc < 0 || cc >= cols) {
              continue;
            }
            const float value = filled.at<float>(rr, cc);
            if (std::isfinite(value)) {
              minNeighbor = std::min(minNeighbor, value);
              hasNeighbor = true;
            }
          }
        }
        if (hasNeighbor) {
          next.at<float>(r, c) = minNeighbor;
          changed = true;
        }
      }
    }
    filled = next;
    if (!changed) {
      break;
    }
  }

  // Remaining holes (if any) are set to global minimum to keep image operators stable.
  float globalMin = std::numeric_limits<float>::infinity();
  for (int r = 0; r < rows; ++r) {
    const float* rowPtr = filled.ptr<float>(r);
    for (int c = 0; c < cols; ++c) {
      if (std::isfinite(rowPtr[c])) {
        globalMin = std::min(globalMin, rowPtr[c]);
      }
    }
  }
  if (!std::isfinite(globalMin)) {
    globalMin = 0.0F;
  }
  for (int r = 0; r < rows; ++r) {
    float* rowPtr = filled.ptr<float>(r);
    for (int c = 0; c < cols; ++c) {
      if (!std::isfinite(rowPtr[c])) {
        rowPtr[c] = globalMin;
      }
    }
  }
  return filled;
}

grid_map::GridMap PostprocessingPipelineFunctor::runBuiltinFilterHierarchy(const GridMap& inputMap) const {
  const Parameters parameters{parameters_.getData()};
  if (!inputMap.exists(parameters.sourceLayerName_)) {
    RCLCPP_WARN(node_->get_logger(), "Source layer '%s' does not exist; forwarding raw map.",
                parameters.sourceLayerName_.c_str());
    return inputMap;
  }

  const cv::Mat rawHeight = gridMapLayerToCvMat(inputMap, parameters.sourceLayerName_);
  if (rawHeight.empty()) {
    return inputMap;
  }

  // Step 1: in-paint empty cells using local occlusion-border minimum.
  const cv::Mat inpainted = inpaintWithBorderMin(rawHeight, parameters.inpaintIterations_);

  // Step 2: repeated median filtering for outlier rejection.
  cv::Mat denoised = inpainted.clone();
  const int outlierKernel = ensureOddKernel(parameters.outlierMedianKernelSize_);
  for (int i = 0; i < std::max(1, parameters.outlierMedianPasses_); ++i) {
    denoised = medianFilterFloat32(denoised, outlierKernel);
  }

  // Step 3: hs1 = light Gaussian smoothing used for edge gradients.
  cv::Mat hs1;
  cv::GaussianBlur(denoised, hs1, cv::Size(0, 0), std::max(1e-3, parameters.hs1Sigma_));

  // Step 4: virtual floor preprocessing (aggressive median -> delta -> sign-separated masks).
  cv::Mat aggressiveMedian = medianFilterFloat32(denoised, ensureOddKernel(parameters.aggressiveMedianKernelSize_));
  const cv::Mat delta = denoised - aggressiveMedian;

  cv::Mat positiveMask = cv::Mat::zeros(delta.size(), CV_8UC1);  // stepping stones: delta > +tau
  cv::Mat negativeMask = cv::Mat::zeros(delta.size(), CV_8UC1);  // gaps: delta < -tau
  const float threshold = static_cast<float>(std::max(0.0, parameters.deltaHeightMaskThreshold_));
  for (int r = 0; r < delta.rows; ++r) {
    const float* deltaRow = delta.ptr<float>(r);
    uint8_t* posRow = positiveMask.ptr<uint8_t>(r);
    uint8_t* negRow = negativeMask.ptr<uint8_t>(r);
    for (int c = 0; c < delta.cols; ++c) {
      if (deltaRow[c] > threshold) {
        posRow[c] = 255;
      } else if (deltaRow[c] < -threshold) {
        negRow[c] = 255;
      }
    }
  }

  cv::Mat dilated;
  cv::dilate(denoised, dilated,
             cv::getStructuringElement(cv::MORPH_RECT,
                                       cv::Size(ensureOddKernel(parameters.dilationKernelSize_),
                                                ensureOddKernel(parameters.dilationKernelSize_))));
  cv::Mat virtualFloor = denoised.clone();
  // Paper-style split: only expand stepping stones. Gap cells are preserved (not max-filled).
  dilated.copyTo(virtualFloor, positiveMask);
  (void)negativeMask;

  // Step 5: hs2 = strongly smoothed virtual floor.
  cv::Mat hs2;
  cv::GaussianBlur(virtualFloor, hs2, cv::Size(0, 0), std::max(1e-3, parameters.hs2Sigma_));

  grid_map::GridMap outputMap = inputMap;
  if (parameters.replaceSourceWithDenoised_) {
    auto& sourceLayer = outputMap[parameters.sourceLayerName_];
    for (int r = 0; r < sourceLayer.rows(); ++r) {
      for (int c = 0; c < sourceLayer.cols(); ++c) {
        sourceLayer(r, c) = denoised.at<float>(r, c);
      }
    }
  }
  if (!outputMap.exists(parameters.smoothLayerName1_)) {
    outputMap.add(parameters.smoothLayerName1_);
  }
  if (!outputMap.exists(parameters.smoothLayerName2_)) {
    outputMap.add(parameters.smoothLayerName2_);
  }
  auto& hs1Layer = outputMap[parameters.smoothLayerName1_];
  auto& hs2Layer = outputMap[parameters.smoothLayerName2_];
  for (int r = 0; r < hs1.rows; ++r) {
    for (int c = 0; c < hs1.cols; ++c) {
      hs1Layer(r, c) = hs1.at<float>(r, c);
      hs2Layer(r, c) = hs2.at<float>(r, c);
    }
  }

  return outputMap;
}

}  // namespace elevation_mapping
