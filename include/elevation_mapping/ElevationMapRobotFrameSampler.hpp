/**
 * @file ElevationMapRobotFrameSampler.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 以机器人为中心按局部矩形规则采样高程图，将地图系下的 (x,y,z) 变换到机器人基座系并发布 PointCloud2。
 */
#pragma once

#include "elevation_mapping/ElevationMap.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace elevation_mapping {

/**
 * @brief 先在 map 系高程图上取样得到 3D 点，再整体变换到机器人局部系发布，输出 xyz 全部位于机器人局部系。
 *
 * 坐标：先依据机器人局部矩形采样规则确定采样位置，在 map 系内查询对应栅格的 3D 中心点
 * （由 grid_map::GridMap::getPosition3 得到），再左乘 T_{robot<-map}
 * （lookupTransform(robot_base, map_frame)）变换到机器人局部系，因此发布点云的 x/y/z 都是机器人局部系坐标。
 *
 * 点序：右后（right-back）为点云第 0 个点；先在同一列内从后往前，再从右向左切到下一列。
 */
class ElevationMapRobotFrameSampler {
 public:
  /**
   * @param node            与 ElevationMapping 共用同一节点以读取参数
   * @param elevation_map   内部地图引用（须与 ElevationMapping 生命周期一致）
   * @param tf_buffer       与 ElevationMapping 共用的 TF 缓冲
   */
  ElevationMapRobotFrameSampler(rclcpp::Node::SharedPtr node, ElevationMap& elevation_map,
                                std::shared_ptr<tf2_ros::Buffer> tf_buffer);

 private:
  struct Parameters {
    bool enable_{false};
    double publish_rate_hz_{2.0};
    std::string topic_{"elevation_sampled_cloud"};
    bool use_fused_map_{true};
    std::string layer_name_{"elevation"};
    std::string robot_base_frame_id_{"robot"};
    std::string invalid_height_fill_mode_{"none"};
    double invalid_height_body_offset_{0.0};
    bool rotate_output_with_robot_attitude_{true};
    bool publish_index_markers_{false};
    std::string marker_topic_{"elevation_sampled_cloud_indices"};
    double marker_scale_{0.06};
    double longitudinal_length_{2.5};
    double lateral_length_{1.5};
    int longitudinal_samples_{26};
    int lateral_samples_{16};
  };

  void readParameters();
  void timerCallback();

  rclcpp::Node::SharedPtr node_;
  ElevationMap& elevation_map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Parameters params_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace elevation_mapping
