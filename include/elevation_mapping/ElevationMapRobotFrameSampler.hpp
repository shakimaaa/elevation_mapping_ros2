/**
 * @file ElevationMapRobotFrameSampler.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 按栅格步长采样高程图，将地图系下的 (x,y,z) 变换到机器人基座系并发布 PointCloud2。
 */
#pragma once

#include "elevation_mapping/ElevationMap.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

namespace elevation_mapping {

/**
 * @brief 从 ElevationMap 的融合或原始网格中按 stride 下采样，z 取指定层高程，xy 经 TF 转到机器人局部系。
 *
 * 坐标：栅格中心在地图系下的 3D 位置由 grid_map::GridMap::getPosition3 得到，再左乘
 * T_{robot<-map}（lookupTransform(robot_base, map_frame)），使点云随机器人旋转在机体系中变化。
 *
 * 点序（与 ElevationMap 中 size(0)=行、size(1)=列 一致）：最右一列、最下一格为点云第 0 个点；
 * 沿该列自下而上（行索引减小方向上的下一采样格为 1、2、…），该列结束后移到左侧下一列再自下而上，依此类推。
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
    int stride_x_{1};
    int stride_y_{1};
    bool use_fused_map_{true};
    std::string layer_name_{"elevation"};
    std::string robot_base_frame_id_{"robot"};
  };

  void readParameters();
  void timerCallback();

  rclcpp::Node::SharedPtr node_;
  ElevationMap& elevation_map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Parameters params_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace elevation_mapping
