# elevation_mapping_ros2

基于 **ROS 2（Humble）** 的 **机器人中心高程地图** 功能包：融合配准点云与 TF，在滑动窗口栅格上维护高度与不确定性，并发布 `grid_map` 消息。工程面向 **FAST-LIO + Mid-360（或同类激光）** 等常见组合做了参数与启动示例，也可在修改坐标系与话题后用于其它平台。

本包是经典 Catkin 包 [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) 思路在 **ament / colcon** 下的移植与适配，使用 **grid_map**、**PCL**、**kindr**、**tf2** 等栈。

## 主要能力

- **单路或多路点云输入**：未配置 `input_sources_file` 时可用 `point_cloud_topic` 兼容模式；多路时由 YAML 描述各输入与传感器模型。
- **传感器处理器**：激光 / 双目 / 结构光 / 理想模型等，做点云裁剪、体素滤波与测量方差估计。
- **原始图与融合图**：原始层写入与方差传播，融合后发布 `elevation` 等层；可选可见性清理与后处理管道异步发布 `elevation_map_raw`。
- **地图随动**：通过 `track_point_*` 与 TF 将网格中心对齐到机器人参考点。
- **服务**：触发融合、查询子图、清图、掩码替换、参数重载等（部分与 rosbag 相关接口仍为占位实现）。
- **可选：高程栅格采样点云**（`ElevationMapRobotFrameSampler`）：按 `stride` 下采样融合/原始图，将地图系高度点变换到 **`robot_base_frame_id`** 并发布 `sensor_msgs/PointCloud2`，点序为「最右列最下行起，沿列自下而上，再向左列」。

## 依赖与环境

- **ROS 2 Humble**（`rclcpp`、`tf2_ros`、`sensor_msgs`、`geometry_msgs`、`grid_map_*`、`pcl_conversions`、PCL 等）
- **kindr**（工作空间中通常需先 `colcon build` 该包或已安装）
- **Boost**、**Eigen3**、**yaml-cpp**（经 `yaml_cpp_vendor`）

### 用 rosdep 安装系统依赖

在 **ROS 2 工作空间根目录**（与 `src/` 同级）执行，将 `package.xml` 中声明的依赖解析为系统包并安装（路径需与本仓在 `src` 下的实际位置一致）：

```bash
sudo rosdep init   # 若本机尚未初始化，仅需执行一次
rosdep update
rosdep install --from-paths src/elevation/ --ignore-src -r -y
```
### kindr
```bash
git clone https://github.com/ANYbotics/kindr.git

cd kindr
mkdir build
cd build
cmake ..
sudo make install
```

说明：`--from-paths src/elevation/` 会扫描该目录下各功能包的 `package.xml`；`--ignore-src` 跳过已在工作空间内的源码包；`-r` 继续尝试安装其余依赖；`-y` 默认同意安装提示。

## 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to elevation_mapping_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

若使用 IDE 跳转定义，建议加上 `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`，并保证合并后的 `compile_commands.json` 包含本包（见仓库内 `.clangd` / `.vscode` 说明）。

## 运行

**推荐：分拆参数 + Launch**

```bash
ros2 launch elevation_mapping_ros2 fastlio_elevation_mapping.launch.py
```

默认加载：

- `config/robots/fastlio_mid360_robot.yaml`（坐标系、话题、可选 `elevation_sampling`）
- `config/elevation_maps/fastlio_default_map.yaml`（分辨率、窗口、融合与定时器）
- `config/sensor_processors/mid360_laser.yaml`
- `config/postprocessing/postprocessor_pipeline.yaml`

可通过 launch 参数覆盖各 YAML 路径，例如：

```bash
ros2 launch elevation_mapping_ros2 fastlio_elevation_mapping.launch.py \
  robot_params:=/path/to/your_robot.yaml
```

**单文件参数快速试跑**（示例）：

```bash
ros2 run elevation_mapping_ros2 elevation_mapping_ros2_node --ros-args \
  --params-file $(ros2 pkg prefix elevation_mapping_ros2)/share/elevation_mapping_ros2/config/full_params/fastlio_pointcloud.yaml
```

运行前请确认：**点云话题**、**map / base TF 链** 与 YAML 中 `map_frame_id`、`robot_base_frame_id` 一致；FAST-LIO 若只发 `Odometry` 而无 `PoseWithCovarianceStamped`，可将 `robot_pose_with_covariance_topic` 置空，此时运动方差预测关闭，建图仍依赖点云与 TF。

## 配置与参数详解

各 YAML 字段、定时器逻辑、订阅分支等见同目录 **`configparam.md`**（比本 README 更细）。

## 节点与可执行文件

| 名称 | 说明 |
|------|------|
| `elevation_mapping_ros2_node` | 节点名默认 `elevation_mapping`，内建 `ElevationMapping` + 可选 `ElevationMapRobotFrameSampler` |
| `elevation_mapping_ros2_core` | 共享库，供节点链接 |

典型输出话题包括：`elevation_map`（融合 GridMap）、经后处理的原始图话题（由 postprocessing 配置）、以及启用后的 `elevation_sampling.topic` 点云。

## 作者与联系

- **作者**: Beauhowe Zhang（BeauhoweZhang）
- **邮箱**: [zbohao7@gmail.com](mailto:zbohao7@gmail.com)


## 相关文档

- 本仓库：**`configparam.md`** — 配置结构、参数表、启动与调试要点
- 上游概念与算法可参考 ANYbotics 原版 **elevation_mapping** 论文与文档
