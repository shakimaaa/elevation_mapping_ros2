# elevation_mapping_ros2

基于 **ROS 2（Humble）** 的 **机器人中心高程地图** 功能包：融合配准点云与 TF，在滑动窗口栅格上维护高度与不确定性，并发布 `grid_map` 消息。工程面向 **FAST-LIO + Mid-360（或同类激光）** 等常见组合做了参数与启动示例，也可在修改坐标系与话题后用于其它平台。

本包是经典 Catkin 包 [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) 思路在 **ament / colcon** 下的移植与适配，使用 **grid_map**、**PCL**、**kindr**、**tf2** 等栈。

## 主要能力

- **单路或多路点云输入**：未配置 `input_sources_file` 时可用 `point_cloud_topic` 兼容模式；多路时由 YAML 描述各输入与传感器模型。
- **传感器处理器**：激光 / 双目 / 结构光 / 理想模型等，做点云裁剪、体素滤波与测量方差估计。
- **原始图与融合图**：原始层写入与方差传播，融合后发布 `elevation` 等层；可选可见性清理与后处理管道异步发布 `elevation_map_raw`。
- **地图随动**：通过 `track_point_*` 与 TF 将网格中心对齐到机器人参考点。
- **服务**：触发融合、查询子图、清图、掩码替换、参数重载等（部分与 rosbag 相关接口仍为占位实现）。
- **可选：高程栅格采样点云**（`ElevationMapRobotFrameSampler`）：按机器人周围局部矩形规则在 map 系高程图上取样，再整体变换到 **`robot_base_frame_id`** 并发布 `sensor_msgs/PointCloud2`；输出点的 `x/y/z` 全部位于机器人局部系，点序为 `right-back` 起、同列后到前、再由右向左。采样网格始终先在机器人局部系定义，因此可用 `lateral_*` / `longitudinal_*` 精确控制局部平面采样分辨率。
- **可选：采样序号 MarkerArray**：可为每个采样点发布一个 `TEXT_VIEW_FACING` marker，用于在 RViz 中查看采样点序号。
- **采样点云调试脚本**：提供 Python 工具订阅采样点云、按原始顺序解析 `x/y/z`、按二维网格显示，并可按需保存 CSV。

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

### 采样点云常用参数

`config/robots/fastlio_mid360_robot.yaml` 中的 `elevation_sampling` 示例：

```yaml
elevation_sampling:
  enable: true
  publish_rate: 10.0
  topic: elevation_sampled_cloud
  use_fused_map: false
  layer_name: elevation
  invalid_height_fill_mode: none
  invalid_height_body_offset: -0.2
  rotate_output_with_robot_attitude: true
  publish_index_markers: true
  marker_topic: elevation_sampled_cloud_indices
  marker_scale: 0.06
  lateral_samples: 16
  longitudinal_samples: 26
  lateral_length: 1.5
  longitudinal_length: 2.5
```

说明：

- `lateral_samples=16`、`lateral_length=1.5` 对应横向采样间距 `0.1 m`
- `longitudinal_samples=26`、`longitudinal_length=2.5` 对应纵向采样间距 `0.1 m`
- 上面的 `0.1 m` 指的是**机器人局部平面采样分辨率**，即采样网格在局部系中的 `x/y` 采样间距；它并不表示最终三维点之间的欧氏距离恒为 `0.1 m`
- `publish_index_markers=true` 时会额外发布 `MarkerArray`
- `use_fused_map=false` 更适合调试采样逻辑，避免融合图未更新时得到空点云
- `invalid_height_fill_mode` 用于指定高程图该采样点无有效高度时的补值策略：
  `none` 表示跳过该点，`last_valid` 表示沿当前点序使用上一个有效点高度，`body` 表示使用 `map` 系 `z=0` 变换到机器人局部系后的高度进行补值
- `invalid_height_fill_mode=body` 时，补出来的高度表示机器人局部系下 body 相对地面的高度，通常会是负值，正负号会与其它正常采样点保持一致
- `invalid_height_body_offset` 仅在 `invalid_height_fill_mode=body` 时生效，会在 body-to-ground 高度基础上再叠加一个额外偏移量；例如 `-0.1979` 表示再向下偏移 `0.1979 m`
- `rotate_output_with_robot_attitude=true` 时，输出点云会完整跟随机器人姿态变化，`x/y/z` 都会随 yaw / roll / pitch 改变
- `rotate_output_with_robot_attitude=false` 时，输出点会保持在固定的 body 局部规则网格上，便于检查局部平面采样间距是否严格一致

## 调试脚本

包内提供脚本 `print_elevation_sampled_cloud.py`，用于订阅采样点云并按原始顺序解析显示每个点的 `x/y/z`。

先确保环境已加载：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 只打印一帧

```bash
ros2 run elevation_mapping_ros2 print_elevation_sampled_cloud.py --once
```

### 按二维网格显示

```bash
ros2 run elevation_mapping_ros2 print_elevation_sampled_cloud.py \
  --once \
  --show-grid \
  --rows 26 \
  --cols 16
```

### 保存 CSV

```bash
ros2 run elevation_mapping_ros2 print_elevation_sampled_cloud.py \
  --once \
  --save-csv \
  --csv-path /tmp/sampled_cloud.csv
```

### 同时打印网格并保存 CSV

```bash
ros2 run elevation_mapping_ros2 print_elevation_sampled_cloud.py \
  --once \
  --show-grid \
  --rows 26 \
  --cols 16 \
  --save-csv \
  --csv-path /tmp/sampled_cloud.csv
```

注意：

- 不加 `--save-csv` 时不会保存任何 CSV 文件
- 单独传 `--csv-path` 但不加 `--save-csv`，同样不会保存
- `rows` 表示每列中的点数，`cols` 表示列数；默认对应当前采样配置 `26 x 16`

## 节点与可执行文件

| 名称 | 说明 |
|------|------|
| `elevation_mapping_ros2_node` | 节点名默认 `elevation_mapping`，内建 `ElevationMapping` + 可选 `ElevationMapRobotFrameSampler` |
| `elevation_mapping_ros2_core` | 共享库，供节点链接 |
| `print_elevation_sampled_cloud.py` | 订阅采样点云并打印/网格显示/按需保存 CSV 的调试脚本 |

典型输出话题包括：`elevation_map`（融合 GridMap）、经后处理的原始图话题（由 postprocessing 配置）、以及启用后的 `elevation_sampling.topic` 点云。

## 作者与联系

- **作者**: Beauhowe Zhang（BeauhoweZhang）
- **邮箱**: [zbohao7@gmail.com](mailto:zbohao7@gmail.com)


## 相关文档

- 本仓库：**`configparam.md`** — 配置结构、参数表、启动与调试要点
- 上游概念与算法可参考 ANYbotics 原版 **elevation_mapping** 论文与文档
