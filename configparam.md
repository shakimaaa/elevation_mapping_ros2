# elevation_mapping_ros2 配置说明（FAST-LIO / Mid-360）

本文档整理当前仓库里与配置相关的文件、参数含义、以及推荐启动方式。

## 1. 配置文件结构

当前支持两种风格：

- 分拆风格（推荐，和 ROS1 一致）  
  - `config/robots/fastlio_mid360_robot.yaml`
  - `config/elevation_maps/fastlio_default_map.yaml`
  - `config/sensor_processors/mid360_laser.yaml`
  - `config/postprocessing/postprocessor_pipeline.yaml`
- 单文件风格（便于快速试跑）  
  - `config/full_params/fastlio_pointcloud.yaml`

## 2. 分拆风格各文件作用

### 2.1 `robots/fastlio_mid360_robot.yaml`

机器人/话题/坐标系相关参数：

- `map_frame_id`: 地图坐标系（FAST-LIO 常用 `camera_init`）
- `robot_base_frame_id`: 机器人基座坐标系（常用 `body`）
- `track_point_frame_id`, `track_point_x`, `track_point_y`, `track_point_z`: 地图随动参考点
- `point_cloud_topic`: 单路点云输入话题（常用 `/cloud_registered`）
- `robot_pose_with_covariance_topic`: 位姿协方差输入（`PoseWithCovarianceStamped`）
- `robot_pose_cache_size`: 位姿缓存深度
- `num_callback_threads`: 线程数

说明：如果 `robot_pose_with_covariance_topic` 设为空字符串，代码会关闭运动方差预测（仅依赖 TF 与点云建图）。

#### 高程栅格采样点云 `elevation_sampling`（可选）

节点在 `elevation_mapping_node.cpp` 中构造 `ElevationMapRobotFrameSampler`：先按机器人周围局部矩形规则在 **map 系** 高程图上确定采样位置，再读取对应格子的 **地图系 3D 中心**（由 `grid_map::GridMap::getPosition3` 得到，z 为指定层高程），最后整体变换到 **`robot_base_frame_id` 局部系** 后发布 `sensor_msgs/PointCloud2`（`frame_id` 为机器人基座系），因此发布点云的 `x/y/z` 全部处于机器人局部系。

- `elevation_sampling.enable`：是否启用（默认 `false`）。
- `elevation_sampling.publish_rate`：发布频率（Hz）。
- `elevation_sampling.topic`：点云话题名。
- `elevation_sampling.lateral_samples`：横向采样点数，例如 `16`。
- `elevation_sampling.longitudinal_samples`：纵向采样点数，例如 `26`。
- `elevation_sampling.lateral_length`：横向采样矩形宽度（m）。
- `elevation_sampling.longitudinal_length`：纵向采样矩形长度（m）。
  点云中的**点顺序**：`right-back` 为第 0 个点；同一列内从后往前，再从右向左切换到下一列。  
  例如 `lateral_samples=16`、`longitudinal_samples=26` 且 `lateral_length=1.5`、`longitudinal_length=2.5` 时，横纵两个方向的采样间距都为 `0.1 m`。
- `elevation_sampling.rotate_output_with_robot_attitude`：是否让输出点云完整跟随机器人姿态变化。  
  - `true`：输出点的 `x/y/z` 都会随 yaw / roll / pitch 改变。  
  - `false`：输出点保持在固定的 body 局部规则网格上，仅高度 `z` 随地图与姿态变化。
- `elevation_sampling.invalid_height_fill_mode`：当高程图该采样点没有有效高度时的补值策略。  
  - `none`：不补值，直接跳过该点。  
  - `last_valid`：使用当前点序中的上一个有效点高度。  
  - `body`：使用 `map` 系 `z=0` 的点变换到 `robot_base_frame_id` 后的高度补值；该高度通常表示 body 相对地面的高度，在局部系中常为负值，符号会与其它正常采样点保持一致。
- `elevation_sampling.invalid_height_body_offset`：仅在 `invalid_height_fill_mode=body` 时生效，在 body-to-ground 高度基础上追加的额外偏移量。  
  - 例如 `-0.1979` 表示在原有 body 补值高度基础上再减 `0.2 m`。
- `elevation_sampling.use_fused_map`：`true` 用融合图，`false` 用原始图。
- `elevation_sampling.layer_name`：高度图层名，一般为 `elevation`。
- `elevation_sampling.publish_index_markers`：是否额外发布采样点序号 `MarkerArray`。
- `elevation_sampling.marker_topic`：采样点序号 marker 的话题名。
- `elevation_sampling.marker_scale`：文字 marker 的显示尺度。

补充说明：

- 这里的 `0.1 m` 指的是**机器人局部平面采样分辨率**，也就是采样网格在局部系中的 `x/y` 间距；并不表示最终三维点之间的欧氏距离恒为 `0.1 m`。
- 采样位置始终先在机器人局部系中定义，再映射到 `map` 系查询高度，因此局部规则网格和点序在两种输出模式下都保持一致。

需保证 TF 中存在 `map_frame_id` → `robot_base_frame_id` 的变换（与建图一致）。

### 2.2 `elevation_maps/fastlio_default_map.yaml`

地图行为和融合参数：

- 更新与定时：
  - `min_update_rate`：无新点云时“保底预测更新”的最低频率（Hz）。  
    - 代码里会转成 `max_no_update_duration = 1 / min_update_rate`。  
    - 设太小：地图在停更时更新不及时；设太大：CPU 占用上升。  
  - `time_tolerance`：位姿和点云时间对齐的容差（秒）。  
    - 主要影响 `updatePrediction()` 是否接受该时刻位姿。  
    - 设太小：容易报时序不匹配；设太大：可能使用不够“近”的位姿。  
  - `fused_map_publishing_rate`：融合图发布频率（Hz）。  
    - `0`：不启用周期发布（仅服务触发）。  
    - `inf`：每次点云回调里尝试连续融合（更实时但更耗算力）。  
  - `visibility_cleanup_rate`：可见性清理周期（Hz）。  
    - 仅在启用可见性清理且非连续清理模式下创建该定时器。  

- 地图几何：
  - `resolution`：栅格分辨率（m/格）。  
    - 越小地图越精细，但内存和计算量显著增加。  
  - `length_in_x`, `length_in_y`：地图窗口尺寸（m）。  
    - 共同决定网格总大小（`length / resolution`）。  
  - `position_x`, `position_y`：初始地图中心在地图系的位置。  
    - 对启动初始可视范围有影响，运行中会被 `map_.move()` 动态更新。  

- 融合与噪声模型：
  - `min_variance`, `max_variance`：高度方差上下限。  
    - 下限防止过度自信，上限防止数值发散。  
  - `mahalanobis_distance_threshold`：多高度检测阈值。  
    - 超阈值时按“多高度/异常点”分支处理。  
    - 小：更保守，容易拒绝新高度；大：更容易接受跳变。  
  - `multi_height_noise`：判定为多高度时给方差附加的噪声项。  
    - 增大可降低这类点对地图的影响。  
  - `min_horizontal_variance`, `max_horizontal_variance`：水平不确定度上下限。  
    - 影响平面方向的不确定性传播和约束。  
  - `scanning_duration`：同一扫描时间窗（秒）。  
    - 用于判断点是否来自同一扫描周期，影响多高度融合逻辑与清理逻辑。  
  - `increase_height_alpha`：同一扫描内出现更高点时的平滑系数。  
    - `0` 更快跟随新高点，`1` 基本不更新。  

- 清理与服务：
  - `underlying_map_topic`：底图输入话题（可选）。  
    - 非空时会订阅底图并参与底层地图更新逻辑。  
  - `enable_visibility_cleanup`：是否启用可见性清理机制。  
  - `enable_continuous_cleanup`：是否在点云回调中连续清理（而非定时器清理）。  
    - 连续清理实时性高但更耗算力。  
  - `masked_replace_service_mask_layer_name`：`masked_replace` 服务使用的掩码层名。  
    - 掩码非 NaN 区域会覆盖目标层。  

- 初始化：
  - `initialize_elevation_map`：启动时是否执行初始子图高度填充。  
  - `initialization_method`：初始化策略（当前实现主要是平面初始化）。  
  - `length_in_x_init_submap`, `length_in_y_init_submap`：初始化子图尺寸。  
  - `init_submap_height_offset`：初始化高度偏移（相对目标 frame 高度）。  
  - `init_submap_variance`：初始化区域方差初值。  
  - `target_frame_init_submap`：初始化时用于取高度的参考 frame。  
    - 典型设为 `footprint` 或 `base_link`。  

- 运动更新：
  - `robot_motion_map_update.covariance_scale`：位姿协方差缩放系数。  
    - >1：更保守，运动预测引入更大不确定度；<1：更激进。  
    - 当 `robot_pose_with_covariance_topic` 为空时，这一项不会生效（运动更新被关闭）。

### 2.3 `sensor_processors/mid360_laser.yaml`

传感器处理器参数：

- `sensor_processor.type: laser`
- 通用预处理：
  - `sensor_processor.ignore_points_above`
  - `sensor_processor.ignore_points_below`
  - `sensor_processor.apply_voxelgrid_filter`
  - `sensor_processor.voxelgrid_filter_size`
- 激光噪声模型：
  - `sensor_processor.min_radius`
  - `sensor_processor.beam_angle`
  - `sensor_processor.beam_constant`

### 2.4 `postprocessing/postprocessor_pipeline.yaml`

后处理与发布：

- `postprocessor_num_threads`
- `postprocessor_pipeline_name`
- `output_topic`

另外，文件中注释了 `postprocessor_pipeline` 的 filters 链示例。若不配置真实滤波链，节点会警告并退化为直接转发 raw map。

## 3. 推荐启动方式（分拆风格）

使用已提供的 launch：

```bash
ros2 launch elevation_mapping_ros2 fastlio_elevation_mapping.launch.py
```

该 launch 默认加载上面四份分拆配置。

## 4. 参数覆盖示例

只改某一份配置路径：

```bash
ros2 launch elevation_mapping_ros2 fastlio_elevation_mapping.launch.py \
  robot_params:=/absolute/path/your_robot.yaml
```

可覆盖参数：

- `robot_params`
- `map_params`
- `sensor_params`
- `post_params`

## 5. 单文件模式启动

```bash
ros2 run elevation_mapping_ros2 elevation_mapping_ros2_node --ros-args \
  --params-file $(ros2 pkg prefix elevation_mapping_ros2)/share/elevation_mapping_ros2/config/full_params/fastlio_pointcloud.yaml
```

## 6. 常见注意事项

- FAST-LIO 常发 `nav_msgs/Odometry`，而本节点使用 `PoseWithCovarianceStamped`。若需要运动方差预测，请增加一个桥接节点进行消息转换。
- 请确认 TF 链完整：`map_frame_id`、`robot_base_frame_id`、点云 `frame_id` 必须可互相变换。
- 若使用 `input_sources_file` 多路输入配置，`point_cloud_topic` 将不再作为主路径。

## 7. 参数对启动逻辑的影响（按程序流程）

下面是节点从启动到进入稳定运行的主流程，以及参数在哪一步生效。

### 7.1 节点构造阶段

构造顺序大致为：

1. 创建 TF Buffer/Listener
2. `readParameters()`
3. `setupSubscribers()`
4. `setupServices()`
5. `setupTimers()`
6. `initialize()`

因此，配置文件里的参数会在上述阶段被一次性读取并决定后续分支。

### 7.2 `readParameters()` 阶段（决定全局行为）

这一阶段读取大部分参数并决定“系统拓扑”与“算法模式”：

- 坐标系与地图几何  
  - `map_frame_id`、`resolution`、`length_in_x/y`、`position_x/y`
  - 直接决定网格初始化方式与地图 frame。
- 传感器处理器类型  
  - `sensor_processor.type`
  - 决定实例化 `laser/stereo/structured_light/perfect` 哪个处理器，类型错会直接启动失败。
- 频率与时间参数  
  - `min_update_rate`、`fused_map_publishing_rate`、`visibility_cleanup_rate`、`time_tolerance`
  - 影响后续 timer 是否创建、周期多长、预测是否允许。
- 地图模型参数  
  - `min_variance`、`max_variance`、`mahalanobis_distance_threshold` 等
  - 影响点融合时更新规则和噪声上下界。
- 初始化参数  
  - `initialize_elevation_map` 等
  - 决定启动后是否进行平面子图初始化。

### 7.3 `setupSubscribers()` 阶段（决定订阅路径）

这一阶段最关键的分支是：

- 如果配置了 `input_sources_file` 且解析成功：  
  走多路输入，`point_cloud_topic` 不作为主输入。
- 否则若存在 `point_cloud_topic`：  
  走单路兼容订阅。

位姿订阅逻辑：

- `robot_pose_with_covariance_topic` 非空：创建位姿订阅与缓存，后续启用运动方差预测。
- `robot_pose_with_covariance_topic` 为空：设置忽略运动更新，建图只依赖点云+TF。

### 7.4 `setupTimers()` 阶段（决定周期任务）

- `min_update_rate` 决定“无点云时的保底更新定时器”触发间隔。
- `fused_map_publishing_rate`：
  - `>0` 创建融合发布定时器；
  - `=0` 不创建（仅服务触发）；
  - `inf` 进入“在点云回调里连续融合”模式。
- `visibility_cleanup_rate` + `enable_visibility_cleanup` + `enable_continuous_cleanup`  
  共同决定是否创建可见性清理定时器。

### 7.5 `initialize()` 阶段（启动后首帧行为）

- `initialize_elevation_map=true` 时，会尝试用 `target_frame_init_submap` 对齐 TF 并写入初始子图高度；
- 若 TF 不可用，仅跳过初始化，不会导致整节点崩溃。

### 7.6 运行期行为受哪些参数直接控制

- 点云融合开关：`enable_updates` / `disable_updates` 服务会动态影响更新状态。
- 地图随动：`track_point_*` + `track_point_frame_id` 决定 `map_.move(...)` 的目标点。
- 传感器预处理：`sensor_processor.*` 决定裁剪、下采样、测量方差模型。
- 后处理：`postprocessor_pipeline_name` 和 `output_topic` 决定后处理链是否配置成功及发布话题。


