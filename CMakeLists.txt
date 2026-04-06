cmake_minimum_required(VERSION 3.16)
project(elevation_mapping_ros2)

# 生成 compile_commands.json，供 clangd / Cursor 跳转定义与补全（colcon 会合并到 workspace/build/）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(kindr REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)

set(EM_TARGETS
  src/PointXYZRGBConfidenceRatio.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/StructuredLightSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
  src/input_sources/Input.cpp
  src/input_sources/InputSourceManager.cpp
  src/postprocessing/PostprocessingPipelineFunctor.cpp
  src/postprocessing/PostprocessingWorker.cpp
  src/postprocessing/PostprocessorPool.cpp
  src/ElevationMap.cpp
  src/ElevationMapRobotFrameSampler.cpp
  src/ElevationMapping.cpp
)

add_library(${PROJECT_NAME}_core SHARED
  ${EM_TARGETS}
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# find_package(kindr REQUIRED) 必须在 ament_target_dependencies(... kindr ...) 之前，
# 否则 ament 会报 “package name 'kindr' was not found before”。
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_conversions
  kindr
  yaml_cpp_vendor
  filters
  pluginlib
)

target_link_libraries(${PROJECT_NAME}_core
  ${PCL_LIBRARIES}
  Boost::thread
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_node
  src/elevation_mapping_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

install(
  TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_conversions
  kindr
  yaml_cpp_vendor
  filters
  pluginlib
)

ament_package()