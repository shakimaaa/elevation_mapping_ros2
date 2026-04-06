"""FAST-LIO 示例：加载分拆 YAML 并启动 elevation_mapping 节点。

Author: Beauhowe Zhang <zbohao7@gmail.com>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("elevation_mapping_ros2")

    default_robot = os.path.join(
        pkg_share, "config", "robots", "fastlio_mid360_robot.yaml"
    )
    default_map = os.path.join(
        pkg_share, "config", "elevation_maps", "fastlio_default_map.yaml"
    )
    default_sensor = os.path.join(
        pkg_share, "config", "sensor_processors", "mid360_laser.yaml"
    )
    default_post = os.path.join(
        pkg_share, "config", "postprocessing", "postprocessor_pipeline.yaml"
    )

    robot_params = LaunchConfiguration("robot_params")
    map_params = LaunchConfiguration("map_params")
    sensor_params = LaunchConfiguration("sensor_params")
    post_params = LaunchConfiguration("post_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_params",
                default_value=default_robot,
                description="Robot/frame/topic parameters YAML",
            ),
            DeclareLaunchArgument(
                "map_params",
                default_value=default_map,
                description="Elevation map behavior parameters YAML",
            ),
            DeclareLaunchArgument(
                "sensor_params",
                default_value=default_sensor,
                description="Sensor processor parameters YAML",
            ),
            DeclareLaunchArgument(
                "post_params",
                default_value=default_post,
                description="Postprocessing parameters YAML",
            ),
            Node(
                package="elevation_mapping_ros2",
                executable="elevation_mapping_ros2_node",
                name="elevation_mapping",
                output="screen",
                parameters=[robot_params, map_params, sensor_params, post_params],
            ),
        ]
    )
