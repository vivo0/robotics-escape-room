"""
Nav2 navigation stack launch for the escape room.

    - static_transform_publisher: base_link → laser_link
    - slam_toolbox:       /scan → /map + map→odom TF
    - nav2 bringup:       costmap + NavFn planner + RPP controller + BT navigator
    - color_detector_node: HSV landmark detection
    - explorer_node:       mission FSM using Nav2 NavigateToPose action

/scan is produced by lidar_sensor.lua injected into CoppeliaSim by build_scene.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory("escape_room")
    nav2_dir = get_package_share_directory("nav2_bringup")

    slam_params = os.path.join(pkg_dir, "config", "slam_toolbox_params.yaml")
    nav2_params = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    rviz_config = os.path.join(pkg_dir, "config", "escape_room.rviz")

    return LaunchDescription(
        [
            # Static TF: base_link → laser_link (TRANSIENT_LOCAL — always
            # received by slam_toolbox regardless of startup order).
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser_link",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0.12",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "laser_link",
                ],
            ),
            # RViz2 visualization (map, scan, costmaps, path)
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                ros_arguments=["--log-level", "WARN"],
                output="log",
            ),
            # SLAM: /scan + odom→base_link → /map + map→odom
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
                ros_arguments=["--log-level", "WARN"],
            ),
            # Nav2: costmap + planner + controller + BT navigator
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": "false",
                    "autostart": "true",
                    "log_level": "warn",
                }.items(),
            ),
            # HSV-based colour landmark detection → /targets/{cube,plate,door}
            Node(
                package="escape_room",
                executable="color_detector_node",
                name="color_detector_node",
                output="screen",
            ),
            # Mission FSM: sends NavigateToPose goals + drives gripper via ZMQ
            Node(
                package="escape_room",
                executable="explorer_node",
                name="explorer_node",
                output="screen",
            ),
        ]
    )
