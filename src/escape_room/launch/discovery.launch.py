"""
Nav2 navigation stack launch for the escape room.

Replaces the old mapper_node + custom A*/pure-pursuit with:
    - lidar_node:         CoppeliaSim ground-truth rays → /scan + base_link→laser TF
    - slam_toolbox:       /scan → /map + map→odom TF
    - nav2 bringup:       costmap + NavFn planner + RPP controller + BT navigator
    - color_detector_node: HSV landmark detection
    - explorer_node:       mission FSM using Nav2 NavigateToPose action
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory('escape_room')
    nav2_dir = get_package_share_directory('nav2_bringup')

    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    rviz_config = os.path.join(pkg_dir, 'config', 'escape_room.rviz')

    return LaunchDescription([
        # RViz2 visualization (map, scan, costmaps, path)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log',
        ),

        # 2D virtual LiDAR: ZMQ ground-truth rays → /scan + base_link→laser TF
        Node(
            package='escape_room',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
        ),

        # SLAM: /scan + odom→base_link → /map + map→odom
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),

        # Nav2: costmap + planner + controller + BT navigator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),

        # HSV-based colour landmark detection → /targets/{cube,plate,door}
        Node(
            package='escape_room',
            executable='color_detector_node',
            name='color_detector_node',
            output='screen',
        ),

        # Mission FSM: sends NavigateToPose goals + drives gripper via ZMQ
        Node(
            package='escape_room',
            executable='explorer_node',
            name='explorer_node',
            output='screen',
        ),
    ])
