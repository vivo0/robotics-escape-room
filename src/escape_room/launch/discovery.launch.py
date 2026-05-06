"""
Discovery-phase launch:
  - static TF: chassis_base_link -> velodyne (Velodyne mount on the robot)
  - mapper_node: builds /map from /velodyne_points

Tune the Velodyne mount via launch args (defaults assume sensor centered
on top of the chassis, ~15 cm above):
    ros2 launch escape_room discovery.launch.py velodyne_z:=0.18
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('velodyne_x', default_value='0.0'),
        DeclareLaunchArgument('velodyne_y', default_value='0.0'),
        DeclareLaunchArgument('velodyne_z', default_value='0.15'),
        # The CoppeliaSim VPL16 script publishes points in a frame whose Y
        # axis is the laser spin axis, not Z. Roll +90° brings that into a
        # ROS-standard Z-up frame so the 2D map slice is horizontal.
        DeclareLaunchArgument('velodyne_roll', default_value='1.5707963'),
        DeclareLaunchArgument('velodyne_pitch', default_value='0.0'),
        DeclareLaunchArgument('velodyne_yaw', default_value='0.0'),
        DeclareLaunchArgument('velodyne_parent_frame',
                              default_value='chassis_base_link'),
        DeclareLaunchArgument('velodyne_child_frame', default_value='velodyne'),
    ]

    velodyne_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_static_tf',
        arguments=[
            '--x', LaunchConfiguration('velodyne_x'),
            '--y', LaunchConfiguration('velodyne_y'),
            '--z', LaunchConfiguration('velodyne_z'),
            '--roll', LaunchConfiguration('velodyne_roll'),
            '--pitch', LaunchConfiguration('velodyne_pitch'),
            '--yaw', LaunchConfiguration('velodyne_yaw'),
            '--frame-id', LaunchConfiguration('velodyne_parent_frame'),
            '--child-frame-id', LaunchConfiguration('velodyne_child_frame'),
        ],
        output='screen',
    )

    mapper = Node(
        package='escape_room',
        executable='mapper_node',
        name='mapper_node',
        output='screen',
    )

    return LaunchDescription(args + [velodyne_static_tf, mapper])
