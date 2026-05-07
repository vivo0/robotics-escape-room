"""
Discovery-phase launch:
    - mapper_node: paints /map from CoppeliaSim's known obstacle layout
    - explorer_node: frontier-based exploration (A* + pure pursuit)

Disable autonomous driving (mapping only) with:

    ros2 launch escape_room discovery.launch.py explore:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    explore_arg = DeclareLaunchArgument(
        'explore',
        default_value='true',
        description='Run explorer_node alongside the mapper.',
    )

    mapper = Node(
        package='escape_room',
        executable='mapper_node',
        name='mapper_node',
        output='screen',
    )

    explorer = Node(
        package='escape_room',
        executable='explorer_node',
        name='explorer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    return LaunchDescription([explore_arg, mapper, explorer])
