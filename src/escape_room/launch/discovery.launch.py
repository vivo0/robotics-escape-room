"""
Discovery-phase launch:
    - mapper_node:         /map from CoppeliaSim's known obstacle layout
    - explorer_node:       frontier-based exploration (A* + pure pursuit)
    - color_detector_node: red/green/blue landmark detection on /camera
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(package='escape_room', executable='mapper_node',
             name='mapper_node', output='screen'),
        Node(package='escape_room', executable='explorer_node',
             name='explorer_node', output='screen'),
        Node(package='escape_room', executable='color_detector_node',
             name='color_detector_node', output='screen'),
    ])
