#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/nvidia/Maps/d215_map.yaml',
        description='Absolute path to the map yaml file.',
    )

    trajectory_yaml_arg = DeclareLaunchArgument(
        'trajectory_yaml',
        default_value='/home/nvidia/ros2/src/qcar2_nodes/config/qcar2_trajectory.yaml',
        description='Absolute path to the QCar2 Bezier-7 trajectory yaml file.',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml')}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    bezier_visualizer = Node(
        package='qcar2_nodes',
        executable='qcar2_bezier',
        name='qcar2_bezier',
        output='screen',
        parameters=[LaunchConfiguration('trajectory_yaml')],
    )

    return LaunchDescription([
        map_yaml_arg,
        trajectory_yaml_arg,
        map_server,
        lifecycle_manager,
        bezier_visualizer,
    ])
