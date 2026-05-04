from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar2_nodes')
    params = os.path.join(pkg_share, 'config', 'qcar2_NLMPC.yaml')

    bezier = Node(
        package='qcar2_nodes',
        executable='qcar2_bezier',
        name='qcar2_bezier',
        output='screen',
        parameters=[params],
    )

    nlmpc = Node(
        package='qcar2_nodes',
        executable='qcar2_nlmpc',
        name='qcar2_nlmpc',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([bezier, nlmpc])
