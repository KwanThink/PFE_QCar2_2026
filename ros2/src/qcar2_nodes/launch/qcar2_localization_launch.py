import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('qcar2_nodes')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    device_type = LaunchConfiguration('device_type')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        },
        convert_types=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the localization stack'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/nvidia/Maps/d215_map.yaml',
        description='Full path to the map yaml file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'qcar2_localization_and_nav.yaml'),
        description='Full path to the localization parameter file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically bring map_server and amcl to the active state'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level'
    )

    declare_device_type_cmd = DeclareLaunchArgument(
        'device_type',
        default_value='physical',
        description='physical or virtual'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz2 together with localization'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RViz config file'
    )

    qcar2_hardware_node = Node(
        package='qcar2_nodes',
        executable='qcar2_hardware',
        name='qcar2_hardware',
        output='screen',
        parameters=[{'device_type': device_type}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    qcar2_odometry_node = Node(
        package='qcar2_nodes',
        executable='qcar2_odometry',
        name='qcar2_odometry',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    lidar_node = Node(
        package='qcar2_nodes',
        executable='lidar',
        name='lidar',
        output='screen',
        parameters=[{'device_type': device_type}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    fixed_lidar_frame_physical = Node(
        package='qcar2_nodes',
        executable='fixed_lidar_frame',
        name='fixed_lidar_frame',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(["'", device_type, "' == 'physical'"])),
    )

    fixed_lidar_frame_virtual = Node(
        package='qcar2_nodes',
        executable='fixed_lidar_frame_virtual',
        name='fixed_lidar_frame_virtual',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(["'", device_type, "' == 'virtual'"])),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', log_level],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    localization_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),
        qcar2_hardware_node,
        qcar2_odometry_node,
        lidar_node,
        fixed_lidar_frame_physical,
        fixed_lidar_frame_virtual,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization_node,
    ])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_device_type_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)

    ld.add_action(localization_group)
    ld.add_action(rviz_node)

    return ld
