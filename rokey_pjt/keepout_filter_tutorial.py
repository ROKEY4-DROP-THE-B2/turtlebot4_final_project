import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
# RewrittenYaml 완전히 제거

def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('nav2_bringup')

    # 모든 불리언은 LaunchConfiguration으로
    namespace          = LaunchConfiguration('namespace')
    keepout_mask_yaml  = LaunchConfiguration('keepout_mask')
    use_sim_time       = LaunchConfiguration('use_sim_time')
    autostart          = LaunchConfiguration('autostart')
    params_file        = LaunchConfiguration('params_file')
    use_composition    = LaunchConfiguration('use_composition')
    container_name     = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn        = LaunchConfiguration('use_respawn')
    use_keepout_zones  = LaunchConfiguration('use_keepout_zones')
    log_level          = LaunchConfiguration('log_level')

    lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']

    # tf 리매핑
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # RewrittenYaml 대신 그냥 params_file을 그대로 건넴
    configured_params = ParameterFile(params_file, allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # 인자 선언 (기본값: robot2도 가능하지만 여기선 일반값)
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='robot1', description='Top-level namespace')

    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask', default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
        description='Full path to keepout mask yaml file to load',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time')
    declare_params_file_cmd  = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )
    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='false', description='Use composition')
    declare_container_name_cmd  = DeclareLaunchArgument('container_name', default_value='nav2_container', description='container name')
    declare_use_respawn_cmd     = DeclareLaunchArgument('use_respawn', default_value='false', description='respawn if crash')
    declare_use_keepout_zones_cmd = DeclareLaunchArgument('use_keepout_zones', default_value='true', description='enable keepout zones')
    declare_log_level_cmd       = DeclareLaunchArgument('log_level', default_value='info', description='log level')

    # 누락되기 쉬운 autostart 인자
    declare_autostart_cmd       = DeclareLaunchArgument('autostart', default_value='true', description='autostart lifecycle')

    # Non-composition 경로
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            PushRosNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),

            # keepout 전용 MapServer (토픽 충돌 방지: topic_name=keepout_mask)
            Node(
                condition=IfCondition(use_keepout_zones),
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': keepout_mask_yaml, 'topic_name': 'keepout_mask'}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # CostmapFilterInfoServer
            Node(
                condition=IfCondition(use_keepout_zones),
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='keepout_costmap_filter_info_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            # Lifecycle Manager
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout_zone',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # Composition 경로
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushRosNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),

            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(use_keepout_zones),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='keepout_filter_mask_server',
                        parameters=[configured_params, {'yaml_filename': keepout_mask_yaml, 'topic_name': 'keepout_mask'}],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='keepout_costmap_filter_info_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                ],
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_keepout_zone',
                        parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}],
                    ),
                ],
            ),
        ],
    )

    # LaunchDescription
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    # Declare
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_autostart_cmd)

    # Actions
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
