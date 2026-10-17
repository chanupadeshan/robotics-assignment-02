"""Independent localization lifecycle, TF gate, then standalone Nav2 servers."""
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def setup(context):
    world = Path(get_package_share_directory('maze_world'))
    runpy.run_path(str(world / 'launch/launch_support.py'))['guard'](
        'localization', {'amcl', 'slam_toolbox', 'map_server', 'planner_server', 'bt_navigator'})
    params = ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)
    common = [params, {'use_sim_time': True}]
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_collision_monitor', 'collision_monitor'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]
    navigation = []
    for package, executable in servers:
        remaps = []
        if executable in ('controller_server', 'behavior_server'):
            remaps = [('cmd_vel', 'cmd_vel_nav')]
        elif executable == 'velocity_smoother':
            remaps = [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]
        navigation.append(Node(package=package, executable=executable, name=executable,
                               parameters=common, remappings=remaps, output='screen'))
    navigation.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': [name for _, name in servers], 'bond_timeout': 10.0}]))
    initializer = Node(
        package='maze_navigation', executable='initialize_localization.py',
        name='initialize_localization', output='screen',
        parameters=[{'use_sim_time': True, 'x': LaunchConfiguration('initial_x'),
                     'y': LaunchConfiguration('initial_y'),
                     'yaw': LaunchConfiguration('initial_yaw')}])

    def after_localization(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Localization initialization failed'))]
        return navigation

    rviz_config = LaunchConfiguration('rviz_config')
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=common + [{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server', 'amcl'], 'bond_timeout': 10.0}]),
        RegisterEventHandler(OnProcessExit(target_action=initializer, on_exit=after_localization)),
        initializer,
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('maze_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(share / 'maps/maze_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config',
                              default_value=str(share / 'config/maze_navigation.rviz')),
        DeclareLaunchArgument('initial_x', default_value='-0.764'),
        DeclareLaunchArgument('initial_y', default_value='-1.250'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
