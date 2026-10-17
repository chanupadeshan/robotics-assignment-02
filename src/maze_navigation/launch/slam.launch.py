"""Genuine LiDAR mapping mode, mutually exclusive with AMCL navigation."""
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    world = Path(get_package_share_directory('maze_world'))
    runpy.run_path(str(world / 'launch/launch_support.py'))['guard'](
        'localization', {'amcl', 'slam_toolbox', 'map_server', 'planner_server', 'bt_navigator'})
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                          'use_lifecycle_manager': 'false',
                          'slam_params_file': LaunchConfiguration('params_file')}.items())]


def generate_launch_description():
    share = Path(get_package_share_directory('maze_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/slam_params.yaml')),
        OpaqueFunction(function=setup),
    ])
