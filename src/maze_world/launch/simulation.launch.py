"""One corrected maze, one Burger, one bridge, and simulation-time TF."""
import os
from pathlib import Path
import runpy
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('maze_world'))
    runpy.run_path(str(share / 'launch/launch_support.py'))['guard'](
        'simulation', {'robot_state_publisher', 'ros_gz_bridge'})
    services = subprocess.run(['gz', 'service', '-l'], capture_output=True, text=True,
                              timeout=8, check=False)
    if '/world/maze/create' in services.stdout:
        raise RuntimeError('A maze Gazebo server already exists in this GZ_PARTITION.')
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    command = ['gz', 'sim', '-s', '-r', '-v', '3']
    if headless:
        command += ['--headless-rendering']
    command += [str(share / 'worlds/maze_fixed.sdf')]
    server = ExecuteProcess(cmd=command, output='screen')
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(tb3 / 'launch/spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '-0.75', 'y_pose': '-1.25'}.items())
    # ros_gz_sim create waits for the world's create service; no fixed startup sleep.
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        server, spawn,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True,
                          'robot_description': (tb3 / 'urdf/turtlebot3_burger.urdf').read_text(),
                          'frame_prefix': ''}]),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '3'], output='screen',
                       condition=IfCondition(LaunchConfiguration('gui'))),
    ]


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='EGL rendering for a server without DISPLAY'),
        DeclareLaunchArgument('partition',
                              default_value=os.environ.get('GZ_PARTITION', 'robotics_test')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('partition')),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               tb3 + '/models:' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        OpaqueFunction(function=setup),
    ])
