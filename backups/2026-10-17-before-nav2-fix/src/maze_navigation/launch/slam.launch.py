
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory("maze_navigation"),
        "config",
        "slam_params.yaml"
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            config_path,
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        slam_node
    ])
