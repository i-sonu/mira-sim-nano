"""Launch the headless (or GUI) PyBullet AUV simulator.

  ros2 launch mira_pybullet_sim sim.launch.py            # headless (default)
  ros2 launch mira_pybullet_sim sim.launch.py gui:=true  # local GUI debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("mira_pybullet_sim")
    default_params = os.path.join(pkg, "config", "sim_params.yaml")

    gui_arg = DeclareLaunchArgument(
        "gui", default_value="false",
        description="Run PyBullet in GUI mode (local debug only). Default headless.")
    params_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Path to the YAML parameter file.")

    sim_node = Node(
        package="mira_pybullet_sim",
        executable="sim_node",
        name="mira_pybullet_sim",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"gui": LaunchConfiguration("gui")},
        ],
    )

    return LaunchDescription([gui_arg, params_arg, sim_node])
