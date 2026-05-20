"""Top-level rover bringup (behavior-equivalent to the old mal_startup.launch.py).

Composition:
  backend.launch.py  -> sensing/actuation sources + static sensor TFs (real|sim|bag)
  ekf_node           -> fuses inputs into /odom (robot_localization)
  slam_toolbox       -> online async mapping (/map + map->odom)

This reproduces mal_startup's node graph with the sources behind a backend switch.
For full navigation (planner + pursuit + localization), use slam_nav.launch.py /
odom_nav.launch.py.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    ekf_config = os.path.join(
        get_package_share_directory("rover_state_estimation"), "config", "ekf.yaml")
    slam_params = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "config", "mapper_params_online_async.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="real",
                              choices=["real", "sim", "bag"],
                              description="Sensing/actuation source"),

        # Sensing/actuation sources + static sensor TFs (swappable backend)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("rover_bringup"), "launch", "backend.launch.py"])),
            launch_arguments={"backend": backend}.items(),
        ),

        # EKF fusion -> /odom (was mal_startup's robot_localization node)
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen", parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom")],
        ),

        # slam_toolbox online async mapping -> /map + map->odom
        Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[slam_params, {
                "scan_topic": "/scan",
                "base_frame": "base_link",
                "odom_frame": "odom",
                "map_frame": "map",
                "provide_odom_frame": False,
            }],
        ),
    ])
