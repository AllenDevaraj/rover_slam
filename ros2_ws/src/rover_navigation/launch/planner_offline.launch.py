#!/usr/bin/env python3
"""
Full SLAM + navigation launch.

Modes:
  mapping    — build a new map while driving (teleop or joystick)
  planning   — load a saved map, localize, and run A* + Pure Pursuit

Usage:
  ros2 launch rover_navigation slam_nav.launch.py mode:=mapping
  ros2 launch rover_navigation slam_nav.launch.py mode:=planning

  terminal_rviz (tviz) uses a full-screen TUI. If the UI still glitches inside
  ``ros2 launch`` (mixed with other nodes' logs), run without it and start TViz
  in a second terminal::

    ros2 launch rover_navigation slam_nav.launch.py mode:=mapping tviz:=false
    ros2 run terminal_rviz terminal_rviz_node --ros-args -p config_file:=<path>

  where ``<path>`` is ``share/rover_navigation/config/config.nathan`` under your
  ``rover_navigation`` install prefix (``ros2 pkg prefix rover_navigation``).
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _default_map_yaml():
    maps_dir = os.path.join(get_package_share_directory('rover_localization'), 'maps')
    last_file = os.path.join(maps_dir, 'last_map')
    if os.path.isfile(last_file):
        with open(last_file) as f:
            name = f.read().strip()
        candidate = os.path.join(maps_dir, name + '.yaml')
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(maps_dir, 'map_April_27_3_52.yaml')


def launch_setup(context, *args, **kwargs):

    amcl_config = os.path.join(
        get_package_share_directory('rover_localization'), 'config', 'amcl.yaml')

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
    )


    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )

    nodes = []

    nodes += [
        static_tf_laser,
        static_tf_map_odom,
        static_tf_odom_base,
    ]

    map_yaml = LaunchConfiguration('map').perform(context)

    pose_file = map_yaml.replace('.yaml', '.pose.yaml')
    if os.path.exists(pose_file):
        with open(pose_file) as f:
            _p = yaml.safe_load(f)
        initial_pose = {
            'x':   float(_p.get('x',   0.0)),
            'y':   float(_p.get('y',   0.0)),
            'z':   0.0,
            'yaw': float(_p.get('yaw', 0.0)),
        }
        print(f'[planner_offline] Seeding AMCL from {pose_file}: '
              f"x={initial_pose['x']:.3f} y={initial_pose['y']:.3f} "
              f"yaw={initial_pose['yaw']:.3f} rad")
    else:
        initial_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        print('[planner_offline] No pose file found — AMCL initializing at (0, 0, 0)')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False,
        }],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config, {'initial_pose': initial_pose}],
    )

    # Lifecycle manager activates map_server and amcl automatically
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # Planner + controller start after AMCL has time to localize
    planner_nodes = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rover_navigation',
                executable='astar_planner',
                name='astar_planner',
                output='screen',
            ),
            Node(
                package='rover_navigation',
                executable='pure_pursuit',
                name='pure_pursuit',
                output='screen',
            ),
        ],
    )

    # Delay lifecycle manager so LiDAR is publishing before AMCL activates
    nodes += [
        map_server,
        amcl_node,
        TimerAction(period=3.0, actions=[lifecycle_manager]),
        planner_nodes,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            description='mapping or planning',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=_default_map_yaml(),
            description='Full path to map YAML. A matching .pose.yaml seeds AMCL initial_pose.',
        ),
        DeclareLaunchArgument(
            'tviz',
            default_value='true',
            description='If true, start terminal_rviz in this launch (use tviz:=false '
            'and run terminal_rviz in a second terminal if the UI glitches).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
