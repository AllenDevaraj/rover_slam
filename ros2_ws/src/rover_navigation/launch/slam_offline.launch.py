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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_MAPS_DIR = os.path.join(get_package_share_directory('rover_localization'), 'maps')


def launch_setup(context, *args, **kwargs):
    nodes = []
    mode = LaunchConfiguration('mode').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    # slam_toolbox expects the path without extension
    map_file_name = map_yaml[:-5] if map_yaml.endswith('.yaml') else map_yaml

    slam_config = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'), 'config',
        'mapper_params_online_async.yaml' if mode == 'mapping'
        else 'mapper_params_localization.yaml'
    ])

    rviz_config = os.path.join(
        get_package_share_directory('rover_navigation'), 'rviz', 'slam_offline.rviz'
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    fake_odom_node = Node(
        package='rover_tools',
        executable='fake_odom',
        name='fake_odom',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    slam_params = {
        'scan_topic': '/scan',
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'provide_odom_frame': False,
        'minimum_travel_distance': 0.2,
        'minimum_travel_heading': 0.2,
        'use_sim_time': True,
    }
    if mode != 'mapping':
        slam_params['map_file_name'] = map_file_name

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, slam_params],
    )

    nodes += [
        static_tf_laser,
        fake_odom_node,
        rviz_node,

        # slam_toolbox delayed 3s to ensure /scan and TFs are ready
        TimerAction(period=3.0, actions=[slam_node]),
    ]

    # if mode == 'planning':
    #     nodes += [
    #         Node(
    #             package='rover_navigation',
    #             executable='astar_planner',
    #             name='astar_planner',
    #             output='screen',
    #         ),
    #         Node(
    #             package='rover_navigation',
    #             executable='pure_pursuit',
    #             name='pure_pursuit',
    #             output='screen',
    #         ),
    #     ]

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
            default_value=os.path.join(_MAPS_DIR, 'map_April_27_3_52.yaml'),
            description='Full path to map YAML (localization mode only).',
        ),
        DeclareLaunchArgument(
            'tviz',
            default_value='true',
            description='If true, start terminal_rviz in this launch (use tviz:=false '
            'and run terminal_rviz in a second terminal if the UI glitches).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
