#!/usr/bin/env python3
"""
Odometry-only navigation — no AMCL.

Loads a saved map for A* planning but fixes map->odom as a static identity
transform. All localization comes from EKF (rf2o + fake_odom + IMU).
The robot must start at the map origin.

Usage:
  ros2 launch rover_bringup odom_nav.launch.py
  ros2 launch rover_bringup odom_nav.launch.py map:=/full/path/to/map.yaml
  ros2 launch rover_bringup odom_nav.launch.py tviz:=false
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    use_tviz = LaunchConfiguration('tviz').perform(context).lower() in ('true', '1', 'yes')
    map_yaml = LaunchConfiguration('map').perform(context)

    lidar_dir = PathJoinSubstitution([FindPackageShare('rplidar_ros'), 'launch'])
    ekf_config = os.path.join(
        get_package_share_directory('rover_state_estimation'), 'config', 'ekf.yaml')
    tviz_config = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'config.nathan')

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_tf',
        arguments=['0', '0', '0', '-0.00356', '0.00563', '0', 'base_link', 'imu_link'],
        output='screen',
    )

    # Fixed map->odom: robot must start at the map origin.
    # EKF owns odom->base_link; no AMCL corrections applied.
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    EKF_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odom')],
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        ros_arguments=['--log-level', 'FATAL'],
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/rf2o_odom',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'freq': 10.0,
        }],
    )

    init_rf2o_pose = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/base_pose_ground_truth',
                    'nav_msgs/msg/Odometry',
                    '{header: {frame_id: odom}, pose: {pose: {orientation: {w: 1.0}}}}',
                ],
                output='log',
            ),
        ],
    )

    rover_node = Node(
        package='rover_base',
        executable='rover_node',
        arguments=['-m', 'rover_base.rover_node'],
        name='rover_node',
        output='screen',
        parameters=[{
            'connection_string': '/dev/ttyACM1',
            'baud_rate': 115200,
            'control_frequency': 20.0,
        }],
    )

    fake_odom_node = Node(
        package='rover_tools',
        executable='fake_odom',
        name='fake_odom',
        output='screen',
        parameters=[{'publish_tf': False}],
        remappings=[('odom', '/fake_odom')],
    )

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

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    planner_nodes = TimerAction(
        period=5.0,
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

    nodes = []

    if use_tviz:
        nodes.append(
            Node(
                package='rover_terminal_viz',
                executable='terminal_rviz_node',
                name='terminal_rviz',
                output='screen',
                emulate_tty=True,
                output_format='{line}',
                parameters=[{'config_file': tviz_config}],
            ),
        )

    nodes += [
        IncludeLaunchDescription(
            PathJoinSubstitution([lidar_dir, 'view_mal_rplidar.launch.py'])
        ),
        static_tf_laser,
        static_tf_imu,
        static_tf_map_odom,
        rf2o_node,
        init_rf2o_pose,
        fake_odom_node,
        EKF_odom,
        rover_node,
        map_server,
        TimerAction(period=3.0, actions=[lifecycle_manager]),
        planner_nodes,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=_default_map_yaml(),
            description='Full path to map YAML for A* planning.',
        ),
        DeclareLaunchArgument(
            'tviz',
            default_value='true',
            description='If true, start terminal_rviz in this launch.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
