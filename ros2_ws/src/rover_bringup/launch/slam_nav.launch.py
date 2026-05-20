#!/usr/bin/env python3
"""
Full SLAM + navigation launch.

Modes:
  mapping    — build a new map while driving (teleop or joystick)
  planning   — load a saved map, localize, and run A* + Pure Pursuit

Usage:
  ros2 launch rover_bringup slam_nav.launch.py mode:=mapping
  ros2 launch rover_bringup slam_nav.launch.py mode:=planning

  terminal_rviz (tviz) uses a full-screen TUI. If the UI still glitches inside
  ``ros2 launch`` (mixed with other nodes' logs), run without it and start TViz
  in a second terminal::

    ros2 launch rover_bringup slam_nav.launch.py mode:=mapping tviz:=false
    ros2 run terminal_rviz terminal_rviz_node --ros-args -p config_file:=<path>

  where ``<path>`` is ``share/rover_bringup/config/config.nathan`` under your
  ``rover_bringup`` install prefix (``ros2 pkg prefix rover_bringup``).
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
    mode = LaunchConfiguration('mode').perform(context)
    use_tviz = LaunchConfiguration('tviz').perform(context).lower() in (
        'true', '1', 'yes')
# nothing
    lidar_dir = PathJoinSubstitution([FindPackageShare('rplidar_ros'), 'launch'])
    ekf_config = os.path.join(
        get_package_share_directory('rover_state_estimation'), 'config', 'ekf.yaml')
    tviz_config = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'config.nathan')
    amcl_config = os.path.join(
        get_package_share_directory('rover_localization'), 'config', 'amcl.yaml')

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen',
    )

    # IMU tilt correction derived from accel_bias in imu_calibration.yaml:
    #   pitch =  arctan(0.0552 / 9.81) =  0.00563 rad
    #   roll  = -arctan(0.0349 / 9.81) = -0.00356 rad
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_tf',
        arguments=['0', '0', '0', '-0.00356', '0.00563', '0', 'base_link', 'imu_link'],
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

    # RF2O blocks until it receives an init pose on /base_pose_ground_truth.
    # Publish a one-shot zero pose a few seconds after launch so it starts.
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

    fake_odom_node = Node(
        package='rover_tools',
        executable='fake_odom',
        name='fake_odom',
        output='screen',
        parameters=[{'publish_tf': False}],
        remappings=[('odom', '/fake_odom')],
    )

    realsense_node = Node(
        package='rover_camera',
        executable='ros_stream',
        name='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'pointcloud.enable': False,
        }],
    )

    # Common nodes for both modes
    nodes += [
        IncludeLaunchDescription(
            PathJoinSubstitution([lidar_dir, 'view_mal_rplidar.launch.py'])
        ),
        static_tf_laser,
        static_tf_imu,
        rf2o_node,
        init_rf2o_pose,
        fake_odom_node,
        EKF_odom,
        rover_node,
        #realsense_node,
    ]

    if mode == 'mapping':
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'), 'config',
                    'mapper_params_online_async.yaml'
                ]),
                {
                    'scan_topic': '/scan',
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'provide_odom_frame': False,
                    'minimum_travel_distance': 0.2,
                    'minimum_travel_heading': 0.2,
                },
            ],
        )
        nodes.append(TimerAction(period=1.0, actions=[slam_node]))

    else:  # planning
        map_yaml = LaunchConfiguration('map').perform(context)

        # Load saved pose if it exists next to the map YAML, else default to origin.
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
            print(f'[slam_nav] Seeding AMCL from {pose_file}: '
                  f"x={initial_pose['x']:.3f} y={initial_pose['y']:.3f} "
                  f"yaw={initial_pose['yaw']:.3f} rad")
        else:
            initial_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
            print('[slam_nav] No pose file found — AMCL initializing at (0, 0, 0)')

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

        # Delay lifecycle manager so LiDAR + EKF TF are publishing before AMCL activates
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
            description='Full path to map YAML (planning mode). '
                        'A matching .pose.yaml seeds AMCL initial_pose.',
        ),
        DeclareLaunchArgument(
            'tviz',
            default_value='true',
            description='If true, start terminal_rviz in this launch (use tviz:=false '
            'and run terminal_rviz in a second terminal if the UI glitches).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
