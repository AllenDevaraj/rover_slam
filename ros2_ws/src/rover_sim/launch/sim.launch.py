#!/usr/bin/env python3
"""Top-level sim entry — the sim analog of rover_bringup/slam_nav.launch.py.
Swaps the hardware layer for Gazebo and sets use_sim_time:=true everywhere.

  ros2 launch rover_sim sim.launch.py mode:=mapping  drive:=diff world:=<file> driver:=line_follower
  ros2 launch rover_sim sim.launch.py mode:=planning drive:=diff world:=<file> map:=<yaml>
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    rover_sim = get_package_share_directory('rover_sim')
    mode = LaunchConfiguration('mode').perform(context)
    driver = LaunchConfiguration('driver').perform(context)
    world = LaunchConfiguration('world').perform(context) or \
        os.path.join(rover_sim, 'worlds', 'corridor.world')
    drive = LaunchConfiguration('drive')

    sim_time = {'use_sim_time': True}
    ekf_config = os.path.join(get_package_share_directory('rover_state_estimation'),
                              'config', 'ekf.yaml')

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rover_sim'), 'launch', 'spawn.launch.py'])),
        launch_arguments={'drive': drive, 'world': world}.items(),
    )

    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='log', ros_arguments=['--log-level', 'FATAL'],
        parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/rf2o_odom',
                     'publish_tf': False, 'base_frame_id': 'base_link',
                     'odom_frame_id': 'odom', 'freq': 10.0, **sim_time}],
    )
    init_rf2o = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/base_pose_ground_truth',
             'nav_msgs/msg/Odometry',
             '{header: {frame_id: odom}, pose: {pose: {orientation: {w: 1.0}}}}'],
        output='log')])
    fake_odom = Node(
        package='rover_tools', executable='fake_odom', name='fake_odom', output='screen',
        parameters=[{'publish_tf': False, **sim_time}], remappings=[('odom', '/fake_odom')])
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_config, sim_time],
        remappings=[('odometry/filtered', '/odom')])

    nodes = [spawn, rf2o, init_rf2o, fake_odom, ekf]

    if mode == 'mapping':
        slam = Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                        'config', 'mapper_params_online_async.yaml']),
                        {'scan_topic': '/scan', 'base_frame': 'base_link',
                         'odom_frame': 'odom', 'map_frame': 'map',
                         'provide_odom_frame': False,
                         'minimum_travel_distance': 0.2,
                         'minimum_travel_heading': 0.2, **sim_time}])
        nodes.append(TimerAction(period=3.0, actions=[slam]))

    if mode == 'planning':
        map_yaml = LaunchConfiguration('map').perform(context)
        if not map_yaml:
            map_yaml = os.path.join(
                get_package_share_directory('rover_localization'),
                'maps', 'map_April_27_3_52.yaml')
        amcl_config = os.path.join(
            get_package_share_directory('rover_localization'), 'config', 'amcl.yaml')
        map_server = Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{'yaml_filename': map_yaml, **sim_time}])
        amcl = Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[amcl_config, sim_time])
        lifecycle = Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'autostart': True,
                         'node_names': ['map_server', 'amcl'], **sim_time}])
        planner = Node(package='rover_navigation', executable='astar_planner',
                       name='astar_planner', output='screen', parameters=[sim_time])
        pursuit = Node(package='rover_navigation', executable='pure_pursuit',
                       name='pure_pursuit', output='screen',
                       parameters=[{'use_sim': False, **sim_time}])
        nodes += [map_server, amcl, TimerAction(period=3.0, actions=[lifecycle]),
                  TimerAction(period=10.0, actions=[planner, pursuit])]

    if driver == 'line_follower':
        nodes.append(Node(package='rover_behaviors', executable='line_follower',
                          name='line_follower', output='screen', parameters=[sim_time]))
    elif driver == 'teleop':
        nodes.append(Node(package='rover_teleop', executable='keyboard_teleop',
                          name='rover_teleop', output='screen',
                          remappings=[('cmd_vel', '/cmd_vel_teleop')]))

    nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                      arguments=['-d', os.path.join(rover_sim, 'rviz', 'sim.rviz')],
                      parameters=[sim_time]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'planning']),
        DeclareLaunchArgument('drive', default_value='diff', choices=['diff', 'ackermann']),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('driver', default_value='line_follower',
                              choices=['line_follower', 'teleop', 'none']),
        DeclareLaunchArgument('map', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
