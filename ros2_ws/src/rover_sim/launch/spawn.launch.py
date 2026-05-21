#!/usr/bin/env python3
"""backend:=sim source layer: Gazebo Fortress + robot_state_publisher + spawn +
ros_gz bridge + cmd_vel relay. Produces /scan,/imu,/camera/color/image_raw and
moves the robot on /cmd_vel.

Args:
  drive    : diff | ackermann
  world    : world file path (empty -> corridor.world)
  headless : true  -> run the gz server only (no GUI), for CI / no-display machines
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    rover_sim = get_package_share_directory('rover_sim')
    rover_desc = get_package_share_directory('rover_description')

    drive = LaunchConfiguration('drive').perform(context)
    world = LaunchConfiguration('world').perform(context) or \
        os.path.join(rover_sim, 'worlds', 'corridor.world')
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')

    # Help gz find world-referenced resources (worlds use inline geometry, but harmless).
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = \
        os.path.join(rover_sim, 'worlds') + os.pathsep + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    gz_args = world + ' -r' + (' -s --headless-rendering' if headless else '')

    urdf_xacro = os.path.join(rover_desc, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro, ' sim_mode:=true', ' drive_model:=', drive]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args}.items(),
    )
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.1'],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(rover_sim, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )
    relay = Node(
        package='rover_sim', executable='cmd_vel_relay', output='screen',
        parameters=[{'output_topic': '/model/rover/cmd_vel', 'use_sim_time': True}],
    )
    return [gz_sim, rsp, spawn, bridge, relay]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drive', default_value='diff', choices=['diff', 'ackermann']),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
