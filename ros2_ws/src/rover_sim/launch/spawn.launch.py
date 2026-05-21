#!/usr/bin/env python3
"""backend:=sim source layer: Gazebo Fortress + robot_state_publisher + spawn +
ros_gz bridge + cmd_vel relay. Produces /scan,/imu,/camera/color/image_raw and
moves the robot on /cmd_vel. Args: drive (diff|ackermann), world (file path)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rover_sim = get_package_share_directory('rover_sim')
    rover_desc = get_package_share_directory('rover_description')
    default_world = os.path.join(rover_sim, 'worlds', 'corridor.world')

    drive = LaunchConfiguration('drive')
    world = LaunchConfiguration('world')

    urdf_xacro = os.path.join(rover_desc, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro, ' sim_mode:=true', ' drive_model:=', drive]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [world, ' -r']}.items(),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
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

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                               os.path.join(rover_sim, 'worlds')),
        DeclareLaunchArgument('drive', default_value='diff', choices=['diff', 'ackermann']),
        DeclareLaunchArgument('world', default_value=default_world),
        gz_sim, rsp, spawn, bridge, relay,
    ])
