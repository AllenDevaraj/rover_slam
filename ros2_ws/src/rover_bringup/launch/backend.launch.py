import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _real():
    # Distro-robust equality (LaunchConfigurationEquals was removed in newer ROS 2 distros).
    # Returns a FRESH condition each call so it is not shared across actions.
    return IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'real'"]))


def _sim():
    return IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'sim'"]))


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory("rover_sim"), "worlds", "corridor.world")

    lidar_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rplidar_ros"), "launch", "view_mal_rplidar.launch.py"])),
        condition=_real(),
    )
    camera_real = Node(
        package="rover_camera", executable="ros_stream", output="screen",
        condition=_real(),
    )
    base_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rover_base"), "launch", "base.launch.py"])),
        condition=_real(),
    )
    # Static sensor TFs (were in mal_startup) -- REAL backend only.
    # In sim these frames come from robot_state_publisher (the URDF).
    tf_laser = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_link_to_laser_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
        output="screen", condition=_real(),
    )
    tf_imu = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_link_to_imu_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        output="screen", condition=_real(),
    )

    # SIM backend: Gazebo Fortress source layer (rover_sim/spawn.launch.py) --
    # produces /scan,/imu,/camera and moves the robot on /cmd_vel, exactly the
    # same topic contract as 'real'. Selected by backend:=sim.
    sim_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rover_sim"), "launch", "spawn.launch.py"])),
        launch_arguments={"drive": LaunchConfiguration("drive"),
                          "world": LaunchConfiguration("world")}.items(),
        condition=_sim(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="real",
                              choices=["real", "sim", "bag"],
                              description="Sensing/actuation source"),
        DeclareLaunchArgument("drive", default_value="diff",
                              choices=["diff", "ackermann"],
                              description="Sim drive model (used when backend:=sim)"),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Sim world file (used when backend:=sim)"),
        tf_laser, tf_imu,
        lidar_real, camera_real, base_real,
        sim_backend,
        # NOTE: 'real' reproduces mal_startup's sources (rplidar + rover_camera +
        # rover_base + the two static TFs). 'sim' brings up Gazebo Fortress via
        # rover_sim honoring the SAME /scan,/odom,/imu contract. 'bag' (ros2 bag
        # play) remains a stub.
    ])
