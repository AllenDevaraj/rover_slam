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


def generate_launch_description():
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
    # Static sensor TFs (were in mal_startup) — needed by mapping in every backend.
    tf_laser = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_link_to_laser_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
        output="screen",
    )
    tf_imu = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_link_to_imu_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="real",
                              choices=["real", "sim", "bag"],
                              description="Sensing/actuation source"),
        tf_laser, tf_imu,
        lidar_real, camera_real, base_real,
        # NOTE: only the 'real' backend is wired -- it reproduces mal_startup's sources
        # exactly (rplidar + rover_camera + rover_base + the two static TFs). 'sim'
        # (Gazebo via rover_description's gazebo_control.xacro) and 'bag' (ros2 bag play)
        # honor the SAME /scan,/odom,/imu contract and can be added later WITHOUT touching
        # any consumer. Wiring them is new functionality, intentionally out of scope here.
    ])
