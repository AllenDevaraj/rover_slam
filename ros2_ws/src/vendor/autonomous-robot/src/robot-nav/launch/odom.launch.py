import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription,  TimerAction

def generate_launch_description():
    package_name='robot-nav'
    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch/ydlidar.launch.py'))
    )

    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch/launch_robot_ackermann.launch.py'))
    )

    launch_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch/imu.launch.py'))
    )


    return LaunchDescription([
        launch_lidar,
        TimerAction(
            period=5.0,
            actions=[launch_robot],
        ),
        TimerAction(
            period=15.0,
            actions=[launch_imu],
        ),
    ])