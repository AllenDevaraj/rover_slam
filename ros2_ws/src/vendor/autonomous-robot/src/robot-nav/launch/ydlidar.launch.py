import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    pwm_node = Node(
        package='pwm_node',
        executable='pwm_node',  
        name='pwm_node',
        output='screen'
    )

    # launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('ydlidar_ros2_driver'),
    #             'launch/ydlidar_launch.py'))
    # )

    return LaunchDescription([
        pwm_node,
        # launch_include
    ])