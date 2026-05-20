from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_dir = PathJoinSubstitution([FindPackageShare('rover_base'),'launch'])

    return LaunchDescription([
        Node(
            package='rover_behaviors',
            executable='line_follower',
            name='line_follower',
            output='screen',
        ),
        Node(
            package='rover_camera',
            executable='ros_stream',
            output='screen'
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([imu_dir, 'base.launch.py'])
        ),
        Node(
            package='rover_behaviors',
            executable='lap_monitor',
            name='loop_closure_detector',
            output='screen',
        ),
    ])
