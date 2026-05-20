from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lidar_dir = PathJoinSubstitution([FindPackageShare('rplidar_ros'), 'launch'])
    imu_dir = PathJoinSubstitution([FindPackageShare('rover_base'),'launch'])
    return LaunchDescription([
        Node(
            package='rover_camera',
            executable='ros_stream',
            output='screen'
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([imu_dir, 'base.launch.py'])
        ),
    ])
