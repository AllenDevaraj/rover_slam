from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # lidar_dir = PathJoinSubstitution([FindPackageShare('rplidar_ros'), 'launch'])
    # imu_dir = PathJoinSubstitution([FindPackageShare('rover_base'),'launch'])
    return LaunchDescription([
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
    ])
