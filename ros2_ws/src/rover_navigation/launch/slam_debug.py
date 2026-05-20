from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # lidar_dir = PathJoinSubstitution([FindPackageShare('rplidar_ros'), 'launch'])
    # imu_dir = PathJoinSubstitution([FindPackageShare('rover_base'),'launch'])
    ekf_config = os.path.join(
            get_package_share_directory('rover_state_estimation'),
                    'config',
                    'ekf.yaml'
                )
    EKF_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False}
        ],
        remappings=[('odometry/filtered', '/odom')]
    )
    fake_odom_node = Node(
        package='rover_tools',
        executable='fake_odom',
        name='fake_odom',
        output='screen',
        parameters=[{'velocity_scale': 1.12}],
        remappings=[('/odom', 'fake_odom/raw')],
    )
    return LaunchDescription([
        fake_odom_node,
        EKF_odom,
    ])
