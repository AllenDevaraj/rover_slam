from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/the2xman/Downloads/2nd_lidar_bag/2nd_lidar_great',
        description='Path to the rosbag directory to play'
    )

    config_path = os.path.join(
        get_package_share_directory('rover_behaviors'),
        'config',
        'tviz_config.nathan'
    )

    # Play rosbag
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path'), '--loop'],
        output='screen'
    )

    # Color follower node
    color_follower = Node(
        package='rover_behaviors',
        executable='color_follower',
        name='color_follower',
        output='screen',
    )

    # Terminal RViz node
    terminal_rviz = Node(
        package='rover_terminal_viz',
        executable='terminal_rviz_node',
        name='terminal_rviz',
        output='screen',
    )

    return LaunchDescription([
        bag_path_arg,
        rosbag_play,
        color_follower,
        terminal_rviz,
    ])
