from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_behaviors',
            executable='color_follower',
            name='color_follower',
            output='screen',
        ),
    ])
