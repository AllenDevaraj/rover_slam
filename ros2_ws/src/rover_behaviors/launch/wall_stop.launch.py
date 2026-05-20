from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	return LaunchDescription([
		Node(
			package='rover_behaviors',
			executable='wall_stop',
			name='lidar_wall_stop',
			output='screen',
		),
		Node(
			package='rover_behaviors',
			executable='wall_follow_pid',
			name='pid_control_node',
			output='screen',
		),
	])
