from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('robot-nav'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            #parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_in','/cmd_vel_out'),
                        ('/cmd_vel_out','/ackermann_steering_controller/reference')]
         )
    
    twist_mux_params = os.path.join(get_package_share_directory("robot-nav"), "config", "twist_mux.yaml")
    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_params]
        )   


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        twist_stamper,
        twist_mux
    ])