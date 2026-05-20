import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rover_localization')
    amcl_yaml = os.path.join(pkg, 'config', 'amcl.yaml')
    default_map = os.path.join(pkg, 'maps', 'map.yaml')  # override with map:=<path>
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Saved map yaml to localize on'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[amcl_yaml, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['map_server', 'amcl']}],
        ),
    ])
