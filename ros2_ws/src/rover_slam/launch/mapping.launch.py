from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Minimal mapping bringup for slam_toolbox:
    - LiDAR driver publishes /scan in frame 'laser'
    - Static TF publishes base_link -> laser (adjust xyz/rpy to your robot)
    - slam_toolbox provides localization + publishes odom/map TF as configured
    """

    lidar_dir = PathJoinSubstitution([FindPackageShare("rplidar_ros"), "launch"])
    slam_params = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "config", "mapper_params_online_async.yaml"]
    )

    static_base_to_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser_tf",
        # x y z yaw pitch roll frame child_frame
        # NOTE: replace 0 0 0 with measured offsets.
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
        output="screen",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {
                "scan_topic": "/scan",
                "base_frame": "base_link",
                "odom_frame": "odom",
                "map_frame": "map",
                # If you don't have wheel odom, let slam_toolbox provide the odom TF.
                "provide_odom_frame": True,
            },
        ],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PathJoinSubstitution([lidar_dir, "view_mal_rplidar.launch.py"])),
            static_base_to_laser_tf,
            slam_toolbox,
        ]
    )


