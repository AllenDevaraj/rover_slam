cmake_minimum_required(VERSION 3.22)
project(tf2_web_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 

add_executable(map_pose_publisher src/map_pose_publisher.cpp)
ament_target_dependencies(map_pose_publisher
  rclcpp
  geometry_msgs
  tf2_ros
)


add_executable(camera_to_map_transformer src/camera_to_map_transformer.cpp)
ament_target_dependencies(camera_to_map_transformer
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  std_msgs
)

install(TARGETS 
  map_pose_publisher
  camera_to_map_transformer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()