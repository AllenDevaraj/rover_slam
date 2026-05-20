cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)
# meshes/ is optional — this robot is built from primitive shapes, so it may not exist.
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
