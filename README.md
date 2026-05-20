# Advanced_Robo_Project

ROS 2 (Humble) SLAM mobile-rover stack, organized into modular `rover_*` packages.
See **`CLAUDE.md`** for the package map, build instructions, and the backend
contract, and `docs/superpowers/specs/` for the architecture rationale.

## Hardware setup

- **Camera** (RealSense on RPi 4): https://github.com/scottnon/robo_realsense
  — TODO: set up udev rules.
- **LiDAR** (RPLIDAR A1): https://github.com/Slamtec/rplidar_ros/tree/ros2
  — TODO: udev rules for rplidar.
- **IMU**: https://github.com/Ian-McConachie-CU/ROBO_rover/tree/main/robo_rover
  — TODO: serial-access permissions.

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

System deps for `slam_toolbox`: `sudo apt install -y libsuitesparse-dev libceres-dev ros-humble-libg2o`.

## Run

```bash
# Top-level bringup (camera + lidar + base + EKF + slam), backend switchable
ros2 launch rover_bringup rover.launch.py backend:=real

# Full SLAM + navigation
ros2 launch rover_bringup slam_nav.launch.py mode:=mapping

# Color follower (behavior)
ros2 launch rover_behaviors color_follower.launch.py
ros2 run rover_behaviors color_follower

# Bag playback
ros2 bag play <rosbag directory>
```

## Notes

- Stall detection (HW4): `rover_behaviors stall_detector` watches `/imu` for a
  stopped state and bumps velocities, returning to nominal after the stall clears.
- SLAM reference: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html
