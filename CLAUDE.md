# Advanced_Robo_Project — `rover_*` SLAM stack

ROS 2 (Humble) SLAM mobile rover, organized into a modular `rover_*` architecture
modeled on `vla_SO-ARM101`. Design + plan:
- `docs/superpowers/specs/2026-05-20-rover-modular-architecture-design.md`
- `docs/superpowers/plans/2026-05-20-rover-modular-architecture.md`

## Workspace layout (`ros2_ws/src/`)

| Package | Build | Responsibility |
|---|---|---|
| `rover_description` | cmake | URDF/xacro model + `robot_state_publisher` (dependency root) |
| `rover_base` | python | ArduPilot base: consumes `/cmd_vel`, publishes `/odom` + raw `/imu` |
| `rover_camera` | python | RealSense color stream publisher (`/camera/color/image_raw`) |
| `rover_state_estimation` | python | rf2o + robot_localization EKF fusion → `/odom` |
| `rover_slam` | python | Live `slam_toolbox` mapping → `/map` + `map→odom` |
| `rover_localization` | python | amcl + map_server localization on a saved map + `save_map` + `maps/` |
| `rover_navigation` | python | A* planner + pure pursuit (`/goal_pose`→`/plan`→`/cmd_vel`) |
| `rover_teleop` | python | Keyboard / joystick → `/cmd_vel` |
| `rover_behaviors` | python | Reactive autonomy → `/cmd_vel` (line, color, wall-stop, wall-follow) |
| `rover_gui` | python | PyQt operator GUI |
| `rover_terminal_viz` | cmake | Terminal-based visualizer (node: `terminal_rviz_node`) |
| `rover_tools` | python | Dev / calibration / debug utilities (not runtime) |
| `rover_bringup` | python | Top-level layered launch + `backend:=real\|sim\|bag` switch |
| `vendor/` | — | Third-party: `slam_toolbox`, `rplidar_ros`, `rf2o_laser_odometry`, `autonomous-robot` |

The dependency graph is acyclic and rooted on `rover_description`; the leaf
packages (`rover_base`, `rover_camera`, `rover_teleop`, `rover_behaviors`,
`rover_gui`, `rover_terminal_viz`, `rover_tools`) couple to the rest only through
ROS topics at runtime.

## Build

```bash
cd ros2_ws
colcon build --symlink-install        # --symlink-install is required so save_map
source install/setup.bash             # persists maps into rover_localization/maps/
```

**System build dependencies for `slam_toolbox`** (needed for a clean build; the
runtime libs may be present but the dev headers are required to compile):

```bash
sudo apt install -y libsuitesparse-dev libceres-dev ros-humble-libg2o
```

**System runtime dependencies** (the SLAM/nav core needs these — install if missing):

```bash
sudo apt install -y ros-humble-robot-localization \
  ros-humble-nav2-map-server ros-humble-nav2-amcl ros-humble-nav2-lifecycle-manager
```

**Simulation (Gazebo Fortress) dependencies** (for `rover_sim` / `backend:=sim`):

```bash
sudo apt install -y ros-humble-ros-gz ros-humble-gz-ros2-control   # Fortress (ign gazebo 6) + ROS bridge
```

**Pip-only runtime deps** (no rosdep keys, install with pip as needed):
`pyrealsense2` (rover_camera / rover_tools), `pymavlink` (rover_base),
`simple_pid` (rover_behaviors).

If a vendored C++ package fails with "source directory does not exist" after a
move, clear its stale build cache: `rm -rf build/<pkg> install/<pkg>` then rebuild
(a full `rm -rf build install log` rebuild avoids this entirely).

## Run

```bash
ros2 launch rover_bringup rover.launch.py backend:=real   # camera+lidar+base+EKF+slam
ros2 launch rover_bringup slam_nav.launch.py mode:=mapping # full SLAM + navigation
ros2 launch rover_bringup odom_nav.launch.py              # navigation on odom
ros2 launch rover_localization localization.launch.py map:=<path/to/map.yaml>
```

### Simulation (Gazebo Fortress — no hardware)

```bash
# Mapping: drive in sim and build a map (camera line-follower on blue tape, or teleop)
ros2 launch rover_sim sim.launch.py mode:=mapping  drive:=diff driver:=line_follower
# Planning + execution: load a saved map, set an RViz "2D Goal Pose" -> A* -> pure pursuit
ros2 launch rover_sim sim.launch.py mode:=planning drive:=diff map:=<path/to/map.yaml>
#   drive:=diff|ackermann   world:=<file>   driver:=line_follower|teleop|none
#   headless:=true (gz server only, CI / no display)   x:=/y:=/yaw:= spawn pose
# Generate a Gazebo world from any saved slam map (co-registered for localization):
ros2 run rover_sim map2world <map.yaml> <out.world>
```

The sim only produces `/scan`, `/imu`, `/camera/color/image_raw` and consumes
`/cmd_vel`; rf2o + EKF + slam_toolbox + A* + pure_pursuit run unchanged. Both a
DiffDrive and an AckermannSteering model are supported (`drive:=`). Worlds:
`corridor.world` (blue-tape loop) and `building.world` (generated from the real
`map_April_27_3_52` map). See `docs/superpowers/specs/2026-05-21-rover-sim-backend-design.md`.

## Backend contract (swappable)

`/scan`, `/odom`, `/imu`, `/camera/color/image_raw` are produced by the selected
backend (`real` | `sim` | `bag`); the SLAM / navigation / behavior core consumes
them and never depends on which backend is active. `real` (rplidar +
`rover_camera` + `rover_base`, exactly as the old `mal_startup`) and `sim`
(Gazebo Fortress via the `rover_sim` package — `gpu_lidar` + `imu` + `camera` +
DiffDrive/AckermannSteering) are both wired and honor the same topic contract;
`bag` (`ros2 bag play`) remains a stub.

## Notes / known issues (preserved, pre-existing)

- `rover_behaviors` registers 5 working executables (`line_follower`,
  `color_follower`, `stall_detector`, `wall_stop`, `wall_follow_pid`).
  `lap_monitor.py` (main() mis-indented inside the class) and `auto_lidar.py`
  (`lidar_callback` mis-indented inside `__init__`) are preserved as modules but
  **not** registered — both had pre-existing bugs and were never installed
  executables. Fix the indentation to enable them.
