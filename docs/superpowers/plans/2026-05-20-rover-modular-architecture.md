# Modular `rover_*` Architecture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-cut the `ros2_ws` SLAM-rover workspace into 13 single-responsibility `rover_*` packages plus a `src/vendor/` area, preserving on-wire behavior, mirroring `vla_SO-ARM101`'s modular architecture.

**Architecture:** Acyclic package graph rooted on a first-party `rover_description` (URDF). Leaf data-source/consumer packages couple only through ROS topics. A top-level `rover_bringup` composes everything via layered `IncludeLaunchDescription` and a `backend:=real|sim|bag` switch. Third-party code is isolated under `src/vendor/`.

**Tech Stack:** ROS 2 (rclpy / ament_python / ament_cmake), colcon, slam_toolbox, rf2o_laser_odometry, robot_localization, rplidar_ros, Nav2 amcl/map_server, PyQt5.

**Spec:** `docs/superpowers/specs/2026-05-20-rover-modular-architecture-design.md`

---

## Conventions (read once, applies to every task)

**Workspace root:** `/home/the2xman/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws`
All `colcon`/`ros2` commands run from there unless stated. Branch: `modularizing`.

**This is a repackaging effort, not feature work — so the per-task "test" is not pytest.** The verification gate for every task is:
1. `colcon build` (whole or `--packages-select <pkg>`) exits 0.
2. Migrated executables resolve: `ros2 pkg executables <pkg>` lists the expected entry points.
3. Migrated launch files load without error (see "launch check" below).
4. ROS **node names and topics inside each migrated node are unchanged** (we rename files/packages/entry points, never the `super().__init__('node_name')` string or topic names).

**Build with symlink-install** (required so `rover_localization`'s `save_map` persists maps to source and so iterative builds are fast):
```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Launch check (hardware-free).** A launch file that uses `get_package_share_directory(...)` raises `PackageNotFoundError` immediately if a package name is wrong, so this surfaces rename mistakes without a robot:
```bash
ros2 launch <pkg> <file>.launch.py --show-args
# Expected: prints "Arguments (pass arguments as '<name>:=<value>'):" and exits 0,
# OR (if no declared args) begins launching nodes — Ctrl-C is fine; a PackageNotFoundError / ModuleNotFoundError is a FAILURE.
```

**New-package manifest standard** (every new `rover_*` package uses these field values):
- `package.xml`: `<version>0.0.1</version>`, `<license>Apache-2.0</license>`,
  `<maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>`,
  format 3.
- `setup.py` (ament_python): `maintainer='The2xMan'`, `maintainer_email='allendevaraj33333@gmail.com'`, `license='Apache-2.0'`.
- pip-only deps (`pyrealsense2`, `pymavlink`, `simple_pid`) are **not** declared as
  `<exec_depend>` (no standard rosdep key); they are noted in each package's README
  comment. Declared rosdep keys we DO use: `rclpy`, `sensor_msgs`, `geometry_msgs`,
  `nav_msgs`, `std_msgs`, `rcl_interfaces`, `tf2_ros`, `tf2_geometry_msgs`,
  `cv_bridge`, `python3-numpy`, `python3-opencv`, `python3-pyqt5`, `python3-pyqtgraph`,
  `robot_localization`, `slam_toolbox`, `rf2o_laser_odometry`, `nav2_amcl`,
  `nav2_map_server`, `robot_state_publisher`, `joint_state_publisher`,
  `xacro`, `rviz2`, `tf2_ros`.

**Package-name remap table (AUTHORITATIVE — used whenever editing a launch/yaml/code reference).**
References to *vendored* packages (`rplidar_ros`, `slam_toolbox`, `rf2o_laser_odometry`, `robot_localization`, `nav2_*`, `tf2_ros`) resolve by name and are **left unchanged** by the vendor move.

| Old reference | New reference (depends on WHAT is referenced) |
|---|---|
| `robo_realsense` (`ros_stream`) | `rover_camera` |
| `robo_teleop` | `rover_teleop` |
| `my_gui_pkg` | `rover_gui` |
| `terminal_rviz` | `rover_terminal_viz` |
| `line_follower` / `color_follower` / `mal_hw3_pkg` (any node) | `rover_behaviors` |
| `robo_rover` → `rover_node`, `velocity_controller`, `rover_launch.py` | `rover_base` |
| `robo_rover` → `imu_calib_values` + `calibration_results/` | `rover_tools` (tool) / `rover_base` (data) |
| `robo_rover` → `auto_lidar_node` | `rover_behaviors` |
| `main_mal_launch` → `ekf.yaml` | `rover_state_estimation` |
| `main_mal_launch` → top launch files | `rover_bringup` |
| `mal_planner` → `astar_planner`, `pure_pursuit`, `goal_pub` | `rover_navigation` |
| `mal_planner` → `save_map`, `amcl.yaml`, `maps/` | `rover_localization` |
| `mal_planner` → `fake_odom`, `calibrate_velocity`, `debug_vel` | `rover_tools` |
| `mal_planner` → slam mapping launch + slam params | `rover_slam` |
| `mal_planner` → `slam_nav`/`odom_nav` (composed) launches | `rover_bringup` |
| `mal_planner` → `planner_offline`/`slam_offline`/`slam_planning`/`slam_debug` | `rover_navigation` |

---

## Task 0: Baseline & safety net

**Files:** none (records only).

- [ ] **Step 1: Confirm branch and clean tree**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project
git rev-parse --abbrev-ref HEAD   # Expected: modularizing
git status --short                # Expected: empty (docs/ already committed)
```

- [ ] **Step 2: Baseline build (record the starting state)**

```bash
cd ros2_ws
colcon build --symlink-install 2>&1 | tee /tmp/baseline_build.log
echo "exit=$?"
```
Expected: note the exit code and which packages currently fail (the goal is to not make
things *worse*; pre-existing failures are documented, not necessarily fixed).

- [ ] **Step 3: Record the baseline runtime surface**

```bash
source install/setup.bash
ros2 pkg executables 2>/dev/null | grep -E "mal_planner|robo_rover|robo_realsense|robo_teleop|line_follower|color_follower|mal_hw3_pkg|my_gui_pkg|terminal_rviz" | sort | tee /tmp/baseline_executables.txt
# Static node-graph snapshot of the top-level bringup (set of package/executable Node() decls):
grep -rEn "package=|executable=" src/main_mal_launch/launch/mal_startup.launch.py | tee /tmp/baseline_mal_startup_nodes.txt
```
Expected: `/tmp/baseline_executables.txt` and `/tmp/baseline_mal_startup_nodes.txt` exist
and are non-empty. These are the comparison targets for Task 13/14.

- [ ] **Step 4: Commit the baseline note**

```bash
mkdir -p docs/superpowers/notes
cp /tmp/baseline_executables.txt docs/superpowers/notes/baseline-executables.txt
cp /tmp/baseline_mal_startup_nodes.txt docs/superpowers/notes/baseline-mal_startup-nodes.txt
git add docs/superpowers/notes/
git commit -m "chore: record pre-migration baseline (executables + bringup node graph)"
```

---

## Task 1: Vendor isolation — move third-party packages to `src/vendor/`

**Files:**
- Move: `ros2_ws/src/{slam_toolbox,rplidar_ros,rf2o_laser_odometry,autonomous-robot}` → `ros2_ws/src/vendor/`
- Modify: `.gitmodules` (the `autonomous-robot` submodule path)

- [ ] **Step 1: Create the vendor dir and move the three plain packages**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p vendor
git mv slam_toolbox vendor/slam_toolbox
git mv rplidar_ros vendor/rplidar_ros
git mv rf2o_laser_odometry vendor/rf2o_laser_odometry
```

- [ ] **Step 2: Move the submodule and fix `.gitmodules`**

`autonomous-robot` is a git submodule, so its path is tracked in `.gitmodules`.

```bash
git mv autonomous-robot vendor/autonomous-robot
# Verify .gitmodules now points at the new path:
grep -n "autonomous-robot" ../../.gitmodules
```
If the `path =` line still reads `ros2_ws/src/autonomous-robot` (old), edit it to
`ros2_ws/src/vendor/autonomous-robot` so the submodule resolves:
```bash
sed -i 's#ros2_ws/src/autonomous-robot#ros2_ws/src/vendor/autonomous-robot#' ../../.gitmodules
git submodule sync
git add ../../.gitmodules
```
Expected: `git submodule status` shows `vendor/autonomous-robot` with a commit hash, no errors.

- [ ] **Step 3: Rebuild — colcon scans `src/` recursively, so vendor packages still build**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install 2>&1 | tail -20
source install/setup.bash
ros2 pkg prefix slam_toolbox && ros2 pkg prefix rplidar_ros && ros2 pkg prefix rf2o_laser_odometry
```
Expected: build exit 0 (same set of packages as baseline), and the three `ros2 pkg prefix`
calls all print an install path (proves the move didn't lose them).

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "refactor: isolate third-party packages under src/vendor/"
```

---

## Task 2: `rover_description` — the first-party URDF root (ament_cmake)

**Files:**
- Create: `ros2_ws/src/rover_description/package.xml`
- Create: `ros2_ws/src/rover_description/CMakeLists.txt`
- Create: `ros2_ws/src/rover_description/launch/rsp.launch.py`
- Copy in: `ros2_ws/src/rover_description/{urdf,meshes,config}/` from the vendored description

- [ ] **Step 1: Scaffold the package and copy the model**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_description/launch
# Copy (not move — the vendored repo keeps its own copy) the xacro/urdf + any meshes:
cp -r vendor/autonomous-robot/src/robot-nav/description rover_description/urdf
# If meshes live elsewhere in robot-nav, copy them too (check and copy if present):
[ -d vendor/autonomous-robot/src/robot-nav/meshes ] && cp -r vendor/autonomous-robot/src/robot-nav/meshes rover_description/meshes || true
ls rover_description/urdf   # Expected: robot.urdf.xacro, robot_core.xacro, lidar.xacro, imu.xacro, camera.xacro, depth_camera.xacro, ros2_control.xacro, gazebo_control.xacro, inertial_macros.xacro
```

- [ ] **Step 2: Write `rover_description/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_description</name>
  <version>0.0.1</version>
  <description>URDF/xacro model, meshes, and robot_state_publisher bringup for the rover. Dependency root of the rover_* stack.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write `rover_description/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)
# meshes/ and config/ are optional — install them only if present.
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
```

- [ ] **Step 4: Write `rover_description/launch/rsp.launch.py` (robot_state_publisher)**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg = get_package_share_directory("rover_description")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
```
> If the top-level xacro file is named differently than `robot.urdf.xacro`, set
> `xacro_file` to the actual entry file found in Step 1's `ls`.

- [ ] **Step 5: Build and verify the model parses**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_description
source install/setup.bash
xacro $(ros2 pkg prefix rover_description)/share/rover_description/urdf/robot.urdf.xacro > /tmp/rover.urdf && echo "XACRO OK: $(wc -l </tmp/rover.urdf) lines"
ros2 launch rover_description rsp.launch.py --show-args
```
Expected: `XACRO OK: N lines` (N>0) and `--show-args` lists `use_sim_time`.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/rover_description
git commit -m "feat: add rover_description URDF root + robot_state_publisher launch"
```

---

## Task 3: `rover_camera` (leaf, ament_python) — RealSense color publisher

**Files:**
- Create: `ros2_ws/src/rover_camera/{package.xml,setup.py,setup.cfg,resource/rover_camera,rover_camera/__init__.py}`
- Move: `robo_realsense/robo_realsense/ros_stream.py` → `rover_camera/rover_camera/ros_stream.py`

- [ ] **Step 1: Scaffold + move the node**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_camera/rover_camera rover_camera/resource
touch rover_camera/resource/rover_camera
git mv robo_realsense/robo_realsense/ros_stream.py rover_camera/rover_camera/ros_stream.py
touch rover_camera/rover_camera/__init__.py
```

- [ ] **Step 2: Write `rover_camera/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_camera</name>
  <version>0.0.1</version>
  <description>Publishes the RealSense color stream as a ROS Image (/camera/color/image_raw).</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <!-- runtime pip dep: pyrealsense2 (no rosdep key) -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write `rover_camera/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'rover_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Publishes the RealSense color stream as a ROS Image.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_stream = rover_camera.ros_stream:main',
        ],
    },
)
```

- [ ] **Step 4: Write `rover_camera/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_camera
[install]
install_scripts=$base/lib/rover_camera
```

- [ ] **Step 5: Build, verify, remove the emptied source package, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_camera
source install/setup.bash
ros2 pkg executables rover_camera   # Expected: rover_camera ros_stream
git rm -r src/robo_realsense        # ros_stream.py already moved; desktop_stream.py handled in Task 6
git add src/rover_camera
git commit -m "refactor: extract rover_camera (RealSense publisher) from robo_realsense"
```
> Note: `robo_realsense/desktop_stream.py` is a non-ROS dev script destined for
> `rover_tools` (Task 6). Before `git rm`, move it out: `git mv src/robo_realsense/desktop_stream.py /tmp/desktop_stream.py` and restore it in Task 6, **or** reorder so Task 6 runs first. Simplest: run `git mv src/robo_realsense/desktop_stream.py src/robo_realsense/_keep_desktop_stream.py` is NOT needed — instead copy it aside now:
```bash
cp src/robo_realsense/desktop_stream.py /tmp/desktop_stream.py   # rescued for Task 6
```
(Do this copy BEFORE the `git rm` above.)

---

## Task 4: `rover_teleop` (leaf, ament_python)

**Files:**
- Create: `ros2_ws/src/rover_teleop/{package.xml,setup.py,setup.cfg,resource/rover_teleop,rover_teleop/__init__.py}`
- Move: `robo_teleop/robo_teleop/{teleop_node.py,joystick_teleop.py}` → `rover_teleop/rover_teleop/`

- [ ] **Step 1: Scaffold + move**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_teleop/rover_teleop rover_teleop/resource
touch rover_teleop/resource/rover_teleop
git mv robo_teleop/robo_teleop/teleop_node.py rover_teleop/rover_teleop/teleop_node.py
git mv robo_teleop/robo_teleop/joystick_teleop.py rover_teleop/rover_teleop/joystick_teleop.py
touch rover_teleop/rover_teleop/__init__.py
```

- [ ] **Step 2: Write `rover_teleop/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_teleop</name>
  <version>0.0.1</version>
  <description>Turns keyboard and joystick input into /cmd_vel.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write `rover_teleop/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'rover_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Turns keyboard and joystick input into /cmd_vel.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = rover_teleop.teleop_node:main',
            'joystick_teleop = rover_teleop.joystick_teleop:main',
        ],
    },
)
```

- [ ] **Step 4: Write `rover_teleop/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_teleop
[install]
install_scripts=$base/lib/rover_teleop
```

- [ ] **Step 5: Build, verify, remove source, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_teleop
source install/setup.bash
ros2 pkg executables rover_teleop   # Expected: rover_teleop joystick_teleop / keyboard_teleop
git rm -r src/robo_teleop
git add src/rover_teleop
git commit -m "refactor: extract rover_teleop from robo_teleop"
```

---

## Task 5: `rover_gui` (leaf, ament_python)

**Files:**
- Create: `ros2_ws/src/rover_gui/{package.xml,setup.py,setup.cfg,resource/rover_gui,rover_gui/__init__.py}`
- Move: `my_gui_pkg/my_gui_pkg/gui_node.py` → `rover_gui/rover_gui/gui_node.py`

- [ ] **Step 1: Scaffold + move**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_gui/rover_gui rover_gui/resource
touch rover_gui/resource/rover_gui
git mv my_gui_pkg/my_gui_pkg/gui_node.py rover_gui/rover_gui/gui_node.py
touch rover_gui/rover_gui/__init__.py
```

- [ ] **Step 2: Write `rover_gui/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_gui</name>
  <version>0.0.1</version>
  <description>PyQt operator GUI for monitoring and commanding the rover.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>python3-pyqt5</exec_depend>
  <exec_depend>python3-pyqtgraph</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write `rover_gui/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'rover_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='PyQt operator GUI for monitoring and commanding the rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = rover_gui.gui_node:main',
        ],
    },
)
```

- [ ] **Step 4: Write `rover_gui/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_gui
[install]
install_scripts=$base/lib/rover_gui
```

- [ ] **Step 5: Build, verify, remove source, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_gui
source install/setup.bash
ros2 pkg executables rover_gui   # Expected: rover_gui gui_node
git rm -r src/my_gui_pkg
git add src/rover_gui
git commit -m "refactor: extract rover_gui from my_gui_pkg"
```

---

## Task 6: `rover_tools` (leaf, ament_python) — dev/calibration/debug utilities

**Files:**
- Create: `ros2_ws/src/rover_tools/{package.xml,setup.py,setup.cfg,resource/rover_tools,rover_tools/__init__.py}`
- Move: `mal_planner/scripts/{fake_odom.py,calibrate_velocity.py,debug_vel.py}` → `rover_tools/rover_tools/`
- Move: `robo_rover/robo_rover/imu_calib_values.py` → `rover_tools/rover_tools/imu_calib.py`
- Restore: `/tmp/desktop_stream.py` → `rover_tools/rover_tools/desktop_stream.py`

- [ ] **Step 1: Scaffold + move tools in**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_tools/rover_tools rover_tools/resource
touch rover_tools/resource/rover_tools rover_tools/rover_tools/__init__.py
git mv mal_planner/scripts/fake_odom.py rover_tools/rover_tools/fake_odom.py
git mv mal_planner/scripts/calibrate_velocity.py rover_tools/rover_tools/calibrate_velocity.py
git mv mal_planner/scripts/debug_vel.py rover_tools/rover_tools/debug_vel.py
git mv robo_rover/robo_rover/imu_calib_values.py rover_tools/rover_tools/imu_calib.py
cp /tmp/desktop_stream.py rover_tools/rover_tools/desktop_stream.py
git add rover_tools/rover_tools/desktop_stream.py
```
> The moved scripts use `def main():` entry points. `imu_calib.py` writes its YAML to an
> `output_file` ROS param (default was a path relative to the old package). Leave the
> param mechanism as-is; the default falling back to CWD is acceptable for a calibration
> tool. The consumed `calibration_results/imu_calibration.yaml` data goes to `rover_base`
> in Task 8.

- [ ] **Step 2: Write `rover_tools/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_tools</name>
  <version>0.0.1</version>
  <description>Developer, calibration, and debug utilities (not part of the runtime stack).</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <!-- runtime pip dep: pyrealsense2 (desktop_stream, no rosdep key) -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write `rover_tools/setup.py`**

```python
from setuptools import find_packages, setup

package_name = 'rover_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Developer, calibration, and debug utilities (not part of the runtime stack).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_odom = rover_tools.fake_odom:main',
            'calibrate_velocity = rover_tools.calibrate_velocity:main',
            'debug_vel = rover_tools.debug_vel:main',
            'imu_calib = rover_tools.imu_calib:main',
            'desktop_stream = rover_tools.desktop_stream:main',
        ],
    },
)
```
> `desktop_stream.py` currently runs its loop at module top-level (no `main()`). Wrap its
> body in `def main():` and add `if __name__ == '__main__': main()` — this is a structural
> wrap, not a logic change.

- [ ] **Step 4: Write `rover_tools/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_tools
[install]
install_scripts=$base/lib/rover_tools
```

- [ ] **Step 5: Build, verify, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_tools
source install/setup.bash
ros2 pkg executables rover_tools   # Expected: 5 entries (fake_odom, calibrate_velocity, debug_vel, imu_calib, desktop_stream)
git add src/rover_tools
git commit -m "feat: add rover_tools (dev/calibration/debug utilities)"
```

---

## Task 7: `rover_behaviors` (leaf, ament_python) — reactive autonomy

**Files:**
- Create: `ros2_ws/src/rover_behaviors/{package.xml,setup.py,setup.cfg,resource/rover_behaviors,rover_behaviors/__init__.py}`
- Move + rename:
  - `line_follower/line_follower/follower_node.py` → `rover_behaviors/rover_behaviors/line_follower.py`
  - `color_follower/color_follower/follower_node.py` → `rover_behaviors/rover_behaviors/color_follower.py`
  - `line_follower/line_follower/stall_detector.py` → `rover_behaviors/rover_behaviors/stall_detector.py`
  - `line_follower/line_follower/loop_closure_detector.py` → `rover_behaviors/rover_behaviors/lap_monitor.py`
  - `mal_hw3_pkg/mal_hw3_pkg/lidar_wall_stop.py` → `rover_behaviors/rover_behaviors/wall_stop.py`
  - `mal_hw3_pkg/mal_hw3_pkg/pid_control_node.py` → `rover_behaviors/rover_behaviors/wall_follow_pid.py`
  - `robo_rover/robo_rover/auto_lidar_node.py` → `rover_behaviors/rover_behaviors/auto_lidar.py`

- [ ] **Step 1: Scaffold + move/rename all behavior modules**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_behaviors/rover_behaviors rover_behaviors/resource rover_behaviors/launch
touch rover_behaviors/resource/rover_behaviors rover_behaviors/rover_behaviors/__init__.py
git mv line_follower/line_follower/follower_node.py rover_behaviors/rover_behaviors/line_follower.py
git mv color_follower/color_follower/follower_node.py rover_behaviors/rover_behaviors/color_follower.py
git mv line_follower/line_follower/stall_detector.py rover_behaviors/rover_behaviors/stall_detector.py
git mv line_follower/line_follower/loop_closure_detector.py rover_behaviors/rover_behaviors/lap_monitor.py
git mv mal_hw3_pkg/mal_hw3_pkg/lidar_wall_stop.py rover_behaviors/rover_behaviors/wall_stop.py
git mv mal_hw3_pkg/mal_hw3_pkg/pid_control_node.py rover_behaviors/rover_behaviors/wall_follow_pid.py
git mv robo_rover/robo_rover/auto_lidar_node.py rover_behaviors/rover_behaviors/auto_lidar.py
```
> File names change; the `Node`-subclass `super().__init__('...')` node names and all topic
> names stay exactly as written — behavior on the wire is unchanged.

- [ ] **Step 2: Move the behavior launch files and repoint their package references**

The existing behavior launches reference the old package names. Move them and update.

```bash
git mv line_follower/launch/line_follower.launch.py rover_behaviors/launch/line_follower.launch.py
git mv color_follower/launch/color_follower.launch.py rover_behaviors/launch/color_follower.launch.py
git mv color_follower/launch/color_follower_tviz.launch.py rover_behaviors/launch/color_follower_tviz.launch.py
git mv mal_hw3_pkg/launch/lidar_wall_stop.launch.py rover_behaviors/launch/wall_stop.launch.py
```
Then edit each moved launch file per the remap table:
- `package='line_follower'` / `package='color_follower'` / `package='mal_hw3_pkg'` → `package='rover_behaviors'`
- `executable='follower_node'` → `executable='line_follower'` (in line launch) or `'color_follower'` (in color launch)
- `executable='lidar_wall_stop'` → `executable='wall_stop'`; `executable='pid_control_node'` → `executable='wall_follow_pid'`
- `package='terminal_rviz'` → `package='rover_terminal_viz'` (in `color_follower_tviz.launch.py`)
- `get_package_share_directory('line_follower'|'color_follower'|'mal_hw3_pkg')` → `'rover_behaviors'`

Verify no stale names remain:
```bash
grep -rnE "line_follower'|color_follower'|mal_hw3_pkg|follower_node|lidar_wall_stop|pid_control_node|terminal_rviz" rover_behaviors/launch/
# Expected: only NEW executable names (line_follower/color_follower as executables) — no package=line_follower, no terminal_rviz, no follower_node.
```

- [ ] **Step 3: Write `rover_behaviors/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_behaviors</name>
  <version>0.0.1</version>
  <description>Reactive autonomy behaviors (line/color following, wall stop/follow, hallway seek, lap completion) producing /cmd_vel.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>rcl_interfaces</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <!-- runtime pip dep: simple_pid (wall_stop, no rosdep key) -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `rover_behaviors/setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_behaviors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Reactive autonomy behaviors producing /cmd_vel.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = rover_behaviors.line_follower:main',
            'color_follower = rover_behaviors.color_follower:main',
            'stall_detector = rover_behaviors.stall_detector:main',
            'lap_monitor = rover_behaviors.lap_monitor:main',
            'wall_stop = rover_behaviors.wall_stop:main',
            'wall_follow_pid = rover_behaviors.wall_follow_pid:main',
            'auto_lidar = rover_behaviors.auto_lidar:main',
        ],
    },
)
```

- [ ] **Step 5: Write `rover_behaviors/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_behaviors
[install]
install_scripts=$base/lib/rover_behaviors
```

- [ ] **Step 6: Build, verify, remove emptied source packages, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_behaviors
source install/setup.bash
ros2 pkg executables rover_behaviors   # Expected: 7 entries
git rm -r src/line_follower src/color_follower src/mal_hw3_pkg
git add src/rover_behaviors
git commit -m "refactor: consolidate reactive behaviors into rover_behaviors"
```

---

## Task 8: `rover_base` (leaf, ament_python) — ArduPilot base driver

**Files:**
- Create: `ros2_ws/src/rover_base/{package.xml,setup.py,setup.cfg,resource/rover_base,rover_base/__init__.py}`
- Move: `robo_rover/robo_rover/{rover_node.py,velocity_controller.py}` → `rover_base/rover_base/`
- Move: `robo_rover/launch/rover_launch.py` → `rover_base/launch/base.launch.py`
- Move: `robo_rover/calibration_results/` → `rover_base/calibration_results/`

- [ ] **Step 1: Scaffold + move base nodes, launch, calibration data**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_base/rover_base rover_base/resource rover_base/launch
touch rover_base/resource/rover_base rover_base/rover_base/__init__.py
git mv robo_rover/robo_rover/rover_node.py rover_base/rover_base/rover_node.py
git mv robo_rover/robo_rover/velocity_controller.py rover_base/rover_base/velocity_controller.py
git mv robo_rover/launch/rover_launch.py rover_base/launch/base.launch.py
git mv robo_rover/calibration_results rover_base/calibration_results
```

- [ ] **Step 2: Repoint `base.launch.py` to the console-script executable**

The old launch invoked the node via `executable='python3', arguments=['-m', 'robo_rover.rover_node']`.
Replace that `Node(...)` block's executable spec with the clean console script now that
`rover_base` registers it:
- Change `executable='python3'` → `executable='rover_node'`, `package` set to `'rover_base'`,
  and remove the `arguments=['-m', 'robo_rover.rover_node']` line.

Resulting node block:
```python
    rover_node = Node(
        package='rover_base',
        executable='rover_node',
        name='rover_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'connection_string': LaunchConfiguration('connection_string'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'imu_frequency': LaunchConfiguration('imu_frequency'),
        }],
    )
```

- [ ] **Step 3: Write `rover_base/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_base</name>
  <version>0.0.1</version>
  <description>Drives the ArduPilot rover base: consumes /cmd_vel, publishes /odom and raw /imu.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <!-- runtime pip dep: pymavlink (no rosdep key) -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `rover_base/setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'calibration_results'), glob('calibration_results/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Drives the ArduPilot rover base.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_node = rover_base.rover_node:main',
            'velocity_controller = rover_base.velocity_controller:main',
        ],
    },
)
```

- [ ] **Step 5: Write `rover_base/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_base
[install]
install_scripts=$base/lib/rover_base
```

- [ ] **Step 6: Build, verify, remove the now-empty `robo_rover`, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_base
source install/setup.bash
ros2 pkg executables rover_base     # Expected: rover_base rover_node / velocity_controller
ros2 launch rover_base base.launch.py --show-args   # Expected: lists connection_string, baud_rate, ...
git rm -r src/robo_rover            # rover_node, velocity_controller, imu_calib, auto_lidar, calibration_results, rover_launch all already moved
git add src/rover_base
git commit -m "refactor: extract rover_base (ArduPilot base driver) from robo_rover"
```

---

## Task 9: `rover_state_estimation` (ament_python) — rf2o + EKF fusion (dedupe ekf)

**Files:**
- Create: `ros2_ws/src/rover_state_estimation/{package.xml,setup.py,setup.cfg,resource/rover_state_estimation,rover_state_estimation/__init__.py,config/ekf.yaml,launch/state_estimation.launch.py}`
- Source: `main_mal_launch/config/ekf.yaml` (canonical); delete orphan `robo_rover/Config/ekf.yaml` (already gone with robo_rover, but confirm)

- [ ] **Step 1: Scaffold + move the canonical ekf config**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_state_estimation/rover_state_estimation rover_state_estimation/resource rover_state_estimation/config rover_state_estimation/launch
touch rover_state_estimation/resource/rover_state_estimation rover_state_estimation/rover_state_estimation/__init__.py
git mv main_mal_launch/config/ekf.yaml rover_state_estimation/config/ekf.yaml
# The duplicate robo_rover/Config/ekf.yaml was already deleted up-front as an orphan
# (see "chore: delete unused robo_rover ekf.yaml" in git log). Confirm it is gone:
test ! -e robo_rover/Config/ekf.yaml && echo "orphan ekf.yaml already removed OK"
```

- [ ] **Step 2: Write `rover_state_estimation/launch/state_estimation.launch.py`**

This composes the fused-odometry layer: rf2o laser odometry + the robot_localization EKF.
(`rf2o_laser_odometry` resolves by package name — unaffected by the vendor move.)

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('rover_state_estimation'), 'config', 'ekf.yaml'
    )
    rf2o_launch = PathJoinSubstitution(
        [FindPackageShare('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py']
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rf2o_launch)),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', 'odom')],
        ),
    ])
```
> This mirrors exactly the EKF node + config that `mal_startup.launch.py` ran, plus the
> rf2o include that the nav launches ran — now in one place. Behavior preserved.

- [ ] **Step 3: Write `rover_state_estimation/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_state_estimation</name>
  <version>0.0.1</version>
  <description>Fuses laser odometry (rf2o) and IMU into a single /odom via a robot_localization EKF.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>rf2o_laser_odometry</exec_depend>
  <exec_depend>rover_description</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `rover_state_estimation/setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_state_estimation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Fuses laser odometry and IMU into a single /odom via an EKF.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
```

- [ ] **Step 5: Write `rover_state_estimation/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_state_estimation
[install]
install_scripts=$base/lib/rover_state_estimation
```

- [ ] **Step 6: Build, verify, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_state_estimation
source install/setup.bash
ros2 launch rover_state_estimation state_estimation.launch.py --show-args   # Expected: loads (no PackageNotFoundError)
git add src/rover_state_estimation
git commit -m "refactor: add rover_state_estimation (rf2o + EKF), dedupe canonical ekf.yaml"
```

---

## Task 10: `rover_slam` (ament_python) — live mapping

**Files:**
- Create: `ros2_ws/src/rover_slam/{package.xml,setup.py,setup.cfg,resource/rover_slam,rover_slam/__init__.py,launch/mapping.launch.py}`
- Source content from: `main_mal_launch/launch/slam.launch.py` (the minimal mapping bringup)

- [ ] **Step 1: Scaffold + create the mapping launch**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_slam/rover_slam rover_slam/resource rover_slam/launch
touch rover_slam/resource/rover_slam rover_slam/rover_slam/__init__.py
git mv main_mal_launch/launch/slam.launch.py rover_slam/launch/mapping.launch.py
```

- [ ] **Step 2: Repoint references in `mapping.launch.py`**

The moved file references vendored packages only (`rplidar_ros`, `slam_toolbox`) — both
resolve by name, so **no package-name edits are required**. Confirm:
```bash
grep -nE "get_package_share_directory|FindPackageShare|package=" rover_slam/launch/mapping.launch.py
# Expected: only rplidar_ros / slam_toolbox / tf2_ros — all vendored-by-name, leave as-is.
```

- [ ] **Step 3: Write `rover_slam/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_slam</name>
  <version>0.0.1</version>
  <description>Builds a live map with slam_toolbox (online async/sync); publishes /map and the map->odom TF.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>rplidar_ros</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>rover_description</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `rover_slam/setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Builds a live map with slam_toolbox.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
```

- [ ] **Step 5: Write `rover_slam/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_slam
[install]
install_scripts=$base/lib/rover_slam
```

- [ ] **Step 6: Build, verify, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_slam
source install/setup.bash
ros2 launch rover_slam mapping.launch.py --show-args   # Expected: loads (no PackageNotFoundError)
git add src/rover_slam
git commit -m "refactor: add rover_slam (live slam_toolbox mapping)"
```

---

## Task 11: `rover_localization` (ament_python) — localize on a saved map

**Files:**
- Create: `ros2_ws/src/rover_localization/{package.xml,setup.py,setup.cfg,resource/rover_localization,rover_localization/__init__.py}`
- Move: `mal_planner/scripts/save_map.py` → `rover_localization/rover_localization/save_map.py`
- Move: `mal_planner/config/amcl.yaml` → `rover_localization/config/amcl.yaml`
- Move: `mal_planner/maps/` → `rover_localization/maps/`
- Move (if present): `mal_planner/launch/{localization,amcl}*.launch.py` → `rover_localization/launch/`

- [ ] **Step 1: Scaffold + move localization assets**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_localization/rover_localization rover_localization/resource rover_localization/config rover_localization/launch rover_localization/maps
touch rover_localization/resource/rover_localization rover_localization/rover_localization/__init__.py
git mv mal_planner/scripts/save_map.py rover_localization/rover_localization/save_map.py
git mv mal_planner/config/amcl.yaml rover_localization/config/amcl.yaml
git mv mal_planner/maps/* rover_localization/maps/ 2>/dev/null; true
```

- [ ] **Step 2: Fix any in-code map/config path that named the old package**

```bash
grep -rnE "mal_planner|main_mal_launch" rover_localization/rover_localization/save_map.py
# For any get_package_share_directory('mal_planner') → 'rover_localization';
# for any hard-coded '.../mal_planner/maps' → '.../rover_localization/maps'.
```
Apply the substitutions found. `save_map` subscribes to the `/map` *topic* (no build dep on
`rover_slam`). Saved maps land in `rover_localization/maps/` (persisted via `--symlink-install`).

- [ ] **Step 2b: Write `rover_localization/launch/localization.launch.py` (map_server + amcl)**

This is the "localize on a saved map" launch the spec calls for. It uses the migrated
`amcl.yaml` and a `map:=` argument (override to pick a specific saved map).

```python
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
```
> `default_map` points at `maps/map.yaml`; the migrated maps are named like
> `map_April_30_16_52.yaml`, so always launch with `map:=<full path>` (or rename one to
> `map.yaml`). `--show-args` verification (Step 7) only loads the description, so a missing
> default map file does not fail that check.

- [ ] **Step 3: Write `rover_localization/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_localization</name>
  <version>0.0.1</version>
  <description>Localizes the rover on a saved map (amcl + slam_toolbox localization + map_server) and saves maps.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_amcl</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>rover_description</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `rover_localization/setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Localizes the rover on a saved map and saves maps.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_map = rover_localization.save_map:main',
        ],
    },
)
```

- [ ] **Step 5: Write `rover_localization/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_localization
[install]
install_scripts=$base/lib/rover_localization
```

- [ ] **Step 6: Build, verify, commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_localization
source install/setup.bash
ros2 pkg executables rover_localization   # Expected: rover_localization save_map
ls $(ros2 pkg prefix rover_localization)/share/rover_localization/maps   # Expected: the migrated *.yaml/*.pgm maps
git add src/rover_localization
git commit -m "refactor: add rover_localization (amcl/map_server + save_map + maps)"
```

---

## Task 12: `rover_navigation` (ament_python) — planning + path following

**Files:**
- Create: `ros2_ws/src/rover_navigation/{package.xml,setup.py,setup.cfg,resource/rover_navigation,rover_navigation/__init__.py,launch/}`
- Move: `mal_planner/scripts/{astar_planner.py,pure_pursuit.py,goal_pub.py}` → `rover_navigation/rover_navigation/`
- Move: `mal_planner/launch/{planner_offline,slam_offline,slam_planning,slam_debug}*` → `rover_navigation/launch/`
- Move: `mal_planner/rviz/` (if present) → `rover_navigation/rviz/`

- [ ] **Step 1: Scaffold + move planner nodes and offline/debug launches**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_navigation/rover_navigation rover_navigation/resource rover_navigation/launch
touch rover_navigation/resource/rover_navigation rover_navigation/rover_navigation/__init__.py
git mv mal_planner/scripts/astar_planner.py rover_navigation/rover_navigation/astar_planner.py
git mv mal_planner/scripts/pure_pursuit.py rover_navigation/rover_navigation/pure_pursuit.py
git mv mal_planner/scripts/goal_pub.py rover_navigation/rover_navigation/goal_pub.py
git mv mal_planner/launch/planner_offline.launch.py rover_navigation/launch/planner_offline.launch.py
git mv mal_planner/launch/slam_offline.launch.py rover_navigation/launch/slam_offline.launch.py
git mv mal_planner/launch/slam_planning.launch.py rover_navigation/launch/slam_planning.launch.py
git mv mal_planner/launch/slam_debug.py rover_navigation/launch/slam_debug.py
[ -d mal_planner/rviz ] && git mv mal_planner/rviz rover_navigation/rviz || true
```

- [ ] **Step 2: Repoint references in the moved launches per the remap table**

These launches reference several old packages. Apply contextually (see the authoritative
table in Conventions):
- `mal_planner` planner scripts → `package='rover_navigation'`, executables `astar_planner`/`pure_pursuit`/`goal_pub`
- `fake_odom` → `package='rover_tools'`, executable `fake_odom`
- `save_map` / `amcl` / `maps` / `amcl.yaml` paths → `rover_localization`
- slam params / `slam_toolbox` / `rplidar_ros` / `rf2o_laser_odometry` / `robot_localization` → unchanged (vendored by name)
- ekf.yaml path (if referenced) → `get_package_share_directory('rover_state_estimation')`

```bash
grep -rnE "mal_planner|main_mal_launch|robo_rover|robo_realsense" rover_navigation/launch/
# Expected after edits: NO matches for these old first-party names.
```

- [ ] **Step 3: Write `rover_navigation/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_navigation</name>
  <version>0.0.1</version>
  <description>Plans a path to a goal (A*) and follows it (pure pursuit): /goal_pose -> /plan -> /cmd_vel.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>
  <exec_depend>rover_description</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write `rover_navigation/setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='A* planner and pure-pursuit controller for map-localized navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = rover_navigation.astar_planner:main',
            'pure_pursuit = rover_navigation.pure_pursuit:main',
            'goal_pub = rover_navigation.goal_pub:main',
        ],
    },
)
```

- [ ] **Step 5: Write `rover_navigation/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_navigation
[install]
install_scripts=$base/lib/rover_navigation
```

- [ ] **Step 6: Build, verify, commit (mal_planner removal happens in Task 13 after its composed launches move)**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_navigation
source install/setup.bash
ros2 pkg executables rover_navigation   # Expected: astar_planner, pure_pursuit, goal_pub
git add src/rover_navigation
git commit -m "refactor: add rover_navigation (A* planner + pure pursuit + goal_pub)"
```

---

## Task 13: `rover_terminal_viz` (ament_cmake C++) — rename of terminal_rviz

**Files:**
- Rename: `ros2_ws/src/terminal_rviz/` → `ros2_ws/src/rover_terminal_viz/`
- Modify: `package.xml` (`<name>`), `CMakeLists.txt` (`project()`), `include/terminal_rviz/` dir + `#include` paths, `install()` rules.

- [ ] **Step 1: Move the package dir and the include subdir**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
git mv terminal_rviz rover_terminal_viz
git mv rover_terminal_viz/include/terminal_rviz rover_terminal_viz/include/rover_terminal_viz
```

- [ ] **Step 2: Update the package name in manifest, CMake, and include paths**

```bash
cd rover_terminal_viz
# package.xml <name>
sed -i 's#<name>terminal_rviz</name>#<name>rover_terminal_viz</name>#' package.xml
# CMake project name (covers project(terminal_rviz ...) and ${PROJECT_NAME} usage)
sed -i 's#project(terminal_rviz#project(rover_terminal_viz#' CMakeLists.txt
# #include paths that referenced the old include dir
grep -rl '#include "terminal_rviz/' src include 2>/dev/null | xargs -r sed -i 's#\#include "terminal_rviz/#\#include "rover_terminal_viz/#g'
# Any other literal install/share path references to the old name in CMake:
sed -i 's#terminal_rviz#rover_terminal_viz#g' CMakeLists.txt
```
> Keep the **executable/node name `terminal_rviz_node` unchanged** — only the *package*
> name changes. If `CMakeLists.txt` defines `add_executable(terminal_rviz_node ...)`, the
> blanket `s#terminal_rviz#rover_terminal_viz#g` above will have renamed it to
> `rover_terminal_viz_node`. Revert just that token so the node name is preserved:
```bash
sed -i 's#rover_terminal_viz_node#terminal_rviz_node#g' CMakeLists.txt
```
Verify maintainer/description are filled (fix if empty):
```bash
grep -E "<maintainer|<description" package.xml
```

- [ ] **Step 3: Build and verify the node still resolves under the new package**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
colcon build --symlink-install --packages-select rover_terminal_viz 2>&1 | tail -15
source install/setup.bash
ros2 pkg executables rover_terminal_viz   # Expected: rover_terminal_viz terminal_rviz_node
```

- [ ] **Step 4: Commit**

```bash
git add -A src/rover_terminal_viz
git commit -m "refactor: rename terminal_rviz package to rover_terminal_viz (node name preserved)"
```

---

## Task 14: `rover_bringup` (ament_python) — top-level composition + backend switch

**Files:**
- Create: `ros2_ws/src/rover_bringup/{package.xml,setup.py,setup.cfg,resource/rover_bringup,rover_bringup/__init__.py,launch/}`
- Create: `rover_bringup/launch/{backend.launch.py,rover.launch.py}`
- Move: `main_mal_launch/launch/mal_startup.launch.py` content → `rover_bringup/launch/rover.launch.py`
- Move: `mal_planner/launch/{slam_nav,odom_nav}.launch.py` → `rover_bringup/launch/`
- Move: `main_mal_launch/launch/line_follower.launch.py` → `rover_bringup/launch/`

- [ ] **Step 1: Scaffold + move composed launches**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
mkdir -p rover_bringup/rover_bringup rover_bringup/resource rover_bringup/launch
touch rover_bringup/resource/rover_bringup rover_bringup/rover_bringup/__init__.py
git mv mal_planner/launch/slam_nav.launch.py rover_bringup/launch/slam_nav.launch.py
git mv mal_planner/launch/odom_nav.launch.py rover_bringup/launch/odom_nav.launch.py
git mv main_mal_launch/launch/line_follower.launch.py rover_bringup/launch/line_follower.launch.py
git mv main_mal_launch/launch/mal_startup.launch.py rover_bringup/launch/rover.launch.py
```

- [ ] **Step 2: Write `rover_bringup/launch/backend.launch.py` (the swappable sensing/actuation backend)**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _real():
    # Distro-robust equality (LaunchConfigurationEquals was removed in newer ROS 2 distros).
    # Returns a FRESH condition each call so it is not shared across actions.
    return IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'real'"]))


def generate_launch_description():
    lidar_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rplidar_ros"), "launch", "view_mal_rplidar.launch.py"])),
        condition=_real(),
    )
    camera_real = Node(
        package="rover_camera", executable="ros_stream", output="screen",
        condition=_real(),
    )
    base_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rover_base"), "launch", "base.launch.py"])),
        condition=_real(),
    )
    # Static sensor TFs (were in mal_startup) — needed by mapping in every backend.
    tf_laser = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_link_to_laser_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser"],
        output="screen",
    )
    tf_imu = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_link_to_imu_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="real",
                              choices=["real", "sim", "bag"],
                              description="Sensing/actuation source"),
        tf_laser, tf_imu,
        lidar_real, camera_real, base_real,
        # NOTE: only the 'real' backend is wired — it reproduces mal_startup's sources
        # exactly (rplidar + rover_camera + rover_base + the two static TFs). 'sim'
        # (Gazebo via rover_description's gazebo_control.xacro) and 'bag' (ros2 bag play)
        # honor the SAME /scan,/odom,/imu contract and can be added later WITHOUT touching
        # any consumer. Wiring them is new functionality, intentionally out of scope here
        # (spec non-goal: "no new features").
    ])
```
> This preserves the `real` backend exactly as `mal_startup` had it (rplidar +
> rover_camera + rover_base + the two static TFs).

- [ ] **Step 3: Rewrite `rover_bringup/launch/rover.launch.py` (was mal_startup) to compose layers**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg, rel):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), *rel])
        ),
        launch_arguments={}.items(),
    )


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="real",
                              choices=["real", "sim", "bag"]),
        # 1) robot model
        _include("rover_description", ["launch", "rsp.launch.py"]),
        # 2) sensing/actuation backend (real|sim|bag)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("rover_bringup"), "launch", "backend.launch.py"])),
            launch_arguments={"backend": backend}.items(),
        ),
        # 3) fused odometry (rf2o + EKF)
        _include("rover_state_estimation", ["launch", "state_estimation.launch.py"]),
        # 4) live mapping (slam_toolbox)
        _include("rover_slam", ["launch", "mapping.launch.py"]),
    ])
```
> Layer-for-layer this reproduces `mal_startup`'s node set — model (new, was implicit),
> camera+lidar+base+TFs (backend), EKF (state_estimation), slam_toolbox (slam) — now
> composed and backend-parameterized.

- [ ] **Step 4: Repoint references in the moved `slam_nav` / `odom_nav` / `line_follower` launches**

Apply the remap table to each:
```bash
grep -rnE "mal_planner|main_mal_launch|robo_rover|robo_realsense|line_follower'|color_follower'|mal_hw3_pkg" rover_bringup/launch/slam_nav.launch.py rover_bringup/launch/odom_nav.launch.py rover_bringup/launch/line_follower.launch.py
```
- planner/pursuit/goal executables → `package='rover_navigation'`
- `fake_odom` → `package='rover_tools'`
- ekf.yaml → `rover_state_estimation`; amcl/maps/save_map → `rover_localization`
- camera `ros_stream` → `package='rover_camera'`; base → `rover_base`; behaviors → `rover_behaviors`
- slam/rplidar/rf2o/robot_localization → unchanged
Re-run the grep; expected: no old first-party names remain.

- [ ] **Step 5: Write `rover_bringup/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>rover_bringup</name>
  <version>0.0.1</version>
  <description>Top-level layered launch composition with a real|sim|bag backend switch.</description>
  <maintainer email="allendevaraj33333@gmail.com">The2xMan</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rover_description</exec_depend>
  <exec_depend>rover_base</exec_depend>
  <exec_depend>rover_camera</exec_depend>
  <exec_depend>rover_state_estimation</exec_depend>
  <exec_depend>rover_slam</exec_depend>
  <exec_depend>rover_localization</exec_depend>
  <exec_depend>rover_navigation</exec_depend>
  <exec_depend>rover_behaviors</exec_depend>
  <exec_depend>rplidar_ros</exec_depend>
  <exec_depend>rf2o_laser_odometry</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 6: Write `rover_bringup/setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The2xMan',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Top-level layered launch composition with a real|sim|bag backend switch.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
```

- [ ] **Step 7: Write `rover_bringup/setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_bringup
[install]
install_scripts=$base/lib/rover_bringup
```

- [ ] **Step 8: Remove emptied legacy launch packages, build whole workspace, verify**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
# main_mal_launch and mal_planner should now be empty of source (only leftover dirs/manifests):
ls main_mal_launch mal_planner
git rm -r main_mal_launch mal_planner
cd ..
colcon build --symlink-install 2>&1 | tail -25
source install/setup.bash
ros2 launch rover_bringup rover.launch.py --show-args   # Expected: lists 'backend'; no PackageNotFoundError
git add -A
git commit -m "feat: add rover_bringup (layered composition + backend switch); remove legacy launch packages"
```

---

## Task 15: Tidy, docs, and final verification

**Files:**
- Create: `CLAUDE.md` (root) — authoritative package table + backend contract
- Modify: `README.md` (root)
- Create: `ros2_ws/src/rover_*/README.md` notes for pip-only deps (optional, brief)

- [ ] **Step 1: Confirm no legacy first-party packages remain**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/src
ls -d */ | sort
# Expected first-party: rover_base rover_behaviors rover_bringup rover_camera rover_description
#   rover_gui rover_localization rover_navigation rover_slam rover_state_estimation
#   rover_terminal_viz rover_tools  (12 dirs) + vendor/
# Expected GONE: mal_planner main_mal_launch mal_hw3_pkg robo_rover robo_realsense
#   robo_teleop line_follower color_follower my_gui_pkg terminal_rviz
test ! -e ../src/main_mal_launch && echo "legacy gone OK"
# Confirm exactly one ekf.yaml:
find . -name ekf.yaml   # Expected: ./rover_state_estimation/config/ekf.yaml only
```

- [ ] **Step 2: Confirm runtime surface matches the baseline (no lost executables)**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
source install/setup.bash
ros2 pkg executables | grep -E "^rover_" | sort | tee /tmp/new_executables.txt
# Compare COUNT of node-bearing entries against baseline-executables.txt (names changed,
# but every old runtime executable must have a rover_* counterpart). Reconcile any missing.
diff <(sort docs/superpowers/notes/baseline-executables.txt | awk '{print $2}') <(awk '{print $2}' /tmp/new_executables.txt) || echo "(review name remaps; ensure no capability lost)"
```

- [ ] **Step 3: Write root `CLAUDE.md` (authoritative package table)**

```markdown
# Advanced_Robo_Project — rover_* SLAM stack

ROS 2 SLAM mobile rover. Modular `rover_*` architecture (see
`docs/superpowers/specs/2026-05-20-rover-modular-architecture-design.md`).

## Workspace layout (`ros2_ws/src/`)

| Package | Build | Responsibility |
|---|---|---|
| rover_description | cmake | URDF/xacro + robot_state_publisher (dependency root) |
| rover_base | python | ArduPilot base: /cmd_vel -> motors, publishes /odom + /imu |
| rover_camera | python | RealSense color stream publisher |
| rover_state_estimation | python | rf2o + EKF fusion -> /odom |
| rover_slam | python | Live slam_toolbox mapping -> /map |
| rover_localization | python | amcl/map_server localization + save_map + maps/ |
| rover_navigation | python | A* planner + pure pursuit (/goal_pose->/plan->/cmd_vel) |
| rover_teleop | python | Keyboard/joystick -> /cmd_vel |
| rover_behaviors | python | Reactive autonomy (line/color/wall/lap) -> /cmd_vel |
| rover_gui | python | PyQt operator GUI |
| rover_terminal_viz | cmake | Terminal visualizer (node: terminal_rviz_node) |
| rover_tools | python | Dev/calibration/debug utilities (not runtime) |
| rover_bringup | python | Top-level layered launch + backend:=real|sim|bag |
| vendor/ | — | slam_toolbox, rplidar_ros, rf2o_laser_odometry, autonomous-robot |

## Run

    cd ros2_ws && colcon build --symlink-install && source install/setup.bash
    ros2 launch rover_bringup rover.launch.py backend:=real

## Backend contract (swappable)
/scan, /odom, /imu, /camera/color/image_raw are produced by the backend
(real|sim|bag); the slam/nav/behavior core consumes them and never depends on
which backend is active.

## Build notes
- pip-only deps (not rosdep): pyrealsense2 (rover_camera/rover_tools),
  pymavlink (rover_base), simple_pid (rover_behaviors).
- `--symlink-install` is required for save_map to persist into rover_localization/maps.
```

- [ ] **Step 4: Update root `README.md`**

Replace the stale README body with a short pointer:
```markdown
# Advanced_Robo_Project

ROS 2 SLAM mobile-rover stack, organized into modular `rover_*` packages.
See `CLAUDE.md` for the package map and run instructions, and
`docs/superpowers/specs/2026-05-20-rover-modular-architecture-design.md` for the
architecture rationale.
```

- [ ] **Step 5: Final clean build from scratch + commit**

```bash
cd ~/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws
rm -rf build install log
colcon build --symlink-install 2>&1 | tail -30
echo "build exit=$?"
source install/setup.bash
ros2 launch rover_bringup rover.launch.py --show-args
cd ..
git add -A
git commit -m "docs: add CLAUDE.md package map + backend contract; refresh README"
```
Expected: clean build exit 0; `--show-args` lists `backend`; `git status` clean.

---

## Self-review notes (for the executor)

- **Behavior preservation:** every node's `super().__init__('name')` string and all topic
  names are untouched. Only file names, package names, entry-point names, and launch
  references change. If you find yourself editing topic strings or node names, STOP — that
  is out of scope.
- **The contextual remap (mal_planner / robo_rover) is the #1 risk.** Always consult the
  authoritative table in Conventions; never blanket-sed `mal_planner` → one package.
- **Vendored names resolve by package name** (`rplidar_ros`, `slam_toolbox`,
  `rf2o_laser_odometry`, `robot_localization`, `nav2_*`): never rewrite those references.
- **If a `colcon build` fails,** fix before moving to the next task — the build-green
  invariant is what makes this safe.
