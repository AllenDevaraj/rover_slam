# Rover SLAM Gazebo Fortress Sim Backend — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a swappable `backend:=sim` that runs the rover in Gazebo Fortress so the full SLAM → plan → pure-pursuit pipeline can be verified without hardware, with both DiffDrive and Ackermann drive models.

**Architecture:** A new `rover_sim` package owns all simulation assets (worlds, a PGM→world converter, a `ros_gz` topic-bridge config, a `cmd_vel` relay, RViz, and launch files). `rover_description` gains sim-only xacros (sensors + two drive modules) gated by `sim_mode`/`drive_model` args. The sim only emits `/scan`, `/imu`, `/camera/color/image_raw` and moves the robot on `/cmd_vel`; rf2o + fake_odom + EKF + slam_toolbox + A* + pure_pursuit run unchanged.

**Tech Stack:** ROS 2 Humble, Gazebo Fortress (`ign gazebo` 6.17), `ros_gz_sim`/`ros_gz_bridge`/`ros_gz_image` (Fortress build, 0.244.20), `ament_python`, `xacro`, SDF, `pytest`.

**Reference spec:** `docs/superpowers/specs/2026-05-21-rover-sim-backend-design.md`

**Conventions used throughout:**
- All commands run from `~/rover_slam/ros2_ws` unless noted. Build with
  `MAKEFLAGS="-j1" colcon build --symlink-install --parallel-workers 1` (machine is OOM-capped).
- Source `/opt/ros/humble/setup.bash` then `install/setup.bash` before ROS commands.
- Verified Fortress system-plugin filenames (used in xacro `<plugin filename=...>`):
  `ignition-gazebo-diff-drive-system`, `ignition-gazebo-ackermann-steering-system`,
  `ignition-gazebo-sensors-system`, `ignition-gazebo-imu-system`,
  `ignition-gazebo-joint-state-publisher-system`. C++ names live under
  `ignition::gazebo::systems::{DiffDrive,AckermannSteering,Sensors,Imu,JointStatePublisher}`.
- Bridge gz type names are `ignition.msgs.*` (Fortress).
- Branch: work happens on `sim-backend` (already created).

---

## Phase 0 — `rover_sim` package + Python tooling (TDD)

### Task 1: Create the `rover_sim` package skeleton

**Files:**
- Create: `ros2_ws/src/rover_sim/package.xml`
- Create: `ros2_ws/src/rover_sim/setup.py`
- Create: `ros2_ws/src/rover_sim/setup.cfg`
- Create: `ros2_ws/src/rover_sim/resource/rover_sim`
- Create: `ros2_ws/src/rover_sim/rover_sim/__init__.py`

- [ ] **Step 1: Create `package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_sim</name>
  <version>0.0.0</version>
  <description>Gazebo Fortress simulation backend for the rover: worlds, ros_gz bridge, spawn/launch, producing the /scan,/imu,/camera topic contract.</description>
  <maintainer email="allendevaraj33333@gmail.com">Allen Devaraj</maintainer>
  <license>MIT</license>

  <exec_depend>rover_description</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_image</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>rover_state_estimation</exec_depend>
  <exec_depend>rf2o_laser_odometry</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>rover_navigation</exec_depend>
  <exec_depend>rover_localization</exec_depend>
  <exec_depend>rover_behaviors</exec_depend>
  <exec_depend>rover_teleop</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create `setup.py`**

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Allen Devaraj',
    maintainer_email='allendevaraj33333@gmail.com',
    description='Gazebo Fortress simulation backend for the rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = rover_sim.cmd_vel_relay:main',
            'map2world = rover_sim.map_to_world:main',
        ],
    },
)
```

- [ ] **Step 3: Create `setup.cfg`**

```ini
[develop]
script_dir=$base/lib/rover_sim
[install]
install_scripts=$base/lib/rover_sim
```

- [ ] **Step 4: Create the package marker and module init (both empty files)**

```bash
mkdir -p ros2_ws/src/rover_sim/resource ros2_ws/src/rover_sim/rover_sim \
         ros2_ws/src/rover_sim/launch ros2_ws/src/rover_sim/worlds \
         ros2_ws/src/rover_sim/config ros2_ws/src/rover_sim/rviz \
         ros2_ws/src/rover_sim/test
touch ros2_ws/src/rover_sim/resource/rover_sim
touch ros2_ws/src/rover_sim/rover_sim/__init__.py
```

- [ ] **Step 5: Build and verify the package is discovered**

Run: `cd ~/rover_slam/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select rover_sim`
Expected: `Finished <<< rover_sim` exit 0.
Run: `colcon list | grep rover_sim`
Expected: `rover_sim	src/rover_sim	(ros.ament_python)`

- [ ] **Step 6: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim
git commit -m "feat(rover_sim): scaffold simulation backend package"
```

---

### Task 2: `cmd_vel_relay` node (TDD)

The relay merges the two `/cmd_vel` producers (autonomy publishes `TwistStamped`, teleop publishes `Twist`) into one plain `Twist` topic the Gazebo drive system subscribes to.

**Files:**
- Create: `ros2_ws/src/rover_sim/rover_sim/cmd_vel_relay.py`
- Test: `ros2_ws/src/rover_sim/test/test_cmd_vel_relay.py`

- [ ] **Step 1: Write the failing test**

```python
# ros2_ws/src/rover_sim/test/test_cmd_vel_relay.py
import rclpy
import pytest
from geometry_msgs.msg import Twist, TwistStamped
from rover_sim.cmd_vel_relay import CmdVelRelay


@pytest.fixture(scope='module', autouse=True)
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_stamped_callback_publishes_inner_twist():
    node = CmdVelRelay()
    captured = []
    node.pub.publish = lambda m: captured.append(m)  # intercept output

    msg = TwistStamped()
    msg.twist.linear.x = 0.5
    msg.twist.angular.z = -0.3
    node._stamped_cb(msg)

    assert len(captured) == 1
    assert isinstance(captured[0], Twist)
    assert captured[0].linear.x == 0.5
    assert captured[0].angular.z == -0.3
    node.destroy_node()


def test_teleop_twist_passthrough():
    node = CmdVelRelay()
    captured = []
    node.pub.publish = lambda m: captured.append(m)

    msg = Twist()
    msg.linear.x = 0.25
    node._twist_cb(msg)

    assert len(captured) == 1
    assert captured[0].linear.x == 0.25
    node.destroy_node()
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd ~/rover_slam/ros2_ws && source /opt/ros/humble/setup.bash && python3 -m pytest src/rover_sim/test/test_cmd_vel_relay.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'rover_sim.cmd_vel_relay'`.

- [ ] **Step 3: Write the implementation**

```python
# ros2_ws/src/rover_sim/rover_sim/cmd_vel_relay.py
#!/usr/bin/env python3
"""Relay /cmd_vel (TwistStamped, from autonomy) and /cmd_vel_teleop (Twist, from
keyboard teleop) onto a single plain Twist topic consumed by the Gazebo drive
system via ros_gz_bridge. Resolves the pre-existing cmd_vel type split."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('output_topic', '/model/rover/cmd_vel')
        out = self.get_parameter('output_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, out, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._stamped_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self._twist_cb, 10)
        self.get_logger().info(
            f'cmd_vel_relay: /cmd_vel (TwistStamped) + /cmd_vel_teleop (Twist) -> {out} (Twist)')

    def _stamped_cb(self, msg: TwistStamped) -> None:
        self.pub.publish(msg.twist)

    def _twist_cb(self, msg: Twist) -> None:
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest src/rover_sim/test/test_cmd_vel_relay.py -v`
Expected: 2 passed.

- [ ] **Step 5: Build and confirm the entry point resolves**

Run: `colcon build --symlink-install --packages-select rover_sim && source install/setup.bash && ros2 pkg executables rover_sim`
Expected: includes `rover_sim cmd_vel_relay`.

- [ ] **Step 6: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim/rover_sim/cmd_vel_relay.py ros2_ws/src/rover_sim/test/test_cmd_vel_relay.py
git commit -m "feat(rover_sim): add cmd_vel_relay (TwistStamped+Twist -> Twist)"
```

---

### Task 3: `map_to_world.py` PGM→SDF converter (TDD)

Converts a slam_toolbox occupancy map (PGM + YAML) into a Gazebo SDF world of
extruded walls, preserving the map origin/resolution so the world is
co-registered with the source map.

**Files:**
- Create: `ros2_ws/src/rover_sim/rover_sim/map_to_world.py`
- Test: `ros2_ws/src/rover_sim/test/test_map_to_world.py`

- [ ] **Step 1: Write the failing test**

```python
# ros2_ws/src/rover_sim/test/test_map_to_world.py
import xml.etree.ElementTree as ET
from rover_sim.map_to_world import (
    parse_pgm, occupied_grid, merge_row_runs, runs_to_boxes, build_sdf,
)


def _tiny_pgm(tmp_path):
    # 4x3 P5 PGM: a 2-cell horizontal wall (value 0=occupied) on the top row,
    # everything else free (254). Row-major, origin bottom-left after y-flip.
    w, h = 4, 3
    pix = [254] * (w * h)
    pix[0] = 0   # (row0,col0)
    pix[1] = 0   # (row0,col1)
    data = f'P5\n{w} {h}\n255\n'.encode() + bytes(pix)
    p = tmp_path / 'm.pgm'
    p.write_bytes(data)
    return str(p), w, h


def test_parse_pgm(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    gw, gh, pix = parse_pgm(path)
    assert (gw, gh) == (w, h)
    assert len(pix) == w * h
    assert pix[0] == 0 and pix[2] == 254


def test_occupied_grid_thresholds(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    assert grid[0][0] is True and grid[0][1] is True
    assert grid[0][2] is False and grid[1][0] is False


def test_merge_row_runs(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    runs = merge_row_runs(grid)          # list of (row, col_start, length)
    assert (0, 0, 2) in runs
    assert len(runs) == 1


def test_runs_to_boxes_world_coords(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    runs = merge_row_runs(grid)
    boxes = runs_to_boxes(runs, resolution=0.5, origin=(0.0, 0.0), img_h=h)
    assert len(boxes) == 1
    b = boxes[0]
    # 2 cells * 0.5 m wide
    assert abs(b['sx'] - 1.0) < 1e-6
    assert abs(b['sy'] - 0.5) < 1e-6
    # row 0 is the TOP image row -> highest world y after flip
    assert abs(b['y'] - ((h - 0 - 0.5) * 0.5 + 0.0)) < 1e-6


def test_build_sdf_is_valid_xml_with_one_wall(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    boxes = runs_to_boxes(merge_row_runs(grid), 0.5, (0.0, 0.0), h)
    sdf = build_sdf(boxes, wall_height=1.0, world_name='m')
    root = ET.fromstring(sdf)            # raises if malformed
    assert root.tag == 'sdf'
    assert len(root.findall('.//world')) == 1
    # one wall link inside the walls model
    assert len(root.findall(".//model[@name='walls']/link")) == 1
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest src/rover_sim/test/test_map_to_world.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'rover_sim.map_to_world'`.

- [ ] **Step 3: Write the implementation**

```python
# ros2_ws/src/rover_sim/rover_sim/map_to_world.py
#!/usr/bin/env python3
"""Convert a slam_toolbox occupancy map (PGM + YAML) into a Gazebo (Fortress) SDF
world of extruded walls. Preserves the map's origin/resolution so the generated
world is spatially co-registered with the source map (localize on the source
map.yaml directly, no re-mapping)."""
import argparse
import os
import sys

import yaml


def parse_pgm(path):
    """Parse a binary (P5) PGM. Returns (width, height, list_of_int_pixels)."""
    with open(path, 'rb') as f:
        data = f.read()
    assert data[:2] == b'P5', 'only binary P5 PGM supported'
    # tokenize the ASCII header (magic, width, height, maxval), skipping comments
    idx = 2
    tokens = []
    while len(tokens) < 3:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':                  # comment to end of line
            while idx < len(data) and data[idx:idx + 1] != b'\n':
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(int(data[start:idx]))
    width, height, _maxval = tokens
    idx += 1                                            # single whitespace after maxval
    pix = list(data[idx:idx + width * height])
    return width, height, pix


def occupied_grid(pix, w, h, negate, occupied_thresh):
    """Return grid[row][col] = True where the cell is an obstacle.
    Mirrors nav2 map_server semantics: p = (255-v)/255 (negate=0)."""
    grid = [[False] * w for _ in range(h)]
    for r in range(h):
        for c in range(w):
            v = pix[r * w + c]
            p = (v / 255.0) if negate else (255.0 - v) / 255.0
            grid[r][c] = p > occupied_thresh
    return grid


def merge_row_runs(grid):
    """Merge consecutive occupied cells in each row into (row, col_start, length)."""
    runs = []
    for r, row in enumerate(grid):
        c = 0
        w = len(row)
        while c < w:
            if row[c]:
                start = c
                while c < w and row[c]:
                    c += 1
                runs.append((r, start, c - start))
            else:
                c += 1
    return runs


def runs_to_boxes(runs, resolution, origin, img_h):
    """Convert row-runs to world-frame box descriptors.
    Image row 0 is the TOP; map y increases upward, so flip with img_h."""
    ox, oy = origin
    boxes = []
    for (r, c0, length) in runs:
        sx = length * resolution
        sy = resolution
        x = ox + (c0 + length / 2.0) * resolution
        y = oy + (img_h - r - 0.5) * resolution
        boxes.append({'x': x, 'y': y, 'sx': sx, 'sy': sy})
    return boxes


def build_sdf(boxes, wall_height, world_name):
    links = []
    for i, b in enumerate(boxes):
        links.append(f"""      <link name="wall_{i}">
        <pose>{b['x']:.3f} {b['y']:.3f} {wall_height / 2:.3f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{b['sx']:.3f} {b['sy']:.3f} {wall_height:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{b['sx']:.3f} {b['sy']:.3f} {wall_height:.3f}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>""")
    walls = '\n'.join(links)
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{world_name}">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="walls"><static>true</static>
{walls}
    </model>
  </world>
</sdf>
"""


def main(argv=None):
    ap = argparse.ArgumentParser(description='slam_toolbox PGM map -> Gazebo SDF world')
    ap.add_argument('map_yaml', help='path to the map .yaml')
    ap.add_argument('out_world', help='output .world path')
    ap.add_argument('--wall-height', type=float, default=1.0)
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)
    map_dir = os.path.dirname(os.path.abspath(args.map_yaml))
    pgm_path = meta['image'] if os.path.isabs(meta['image']) else os.path.join(map_dir, meta['image'])
    resolution = float(meta['resolution'])
    origin = (float(meta['origin'][0]), float(meta['origin'][1]))
    negate = int(meta.get('negate', 0))
    occ = float(meta.get('occupied_thresh', 0.65))

    w, h, pix = parse_pgm(pgm_path)
    grid = occupied_grid(pix, w, h, negate, occ)
    boxes = runs_to_boxes(merge_row_runs(grid), resolution, origin, h)
    name = os.path.splitext(os.path.basename(args.out_world))[0]
    sdf = build_sdf(boxes, args.wall_height, name)
    with open(args.out_world, 'w') as f:
        f.write(sdf)
    print(f'Wrote {args.out_world}: {len(boxes)} wall boxes from {w}x{h} map')


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `python3 -m pytest src/rover_sim/test/test_map_to_world.py -v`
Expected: 5 passed.

- [ ] **Step 5: Build + confirm `map2world` entry point**

Run: `colcon build --symlink-install --packages-select rover_sim && source install/setup.bash && ros2 pkg executables rover_sim`
Expected: includes `rover_sim cmd_vel_relay` and `rover_sim map2world`.

- [ ] **Step 6: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim/rover_sim/map_to_world.py ros2_ws/src/rover_sim/test/test_map_to_world.py ros2_ws/src/rover_sim/setup.py
git commit -m "feat(rover_sim): add map_to_world PGM->SDF converter"
```

---

## Phase 1 — Worlds

### Task 4: Generate `building.world` from the real map

**Files:**
- Create (generated): `ros2_ws/src/rover_sim/worlds/building.world`

- [ ] **Step 1: Generate the world from the committed map**

Run:
```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
ros2 run rover_sim map2world \
  src/rover_localization/maps/map_April_27_3_52.yaml \
  src/rover_sim/worlds/building.world
```
Expected: `Wrote .../building.world: <N> wall boxes from 1068x878 map` (N in the hundreds).

- [ ] **Step 2: Verify the SDF parses in Gazebo (headless, 3 s) — and confirm a display is available**

Run:
```bash
timeout 12 ign gazebo -s -r --iterations 200 src/rover_sim/worlds/building.world ; echo "exit=$?"
```
Expected: server runs without SDF parse errors and exits (`exit=0` or `124` from timeout). If you see `Unable to find or download file` or XML errors, fix `build_sdf` and regenerate.
NOTE: `-s` runs server-only (no GUI), so this works headless. The GUI (`ign gazebo <world>`) needs a display/GPU — see Task 13 note.

- [ ] **Step 3: Build (so the world installs into share/) and commit**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_sim
cd ~/rover_slam
git add ros2_ws/src/rover_sim/worlds/building.world
git commit -m "feat(rover_sim): add building.world generated from map_April_27_3_52"
```

---

### Task 5: Hand-write the representative `corridor.world` with blue tape

**Files:**
- Create: `ros2_ws/src/rover_sim/worlds/corridor.world`

- [ ] **Step 1: Create the world** (a rectangular ring corridor with four walls + an inner block, plus a blue-tape loop with right-angle corners; tape color RGB≈(0,153,255))

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="corridor">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Outer ring walls (8x6 m room), 1 m tall, 0.1 m thick -->
    <model name="walls"><static>true</static>
      <link name="n"><pose>0  3 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>8 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>8 0.1 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
      <link name="s"><pose>0 -3 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>8 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>8 0.1 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
      <link name="e"><pose>4  0 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>0.1 6 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.1 6 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
      <link name="w"><pose>-4 0 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>0.1 6 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.1 6 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
      <!-- Inner block (2.4x1.4 m) to force a ring corridor -->
      <link name="in_n"><pose>0  0.7 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>2.4 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>2.4 0.1 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
      <link name="in_s"><pose>0 -0.7 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>2.4 0.1 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>2.4 0.1 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
      <link name="in_e"><pose>1.2 0 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>0.1 1.4 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.1 1.4 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
      <link name="in_w"><pose>-1.2 0 0.5 0 0 0</pose>
        <collision name="c"><geometry><box><size>0.1 1.4 1</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.1 1.4 1</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link>
    </model>

    <!-- Blue tape loop (mid-corridor, ~2.5 m radius rectangle), thin flat strips on the floor.
         Right-angle corners. Color RGB (0,153,255) = (0, 0.6, 1). Emissive so the camera sees it. -->
    <model name="blue_tape"><static>true</static>
      <link name="t_n"><pose>0  2.0 0.005 0 0 0</pose>
        <visual name="v"><geometry><box><size>5.0 0.08 0.01</size></box></geometry>
          <material><ambient>0 0.6 1 1</ambient><diffuse>0 0.6 1 1</diffuse><emissive>0 0.45 0.8 1</emissive></material></visual></link>
      <link name="t_s"><pose>0 -2.0 0.005 0 0 0</pose>
        <visual name="v"><geometry><box><size>5.0 0.08 0.01</size></box></geometry>
          <material><ambient>0 0.6 1 1</ambient><diffuse>0 0.6 1 1</diffuse><emissive>0 0.45 0.8 1</emissive></material></visual></link>
      <link name="t_e"><pose>2.5 0 0.005 0 0 0</pose>
        <visual name="v"><geometry><box><size>0.08 4.0 0.01</size></box></geometry>
          <material><ambient>0 0.6 1 1</ambient><diffuse>0 0.6 1 1</diffuse><emissive>0 0.45 0.8 1</emissive></material></visual></link>
      <link name="t_w"><pose>-2.5 0 0.005 0 0 0</pose>
        <visual name="v"><geometry><box><size>0.08 4.0 0.01</size></box></geometry>
          <material><ambient>0 0.6 1 1</ambient><diffuse>0 0.6 1 1</diffuse><emissive>0 0.45 0.8 1</emissive></material></visual></link>
    </model>
  </world>
</sdf>
```

- [ ] **Step 2: Verify it parses (headless server)**

Run: `cd ~/rover_slam/ros2_ws && timeout 12 ign gazebo -s -r --iterations 200 src/rover_sim/worlds/corridor.world ; echo "exit=$?"`
Expected: no SDF/XML parse errors; clean exit.

- [ ] **Step 3: Build + commit**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_sim
cd ~/rover_slam
git add ros2_ws/src/rover_sim/worlds/corridor.world
git commit -m "feat(rover_sim): add representative corridor.world with blue tape loop"
```

---

## Phase 2 — Sim robot description (DiffDrive path)

### Task 6: `gz_sensors.xacro` (lidar + imu + camera)

**Files:**
- Create: `ros2_ws/src/rover_description/urdf/gz_sensors.xacro`

- [ ] **Step 1: Create the sensors xacro** (frame ids match existing links: `laser_frame`, `imu_link`, `camera_link`)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 2D LiDAR -> /scan -->
  <gazebo reference="laser_frame">
    <sensor name="laser" type="gpu_lidar">
      <topic>scan</topic>
      <gz_frame_id>laser_frame</gz_frame_id>
      <update_rate>10</update_rate>
      <always_on>1</always_on>
      <visualize>true</visualize>
      <lidar>
        <scan><horizontal><samples>360</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
        <range><min>0.3</min><max>12.0</max><resolution>0.01</resolution></range>
      </lidar>
    </sensor>
  </gazebo>

  <!-- IMU -> /imu -->
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <topic>imu</topic>
      <gz_frame_id>imu_link</gz_frame_id>
      <update_rate>50</update_rate>
      <always_on>1</always_on>
    </sensor>
  </gazebo>
  <gazebo>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
  </gazebo>

  <!-- Forward camera -> /camera/color/image_raw (sees floor tape in lower FOV) -->
  <gazebo reference="camera_link">
    <sensor name="color_camera" type="camera">
      <topic>camera/color/image_raw</topic>
      <gz_frame_id>camera_link</gz_frame_id>
      <update_rate>20</update_rate>
      <always_on>1</always_on>
      <camera>
        <horizontal_fov>1.089</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>20.0</far></clip>
      </camera>
    </sensor>
  </gazebo>
</robot>
```

- [ ] **Step 2: (No standalone test — verified when the URDF expands in Task 8.)** Proceed.

- [ ] **Step 3: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_description/urdf/gz_sensors.xacro
git commit -m "feat(rover_description): add Fortress gz sensors xacro (lidar/imu/camera)"
```

---

### Task 7: `gz_diff_drive.xacro` + `gz_joint_state.xacro`

**Files:**
- Create: `ros2_ws/src/rover_description/urdf/gz_diff_drive.xacro`
- Create: `ros2_ws/src/rover_description/urdf/gz_joint_state.xacro`

- [ ] **Step 1: Create `gz_diff_drive.xacro`** (drives the two rear wheels; wheel_separation = rear_wheel_track 0.175, wheel_radius 0.034; odom/TF publishing OFF — EKF owns odom; subscribes the relayed topic `/model/rover/cmd_vel`)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <wheel_separation>0.175</wheel_separation>
      <wheel_radius>0.034</wheel_radius>
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <topic>/model/rover/cmd_vel</topic>
      <!-- Odometry/TF intentionally NOT published here; rf2o+fake_odom+EKF own odom. -->
      <odom_publish_frequency>0</odom_publish_frequency>
    </plugin>
  </gazebo>
</robot>
```

- [ ] **Step 2: Create `gz_joint_state.xacro`** (publishes `/joint_states` so robot_state_publisher can emit wheel TFs)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <gazebo>
    <plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher">
      <topic>joint_states</topic>
    </plugin>
  </gazebo>
</robot>
```

- [ ] **Step 3: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_description/urdf/gz_diff_drive.xacro ros2_ws/src/rover_description/urdf/gz_joint_state.xacro
git commit -m "feat(rover_description): add gz diff-drive + joint-state xacros"
```

---

### Task 8: Wire `robot.urdf.xacro` conditionals + fix steering joints + `Select` artifact

**Files:**
- Modify: `ros2_ws/src/rover_description/urdf/robot.urdf.xacro`
- Modify: `ros2_ws/src/rover_description/urdf/robot_core.xacro:54` (Select artifact)
- Modify: `ros2_ws/src/rover_description/urdf/robot_core.xacro:166,198` (steering joint velocity limits)

- [ ] **Step 1: Replace `robot.urdf.xacro` body with the conditional version**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">

    <xacro:arg name="use_ros2_control" default="true" />
    <xacro:arg name="sim_mode" default="false" />
    <xacro:arg name="drive_model" default="diff" />   <!-- diff | ackermann -->

    <xacro:include filename="robot_core.xacro" />

    <xacro:if value="$(arg sim_mode)">
        <xacro:include filename="gz_sensors.xacro" />
        <xacro:include filename="gz_joint_state.xacro" />
        <xacro:if value="${'$(arg drive_model)' == 'ackermann'}">
            <xacro:include filename="gz_ackermann.xacro" />
        </xacro:if>
        <xacro:unless value="${'$(arg drive_model)' == 'ackermann'}">
            <xacro:include filename="gz_diff_drive.xacro" />
        </xacro:unless>
    </xacro:if>

    <xacro:unless value="$(arg sim_mode)">
        <xacro:include filename="ros2_control.xacro" />
        <xacro:include filename="lidar.xacro" />
        <xacro:include filename="imu.xacro" />
        <xacro:include filename="camera.xacro" />
    </xacro:unless>
</robot>
```

- [ ] **Step 2: Fix the stray `Select` text in `robot_core.xacro`**

Find (line ~54): `<!-- BASE_FOOTPRINT LINK -->Select <joint name="base_footprint_joint" type="fixed">`
Replace with: `<!-- BASE_FOOTPRINT LINK --><joint name="base_footprint_joint" type="fixed">`

- [ ] **Step 3: Fix the front steering joints' `velocity="0.0"` so Ackermann can steer**

In `robot_core.xacro`, both `front_right_wheel_joint` and `front_left_wheel_joint` have:
`<limit lower="..." upper="..." effort="100.0" velocity="0.0" />`
Change `velocity="0.0"` → `velocity="2.0"` on both (leaves diff-drive unaffected; enables Ackermann).

- [ ] **Step 4: Verify the URDF expands in BOTH real and sim/diff modes**

Run:
```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
URDF=src/rover_description/urdf/robot.urdf.xacro
xacro $URDF > /tmp/real.urdf && echo "REAL ok"
xacro $URDF sim_mode:=true drive_model:=diff > /tmp/sim_diff.urdf && echo "SIM/diff ok"
grep -c 'diff-drive-system' /tmp/sim_diff.urdf      # expect 1
grep -c 'gpu_lidar' /tmp/sim_diff.urdf              # expect 1
grep -c 'AckermannArduinoHardware' /tmp/sim_diff.urdf  # expect 0 (real hw excluded in sim)
grep -c 'AckermannArduinoHardware' /tmp/real.urdf      # expect 1 (present in real)
```
Expected: both `xacro` calls succeed; counts are 1/1/0/1 respectively.

- [ ] **Step 5: Build + commit**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_description
cd ~/rover_slam
git add ros2_ws/src/rover_description/urdf/robot.urdf.xacro ros2_ws/src/rover_description/urdf/robot_core.xacro
git commit -m "feat(rover_description): gate sim_mode/drive_model in URDF; fix steering joints + Select artifact"
```

---

## Phase 3 — Bring-up (P0: spawn + bridge + teleop, DiffDrive)

### Task 9: `config/bridge.yaml`

**Files:**
- Create: `ros2_ws/src/rover_sim/config/bridge.yaml`

- [ ] **Step 1: Create the bridge config** (gz Fortress types are `ignition.msgs.*`)

```yaml
# ros_gz_bridge parameter_bridge config (Fortress)
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "ignition.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "ignition.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "ignition.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/color/image_raw"
  gz_topic_name: "/camera/color/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "ignition.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/model/rover/cmd_vel"
  gz_topic_name: "/model/rover/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "ignition.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/default/model/rover/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "ignition.msgs.Model"
  direction: GZ_TO_ROS
```

- [ ] **Step 2: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim/config/bridge.yaml
git commit -m "feat(rover_sim): add ros_gz bridge config (Fortress)"
```

---

### Task 10: `spawn.launch.py` (the backend:=sim source layer)

**Files:**
- Create: `ros2_ws/src/rover_sim/launch/spawn.launch.py`

- [ ] **Step 1: Create the launch file**

```python
#!/usr/bin/env python3
"""backend:=sim source layer: Gazebo Fortress + robot_state_publisher + spawn +
ros_gz bridge + cmd_vel relay. Produces /scan,/imu,/camera/color/image_raw and
moves the robot on /cmd_vel. Args: drive (diff|ackermann), world (file path)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rover_sim = get_package_share_directory('rover_sim')
    rover_desc = get_package_share_directory('rover_description')
    default_world = os.path.join(rover_sim, 'worlds', 'corridor.world')

    drive = LaunchConfiguration('drive')
    world = LaunchConfiguration('world')

    urdf_xacro = os.path.join(rover_desc, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro, ' sim_mode:=true', ' drive_model:=', drive]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [world, ' -r']}.items(),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.1'],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(rover_sim, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )

    relay = Node(
        package='rover_sim', executable='cmd_vel_relay', output='screen',
        parameters=[{'output_topic': '/model/rover/cmd_vel', 'use_sim_time': True}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                               os.path.join(rover_sim, 'worlds')),
        DeclareLaunchArgument('drive', default_value='diff', choices=['diff', 'ackermann']),
        DeclareLaunchArgument('world', default_value=default_world),
        gz_sim, rsp, spawn, bridge, relay,
    ])
```

- [ ] **Step 2: Build + verify the launch parses (`--show-args` does not start Gazebo)**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_sim && source install/setup.bash
ros2 launch rover_sim spawn.launch.py --show-args
```
Expected: prints args `drive`, `world`; no Python import/syntax errors.

- [ ] **Step 3: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim/launch/spawn.launch.py
git commit -m "feat(rover_sim): add spawn.launch.py (Gazebo + bridge + RSP + relay)"
```

---

### Task 11: `rviz/sim.rviz`

**Files:**
- Create: `ros2_ws/src/rover_sim/rviz/sim.rviz`

- [ ] **Step 1: Create a base config, then refine in the GUI.** Start from the nav2 default and add displays. Run (needs display):

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
In RViz set **Fixed Frame = map** and add these displays, then **File → Save Config As** →
`~/rover_slam/ros2_ws/src/rover_sim/rviz/sim.rviz`:
- TF
- RobotModel (Description Topic: `/robot_description`)
- LaserScan (Topic: `/scan`)
- Map (Topic: `/map`)
- Path (Topic: `/plan`)
- Camera (Topic: `/camera/color/image_raw`)
- Tools: ensure "2D Goal Pose" publishes to `/goal_pose`.

If running headless and you cannot open the GUI, create the file with this minimal working config instead:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Global Options:
    Fixed Frame: map
  Displays:
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
    - Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: RobotModel
      Description Topic:
        Value: /robot_description
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan
      Topic:
        Value: /scan
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map
      Topic:
        Value: /map
    - Class: rviz_default_plugins/Path
      Enabled: true
      Name: Path
      Topic:
        Value: /plan
    - Class: rviz_default_plugins/Camera
      Enabled: true
      Name: Camera
      Topic:
        Value: /camera/color/image_raw
  Tools:
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Value: /goal_pose
```

- [ ] **Step 2: Build + commit**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_sim
cd ~/rover_slam
git add ros2_ws/src/rover_sim/rviz/sim.rviz
git commit -m "feat(rover_sim): add sim RViz config"
```

---

### Task 12: Wire `backend.launch.py` `sim` branch

**Files:**
- Modify: `ros2_ws/src/rover_bringup/launch/backend.launch.py`

- [ ] **Step 1: Add a `_sim()` condition and include `rover_sim/spawn.launch.py`.** Insert after the `_real()` helper:

```python
def _sim():
    return IfCondition(PythonExpression(["'", LaunchConfiguration("backend"), "' == 'sim'"]))
```

Add these args + the include inside `generate_launch_description()` (alongside the real sources), and gate the real-only static TFs with `condition=_real()`:

```python
    sim_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rover_sim"), "launch", "spawn.launch.py"])),
        launch_arguments={"drive": LaunchConfiguration("drive"),
                          "world": LaunchConfiguration("world")}.items(),
        condition=_sim(),
    )
```

Add the new launch args (near the `backend` arg):

```python
        DeclareLaunchArgument("drive", default_value="diff", choices=["diff", "ackermann"]),
        DeclareLaunchArgument("world", default_value=""),
```

Add `tf_laser`/`tf_imu` `condition=_real()` (sim gets these from robot_state_publisher), and append `sim_backend` to the returned `LaunchDescription` list.

- [ ] **Step 2: Build + verify both backends parse**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_bringup && source install/setup.bash
ros2 launch rover_bringup backend.launch.py --show-args
```
Expected: args now include `backend`, `drive`, `world`; no errors.

- [ ] **Step 3: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_bringup/launch/backend.launch.py
git commit -m "feat(rover_bringup): wire backend:=sim to rover_sim spawn layer"
```

---

### Task 13: P0 runtime verification (bring-up + teleop)

**Files:** none (verification only).

> **Display note:** Gazebo's GUI + `gpu_lidar`/camera rendering need a display/GPU.
> If this machine is headless, run the gz server headless (the launch uses `-r`; add
> `-s` by setting `world` through a server-only path or run `ign gazebo -s`) and use
> RViz over X-forwarding/VNC, or install `xvfb` and prefix with `xvfb-run -a`.
> Confirm the display situation before this task; record the working invocation.

- [ ] **Step 1: Launch the sim source layer (DiffDrive)**

Terminal A:
```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
ros2 launch rover_sim spawn.launch.py drive:=diff
```
Expected: Gazebo starts with `corridor.world`, the rover spawns.

- [ ] **Step 2: Verify the topic contract publishes**

Terminal B:
```bash
source ~/rover_slam/ros2_ws/install/setup.bash
ros2 topic hz /scan & ros2 topic hz /imu & ros2 topic hz /camera/color/image_raw &
sleep 6; kill %1 %2 %3
ros2 topic echo /clock --once
```
Expected: `/scan` ~10 Hz, `/imu` ~50 Hz, `/camera/color/image_raw` ~20 Hz; `/clock` has increasing time.

- [ ] **Step 3: Drive with keyboard teleop (remapped to /cmd_vel_teleop)**

Terminal C:
```bash
source ~/rover_slam/ros2_ws/install/setup.bash
ros2 run rover_teleop keyboard_teleop --ros-args -r cmd_vel:=/cmd_vel_teleop
```
Press the arrow keys; expected: the rover moves in Gazebo. Verify `ros2 topic echo /model/rover/cmd_vel --once` shows the relayed Twist.

- [ ] **Step 4: Record the result.** If all pass, P0 is green for DiffDrive. No commit (verification only) — note any tuning in the commit message of the next code change.

---

## Phase 4 — Mapping (P1: line-follower + SLAM, DiffDrive)

### Task 14: `sim.launch.py` — mapping mode

**Files:**
- Create: `ros2_ws/src/rover_sim/launch/sim.launch.py`

- [ ] **Step 1: Create the launch file (mapping branch first; planning branch added in Task 16)**

```python
#!/usr/bin/env python3
"""Top-level sim entry — the sim analog of rover_bringup/slam_nav.launch.py.
Swaps the hardware layer for Gazebo and sets use_sim_time:=true everywhere.

  ros2 launch rover_sim sim.launch.py mode:=mapping drive:=diff world:=<file> driver:=line_follower
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, TimerAction, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    rover_sim = get_package_share_directory('rover_sim')
    mode = LaunchConfiguration('mode').perform(context)
    driver = LaunchConfiguration('driver').perform(context)
    world = LaunchConfiguration('world').perform(context) or \
        os.path.join(rover_sim, 'worlds', 'corridor.world')
    drive = LaunchConfiguration('drive')

    sim_time = {'use_sim_time': True}
    ekf_config = os.path.join(get_package_share_directory('rover_state_estimation'),
                              'config', 'ekf.yaml')

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rover_sim'), 'launch', 'spawn.launch.py'])),
        launch_arguments={'drive': drive, 'world': world}.items(),
    )

    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='log', ros_arguments=['--log-level', 'FATAL'],
        parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/rf2o_odom',
                     'publish_tf': False, 'base_frame_id': 'base_link',
                     'odom_frame_id': 'odom', 'freq': 10.0, **sim_time}],
    )
    init_rf2o = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/base_pose_ground_truth',
             'nav_msgs/msg/Odometry',
             '{header: {frame_id: odom}, pose: {pose: {orientation: {w: 1.0}}}}'],
        output='log')])
    fake_odom = Node(
        package='rover_tools', executable='fake_odom', name='fake_odom', output='screen',
        parameters=[{'publish_tf': False, **sim_time}], remappings=[('odom', '/fake_odom')])
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_config, sim_time],
        remappings=[('odometry/filtered', '/odom')])

    nodes = [spawn, rf2o, init_rf2o, fake_odom, ekf]

    if mode == 'mapping':
        slam = Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                        'config', 'mapper_params_online_async.yaml']),
                        {'scan_topic': '/scan', 'base_frame': 'base_link',
                         'odom_frame': 'odom', 'map_frame': 'map',
                         'provide_odom_frame': False,
                         'minimum_travel_distance': 0.2,
                         'minimum_travel_heading': 0.2, **sim_time}])
        nodes.append(TimerAction(period=3.0, actions=[slam]))

    if driver == 'line_follower':
        nodes.append(Node(package='rover_behaviors', executable='line_follower',
                          name='line_follower', output='screen', parameters=[sim_time]))
    elif driver == 'teleop':
        nodes.append(Node(package='rover_teleop', executable='keyboard_teleop',
                          name='rover_teleop', output='screen',
                          remappings=[('cmd_vel', '/cmd_vel_teleop')]))

    nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                      arguments=['-d', os.path.join(rover_sim, 'rviz', 'sim.rviz')],
                      parameters=[sim_time]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'planning']),
        DeclareLaunchArgument('drive', default_value='diff', choices=['diff', 'ackermann']),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('driver', default_value='line_follower',
                              choices=['line_follower', 'teleop', 'none']),
        DeclareLaunchArgument('map', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
```

- [ ] **Step 2: Build + verify it parses**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_sim && source install/setup.bash
ros2 launch rover_sim sim.launch.py mode:=mapping --show-args
```
Expected: args `mode`, `drive`, `world`, `driver`, `map`; no errors.

- [ ] **Step 3: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim/launch/sim.launch.py
git commit -m "feat(rover_sim): add sim.launch.py mapping mode"
```

---

### Task 15: P1 runtime verification (line-follower mapping + save)

**Files:** none (verification + possible tuning).

- [ ] **Step 1: Launch mapping with the line-follower in the blue-tape corridor**

```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
ros2 launch rover_sim sim.launch.py mode:=mapping drive:=diff world:=$(ros2 pkg prefix rover_sim)/share/rover_sim/worlds/corridor.world driver:=line_follower
```
Expected: the rover follows the blue tape; in RViz the `/map` grows; TF `map→odom→base_link` is connected.

- [ ] **Step 2: If the rover does not track the tape, tune `line_follower`** (camera color/threshold or gains). Quick checks:

```bash
ros2 topic echo /camera/color/image_raw --once | head    # confirm images arrive
ros2 param set /line_follower angular_velocity 1.2        # reduce if it over-rotates (default 1.9)
```
If the blue is not detected, adjust the tape material in `corridor.world` toward RGB (0,153,255) (i.e. diffuse `0 0.6 1 1`) and rebuild. Commit any world/param tuning:
```bash
git add -A && git commit -m "fix(rover_sim): tune blue tape color / line_follower for sim"
```

- [ ] **Step 3: Save the map once a full loop is mapped**

```bash
ros2 run rover_localization save_map --ros-args -p map_name:=/tmp/sim_corridor   # adjust to save_map's actual arg
ls -la /tmp/sim_corridor.yaml /tmp/sim_corridor.pgm
```
Expected: a `.yaml` + `.pgm` are written; opening the pgm shows the ring corridor.
(If `save_map`'s parameters differ, inspect `ros2 run rover_localization save_map --ros-args --help` or its source `rover_localization/rover_localization/save_map.py`.)

- [ ] **Step 4: Record results.** P1 green for DiffDrive when a corridor map saves successfully.

---

## Phase 5 — Planning + execution (P2, DiffDrive)

### Task 16: `sim.launch.py` — planning mode

**Files:**
- Modify: `ros2_ws/src/rover_sim/launch/sim.launch.py`

- [ ] **Step 1: Add the planning branch.** After the `if mode == 'mapping':` block, add:

```python
    if mode == 'planning':
        map_yaml = LaunchConfiguration('map').perform(context)
        if not map_yaml:
            map_yaml = os.path.join(
                get_package_share_directory('rover_localization'),
                'maps', 'map_April_27_3_52.yaml')
        amcl_config = os.path.join(
            get_package_share_directory('rover_localization'), 'config', 'amcl.yaml')
        map_server = Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{'yaml_filename': map_yaml, **sim_time}])
        amcl = Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[amcl_config, sim_time])
        lifecycle = Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'autostart': True,
                         'node_names': ['map_server', 'amcl'], **sim_time}])
        planner = Node(package='rover_navigation', executable='astar_planner',
                       name='astar_planner', output='screen', parameters=[sim_time])
        pursuit = Node(package='rover_navigation', executable='pure_pursuit',
                       name='pure_pursuit', output='screen',
                       parameters=[{'use_sim': False, **sim_time}])
        nodes += [map_server, amcl, TimerAction(period=3.0, actions=[lifecycle]),
                  TimerAction(period=10.0, actions=[planner, pursuit])]
```

> Note: `pure_pursuit` runs with `use_sim:=False` — Gazebo + amcl + EKF provide the real `map→base_link` TF, so its internal kinematic fake must stay off.

- [ ] **Step 2: Build + verify planning mode parses**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_sim && source install/setup.bash
ros2 launch rover_sim sim.launch.py mode:=planning --show-args
```
Expected: parses with no error.

- [ ] **Step 3: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim/launch/sim.launch.py
git commit -m "feat(rover_sim): add sim.launch.py planning mode (map_server+amcl+A*+pursuit)"
```

---

### Task 17: P2 runtime verification (localize → goal → plan → drive)

**Files:** none (verification).

- [ ] **Step 1: Launch planning on the co-registered building world + its source map**

```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
WORLD=$(ros2 pkg prefix rover_sim)/share/rover_sim/worlds/building.world
MAP=src/rover_localization/maps/map_April_27_3_52.yaml
ros2 launch rover_sim sim.launch.py mode:=planning drive:=diff world:=$WORLD map:=$MAP
```
Expected: `/map` shows in RViz; amcl localizes (TF `map→odom` appears); the robot model sits on the map.

- [ ] **Step 2: Send a goal and confirm the pipeline**

In RViz, click **2D Goal Pose** in a free area. Then:
```bash
ros2 topic echo /plan --once | head        # A* published a Path
```
Expected: `astar_planner` logs `Published path with N waypoints`; `/plan` shows in RViz; `pure_pursuit` logs pose/targets and the rover drives toward the goal, stopping within 0.25 m.

- [ ] **Step 3: Record results.** P2 green for DiffDrive when the rover reaches a clicked goal.

---

## Phase 6 — Ackermann drive module

### Task 18: `gz_ackermann.xacro`

**Files:**
- Create: `ros2_ws/src/rover_description/urdf/gz_ackermann.xacro`

- [ ] **Step 1: Create the Ackermann steering xacro** (rear wheels drive, front wheels steer; uses wheel_base 0.172, track 0.175; subscribes the relayed `/model/rover/cmd_vel`; no odom/TF)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <gazebo>
    <plugin filename="ignition-gazebo-ackermann-steering-system" name="ignition::gazebo::systems::AckermannSteering">
      <left_joint>rear_left_wheel_joint</left_joint>
      <right_joint>rear_right_wheel_joint</right_joint>
      <left_steering_joint>front_left_wheel_joint</left_steering_joint>
      <right_steering_joint>front_right_wheel_joint</right_steering_joint>
      <kingpin_width>0.175</kingpin_width>
      <steering_limit>0.45</steering_limit>
      <wheel_base>0.172</wheel_base>
      <wheel_separation>0.175</wheel_separation>
      <wheel_radius>0.034</wheel_radius>
      <min_velocity>-1.0</min_velocity>
      <max_velocity>1.0</max_velocity>
      <topic>/model/rover/cmd_vel</topic>
      <odom_publish_frequency>0</odom_publish_frequency>
    </plugin>
  </gazebo>
</robot>
```

- [ ] **Step 2: Verify the URDF expands in ackermann mode**

```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
xacro src/rover_description/urdf/robot.urdf.xacro sim_mode:=true drive_model:=ackermann > /tmp/sim_ack.urdf && echo ok
grep -c 'ackermann-steering-system' /tmp/sim_ack.urdf   # expect 1
grep -c 'diff-drive-system' /tmp/sim_ack.urdf           # expect 0
```
Expected: `ok`; counts 1 / 0.

- [ ] **Step 3: Build + commit**

```bash
cd ~/rover_slam/ros2_ws && colcon build --symlink-install --packages-select rover_description
cd ~/rover_slam
git add ros2_ws/src/rover_description/urdf/gz_ackermann.xacro
git commit -m "feat(rover_description): add gz Ackermann steering xacro"
```

---

### Task 19: Ackermann runtime verification (P0→P2 with drive:=ackermann)

**Files:** none (verification + tuning).

- [ ] **Step 1: P0 bring-up with Ackermann**

```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
ros2 launch rover_sim spawn.launch.py drive:=ackermann
# Terminal B: drive it
ros2 run rover_teleop keyboard_teleop --ros-args -r cmd_vel:=/cmd_vel_teleop
```
Expected: the front wheels visibly steer; the rover drives. If it does not steer, confirm the steering-joint velocity fix (Task 8 Step 3) and the joint names in `gz_ackermann.xacro`.

- [ ] **Step 2: P1 mapping with Ackermann + line-follower**

```bash
ros2 launch rover_sim sim.launch.py mode:=mapping drive:=ackermann world:=$(ros2 pkg prefix rover_sim)/share/rover_sim/worlds/corridor.world driver:=line_follower
```
Expected: maps the corridor. Ackermann turning radius is limited, so the tape loop must be wide enough (the `corridor.world` 5×4 m loop clears the ~0.4 m min radius). Tune `steering_limit` if turns are too tight.

- [ ] **Step 3: P2 planning + execution with Ackermann**

```bash
ros2 launch rover_sim sim.launch.py mode:=planning drive:=ackermann world:=$(ros2 pkg prefix rover_sim)/share/rover_sim/worlds/building.world map:=src/rover_localization/maps/map_April_27_3_52.yaml
```
Expected: rover reaches a clicked goal. `pure_pursuit`'s bicycle model matches Ackermann well; if it weaves, raise `LOOKAHEAD_DISTANCE` in `rover_navigation/pure_pursuit.py`.

- [ ] **Step 4: Final commit (docs)** — update `CLAUDE.md` Run section with the sim commands:

```bash
cd ~/rover_slam
# add a "Simulation (Gazebo Fortress)" subsection documenting:
#   ros2 launch rover_sim sim.launch.py mode:=mapping  drive:=diff|ackermann driver:=line_follower|teleop
#   ros2 launch rover_sim sim.launch.py mode:=planning drive:=diff|ackermann map:=<yaml>
git add CLAUDE.md
git commit -m "docs: document Gazebo Fortress sim backend usage"
```

---

## Self-Review (completed during authoring)

- **Spec coverage:** package layout (T1), cmd_vel relay (T2), map→world converter (T3),
  both worlds (T4–T5), sensors (T6), diff-drive (T7), URDF gating + fixes (T8), bridge (T9),
  spawn layer (T10), RViz (T11), backend wiring (T12), P0 (T13), mapping launch + P1 (T14–T15),
  planning launch + P2 (T16–T17), Ackermann module + verification (T18–T19). ✅
- **Placeholders:** none — every code step has complete content; verification steps have exact commands + expected output.
- **Type/name consistency:** relay output topic `/model/rover/cmd_vel` is used identically in
  T2 (default param), T7/T18 (`<topic>`), T9 (bridge `ROS_TO_GZ`), and T10 (relay param).
  Frame ids (`laser_frame`, `imu_link`, `camera_link`) match the existing URDF links.
  `sim_mode`/`drive_model` args are consistent T8 ↔ T10.
- **Known runtime-dependent items (flagged in-task, not placeholders):** `save_map` argument
  name (T15 Step 3), exact headless display invocation (T13 note), Ackermann steering tuning
  (T19) — each has a concrete discovery/verification step.
