# ROSbot 2R Model Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the Husarion ROSbot 2R as a swappable `model:=rosbot` in the existing Gazebo Fortress sim, driven by our gz `DiffDrive` + sensors on our topic contract, feeding the unchanged rf2o/EKF/slam/nav pipeline.

**Architecture:** Vendor **only `rosbot_description`** (trimmed to the 2R variant). A thin wrapper `rosbot_sim.urdf.xacro` includes the ROSbot's `body` + `wheel` sub-xacros (real chassis/wheels/meshes) — **bypassing the `rosbot` macro's bundled control stack** — and adds our gz `DiffDrive` (on `fl/fr/rl/rr_wheel_joint`) + our `gpu_lidar`/`imu`/`camera` sensors at the ROSbot frames (`cover_link`/`imu_link`/`camera_mount_link`), publishing `/scan`,`/imu`,`/camera/color/image_raw`. `rover_sim` gains `model:=rover|rosbot`; world, bridge, relay, RViz, and the whole downstream pipeline are unchanged.

**Tech Stack:** ROS 2 Humble, Gazebo Fortress, xacro, ament_cmake (vendored description), `ros_gz`.

**Reference spec:** `docs/superpowers/specs/2026-05-21-rosbot-model-integration-design.md`

**Deviation from spec (simplification):** the spec planned to vendor `husarion_components_description` too. Inspection showed the ROSbot `body`/`wheel` sub-xacros reference **only `rosbot_description`** (one internal `vl53lox` component), so by adding our own sensors we **do not need `husarion_components_description`** — fewer vendored repos, less bloat.

**Verified ROSbot 2R facts:** wheel joints `fl_wheel_joint,fr_wheel_joint,rl_wheel_joint,rr_wheel_joint` (continuous, axis `0 1 0`); frames `base_link,body_link,cover_link,imu_link,camera_mount_link`; non-mecanum `wheel_radius=0.0425`, `wheel_separation_y=0.192`. Source clone at `/tmp/rosbot_probe/rosbot_ros` (re-clone if gone: `git clone --depth 1 -b humble https://github.com/husarion/rosbot_ros.git`).

**Conventions:** run from `~/rover_slam/ros2_ws`; `source /opt/ros/humble/setup.bash` then `install/setup.bash`; build with `MAKEFLAGS="-j1" colcon build --symlink-install --parallel-workers 1`. Branch: `rosbot-model`.

---

### Task 1: Vendor `rosbot_description` (2R only)

**Files:**
- Create: `ros2_ws/src/vendor/rosbot_description/` (copied)
- Modify: `ros2_ws/src/vendor/rosbot_description/package.xml`

- [ ] **Step 1: Copy the package and strip its history / trim the XL variant**

```bash
SRC=/tmp/rosbot_probe/rosbot_ros/rosbot_description
DEST=/home/the2xman/rover_slam/ros2_ws/src/vendor/rosbot_description
[ -d "$SRC" ] || (cd /tmp && rm -rf rosbot_probe && mkdir rosbot_probe && cd rosbot_probe && git clone --depth 1 -b humble https://github.com/husarion/rosbot_ros.git)
cp -a "$SRC" "$DEST"
rm -rf "$DEST/.git"
# trim the ROSbot XL variant (keep only 2R) to cut ~half the mesh bloat
rm -rf "$DEST/urdf/rosbot_xl" "$DEST/config/rosbot_xl" "$DEST/meshes/rosbot_xl"
du -sh "$DEST" "$DEST/meshes"
```
Expected: package copied, no `.git`, XL dirs gone.

- [ ] **Step 2: Remove unused exec_depends so colcon can build it standalone**

Edit `ros2_ws/src/vendor/rosbot_description/package.xml` — delete these lines (we bypass the macro/manipulator, so they are unused and would block the build):

```xml
  <exec_depend>husarion_components_description</exec_depend>
  <exec_depend>open_manipulator_x_description</exec_depend>
  <exec_depend condition="$HUSARION_ROS_BUILD_TYPE == hardware">rosbot_hardware_interfaces</exec_depend>
```

- [ ] **Step 3: Build the vendored package**

Run:
```bash
cd ~/rover_slam/ros2_ws && source /opt/ros/humble/setup.bash
MAKEFLAGS="-j1" colcon build --symlink-install --packages-select rosbot_description 2>&1 | tail -4
```
Expected: `Finished <<< rosbot_description`. If it fails on a missing dep, remove that exec_depend line too (only `ament_cmake`, `xacro`, `robot_state_publisher`, `joint_state_publisher`, `rviz2`, `launch`, `launch_ros` are needed).

- [ ] **Step 4: Confirm the 2R body/wheel xacros expand standalone**

```bash
source install/setup.bash
cat > /tmp/probe.xacro <<'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="p">
  <xacro:include filename="$(find rosbot_description)/urdf/rosbot/body.urdf.xacro" ns="body"/>
  <xacro:include filename="$(find rosbot_description)/urdf/rosbot/wheel.urdf.xacro" ns="wheel"/>
  <xacro:body.body wheel_radius="0.0425" namespace=""/>
  <xacro:wheel.wheel mecanum="false" side="fl" wheel_radius="0.0425"/>
  <xacro:wheel.wheel mecanum="false" side="fr" wheel_radius="0.0425"/>
  <xacro:wheel.wheel mecanum="false" side="rl" wheel_radius="0.0425"/>
  <xacro:wheel.wheel mecanum="false" side="rr" wheel_radius="0.0425"/>
</robot>
EOF
xacro /tmp/probe.xacro > /tmp/probe.urdf && echo OK
grep -c '_wheel_joint' /tmp/probe.urdf      # expect 4
grep -cE 'base_link|cover_link|imu_link|camera_mount_link' /tmp/probe.urdf  # expect >=4
```
Expected: `OK`, 4 wheel joints, the frames present. If `$(find rosbot_description)/urdf/rosbot/components/vl53lox.urdf.xacro` errors, confirm `urdf/rosbot/components/` survived the trim (it must — only `rosbot_xl` was removed).

- [ ] **Step 5: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/vendor/rosbot_description
git commit -m "feat(vendor): add Husarion rosbot_description (2R only, manipulator/components deps dropped)"
```

---

### Task 2: ROSbot sim wrapper xacro

**Files:**
- Create: `ros2_ws/src/rover_description/urdf/rosbot_sim.urdf.xacro`

- [ ] **Step 1: Create the wrapper** (ROSbot 2R geometry + our DiffDrive + our sensors on the contract topics)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rosbot">

  <!-- ROSbot 2R geometry (body + 4 wheels), bypassing the rosbot macro's control stack -->
  <xacro:include filename="$(find rosbot_description)/urdf/rosbot/body.urdf.xacro" ns="body"/>
  <xacro:include filename="$(find rosbot_description)/urdf/rosbot/wheel.urdf.xacro" ns="wheel"/>
  <xacro:property name="wr" value="0.0425"/>
  <xacro:body.body wheel_radius="${wr}" namespace=""/>
  <xacro:wheel.wheel mecanum="false" side="fl" wheel_radius="${wr}"/>
  <xacro:wheel.wheel mecanum="false" side="fr" wheel_radius="${wr}"/>
  <xacro:wheel.wheel mecanum="false" side="rl" wheel_radius="${wr}"/>
  <xacro:wheel.wheel mecanum="false" side="rr" wheel_radius="${wr}"/>

  <!-- LiDAR frame on top cover -> /scan -->
  <joint name="laser_joint" type="fixed">
    <parent link="cover_link"/><child link="laser_frame"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
  </joint>
  <link name="laser_frame"/>
  <gazebo reference="laser_frame">
    <sensor name="laser" type="gpu_lidar">
      <topic>scan</topic><gz_frame_id>laser_frame</gz_frame_id>
      <update_rate>10</update_rate><always_on>1</always_on>
      <lidar><scan><horizontal><samples>360</samples><resolution>1</resolution>
        <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
        <range><min>0.3</min><max>12.0</max><resolution>0.01</resolution></range></lidar>
    </sensor>
  </gazebo>

  <!-- IMU -> /imu (imu_link exists in body.urdf.xacro) -->
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu"><topic>imu</topic><gz_frame_id>imu_link</gz_frame_id>
      <update_rate>50</update_rate><always_on>1</always_on></sensor>
  </gazebo>
  <gazebo><plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/></gazebo>

  <!-- Camera -> /camera/color/image_raw (camera_mount_link exists in body.urdf.xacro) -->
  <gazebo reference="camera_mount_link">
    <sensor name="color_camera" type="camera"><topic>camera/color/image_raw</topic>
      <gz_frame_id>camera_mount_link</gz_frame_id><update_rate>20</update_rate><always_on>1</always_on>
      <camera><horizontal_fov>1.089</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>20.0</far></clip></camera>
    </sensor>
  </gazebo>

  <!-- Our DiffDrive on all 4 wheels (left = fl+rl, right = fr+rr) -->
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>fl_wheel_joint</left_joint><left_joint>rl_wheel_joint</left_joint>
      <right_joint>fr_wheel_joint</right_joint><right_joint>rr_wheel_joint</right_joint>
      <wheel_separation>0.192</wheel_separation><wheel_radius>0.0425</wheel_radius>
      <topic>/model/rover/cmd_vel</topic><odom_publish_frequency>0</odom_publish_frequency>
    </plugin>
    <plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher">
      <topic>joint_states</topic>
    </plugin>
  </gazebo>
</robot>
```

- [ ] **Step 2: Verify it expands and is control-stack-free**

```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
xacro src/rover_description/urdf/rosbot_sim.urdf.xacro > /tmp/rosbot.urdf && echo OK
grep -c 'diff-drive-system' /tmp/rosbot.urdf   # expect 1
grep -c 'gpu_lidar' /tmp/rosbot.urdf           # expect 1
grep -c '_wheel_joint' /tmp/rosbot.urdf        # expect 4 (continuous)
grep -c 'gz_ros2_control\|ros2_control' /tmp/rosbot.urdf  # expect 0 (no control stack)
```
Expected: `OK`; counts 1/1/4/0.

- [ ] **Step 3: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_description/urdf/rosbot_sim.urdf.xacro
git commit -m "feat(rover_description): add rosbot_sim wrapper (ROSbot 2R geometry + our DiffDrive/sensors)"
```

---

### Task 3: `model:=rover|rosbot` swap in `rover_sim`

**Files:**
- Modify: `ros2_ws/src/rover_sim/launch/spawn.launch.py`
- Modify: `ros2_ws/src/rover_sim/launch/sim.launch.py`
- Modify: `ros2_ws/src/rover_description/package.xml`
- Modify: `ros2_ws/src/rover_sim/package.xml`

- [ ] **Step 1: Add `model` selection in `spawn.launch.py`.** In `launch_setup`, replace the `urdf_xacro`/`robot_description` block with:

```python
    model = LaunchConfiguration('model').perform(context)
    if model == 'rosbot':
        urdf_xacro = os.path.join(rover_desc, 'urdf', 'rosbot_sim.urdf.xacro')
        robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)
    else:
        urdf_xacro = os.path.join(rover_desc, 'urdf', 'robot.urdf.xacro')
        robot_description = ParameterValue(
            Command(['xacro ', urdf_xacro, ' sim_mode:=true', ' drive_model:=', drive]),
            value_type=str)
```

And add the arg to `generate_launch_description()`:

```python
        DeclareLaunchArgument('model', default_value='rover', choices=['rover', 'rosbot']),
```

- [ ] **Step 2: Pass `model` through `sim.launch.py`.** In its `spawn` include `launch_arguments`, add `'model': LaunchConfiguration('model')`; and add to `generate_launch_description()`:

```python
        DeclareLaunchArgument('model', default_value='rover', choices=['rover', 'rosbot']),
```

- [ ] **Step 3: Declare the dependency.** In `ros2_ws/src/rover_description/package.xml` add (rover_description's wrapper includes rosbot_description):

```xml
  <exec_depend>rosbot_description</exec_depend>
```

(rover_sim already depends on rover_description, so it transitively reaches rosbot_description; no rover_sim manifest change needed.)

- [ ] **Step 4: Build + verify both models' args parse**

```bash
cd ~/rover_slam/ros2_ws && MAKEFLAGS="-j1" colcon build --symlink-install --packages-select rosbot_description rover_description rover_sim 2>&1 | tail -3
source install/setup.bash
ros2 launch rover_sim spawn.launch.py --show-args 2>&1 | grep -A1 "'model'"
ros2 launch rover_sim sim.launch.py mode:=mapping --show-args 2>&1 | grep -A1 "'model'"
```
Expected: `model` arg with choices `['rover','rosbot']` in both.

- [ ] **Step 5: Commit**

```bash
cd ~/rover_slam
git add ros2_ws/src/rover_sim/launch/spawn.launch.py ros2_ws/src/rover_sim/launch/sim.launch.py ros2_ws/src/rover_description/package.xml
git commit -m "feat(rover_sim): add model:=rover|rosbot swap"
```

---

### Task 4: Runtime verification (`model:=rosbot`)

**Files:** none (verification).

- [ ] **Step 1: Headless spawn — sensors publish + robot drives**

```bash
cd ~/rover_slam/ros2_ws && source install/setup.bash
ros2 launch rover_sim spawn.launch.py headless:=true model:=rosbot > /tmp/rosbot_p0.log 2>&1 &
sleep 28
for t in /scan /imu /camera/color/image_raw; do echo -n "$t: "; timeout 7 ros2 topic hz $t 2>&1 | grep -m1 -E 'average rate|no new'; done
# drive
timeout 6 ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.3}}" >/dev/null 2>&1
echo "moved? check gz pose:"; ign model -m rover -p 2>/dev/null | head -3 || ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -m1 Translation
pkill -INT -f 'ros2 launch rover_sim'; sleep 5; pkill -9 -f 'ign gazebo'; pkill -9 -f 'ruby.*ign'; pkill -9 -f parameter_bridge
```
Expected: `/scan` ~10 Hz, `/imu` ~50 Hz, `/camera` ~20 Hz; robot moves on cmd_vel. If the camera faces the wrong way, add `rpy` to a camera optical sub-frame; if a sensor is silent, check `ign topic -l` for the gz topic name and the link exists in `/tmp/rosbot.urdf`.

- [ ] **Step 2: Mapping with the ROSbot**

```bash
ros2 launch rover_sim sim.launch.py mode:=mapping model:=rosbot driver:=none headless:=true rviz:=false x:=0.0 y:=2.0 > /tmp/rosbot_p1.log 2>&1 &
sleep 16
timeout 6 ros2 topic pub -r 10 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.3}}" >/dev/null 2>&1
sleep 3
timeout 6 ros2 topic echo /map --once --field info 2>&1 | grep -E 'width|height'
ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 -A1 Translation
pkill -INT -f 'ros2 launch rover_sim'; sleep 5; for p in 'ign gazebo' 'ruby.*ign' parameter_bridge async_slam ekf_node rf2o fake_odom; do pkill -9 -f "$p"; done
```
Expected: `/map` has nonzero width/height; `map→odom` TF present — slam mapping works with the ROSbot model.

- [ ] **Step 3: Record results.** ROSbot integration is green when sensors publish, it drives, and it maps. Note any camera-orientation or tuning fixes in the relevant commit.

---

### Task 5: Docs

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Document the `model:=` option** in the Simulation section:

```bash
# In CLAUDE.md "Simulation" block, note:
#   model:=rover|rosbot   (rover = first-party; rosbot = vendored Husarion ROSbot 2R)
# e.g. ros2 launch rover_sim sim.launch.py mode:=mapping model:=rosbot driver:=teleop
```

- [ ] **Step 2: Commit**

```bash
cd ~/rover_slam
git add CLAUDE.md
git commit -m "docs: document model:=rosbot sim option"
```

---

## Self-Review (completed during authoring)

- **Spec coverage:** vendoring (T1), wrapper with DiffDrive+sensors (T2), `model:=` swap (T3), runtime verify (T4), docs (T5). The spec's `husarion_components_description` vendoring is intentionally dropped (documented deviation — not needed once we add our own sensors).
- **Placeholders:** none — wrapper xacro, package.xml edits, launch edits, and verification commands are concrete with the verified frames/joints (`fl/fr/rl/rr_wheel_joint`, `cover_link`/`imu_link`/`camera_mount_link`, `wheel_separation=0.192`, `wheel_radius=0.0425`).
- **Consistency:** spawn topic `/model/rover/cmd_vel` matches the existing `cmd_vel_relay` + `bridge.yaml`; `model` arg + choices identical across spawn/sim launches; sensor topics match the existing contract so rf2o/EKF/slam consume them unchanged.
- **Runtime-dependent (flagged, not placeholders):** camera frame orientation (T4 Step 1) and any sensor-topic naming check via `ign topic -l`.
