# Rover SLAM — Gazebo Fortress Simulation Backend — Design

**Date:** 2026-05-21
**Status:** Approved (design); pending implementation plan
**Author:** brainstormed with the user (brainstorm → spec → plan flow)
**Related:** `docs/superpowers/specs/2026-05-20-rover-modular-architecture-design.md`, `CLAUDE.md`

---

## 1. Goal

Reproduce the real-hardware SLAM workflow **entirely in simulation** (Gazebo +
RViz) so the modular `rover_*` repo can be verified without the physical rover.
The real workflow we are reproducing:

1. **Mapping** — `line_follower` uses the camera to follow **blue tape** on the
   floor of a building corridor while `slam_toolbox` builds a map; the map is saved.
2. **Planning** — `astar_planner` runs on that saved map and produces a path.
3. **Execution** — `pure_pursuit` follows the path, driving the rover via `/cmd_vel`.

Keyboard teleop must also be available as an alternative driver.

The deeper purpose is to **generalize the repo**: the simulation is added as a
clean, swappable backend, and supports **both** a differential-drive and an
Ackermann (car-like) robot through the *same* downstream pipeline.

## 2. Guiding principle — the swappable-backend seam

The simulation is a drop-in **`backend:=sim`**. It produces exactly the existing
topic contract and consumes `/cmd_vel`; **every downstream node runs unchanged**,
byte-for-byte identical to the real robot.

```
ign gazebo (world: corridor | building)
  rover model: <drive system> + gpu_lidar + imu + camera
        │  gz transport topics
  ros_gz_bridge  ⇄  /scan  /imu  /camera/color/image_raw  /clock     ◄── the only NEW glue
        │                                       ▲ /cmd_vel (relayed → gz Twist)
  rf2o_laser_odometry   /scan      → /rf2o_odom ┐
  rover_tools fake_odom /imu       → /fake_odom ├─► robot_localization EKF
  (sim publishes NO odom and NO TF)             │      → odom→base_link  + /odom     UNCHANGED
  slam_toolbox          /scan      → /map + map→odom                                  UNCHANGED
  astar_planner   /map + /goal_pose → /plan                                           UNCHANGED
  pure_pursuit    /plan             → /cmd_vel (TwistStamped)                          UNCHANGED
```

**Why the seam is clean:** the EKF (`rover_state_estimation/config/ekf.yaml`)
already fuses `/rf2o_odom` (vx + vyaw from laser), `/fake_odom` (yaw from IMU
gyro) and `/imu`, and it **publishes the `odom→base_link` transform itself**.
Therefore the simulator does **not** need to publish odometry or TF — it only has
to emit `/scan`, `/imu`, `/camera/color/image_raw` and *move the robot* in
response to `/cmd_vel`. This sidesteps any TF/odom ownership conflict and makes
the sim a true drop-in source.

## 3. Constraints & environment (verified)

- **Simulator:** Gazebo **Fortress** (`ign gazebo` 6.17, `libignition-gazebo6`).
  The installed ROS bridge stack targets Fortress:
  - `ros-humble-ros-gz-sim` / `ros-humble-ros-gz-bridge` 0.244.20 → `libignition-{gazebo6,transport11,msgs8}`
  - `ros-humble-gz-ros2-control` 0.7.17 → `libignition-gazebo6`
  - **Gazebo Classic is NOT installed** (and is EOL); Harmonic (`gz sim` 8.10) is
    installed standalone but has **no** ROS bridge here. → **Zero new installs.**
- **Behavior-preserving downstream:** no changes to node logic, topics, or node
  names of `rover_state_estimation`, `rover_slam`, `rover_navigation`,
  `rover_behaviors`, `rover_localization`, `rover_teleop`. The **real backend is
  untouched**.
- **ROS 2 Humble**, `colcon build --symlink-install`.

## 4. Package layout

| Package | Change | Contents |
|---|---|---|
| `rover_description` | edit | Sim xacros (sensors + two drive modules), gated by xacro args. Model belongs with the model. |
| `rover_sim` | **NEW** (ament_python) | All simulation backend assets: worlds, map→world converter, topic-bridge config, `cmd_vel` relay node, sim RViz, and the sim launch files. |
| `rover_bringup` | edit | Wire the `backend.launch.py` `sim` branch to include `rover_sim`'s source layer. |

### 4.1 `rover_description` — sim model additions

New xacro files (all included only when `sim_mode:=true`):

- **`gz_sensors.xacro`** (shared by both drive models):
  - `<sensor type="gpu_lidar">` on `laser_frame`: 360 samples, range 0.3–12 m,
    update 10 Hz, `<gz_frame_id>laser_frame</gz_frame_id>`, gz topic bridged → `/scan`.
  - `<sensor type="imu">` on `imu_link`: bridged → `/imu`.
  - `<sensor type="camera">` on `camera_link`: 640×480, `<gz_frame_id>` on the
    camera optical frame, bridged → `/camera/color/image_raw`. (Camera is
    forward-facing at 13 cm; `line_follower` crops the bottom half, so it sees floor tape.)
  - The Fortress `Sensors` system (rendering) is declared in the **world** SDF;
    the `Imu` system as a model/world plugin.
- **`gz_diff_drive.xacro`**: `gz::sim::systems::DiffDrive` on
  `rear_left_wheel_joint` + `rear_right_wheel_joint`; `wheel_separation` =
  `rear_wheel_track` (0.175), `wheel_radius` (0.034); subscribes the bridged gz
  `cmd_vel`; **odom + odom TF publishing disabled** (EKF owns odom).
- **`gz_ackermann.xacro`**: `gz::sim::systems::AckermannSteering` using the front
  steering joints (`front_left_wheel_joint`, `front_right_wheel_joint`) + rear
  drive; uses `wheel_base` (0.172) and track; odom/TF disabled.
- **`gz_joint_state.xacro`** (optional, shared): `JointStatePublisher` system →
  `/joint_states` so `robot_state_publisher` can publish wheel TFs.

`robot.urdf.xacro` is made conditional (the `sim_mode` / new `drive_model` args
already partly exist):

```xml
<xacro:arg name="sim_mode"    default="false"/>
<xacro:arg name="drive_model" default="diff"/>   <!-- diff | ackermann -->

<xacro:include filename="robot_core.xacro"/>
<xacro:if value="$(arg sim_mode)">
  <xacro:include filename="gz_sensors.xacro"/>
  <xacro:include filename="gz_joint_state.xacro"/>
  <xacro:if    value="${'$(arg drive_model)' == 'ackermann'}">
    <xacro:include filename="gz_ackermann.xacro"/>
  </xacro:if>
  <xacro:unless value="${'$(arg drive_model)' == 'ackermann'}">
    <xacro:include filename="gz_diff_drive.xacro"/>
  </xacro:unless>
</xacro:if>
<xacro:unless value="$(arg sim_mode)">
  <xacro:include filename="ros2_control.xacro"/>   <!-- real Ackermann hardware -->
  <xacro:include filename="lidar.xacro"/>          <!-- classic plugin, real only -->
  <xacro:include filename="imu.xacro"/>
  <xacro:include filename="camera.xacro"/>
</xacro:unless>
```

**Bug fixes folded in** (hygiene, behavior-neutral for real):
- Front steering joints currently have `velocity="0.0"` in `<limit>` (cannot
  move) — set a sane non-zero limit so Ackermann steering works.
- Stray `Select` text artifact before the `base_footprint_joint` in `robot_core.xacro`.

### 4.2 `rover_sim` — new package (ament_python)

```
rover_sim/
  worlds/
    corridor.world          # hand-modeled representative ring corridor + blue tape loop
    building.world          # generated from map_April_27_3_52.pgm (committed)
  scripts/map_to_world.py    # reusable PGM+YAML → SDF world converter (entry point: map2world)
  config/bridge.yaml         # ros_gz_bridge topic map
  rover_sim/cmd_vel_relay.py # entry point: cmd_vel_relay
  rviz/sim.rviz              # robot, TF, /scan, /map, /plan, camera, goal tool
  launch/spawn.launch.py     # the backend:=sim source layer
  launch/sim.launch.py       # one-command entry (mapping | planning)
```

- **`cmd_vel_relay`** (drive-agnostic): subscribes `TwistStamped /cmd_vel`
  (autonomy: `line_follower`, `pure_pursuit`) **and** `Twist /cmd_vel_teleop`
  (keyboard teleop, remapped in the teleop launch), republishes plain
  `geometry_msgs/Twist` on the gz-bridged drive topic. This resolves the
  pre-existing `cmd_vel` type split (autonomy uses `TwistStamped`, teleop uses
  `Twist`) without a single-topic type clash.
- **`config/bridge.yaml`** (`ros_gz_bridge parameter_bridge`):
  - `/clock`  gz→ROS (`rosgraph_msgs/Clock`) — drives `use_sim_time`.
  - `/scan`   gz→ROS (`sensor_msgs/LaserScan`).
  - `/imu`    gz→ROS (`sensor_msgs/Imu`).
  - `/camera/color/image_raw` gz→ROS (`sensor_msgs/Image`; or via `ros_gz_image`).
  - drive `cmd_vel` ROS→gz (`geometry_msgs/Twist` → `gz.msgs.Twist`).
  - **No** gz odom/TF bridged (EKF owns it).
- **`map_to_world.py`**: reads a slam_toolbox PGM + its YAML (`resolution`,
  `origin`, thresholds), treats dark cells as occupied, merges occupied cells
  along rows into boxes (to keep box count low), and emits an SDF world of
  extruded gray walls (≈1 m tall) wrapped with sun + ground plane + physics +
  Sensors/SceneBroadcaster systems. Reusable on any saved map; used to generate
  `building.world` from `map_April_27_3_52.pgm` (1068×878, res 0.05,
  origin [-24.9, -32.6, 0]). The converter **preserves the map's origin and
  resolution**, so the generated world is spatially **co-registered** with the
  source map — Phase 2 can then localize directly on the original
  `map_April_27_3_52.yaml` (no re-mapping needed) because the Gazebo world frame
  and the map frame coincide.

### 4.3 `rover_bringup` — backend wiring

- `backend.launch.py`: implement the `sim` branch → `IncludeLaunchDescription`
  of `rover_sim/spawn.launch.py` (forwarding `drive`, `world`, `use_sim_time`),
  conditioned on `backend == sim`. The real branch (rplidar + camera + base +
  static TFs) is unchanged; the sim branch uses `robot_state_publisher` instead
  of the real static TFs.

## 5. Modular drive model

A single arg `drive:=diff|ackermann` flows: `sim.launch.py` → `spawn.launch.py`
→ `robot_state_publisher` URDF xacro args (`sim_mode:=true drive_model:=…`) →
selects exactly one drive xacro. **Everything else is identical** between the two
(sensors, relay, bridge, world, downstream stack). Each drive type is therefore a
self-contained, separately-launchable, independently-verifiable module.

| | DiffDrive | AckermannSteering |
|---|---|---|
| gz system | `DiffDrive` (rear wheels) | `AckermannSteering` (front steer + rear drive) |
| `/cmd_vel` semantics | linear.x = speed, angular.z = yaw rate | linear.x = speed, angular.z → steering |
| Fidelity to real rover | interface-equivalent | geometry-faithful (real is car-like) |
| Extra work | none | URDF steering-joint fix + steering tuning |

## 6. Worlds

1. **`corridor.world`** — clean, hand-modeled rectangular **ring corridor**
   (perimeter walls + inner block) with a **blue-tape loop** on the floor:
   material RGB ≈ `(0, 153, 255)` / `#0099FF` (matching `line_follower`'s target
   color) with **right-angle (90°) turns** and **≤3 right corners** (matching
   `line_follower`'s corner state machine `num_turns < 3`). Primary world for the
   camera line-follower demo and tuning.
2. **`building.world`** — generated from the real `map_April_27_3_52.pgm` via
   `map_to_world.py`; matches the actual building floor (a ring corridor around a
   central block). Primary world for SLAM + planning + execution on real geometry.
   Driven by teleop or line-follower. Laying blue tape along this world's corridor
   centerline is a **stretch goal** (skeletonize the free-space ring).

## 7. Launch composition

- **`rover_sim/spawn.launch.py`** (`drive:=`, `world:=`, `use_sim_time:=true`):
  `ros_gz_sim/gz_sim.launch.py` (`gz_args:=<world> -r`) + `robot_state_publisher`
  (URDF with `sim_mode:=true drive_model:=<drive>`) + `ros_gz_sim create`
  (spawn from `/robot_description`) + `ros_gz_bridge parameter_bridge`
  (`config_file:=bridge.yaml`) + `cmd_vel_relay`. Sets the gz resource path so the
  worlds are found.
- **`rover_sim/sim.launch.py`** — the sim analog of `rover_bringup/slam_nav.launch.py`,
  swapping the hardware layer for the Gazebo layer and setting `use_sim_time:=true`:
  - `mode:=mapping`: spawn + rf2o (+ one-shot `/base_pose_ground_truth` init,
    reused from `slam_nav`) + fake_odom + EKF + slam_toolbox + RViz +
    `driver:=line_follower|teleop|none`.
  - `mode:=planning map:=<path>`: spawn + rf2o + fake_odom + EKF + map_server +
    amcl + lifecycle_manager + astar_planner + pure_pursuit (`use_sim:=false`,
    because Gazebo+localization provide real TF) + RViz.
  - args: `mode`, `drive`, `world`, `driver`, `map`.

`use_sim_time:=true` is threaded to **every** node (RSP, rf2o, fake_odom, EKF,
slam_toolbox, amcl, map_server, planner, pursuit), fed by the bridged `/clock`.

## 8. TF frames

`robot_state_publisher` (run in the sim backend) publishes
`base_link → laser_frame | imu_link | camera_link` (fixed joints) and the wheel
TFs (from `/joint_states`). The gz sensors' `gz_frame_id`s are set to match
(`laser_frame`, `imu_link`, camera optical frame). rf2o uses `base_frame_id=base_link`;
slam_toolbox `base_frame=base_link`, `odom_frame=odom`, `map_frame=map`,
`provide_odom_frame=false`. (Real uses static TFs to a frame named `laser`; sim
uses `laser_frame` from the model — both consistent within their backend.)

## 9. Topic contract

| Topic | Type | Sim provider → consumer |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | gz gpu_lidar → rf2o, slam_toolbox |
| `/imu` | `sensor_msgs/Imu` | gz imu → fake_odom, EKF |
| `/camera/color/image_raw` | `sensor_msgs/Image` | gz camera → line_follower |
| `/cmd_vel` | `TwistStamped` (autonomy) / `Twist` (teleop via `/cmd_vel_teleop`) | drivers → relay → gz drive |
| `/clock` | `rosgraph_msgs/Clock` | gz → all (use_sim_time) |
| `/rf2o_odom`, `/fake_odom`, `/odom` | `nav_msgs/Odometry` | rf2o / fake_odom / EKF (unchanged) |
| `/map`, `/plan`, `/goal_pose` | OccupancyGrid / Path / PoseStamped | slam·map_server / A* / RViz (unchanged) |
| TF `map→odom→base_link` | tf2 | slam or amcl / EKF (unchanged) |

## 10. Milestones & verification

- **P0 — Bring-up:** `colcon build` clean; `ros2 launch rover_sim spawn.launch.py
  drive:=diff` spawns the rover in `corridor.world`; `ros2 topic hz /scan /imu
  /camera/color/image_raw` all healthy; teleop drives the robot in Gazebo + RViz.
  Repeat for `drive:=ackermann`.
- **P1 — Line-follower mapping:** `ros2 launch rover_sim sim.launch.py
  mode:=mapping drive:=diff world:=corridor driver:=line_follower`; the rover
  follows the blue tape; the map grows in RViz; `ros2 run rover_localization
  save_map` writes a `.yaml`/`.pgm` resembling the corridor.
- **P2 — Planning + execution:** `ros2 launch rover_sim sim.launch.py
  mode:=planning drive:=diff world:=building map:=<…/map_April_27_3_52.yaml>`
  (the source map is co-registered with `building.world`, so no re-mapping is
  needed; a map saved from P1 can be used the same way in `corridor.world`); amcl
  localizes; a goal set in RViz produces a `/plan`; `pure_pursuit` drives the
  rover to within `GOAL_TOLERANCE_M` (0.25 m) of the goal. Validate the
  `ackermann` module too.
- **Cross-cutting:** every launch parses via `--show-args`; entry points resolve.

## 11. Risks & mitigations

1. **`cmd_vel` type split** → relay subscribes both `TwistStamped /cmd_vel` and
   `Twist /cmd_vel_teleop`.
2. **`use_sim_time`** → bridge `/clock`; set true on all nodes (the real
   `slam_nav` hardcodes false).
3. **Ackermann stability** → URDF steering-joint velocity-limit fix + steering PID
   tuning; DiffDrive is the low-risk reference proven first.
4. **`line_follower` tuning** → its gains (esp. `angular ≤ 12`) and the ≤3-corner
   loop assume the real corridor; expect light tuning in `corridor.world`.
5. **Headless / GPU** → `gpu_lidar` + camera need rendering; confirm the display
   situation at implementation (run gz server headless and/or use a software/X
   backend if no GPU display).
6. **`building.world` size/perf** → ~53 × 44 m; the converter merges occupied-cell
   runs into fewer boxes; downsample if needed.
7. **`map_to_world` thresholds** → honor the map YAML `negate`/`occupied_thresh`/
   `free_thresh`; slam_toolbox PGMs use 0=occupied, 254=free, 205=unknown.

## 12. Scope (YAGNI / out of scope)

- Two worlds only; blue-tape line-follow demo lives in the clean `corridor.world`
  (tape in `building.world` is a stretch goal).
- Both drive models are first-class, but P0→P2 is proven on **diff** first, then
  the **ackermann** module is validated.
- No depth camera, no extra Gazebo GUI plugins, no Harmonic/Classic support.
- The real (`backend:=real`) path is not modified.

## 13. Open implementation questions (resolve during plan/impl)

- Exact `ros_gz_bridge` message-type strings for Fortress (camera via
  `ros_gz_image` vs `ros_gz_bridge`).
- Whether `rover_node`/`velocity_controller` (real base) need any sim shim — not
  used in sim, but confirm nothing else expects `motor_cmd`.
- Headless rendering approach on this machine.
- AckermannSteering parameter names/units in Fortress (steering limits, speed).
