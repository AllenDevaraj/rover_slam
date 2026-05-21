# Husarion ROSbot 2R Model Integration — Design

**Date:** 2026-05-21
**Status:** Approved (design); pending implementation plan
**Related:** `docs/superpowers/specs/2026-05-21-rover-sim-backend-design.md`, `CLAUDE.md`

---

## 1. Goal

Add the open-source **Husarion ROSbot 2R** (differential 4-wheeler) as a
**swappable robot model** in the existing Gazebo Fortress sim, alongside the
first-party `rover_description` model. Reuse *our* sim plumbing (gz `DiffDrive`,
`ros_gz_bridge`, `cmd_vel_relay`, worlds, RViz) and *our* topic contract so the
ROSbot feeds the **unchanged** rf2o + EKF + slam_toolbox + A* + pure_pursuit
pipeline. One new launch dimension: `model:=rover|rosbot`.

Non-goal: replacing `rover_description` (it stays the canonical model that
mirrors the real robot) or adopting Husarion's control stack.

## 2. Constraints / facts (verified)

- ROSbot 2R URDF wheel joints: **`fl_wheel_joint`, `fr_wheel_joint`,
  `rl_wheel_joint`, `rr_wheel_joint`** (differential: left = fl+rl, right = fr+rr).
  Frames include `base_link`, `imu_link`.
- `rosbot_description` ships **Fortress-ready** gz content (`gpu_lidar`,
  `gz_ros2_control`, ignition sensors) and `exec_depend`s on
  **`husarion_components_description`** (sensor component xacros, incl.
  `slamtec_rplidar`, camera, `gz_sensor.urdf.xacro` with configurable `<topic>`)
  and `open_manipulator_x_description` (manipulator variant — unused here).
- `rosbot_description` meshes ≈ 16 MB (ROSbot 2R + ROSbot XL); trimming XL cuts bloat.
- Same machine: Gazebo Fortress + `ros_gz` + `gz_ros2_control` already installed;
  no new system deps expected.

## 3. Vendoring (`ros2_ws/src/vendor/`)

- **`rosbot_description`** — vendored from `husarion/rosbot_ros` (humble); strip
  nested `.git`; **trim** `urdf/rosbot_xl/`, `config/rosbot_xl/`,
  `meshes/rosbot_xl/` (keep only ROSbot 2R) to reduce bloat.
- **`husarion_components_description`** — vendored from
  `husarion/husarion_components_description`; strip `.git`.
- **Drop the `open_manipulator_x_description` dependency** from
  `rosbot_description/package.xml` (we never include the manipulator), so the
  workspace builds without vendoring the arm.
- Un-gitignore both (they are first-class vendored packages, like `slam_toolbox`).
- Build type: both are `ament_cmake` description packages (xacro + meshes).

## 4. Integration wrapper

New file: **`rover_description/urdf/rosbot_sim.urdf.xacro`** — a thin wrapper that:

1. Includes the ROSbot 2R macro (`rosbot_macro.urdf.xacro`) to get the real
   chassis + 4 wheels + rplidar + camera + imu (correct meshes/poses).
2. Sets the ROSbot macro's args to **disable its `ros2_control`/`gz_ros2_control`**
   control plugin (so it does not require `husarion_controllers`). The exact arg
   names are read from `rosbot_macro.urdf.xacro` during implementation (e.g.
   `use_ros2_control:=false` / a sim-engine selector); if no clean toggle exists,
   include only the `body`/`wheel`/component sub-xacros and skip
   `common/ros2_control.urdf.xacro`.
3. Adds **our** gz `DiffDrive` system on `fl/fr/rl/rr_wheel_joint`
   (`<left_joint>fl_wheel_joint</left_joint><left_joint>rl_wheel_joint</left_joint>`
   and the rr/fr equivalents), `topic /model/rover/cmd_vel`, odom/TF publishing
   off (EKF owns odom) — same pattern as `gz_diff_drive.xacro`.
4. Ensures the lidar/imu/camera gz sensors publish on the contract topics
   **`/scan`, `/imu`, `/camera/color/image_raw`** — by passing the components'
   `<topic>`/namespace args, or (fallback) attaching our `gz_sensors.xacro`-style
   sensors at the ROSbot's `laser`/`imu`/`camera` frames.
5. Adds a `JointStatePublisher` system (→ `/joint_states`).

The wrapper keeps the ROSbot's frames; `robot_state_publisher` publishes them
from this URDF exactly as for the rover model.

## 5. Swap mechanism

Add `model:=rover|rosbot` (default `rover`) to `rover_sim`:

- **`spawn.launch.py`** — select the xacro by `model`:
  `rover` → `rover_description/urdf/robot.urdf.xacro` (`sim_mode:=true drive_model:=<drive>`);
  `rosbot` → `rover_description/urdf/rosbot_sim.urdf.xacro`.
  Everything else (gz world, `create`, `ros_gz_bridge`, `cmd_vel_relay`, spawn
  pose, headless) is identical.
- **`sim.launch.py`** — declare `model` and pass it through to `spawn.launch.py`.
  The state-estimation / slam / nav / driver layers are unchanged.
- `drive:=` is honored for `model:=rover`; for `model:=rosbot` the drive is its
  own DiffDrive wrapper (the `drive` arg is ignored, documented).

`rover_description` gains an `exec_depend` on `rosbot_description` +
`husarion_components_description` (the wrapper includes them).

## 6. Topic contract & pipeline (unchanged)

`/scan`, `/imu`, `/camera/color/image_raw` produced by the ROSbot's gz sensors;
`/cmd_vel` (TwistStamped/Twist) → `cmd_vel_relay` → `/model/rover/cmd_vel` →
DiffDrive. rf2o + fake_odom + EKF + slam_toolbox + A* + pure_pursuit consume the
same topics with no change. `bridge.yaml` is unchanged (same topic names).

## 7. Verification

- `colcon build` clean (rosbot_description + husarion_components_description +
  rover_description + rover_sim).
- `xacro rosbot_sim.urdf.xacro` expands; contains our DiffDrive + the 4 wheel
  joints + sensor frames; **no** `gz_ros2_control`/husarion controller plugin.
- Headless spawn (`model:=rosbot`): `/scan`, `/imu`, `/camera/color/image_raw`
  publish at sensible rates; TF tree present.
- Drive via `/cmd_vel_teleop` → ROSbot moves in Gazebo.
- `mode:=mapping model:=rosbot` → slam builds a `/map`; `save_map` works.
- `--show-args` parses for both `model` values.

## 8. Risks & mitigations

1. **Disabling the ROSbot control stack cleanly** — prefer its xacro toggle;
   fallback = include only body/wheel/component xacros, not `ros2_control.urdf.xacro`.
2. **Sensor topic naming** — set component `<topic>` args to the contract; verify
   with `ign topic -l` / `ros2 topic hz`.
3. **`open_manipulator_x_description` build dep** — removed from package.xml.
4. **Mesh bloat (~16 MB)** — trim the ROSbot XL variant; keep only 2R.
5. **DiffDrive with 4 wheels** — gz `DiffDrive` accepts multiple `<left_joint>`/
   `<right_joint>` entries; list both wheels per side.
6. **Frame collisions** — the ROSbot uses its own frame names; no overlap with the
   rover model since only one model spawns per run.

## 9. Scope (YAGNI / out of scope)

- ROSbot **2R only** (not XL/mecanum, not the manipulator).
- Reuse the ROSbot's own gz sensors (don't re-model sensors unless the topic
  override fails).
- No Husarion controllers / `gz_ros2_control` / Husarion worlds.
- `rover_description` (real-robot model) is unchanged and remains the default.
