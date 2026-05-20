#!/usr/bin/env python3
"""
Save the current slam_toolbox map in both formats needed for planning mode,
patch free_thresh in the generated YAML, and record the rover's current pose.

Writes five files with a shared prefix:
  <prefix>.pgm        — occupancy grid image
  <prefix>.yaml       — ROS map metadata (free_thresh patched to 0.10)
  <prefix>.posegraph  — slam_toolbox pose graph (required for localization mode)
  <prefix>.data       — slam_toolbox scan data  (required for localization mode)
  <prefix>.pose.yaml  — captured map->base_link pose for seeding AMCL

Usage:
  ros2 run rover_localization save_map
  ros2 run rover_localization save_map -- --name my_map
  ros2 run rover_localization save_map -- --dir /some/other/path
"""

import argparse
import math
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


DEFAULT_DIR = Path(get_package_share_directory('rover_localization')) / 'maps'

FREE_THRESH_TARGET = 0.10


def run(cmd: list[str], label: str) -> bool:
    print(f"\n[{label}]")
    print("  $ " + " ".join(cmd))
    result = subprocess.run(cmd, text=True)
    if result.returncode != 0:
        print(f"  ERROR: command exited with code {result.returncode}")
    return result.returncode == 0


def detect_serialize_service() -> str | None:
    """Return whichever slam_toolbox serialization service is live, or None."""
    try:
        result = subprocess.run(
            ["ros2", "service", "list"],
            text=True, capture_output=True, check=False,
        )
    except FileNotFoundError:
        return None
    services = set((result.stdout or "").splitlines())
    if "/slam_toolbox/serialize_pose_graph" in services:
        return "/slam_toolbox/serialize_pose_graph"
    if "/slam_toolbox/serialize_map" in services:
        return "/slam_toolbox/serialize_map"
    return None


def fix_free_thresh(yaml_path: Path) -> None:
    """Patch free_thresh in the map YAML so nav2_map_server classifies gray
    (unknown) cells as -1 instead of 0.  slam_toolbox writes pixel 205 for
    unknown cells, which gives probability 0.196; the default free_thresh 0.25
    sits above that, so without this patch unknowns become free space."""
    text = yaml_path.read_text()
    patched = re.sub(r'free_thresh:\s*\S+', f'free_thresh: {FREE_THRESH_TARGET}', text)
    yaml_path.write_text(patched)
    print(f"  free_thresh patched to {FREE_THRESH_TARGET}")


def capture_pose(output_path: Path) -> bool:
    """Look up map->base_link TF and write x, y, yaw to a YAML file.
    Returns True on success, False if TF was unavailable."""
    rclpy.init()
    node = Node('_save_map_pose_capture')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
    tf = None
    while rclpy.ok() and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            break
        except Exception:
            pass

    node.destroy_node()
    rclpy.shutdown()

    if tf is None:
        print("  WARNING: map->base_link TF not available within 5 s — skipping pose capture")
        return False

    x   = tf.transform.translation.x
    y   = tf.transform.translation.y
    qz  = tf.transform.rotation.z
    qw  = tf.transform.rotation.w
    yaw = 2.0 * math.atan2(qz, qw)

    pose = {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 6)}
    with open(output_path, 'w') as f:
        yaml.dump(pose, f, default_flow_style=False)

    print(f"  x={x:.3f} m  y={y:.3f} m  yaw={math.degrees(yaw):.1f} deg")
    return True


def main():
    parser = argparse.ArgumentParser(description="Save slam_toolbox map for planning mode")
    parser.add_argument("--dir", default=str(DEFAULT_DIR),
                        help=f"Directory to save into (default: {DEFAULT_DIR})")
    parser.add_argument("--name", default=None,
                        help="Base filename (default: map_<timestamp>)")
    args = parser.parse_args()

    save_dir = Path(args.dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    base_name = args.name or ("map_" + datetime.now().strftime("%B_%-d_%-I_%M"))
    prefix = save_dir / base_name

    print(f"\nSaving map with prefix: {prefix}")
    print(f"  Will write: {base_name}.pgm  .yaml  .posegraph  .data  .pose.yaml\n")

    # Step 1: occupancy grid (PGM + YAML)
    ok = run(
        ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix)],
        "Saving occupancy grid (PGM + YAML)"
    )
    if not ok:
        print("\nFailed to save occupancy grid.")
        print("Check: is nav2_map_server installed? Is slam_toolbox publishing /map?")
        sys.exit(1)

    # Step 2: patch free_thresh so unknown cells reach the planner as -1
    yaml_path = Path(str(prefix) + ".yaml")
    print("\n[Patching free_thresh]")
    if yaml_path.exists():
        fix_free_thresh(yaml_path)
    else:
        print(f"  WARNING: {yaml_path} not found — skipping free_thresh patch")

    # Step 3: pose graph (posegraph + data)
    serialize_service = detect_serialize_service()
    if not serialize_service:
        print("\nFailed to save pose graph.")
        print("Could not find a slam_toolbox serialization service.")
        print("Expected one of:")
        print("  - /slam_toolbox/serialize_pose_graph")
        print("  - /slam_toolbox/serialize_map")
        sys.exit(1)

    ok = run(
        [
            "ros2", "service", "call",
            serialize_service,
            "slam_toolbox/srv/SerializePoseGraph",
            f"{{filename: '{prefix}'}}",
        ],
        "Saving pose graph (posegraph + data)"
    )
    if not ok:
        print("\nFailed to save pose graph.")
        print("Check: is slam_toolbox running in mapping mode?")
        sys.exit(1)

    # Step 4: capture current rover pose from TF
    pose_path = Path(str(prefix) + ".pose.yaml")
    print("\n[Capturing rover pose]")
    capture_pose(pose_path)

    # Summary
    print("\n" + "=" * 60)
    print("Map saved successfully:")
    for ext in (".pgm", ".yaml", ".posegraph", ".data", ".pose.yaml"):
        path = Path(str(prefix) + ext)
        size = f"{path.stat().st_size // 1024} KB" if path.exists() else "not found"
        print(f"  {path.name:45s}  {size}")

    (save_dir / 'last_map').write_text(base_name)
