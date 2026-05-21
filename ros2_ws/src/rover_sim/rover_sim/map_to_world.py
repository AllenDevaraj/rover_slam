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


def greedy_rectangles(grid):
    """Greedy maximal-rectangle decomposition of occupied cells.
    Returns (row, col, height, width). Collapses walls into few large boxes
    (a 1-cell-thick vertical wall becomes ONE tall box, not N row-runs)."""
    h = len(grid)
    w = len(grid[0]) if h else 0
    used = [[False] * w for _ in range(h)]
    rects = []
    for r in range(h):
        for c in range(w):
            if not grid[r][c] or used[r][c]:
                continue
            cw = 0                                  # extend width along the row
            while c + cw < w and grid[r][c + cw] and not used[r][c + cw]:
                cw += 1
            ch = 1                                  # extend height while full width stays occupied
            while r + ch < h:
                if all(grid[r + ch][cc] and not used[r + ch][cc] for cc in range(c, c + cw)):
                    ch += 1
                else:
                    break
            for rr in range(r, r + ch):
                for cc in range(c, c + cw):
                    used[rr][cc] = True
            rects.append((r, c, ch, cw))
    return rects


def rects_to_boxes(rects, resolution, origin, img_h, min_cells=1):
    """Convert rectangles to world-frame box descriptors (y-flipped).
    Drops rectangles smaller than min_cells (noise specks)."""
    ox, oy = origin
    boxes = []
    for (r, c, ch, cw) in rects:
        if ch * cw < min_cells:
            continue
        boxes.append({
            'x': ox + (c + cw / 2.0) * resolution,
            'y': oy + (img_h - (r + ch / 2.0)) * resolution,
            'sx': cw * resolution,
            'sy': ch * resolution,
        })
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
    ap.add_argument('--min-cells', type=int, default=3,
                    help='drop wall rectangles smaller than this many cells (noise)')
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
    boxes = rects_to_boxes(greedy_rectangles(grid), resolution, origin, h,
                           min_cells=args.min_cells)
    name = os.path.splitext(os.path.basename(args.out_world))[0]
    sdf = build_sdf(boxes, args.wall_height, name)
    with open(args.out_world, 'w') as f:
        f.write(sdf)
    print(f'Wrote {args.out_world}: {len(boxes)} wall boxes from {w}x{h} map')


if __name__ == '__main__':
    main()
