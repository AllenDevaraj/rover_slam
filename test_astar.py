#!/usr/bin/env python3
"""
Offline A* test — runs the same algorithm as astar_planner.py on the saved
PGM map and plots the result with matplotlib. No ROS needed.

Edit START_PX and GOAL_PX to your GIMP pixel coordinates.
"""

import heapq
import math
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# --- Map file ---
MAP_PGM  = '/home/the2xman/ROBO_5302_Project/Advanced_Robo_Project/ros2_ws/maps/map_April_27_3_52.pgm'
MAP_YAML_ORIGIN     = (-24.9, -32.6)   # from map_April_27_3_52.yaml
MAP_YAML_RESOLUTION = 0.05

# --- Your GIMP pixel coordinates (col, row) ---
START_PX = (498, 226)
GOAL_PX  = (376, 226)  # 20 ft = 6.096 m behind home in -x direction

# Robot heading at start, in degrees (map frame, 0=+x, 90=+y).
# Used to place a virtual wall behind the robot.
START_YAW_DEG = 0.0

# --- Must match astar_planner.py ---
OCCUPIED_THRESHOLD = 1
INFLATION_RADIUS_M = 0
MIN_OBSTACLE_CELLS = 20

# Virtual wall behind robot — blocks A* from cutting backwards on closed loops.
VIRTUAL_WALL_ENABLE       = True
VIRTUAL_WALL_OFFSET_M     = 0.50
VIRTUAL_WALL_LENGTH_M     = 5.0
VIRTUAL_WALL_THICKNESS_M  = 0.70

# Centerline preference — penalize cells close to walls so A* picks middle paths.
# CENTER_RADIUS_CELLS = how far the wall's "influence" reaches.
# CENTER_PENALTY = max extra cost per cell when right next to a wall.
CENTER_RADIUS_CELLS = 25
CENTER_PENALTY      = 15.0


# ── helpers ──────────────────────────────────────────────────────────────────

def world_to_pixel(wx, wy, origin, res, img_h):
    col = int((wx - origin[0]) / res)
    row = int(img_h - (wy - origin[1]) / res)
    return col, row


def pixel_to_world(col, row, origin, res, img_h):
    wx = origin[0] + col * res
    wy = origin[1] + (img_h - row) * res
    return wx, wy


def filter_small_obstacles(grid):
    rows, cols = len(grid), len(grid[0])
    visited = [[False] * cols for _ in range(rows)]
    result = [row[:] for row in grid]
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] and not visited[r][c]:
                component = []
                queue = deque([(r, c)])
                visited[r][c] = True
                while queue:
                    cr, cc = queue.popleft()
                    component.append((cr, cc))
                    for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
                        nr, nc = cr+dr, cc+dc
                        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] and not visited[nr][nc]:
                            visited[nr][nc] = True
                            queue.append((nr, nc))
                if len(component) < MIN_OBSTACLE_CELLS:
                    for cr, cc in component:
                        result[cr][cc] = False
    return result


def paint_virtual_wall(grid, robot_x, robot_y, yaw, origin, res, img_h):
    """Add a wall-shaped patch behind the robot as occupied."""
    rows, cols = len(grid), len(grid[0])
    fx, fy = math.cos(yaw), math.sin(yaw)
    px, py = -fy, fx
    cx = robot_x - fx * VIRTUAL_WALL_OFFSET_M
    cy = robot_y - fy * VIRTUAL_WALL_OFFSET_M

    n_along = int(VIRTUAL_WALL_LENGTH_M / res) + 1
    n_thick = int(VIRTUAL_WALL_THICKNESS_M / res) + 1
    painted = 0
    for i in range(-n_along // 2, n_along // 2 + 1):
        for j in range(-n_thick // 2, n_thick // 2 + 1):
            wx = cx + i * res * px - j * res * fx
            wy = cy + i * res * py - j * res * fy
            col, row = world_to_pixel(wx, wy, origin, res, img_h)
            if 0 <= row < rows and 0 <= col < cols and not grid[row][col]:
                grid[row][col] = True
                painted += 1
    print(f'Virtual wall painted: {painted} cells')


def inflate(grid, radius_cells):
    rows, cols = len(grid), len(grid[0])
    out = [row[:] for row in grid]
    for r in range(rows):
        for c in range(cols):
            if grid[r][c]:
                for dr in range(-radius_cells, radius_cells + 1):
                    for dc in range(-radius_cells, radius_cells + 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            out[nr][nc] = True
    return out


def wall_distance(grid, max_radius):
    """Multi-source BFS: distance from each free cell to the nearest obstacle.
    Capped at max_radius so we don't waste time on faraway cells."""
    rows, cols = len(grid), len(grid[0])
    dist = [[max_radius] * cols for _ in range(rows)]
    queue = deque()
    for r in range(rows):
        for c in range(cols):
            if grid[r][c]:
                dist[r][c] = 0
                queue.append((r, c))
    while queue:
        r, c = queue.popleft()
        if dist[r][c] >= max_radius:
            continue
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and dist[nr][nc] > dist[r][c] + 1:
                dist[nr][nc] = dist[r][c] + 1
                queue.append((nr, nc))
    return dist


def astar(grid, sr, sc, gr, gc, wall_dist=None):
    rows, cols = len(grid), len(grid[0])

    def h(r, c):
        return math.hypot(r - gr, c - gc)

    def center_penalty(r, c):
        if wall_dist is None:
            return 0.0
        d = wall_dist[r][c]
        if d >= CENTER_RADIUS_CELLS:
            return 0.0
        # Linear ramp: at the wall = full penalty, at edge of radius = 0
        return CENTER_PENALTY * (1.0 - d / CENTER_RADIUS_CELLS)

    heap = [(h(sr, sc), 0.0, sr, sc)]
    came_from = {}
    g_cost = {(sr, sc): 0.0}
    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    diag = math.sqrt(2)

    while heap:
        _, g, r, c = heapq.heappop(heap)
        if (r, c) == (gr, gc):
            path = []
            node = (gr, gc)
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append((sr, sc))
            path.reverse()
            return path
        if g > g_cost.get((r, c), float('inf')):
            continue
        for dr, dc in neighbors:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr][nc]:
                continue
            step = (diag if (dr and dc) else 1.0) + center_penalty(nr, nc)
            ng = g + step
            if ng < g_cost.get((nr, nc), float('inf')):
                g_cost[(nr, nc)] = ng
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(heap, (ng + h(nr, nc), ng, nr, nc))
    return None


# ── main ─────────────────────────────────────────────────────────────────────

img = np.array(Image.open(MAP_PGM))
img_h, img_w = img.shape
origin = MAP_YAML_ORIGIN
res    = MAP_YAML_RESOLUTION

# Build obstacle grid (row-major, same as astar_planner.py)
# gray ~ 125
# black 0
# white 255
# to read as black or on obstacle we need the cell to be  255 - whatever 
raw_grid = [[int(img[r, c]) < (255 - OCCUPIED_THRESHOLD) for c in range(img_w)]for r in range(img_h)]
radius_cells = max(1, int(INFLATION_RADIUS_M / res))
filtered_grid = filter_small_obstacles(raw_grid)
obstacle_grid = inflate(filtered_grid, radius_cells)

# Optional: paint virtual wall behind the robot's start pose
if VIRTUAL_WALL_ENABLE:
    start_wx, start_wy = pixel_to_world(*START_PX, origin, res, img_h)
    paint_virtual_wall(
        obstacle_grid, start_wx, start_wy,
        math.radians(START_YAW_DEG),
        origin, res, img_h,
    )

# Pixel coords → grid row/col
sc, sr = START_PX
gc, gr = GOAL_PX

print(f'Start pixel: col={sc} row={sr}  →  world {pixel_to_world(sc, sr, origin, res, img_h)}')
print(f'Goal  pixel: col={gc} row={gr}  →  world {pixel_to_world(gc, gr, origin, res, img_h)}')

# Sanity checks
if obstacle_grid[sr][sc]:
    print('WARNING: start is inside an obstacle or inflated zone')
if obstacle_grid[gr][gc]:
    print('WARNING: goal is inside an obstacle or inflated zone')

print('Computing wall distance...')
wall_dist = wall_distance(obstacle_grid, CENTER_RADIUS_CELLS)

print('Running A*...')
path = astar(obstacle_grid, sr, sc, gr, gc, wall_dist=wall_dist)

if path is None:
    print('No path found.')
else:
    print(f'Path found: {len(path)} cells')

# ── plot ─────────────────────────────────────────────────────────────────────

obstacle_overlay = np.array([[obstacle_grid[r][c] for c in range(img_w)]
                              for r in range(img_h)], dtype=np.uint8)

fig, axes = plt.subplots(1, 2, figsize=(14, 7))

for ax, title, show_inflation in zip(axes,
                                     ['Raw map', 'Inflated map + A* path'],
                                     [False, True]):
    ax.set_title(title)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255)
    if show_inflation:
        ax.imshow(obstacle_overlay, cmap='Reds', alpha=0.3, vmin=0, vmax=1)
    if path:
        path_cols = [c for r, c in path]
        path_rows = [r for r, c in path]
        ax.plot(path_cols, path_rows, 'b-', linewidth=1.5, label='A* path')
    ax.plot(sc, sr, 'go', markersize=10, label='start')
    ax.plot(gc, gr, 'ro', markersize=10, label='goal')
    ax.legend()

plt.tight_layout()
plt.show()
