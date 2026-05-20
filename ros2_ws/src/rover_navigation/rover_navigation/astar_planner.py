#!/usr/bin/env python3
"""
A* path planner.

Subscribes to:
  /map            (nav_msgs/OccupancyGrid)  — from slam_toolbox / map_server
  /goal_pose      (geometry_msgs/PoseStamped) — set a 2D goal

Publishes:
  /plan           (nav_msgs/Path) — A* path in map frame

Uses TF (map -> base_link) to get current robot position.
Cells with occupancy > OCCUPIED_THRESHOLD are treated as walls.
"""

import heapq
import math
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform


OCCUPIED_THRESHOLD = 5  # cells above this value are obstacles (occupied=100, free=0, unknown=-1)
INFLATION_RADIUS_M = 0.0  # inflate obstacles by this many metres (robot half-width)
MIN_OBSTACLE_CELLS = 20   # clusters smaller than this are noise (footsteps, artifacts)

# Centerline preference — penalize cells close to walls so A* picks middle paths.
CENTER_RADIUS_CELLS = 25   # how far the wall's "influence" reaches
CENTER_PENALTY      = 15.0 # max extra cost per cell when right next to a wall

# Virtual wall behind robot — forces A* to take the long way around closed loops.
VIRTUAL_WALL_ENABLE      = True
VIRTUAL_WALL_OFFSET_M    = 0.50
VIRTUAL_WALL_LENGTH_M    = 40.0
VIRTUAL_WALL_THICKNESS_M = 0.50

# Terminal ASCII visualization of the plan (handy on a headless Pi).
ASCII_VIZ_ENABLE = True
ASCII_VIZ_WIDTH  = 80     # target characters wide; height auto-scaled

# Path downsampling — keep N evenly-spaced waypoints (start + goal always kept).
# Set to 0 or None to publish the full A* path.
PATH_NUM_WAYPOINTS = 125


def _filter_small_obstacles(grid: list[list[bool]]) -> list[list[bool]]:
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
                    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                        nr, nc = cr + dr, cc + dc
                        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] and not visited[nr][nc]:
                            visited[nr][nc] = True
                            queue.append((nr, nc))
                if len(component) < MIN_OBSTACLE_CELLS:
                    for cr, cc in component:
                        result[cr][cc] = False
    return result


def _wall_distance(grid: list[list[bool]], max_radius: int) -> list[list[int]]:
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
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and dist[nr][nc] > dist[r][c] + 1:
                dist[nr][nc] = dist[r][c] + 1
                queue.append((nr, nc))
    return dist


def _inflate(grid: list[list[bool]], radius_cells: int) -> list[list[bool]]:
    rows, cols = len(grid), len(grid[0])
    inflated = [row[:] for row in grid]
    for r in range(rows):
        for c in range(cols):
            if grid[r][c]:
                for dr in range(-radius_cells, radius_cells + 1):
                    for dc in range(-radius_cells, radius_cells + 1):
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < rows and 0 <= nc < cols:
                            inflated[nr][nc] = True
    return inflated


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, qos)
        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.debug_map_pub = self.create_publisher(OccupancyGrid, '/planner_map', map_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._map: OccupancyGrid | None = None
        self._obstacle_grid: list[list[bool]] | None = None
        self._wall_dist: list[list[int]] | None = None
        self._last_path_cells: list[tuple[int, int]] = []
        self._pending_goal: PoseStamped | None = None

        self.create_timer(1.0, self._retry_pending_goal)
        self.get_logger().info('A* planner ready — waiting for /map and /goal_pose')

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg
        w, h = msg.info.width, msg.info.height
        raw = [[msg.data[r * w + c] > OCCUPIED_THRESHOLD or msg.data[r * w + c] < 0
                for c in range(w)] for r in range(h)]
        res = msg.info.resolution
        radius_cells = max(1, int(INFLATION_RADIUS_M / res))
        filtered = _filter_small_obstacles(raw)
        self._obstacle_grid = _inflate(filtered, radius_cells)
        self._wall_dist = _wall_distance(self._obstacle_grid, CENTER_RADIUS_CELLS)
        self.get_logger().info(f'Map received: {w}x{h}, res={res:.3f}m')
        self._publish_debug_map()

    def _retry_pending_goal(self) -> None:
        if self._pending_goal is None:
            return
        self.get_logger().info(
            'TF now available — replanning pending goal',
            throttle_duration_sec=2.0,
        )
        self._goal_cb(self._pending_goal)

    def _goal_cb(self, goal: PoseStamped) -> None:
        if self._map is None:
            self.get_logger().warn('No map yet — ignoring goal')
            return

        try:
            if not self.tf_buffer.can_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0)
            ):
                self.get_logger().warn(
                    'TF map->base_link not ready yet — will retry automatically',
                    throttle_duration_sec=3.0,
                )
                self._pending_goal = goal
                return
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF map->base_link unavailable: {e}')
            self._pending_goal = goal
            return

        self._pending_goal = None

        start_x = tf.transform.translation.x
        start_y = tf.transform.translation.y

        # Optionally paint a virtual wall behind the robot. Done on a fresh
        # copy of the obstacle grid so it doesn't accumulate over goals.
        if VIRTUAL_WALL_ENABLE:
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            self._obstacle_grid = [row[:] for row in self._obstacle_grid]
            self._paint_virtual_wall(start_x, start_y, yaw)

        path = self._astar(start_x, start_y, goal.pose.position.x, goal.pose.position.y)
        if path is None:
            self.get_logger().warn('A*: no path found')
            return

        # Downsample to N evenly-spaced waypoints, keeping start and goal.
        if PATH_NUM_WAYPOINTS and len(path) > PATH_NUM_WAYPOINTS:
            step = (len(path) - 1) / (PATH_NUM_WAYPOINTS - 1)
            path = [path[int(round(i * step))] for i in range(PATH_NUM_WAYPOINTS)]

        plan = Path()
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = 'map'
        for wx, wy in path:
            ps = PoseStamped()
            ps.header = plan.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            plan.poses.append(ps)

        self.plan_pub.publish(plan)
        self._publish_debug_map()
        self.get_logger().info(f'Published path with {len(plan.poses)} waypoints')

        if ASCII_VIZ_ENABLE and self._last_path_cells:
            sr, sc = self._world_to_grid(start_x, start_y)
            gr, gc = self._world_to_grid(goal.pose.position.x, goal.pose.position.y)
            self._print_ascii_viz(sr, sc, gr, gc, self._last_path_cells)

    def _paint_virtual_wall(self, robot_x: float, robot_y: float, yaw: float) -> None:
        """Mark a wall-shaped patch behind the robot as occupied."""
        grid = self._obstacle_grid
        if grid is None:
            return
        rows, cols = len(grid), len(grid[0])
        res = self._map.info.resolution

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
                r, c = self._world_to_grid(wx, wy)
                if 0 <= r < rows and 0 <= c < cols and not grid[r][c]:
                    grid[r][c] = True
                    painted += 1
        self.get_logger().info(
            f'Virtual wall: {painted} cells behind ({robot_x:.2f},{robot_y:.2f}) '
            f'yaw={math.degrees(yaw):.0f}°')

    def _print_ascii_viz(self, sr: int, sc: int, gr: int, gc: int,
                         path_cells: list[tuple[int, int]]) -> None:
        """Print a downsampled ASCII view of the planned path to the log."""
        grid = self._obstacle_grid
        rows, cols = len(grid), len(grid[0])
        step = max(1, cols // ASCII_VIZ_WIDTH)
        out_w = cols // step
        out_h = rows // step

        path_set = set((r // step, c // step) for r, c in path_cells)
        s_disp = (sr // step, sc // step)
        g_disp = (gr // step, gc // step)

        lines = []
        for r in range(out_h):
            row_chars = []
            for c in range(out_w):
                # Block is occupied if any cell within is occupied.
                occ = False
                for dr in range(step):
                    for dc in range(step):
                        rr, cc = r * step + dr, c * step + dc
                        if rr < rows and cc < cols and grid[rr][cc]:
                            occ = True
                            break
                    if occ:
                        break
                if (r, c) == s_disp:
                    row_chars.append('S')
                elif (r, c) == g_disp:
                    row_chars.append('G')
                elif (r, c) in path_set:
                    row_chars.append('*')
                elif occ:
                    row_chars.append('#')
                else:
                    row_chars.append(' ')
            lines.append(''.join(row_chars))
        # Image rows go top-down in y, but map y increases up — flip for display
        lines.reverse()
        self.get_logger().info('Plan preview (S=start G=goal *=path #=wall):\n' +
                               '\n'.join(lines))

    def _publish_debug_map(self) -> None:
        if self._map is None or self._obstacle_grid is None:
            return
        grid = self._obstacle_grid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self._map.info
        msg.data = [100 if grid[r][c] or grid[r][c] == -1 else 0
                    for r in range(len(grid))
                    for c in range(len(grid[0]))]
        self.debug_map_pub.publish(msg)

    def _world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        info = self._map.info
        c = int((wx - info.origin.position.x) / info.resolution)
        r = int((wy - info.origin.position.y) / info.resolution)
        return r, c

    def _grid_to_world(self, r: int, c: int) -> tuple[float, float]:
        info = self._map.info
        wx = info.origin.position.x + (c + 0.5) * info.resolution
        wy = info.origin.position.y + (r + 0.5) * info.resolution
        return wx, wy

    def _astar(self, sx: float, sy: float, gx: float, gy: float) -> list[tuple[float, float]] | None:
        grid = self._obstacle_grid
        wall_dist = self._wall_dist
        rows, cols = len(grid), len(grid[0])

        sr, sc = self._world_to_grid(sx, sy)
        gr, gc = self._world_to_grid(gx, gy)

        if not (0 <= sr < rows and 0 <= sc < cols):
            self.get_logger().warn('Start position outside map bounds')
            return None
        if not (0 <= gr < rows and 0 <= gc < cols):
            self.get_logger().warn('Goal position outside map bounds')
            return None
        if grid[gr][gc]:
            self.get_logger().warn('Goal is inside an obstacle')
            return None

        def h(r, c):
            return math.hypot(r - gr, c - gc)

        def center_penalty(r, c):
            if wall_dist is None:
                return 0.0
            d = wall_dist[r][c]
            if d >= CENTER_RADIUS_CELLS:
                return 0.0
            return CENTER_PENALTY * (1.0 - d / CENTER_RADIUS_CELLS)

        open_heap = [(h(sr, sc), 0.0, sr, sc)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_cost: dict[tuple[int, int], float] = {(sr, sc): 0.0}

        # 8-connected grid — diagonal cost = sqrt(2)
        neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        diag_cost = math.sqrt(2)

        while open_heap:
            _, g, r, c = heapq.heappop(open_heap)
            if (r, c) == (gr, gc):
                # Reconstruct
                path_cells = []
                node = (gr, gc)
                while node in came_from:
                    path_cells.append(node)
                    node = came_from[node]
                path_cells.append((sr, sc))
                path_cells.reverse()
                self._last_path_cells = path_cells
                return [self._grid_to_world(r, c) for r, c in path_cells]

            if g > g_cost.get((r, c), float('inf')):
                continue

            for dr, dc in neighbors:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if grid[nr][nc]:
                    continue
                step = (diag_cost if (dr != 0 and dc != 0) else 1.0) + center_penalty(nr, nc)
                ng = g + step
                if ng < g_cost.get((nr, nc), float('inf')):
                    g_cost[(nr, nc)] = ng
                    came_from[(nr, nc)] = (r, c)
                    heapq.heappush(open_heap, (ng + h(nr, nc), ng, nr, nc))

        return None


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
