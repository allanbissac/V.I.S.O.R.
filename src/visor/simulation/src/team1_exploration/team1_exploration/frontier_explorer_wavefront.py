#!/usr/bin/env python3
import math
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# Bitmask flags (wavefront style)
MAP_OPEN = 1
MAP_CLOSED = 2
FRONTIER_OPEN = 4
FRONTIER_CLOSED = 8


@dataclass
class Frontier:
    points: List[Tuple[int, int]]          # frontier points (mx, my) in map indices (UNKNOWN cells)
    centroid_world: Tuple[float, float]    # centroid (x, y) in map frame
    goal_world: Tuple[float, float]        # nearest FREE goal near centroid (x, y) in map frame
    size: int


class FrontierExplorerWavefront(Node):
    NEIGH8 = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    ]

    def __init__(self):
        super().__init__("frontier_explorer_wavefront")

        # -------- Parameters --------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")  # optional
        self.declare_parameter("use_costmap", False)  # keep False unless you really need costmap
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("navigate_action", "navigate_to_pose")

        # Frontier logic
        # occupancy grid values: unknown=-1, free~0, occupied~100
        self.declare_parameter("occ_free_max", 10)          # <= this counts as "free" for adjacency checks
        self.declare_parameter("occ_obstacle_min", 50)      # >= this counts as obstacle; used to reject frontier points
        self.declare_parameter("min_frontier_points", 10)   # frontier cluster min size
        self.declare_parameter("unknown_fraction_stop", 0.03)

        # Arena bounds
        self.declare_parameter("min_x", -0.55)
        self.declare_parameter("max_x", 5.35)
        self.declare_parameter("min_y", -0.55)
        self.declare_parameter("max_y", 5.35)

        # Scoring
        self.declare_parameter("w_dist", 1.0)
        self.declare_parameter("w_size", 2.0)

        # Timing / retries
        self.declare_parameter("tick_period_s", 1.0)

        # Robot Start coordinates (used only to avoid selecting frontiers too close to start)
        self.declare_parameter("robot_start_x", 0.0)
        self.declare_parameter("robot_start_y", 0.0)
        self.declare_parameter("min_dist_from_start", 1.0)

        # Goal blacklist radius
        self.declare_parameter("blacklist_radius", 0.35)

        # -------- Runtime state --------
        self.map: Optional[OccupancyGrid] = None
        self.costmap: Optional[OccupancyGrid] = None
        self.goal_in_flight = False
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self.blacklist: List[Tuple[float, float]] = []
        self.visited: List[Tuple[float, float]] = []

        # -------- QoS --------
        # slam_toolbox /map is often transient local
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Nav2 costmap topics are typically volatile
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            map_qos,
        )

        if bool(self.get_parameter("use_costmap").value):
            self.costmap_sub = self.create_subscription(
                OccupancyGrid,
                self.get_parameter("costmap_topic").value,
                self._on_costmap,
                costmap_qos,
            )

        # -------- Nav2 client --------
        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter("navigate_action").value
        )

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(
            float(self.get_parameter("tick_period_s").value),
            self._tick,
        )

        self.get_logger().info("FrontierExplorerWavefront started.")

    def _on_map(self, msg: OccupancyGrid):
        self.map = msg

    def _on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg

    # ---------------- Main loop ----------------
    def _tick(self):
        if self.map is None:
            return
        if self.goal_in_flight:
            return

        unknown_frac = self._unknown_fraction_in_bounds()
        if unknown_frac is not None and unknown_frac <= float(self.get_parameter("unknown_fraction_stop").value):
            self.get_logger().info(f"Exploration complete. Unknown fraction={unknown_frac:.3f}")
            self.get_logger().info("Save the map with: ros2 run nav2_map_server map_saver_cli -f ~/arena_map")
            rclpy.shutdown()
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn("Waiting for TF map->robot...")
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for Nav2 NavigateToPose action server...")
            return

        frontiers = self._detect_frontiers_wavefront(robot_xy)

        self.get_logger().info(
            f"Frontiers: {len(frontiers)} | blacklist: {len(self.blacklist)} | unknown_frac: {unknown_frac}"
        )

        if not frontiers:
            self.get_logger().info("No frontiers found right now. Finishing...")
            self.timer.cancel()
            return

        chosen = self._choose_best(frontiers, robot_xy)
        if chosen is None:
            self.get_logger().info("No valid frontier after bounds/blacklist. Stopping.")
            rclpy.shutdown()
            return

        goal_xy = chosen.goal_world
        self._send_nav_goal(robot_xy, goal_xy)

    # ---------------- Wavefront frontier detection ----------------
    def _detect_frontiers_wavefront(self, robot_xy: Tuple[float, float]) -> List[Frontier]:
        m = self.map
        grid = m.data
        w = m.info.width
        h = m.info.height

        occ_free_max = int(self.get_parameter("occ_free_max").value)
        occ_obstacle_min = int(self.get_parameter("occ_obstacle_min").value)
        min_frontier_pts = int(self.get_parameter("min_frontier_points").value)

        # Convert robot world -> map indices
        try:
            start_mx, start_my = self._world_to_map(robot_xy[0], robot_xy[1])
        except Exception:
            # If robot outside map for a moment, just wait
            return []

        # Find nearest free cell to start BFS from
        free_start = self._find_nearest_free(start_mx, start_my, grid, w, h, occ_free_max)
        if free_start is None:
            return []
        sx, sy = free_start

        # classification array (bitmask per cell)
        cls = bytearray(w * h)

        def idx(mx: int, my: int) -> int:
            return my * w + mx

        def neighbors8(mx: int, my: int):
            for dx, dy in self.NEIGH8:
                nx, ny = mx + dx, my + dy
                if 0 <= nx < w and 0 <= ny < h:
                    yield nx, ny

        def is_free(v: int) -> bool:
            return (v >= 0) and (v <= occ_free_max)

        def is_obstacle(v: int) -> bool:
            return (v >= occ_obstacle_min)

        def is_frontier_point(mx: int, my: int) -> bool:
            # Frontier point is UNKNOWN cell adjacent to FREE, and not adjacent to obstacles
            v = grid[idx(mx, my)]
            if v != -1:
                return False

            has_free = False
            for nx, ny in neighbors8(mx, my):
                nv = grid[idx(nx, ny)]
                if is_obstacle(nv):
                    return False
                if is_free(nv):
                    has_free = True
            return has_free

        # Wavefront BFS over map
        map_q = deque()
        map_q.append((sx, sy))
        cls[idx(sx, sy)] |= MAP_OPEN

        frontiers: List[Frontier] = []

        while map_q:
            px, py = map_q.popleft()
            p_i = idx(px, py)

            if cls[p_i] & MAP_CLOSED:
                continue

            # Found a frontier point -> grow frontier cluster
            if is_frontier_point(px, py):
                cls[p_i] |= FRONTIER_OPEN
                frontier_q = deque([(px, py)])
                cluster_pts: List[Tuple[int, int]] = []

                while frontier_q:
                    qx, qy = frontier_q.popleft()
                    q_i = idx(qx, qy)

                    if cls[q_i] & (MAP_CLOSED | FRONTIER_CLOSED):
                        continue

                    if is_frontier_point(qx, qy):
                        cluster_pts.append((qx, qy))
                        for wx, wy in neighbors8(qx, qy):
                            w_i = idx(wx, wy)
                            if cls[w_i] & (FRONTIER_OPEN | FRONTIER_CLOSED | MAP_CLOSED):
                                continue
                            cls[w_i] |= FRONTIER_OPEN
                            frontier_q.append((wx, wy))

                    cls[q_i] |= FRONTIER_CLOSED

                # mark cluster points closed in map search
                for cx, cy in cluster_pts:
                    cls[idx(cx, cy)] |= MAP_CLOSED

                if len(cluster_pts) >= min_frontier_pts:
                    # centroid of UNKNOWN frontier points (world)
                    cx_w, cy_w = self._cluster_centroid_world(cluster_pts)

                    # project centroid to nearest FREE cell (safe nav goal)
                    goal = self._project_to_free(cx_w, cy_w, grid, w, h, occ_free_max)
                    if goal is None:
                        continue
                    gx_w, gy_w = goal

                    # bounds + distance from start filtering
                    if not self._in_bounds_world(cx_w, cy_w):
                        continue
                    if not self._in_bounds_world(gx_w, gy_w):
                        continue

                    sx_w = float(self.get_parameter("robot_start_x").value)
                    sy_w = float(self.get_parameter("robot_start_y").value)
                    if self._dist((cx_w, cy_w), (sx_w, sy_w)) < float(self.get_parameter("min_dist_from_start").value):
                        continue

                    frontiers.append(
                        Frontier(
                            points=cluster_pts,
                            centroid_world=(cx_w, cy_w),
                            goal_world=(gx_w, gy_w),
                            size=len(cluster_pts),
                        )
                    )

            # Expand wavefront (similar to nav2_wfd)
            for vx, vy in neighbors8(px, py):
                v_i = idx(vx, vy)
                if cls[v_i] & (MAP_OPEN | MAP_CLOSED):
                    continue

                v_cost = grid[v_i]
                # Ignore hard obstacles
                if is_obstacle(v_cost):
                    continue

                # Only expand into cells that are near free space (keeps search near reachable region)
                near_free = False
                for nx, ny in neighbors8(vx, vy):
                    if is_free(grid[idx(nx, ny)]):
                        near_free = True
                        break
                if near_free:
                    cls[v_i] |= MAP_OPEN
                    map_q.append((vx, vy))

            cls[p_i] |= MAP_CLOSED

        return frontiers

    def _choose_best(self, frontiers: List[Frontier], robot_xy: Tuple[float, float]) -> Optional[Frontier]:
        w_dist = float(self.get_parameter("w_dist").value)
        w_size = float(self.get_parameter("w_size").value)

        best: Optional[Frontier] = None
        best_score = float("inf")

        for f in frontiers:
            gx, gy = f.goal_world

            # Safety checks
            if not self._in_bounds_world(gx, gy):
                continue
            if self._is_blacklisted((gx, gy)):
                continue

            # Optional: avoid revisiting almost-same goals
            too_close_to_visited = any(self._dist((gx, gy), v) < 0.35 for v in self.visited)
            if too_close_to_visited:
                continue

            dist = self._dist((gx, gy), robot_xy)

            # Prefer nearer + larger frontiers (larger -> smaller penalty)
            score = w_dist * dist + w_size * (1.0 / max(1, f.size))

            if score < best_score:
                best_score = score
                best = f

        return best

    def _cluster_centroid_world(self, cluster_pts: List[Tuple[int, int]]) -> Tuple[float, float]:
        # cluster_pts are (mx,my) of frontier UNKNOWN cells
        sx = 0.0
        sy = 0.0
        for mx, my in cluster_pts:
            wx, wy = self._map_to_world(mx, my)
            sx += wx
            sy += wy
        n = float(len(cluster_pts))
        return (sx / n, sy / n)

    def _project_to_free(
        self,
        wx: float,
        wy: float,
        grid,
        w: int,
        h: int,
        occ_free_max: int,
        max_steps: int = 20000,
    ) -> Optional[Tuple[float, float]]:
        """Project a world point to nearest FREE cell via BFS in map indices."""
        try:
            mx0, my0 = self._world_to_map(wx, wy)
        except Exception:
            return None

        free = self._find_nearest_free(mx0, my0, grid, w, h, occ_free_max, max_steps=max_steps)
        if free is None:
            return None
        mx, my = free
        return self._map_to_world(mx, my)

    def _find_nearest_free(
        self,
        mx0: int,
        my0: int,
        grid,
        w: int,
        h: int,
        occ_free_max: int,
        max_steps: int = 20000,
    ) -> Optional[Tuple[int, int]]:
        """BFS from (mx0,my0) to the nearest FREE cell (0..occ_free_max)."""
        def idx(mx, my): return my * w + mx
        def is_free(v): return (v >= 0) and (v <= occ_free_max)

        q = deque()
        seen = set()
        q.append((mx0, my0))
        seen.add((mx0, my0))

        steps = 0
        while q and steps < max_steps:
            steps += 1
            mx, my = q.popleft()
            v = grid[idx(mx, my)]
            if is_free(v):
                return (mx, my)
            for dx, dy in self.NEIGH8:
                nx, ny = mx + dx, my + dy
                if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in seen:
                    seen.add((nx, ny))
                    q.append((nx, ny))
        return None

    # ---------------- Nav2 goal sending ----------------
    def _send_nav_goal(self, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float], yaw: float = float("nan")):
        rx, ry = robot_xy
        gx, gy = goal_xy
        if math.isnan(yaw):
            yaw = math.atan2(gy - ry, gx - rx)

        if self._is_blacklisted(goal_xy):
            self.get_logger().warn(f"Goal near blacklisted area, skipping: ({gx:.2f},{gy:.2f})")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.get_parameter("goal_frame").value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"Exploration goal -> ({gx:.2f}, {gy:.2f}), yaw={yaw:.2f}")
        self.goal_in_flight = True
        self.last_goal_xy = goal_xy

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected, blacklisting.")
            if self.last_goal_xy and self.last_goal_xy not in self.blacklist:
                self.blacklist.append(self.last_goal_xy)
            self.goal_in_flight = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
            if self.last_goal_xy:
                self.visited.append(self.last_goal_xy)
            self.last_goal_xy = None
            self.goal_in_flight = False
            return

        self.get_logger().warn(f"Goal failed (status={status}), blacklisting last goal.")
        if self.last_goal_xy and self.last_goal_xy not in self.blacklist:
            self.blacklist.append(self.last_goal_xy)
        self.last_goal_xy = None
        self.goal_in_flight = False

    # ---------------- Helper Functions ----------------
    def _dist(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _is_blacklisted(self, xy: Tuple[float, float]) -> bool:
        r = float(self.get_parameter("blacklist_radius").value)
        for bx, by in self.blacklist:
            if self._dist((bx, by), xy) <= r:
                return True
        return False

    def _unknown_fraction_in_bounds(self) -> Optional[float]:
        m = self.map
        w = m.info.width
        h = m.info.height
        total = 0
        unknown = 0

        for my in range(h):
            for mx in range(w):
                wx, wy = self._map_to_world(mx, my)
                if not self._in_bounds_world(wx, wy):
                    continue
                total += 1
                if m.data[my * w + mx] == -1:
                    unknown += 1

        if total == 0:
            return None
        return unknown / total

    def _map_to_world(self, mx: float, my: float) -> Tuple[float, float]:
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        x = origin.x + (mx + 0.5) * res
        y = origin.y + (my + 0.5) * res
        return x, y

    def _world_to_map(self, wx: float, wy: float) -> Tuple[int, int]:
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        if wx < origin.x or wy < origin.y:
            raise ValueError("World coordinates out of bounds (below origin)")
        mx = int((wx - origin.x) / res)
        my = int((wy - origin.y) / res)
        if mx < 0 or my < 0 or mx >= self.map.info.width or my >= self.map.info.height:
            raise ValueError("World coordinates out of bounds (outside grid)")
        return mx, my

    def _in_bounds_world(self, x: float, y: float) -> bool:
        return (
            float(self.get_parameter("min_x").value) <= x <= float(self.get_parameter("max_x").value)
            and float(self.get_parameter("min_y").value) <= y <= float(self.get_parameter("max_y").value)
        )

    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter("goal_frame").value,
                self.get_parameter("robot_frame").value,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None


def main(args=None):
    try:
        rclpy.init(args=args)
        node = FrontierExplorerWavefront()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()