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
from nav2_msgs.action import Spin, NavigateToPose
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus

import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


@dataclass
class Frontier:
    cells: List[Tuple[int, int]]          # (mx, my)
    centroid_world: Tuple[float, float]   # (x, y) in map frame
    size: int


class FrontierExplorer(Node):
    # 8-neighbour offsets
    NEIGH8 = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    ]

    def __init__(self):
        super().__init__("frontier_explorer")

        # -------- Parameters --------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")

        # Frontier logic
        self.declare_parameter("occ_free_max", 50)
        self.declare_parameter("min_frontier_cells", 8)
        self.declare_parameter("unknown_fraction_stop", 0.03)

        # Arena bounds (set from your SDF; add margin from walls)
        self.declare_parameter("min_x", -0.55)
        self.declare_parameter("max_x", 5.35)
        self.declare_parameter("min_y", -0.55)
        self.declare_parameter("max_y", 5.35)

        # Scoring
        self.declare_parameter("w_dist", 1.0)
        self.declare_parameter("w_size", 2.0)

        # Planner reachability filter
        self.declare_parameter("use_planner_filter", False)
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("min_path_length_m", 0.3)

        # Timing / retries
        self.declare_parameter("tick_period_s", 1.0)
        self.declare_parameter("blacklist_radius_m", 1.5)

        self.declare_parameter("goal_offset_m", 3.0)
        self.declare_parameter("min_goal_separation_m", 0.6)         # must move at least this
        self.declare_parameter("min_last_goal_separation_m", 0.8)     # avoid same goal repeatedly
        self.declare_parameter("visited_radius_m", 0.8)

        # Approach behavior
        self.declare_parameter("enable_approach", True)
        self.declare_parameter("approach_radius_m", 0.7)
        self.declare_parameter("approach_radius_growth", 1.4)
        self.declare_parameter("approach_growth_steps", 2)
        self.declare_parameter("final_retry_count", 2)

        # State
        self.mode = "EXPLORE"   # EXPLORE / APPROACH / FINAL
        self.pending_goal_xy = None
        self.approach_queue = []
        self.approach_round = 0
        self.final_retries_left = 0
        self.current_goal_kind = "EXPLORE"  # EXPLORE / APPROACH / FINAL

        # -------- Runtime state --------
        self.map: Optional[OccupancyGrid] = None
        self.goal_in_flight = False
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self.blacklist: List[Tuple[float, float]] = []
        self.visited: List[Tuple[float, float]] = []

        # -------- QoS (slam_toolbox often uses transient local) --------
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            map_qos,
        )

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.plan_client = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter("planner_action").value,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(
            float(self.get_parameter("tick_period_s").value),
            self._tick,
        )

        self.get_logger().info("FrontierExplorer started.")

    def _on_map(self, msg: OccupancyGrid):
        self.map = msg

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

        # --- APPROACH mode ---
        if self.mode == "APPROACH":
            if not self.approach_queue:
                self.approach_round += 1
                if self.approach_round >= int(self.get_parameter("approach_growth_steps").value):
                    self.get_logger().warn("Approach failed; blacklisting original goal.")
                    if self.pending_goal_xy:
                        self.blacklist.append(self.pending_goal_xy)
                    self.mode = "EXPLORE"
                    self.pending_goal_xy = None
                    return

                r = float(self.get_parameter("approach_radius_m").value) * (
                    float(self.get_parameter("approach_radius_growth").value) ** self.approach_round
                )
                self.approach_queue = self._make_approach_candidates(self.pending_goal_xy, robot_xy, r)
                self.get_logger().info(f"Approach queue rebuilt (round={self.approach_round}) size={len(self.approach_queue)}")
                return

            next_xy = self.approach_queue.pop(0)
            self.current_goal_kind = "APPROACH"
            self._send_nav_goal(robot_xy, next_xy)
            return

        # --- FINAL mode ---
        if self.mode == "FINAL" and self.pending_goal_xy is not None:
            snapped = self._snap_to_nearest_free(self.pending_goal_xy[0], self.pending_goal_xy[1], radius_m=0.8)
            if snapped is None:
                self.get_logger().warn("Final goal snap failed; blacklisting.")
                self.blacklist.append(self.pending_goal_xy)
                self.mode = "EXPLORE"
                self.pending_goal_xy = None
                return

            snapped = self._offset_towards_robot(snapped, robot_xy)
            self.current_goal_kind = "FINAL"
            self._send_nav_goal(robot_xy, snapped)
            return

        frontiers = self._detect_frontiers()
        self.get_logger().info(
            f"Frontiers found: {len(frontiers)} | blacklist: {len(self.blacklist)} | unknown_frac: {unknown_frac}"
        )
        if not frontiers:
            self.get_logger().info("No frontiers found right now. Retrying...")
            return

        candidate = self._choose_best(frontiers, robot_xy)
        if candidate is None:
            self.get_logger().info("No valid frontier after blacklist/bounds. Stopping.")
            rclpy.shutdown()
            return

        goal_xy_orig = candidate.centroid_world
        goal_xy = self._offset_towards_robot(goal_xy_orig, robot_xy)

        snapped = self._snap_to_nearest_free(goal_xy[0], goal_xy[1], radius_m=0.8)
        if snapped is None:
            self.get_logger().warn(f"Goal not in free space, snapping failed; blacklisting: {goal_xy}")
            if goal_xy_orig not in self.blacklist:
                self.blacklist.append(goal_xy_orig)
            if goal_xy not in self.blacklist:
                self.blacklist.append(goal_xy)
            return
        goal_xy = snapped

        if self._dist(robot_xy, goal_xy) < float(self.get_parameter("min_goal_separation_m").value):
            self.get_logger().info(f"Skipping goal too close to robot: {goal_xy}")
            if goal_xy_orig not in self.blacklist:
                self.blacklist.append(goal_xy_orig)
            if goal_xy not in self.blacklist:
                self.blacklist.append(goal_xy)
            return

        if self.last_goal_xy is not None and self._dist(self.last_goal_xy, goal_xy) < float(self.get_parameter("min_last_goal_separation_m").value):
            self.get_logger().info(f"Skipping goal too close to last goal: {goal_xy}")
            if goal_xy_orig not in self.blacklist:
                self.blacklist.append(goal_xy_orig)
            if goal_xy not in self.blacklist:
                self.blacklist.append(goal_xy)
            return

        if bool(self.get_parameter("use_planner_filter").value):
            if not self._planner_ready():
                self.get_logger().warn("Planner service not ready; sending goal without filter.")
            else:
                ok = self._is_reachable(robot_xy, goal_xy)
                if not ok:
                    self.get_logger().warn(f"Frontier not reachable, blacklisting: {goal_xy}")
                    if goal_xy_orig not in self.blacklist:
                        self.blacklist.append(goal_xy_orig)
                    if goal_xy not in self.blacklist:
                        self.blacklist.append(goal_xy)
                    return

        self._send_nav_goal(robot_xy, goal_xy)

    def _make_approach_candidates(self, goal_xy, robot_xy, radius_m):
        gx, gy = goal_xy
        raw = [
            (gx + radius_m, gy),
            (gx - radius_m, gy),
            (gx, gy + radius_m),
            (gx, gy - radius_m),
        ]

        cands = []
        for x, y in raw:
            if not self._in_bounds_world(x, y):
                continue

            snapped = self._snap_to_nearest_free(x, y, radius_m=0.8)
            if snapped is None:
                continue

            snapped = self._offset_towards_robot(snapped, robot_xy)

            if self._dist(snapped, robot_xy) < float(self.get_parameter("min_goal_separation_m").value):
                continue

            cands.append(snapped)

        cands.sort(key=lambda p: self._dist(p, robot_xy))
        return cands

    def _world_to_map(self, x: float, y: float):
        m = self.map
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= m.info.width or my >= m.info.height:
            return None
        return (mx, my)

    def _map_to_world_cell_center(self, mx: int, my: int):
        m = self.map
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        return (ox + (mx + 0.5) * res, oy + (my + 0.5) * res)

    def _cell_is_free(self, mx: int, my: int) -> bool:
        m = self.map
        v = m.data[my * m.info.width + mx]
        return (v >= 0) and (v <= int(self.get_parameter("occ_free_max").value))

    def _snap_to_nearest_free(self, gx: float, gy: float, radius_m: float = 0.8):
        ij = self._world_to_map(gx, gy)
        if ij is None:
            return None

        mx0, my0 = ij
        if self._cell_is_free(mx0, my0):
            return (gx, gy)

        m = self.map
        res = m.info.resolution
        r_cells = int(radius_m / res)

        for r in range(1, r_cells + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    mx, my = mx0 + dx, my0 + dy
                    if 0 <= mx < m.info.width and 0 <= my < m.info.height:
                        if self._cell_is_free(mx, my):
                            return self._map_to_world_cell_center(mx, my)
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    mx, my = mx0 + dx, my0 + dy
                    if 0 <= mx < m.info.width and 0 <= my < m.info.height:
                        if self._cell_is_free(mx, my):
                            return self._map_to_world_cell_center(mx, my)

        return None

    def _dist(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    # ---------------- Frontier detection (FIX 1) ----------------
    def _detect_frontiers(self) -> List[Frontier]:
        m = self.map
        grid = m.data
        w = m.info.width
        h = m.info.height
        occ_free_max = int(self.get_parameter("occ_free_max").value)
        min_cells = int(self.get_parameter("min_frontier_cells").value)

        visited: Set[int] = set()
        frontiers: List[Frontier] = []

        for y in range(h):
            for x in range(w):
                idx = y * w + x
                if idx in visited:
                    continue
                if not self._is_frontier_cell(x, y, grid, w, h, occ_free_max):
                    continue

                # Cluster via BFS (8-connected)
                q = deque([(x, y)])
                visited.add(idx)
                cluster: List[Tuple[int, int]] = []

                while q:
                    cx, cy = q.popleft()
                    cluster.append((cx, cy))

                    for dx, dy in self.NEIGH8:
                        nx, ny = cx + dx, cy + dy
                        if nx < 0 or nx >= w or ny < 0 or ny >= h:
                            continue
                        nidx = ny * w + nx
                        if nidx in visited:
                            continue
                        if self._is_frontier_cell(nx, ny, grid, w, h, occ_free_max):
                            visited.add(nidx)
                            q.append((nx, ny))

                if len(cluster) >= min_cells:
                    gx, gy = self._cluster_centroid_world(cluster)
                    if self._in_bounds_world(gx, gy) and not self._is_blacklisted(gx, gy) and not self._is_visited(gx, gy):
                        frontiers.append(Frontier(cluster, (gx, gy), len(cluster)))

        return frontiers

    def _is_frontier_cell(self, x, y, grid, w, h, occ_free_max) -> bool:
        v = grid[y * w + x]
        if v < 0:               # unknown cell itself is not frontier
            return False
        if v > occ_free_max:    # occupied-ish is not frontier
            return False

        # FIX 1: free cell adjacent to unknown (8-neighbourhood)
        for dx, dy in self.NEIGH8:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                if grid[ny * w + nx] == -1:
                    return True
        return False

    def _cluster_centroid_world(self, cluster: List[Tuple[int, int]]) -> Tuple[float, float]:
        mx = sum(c[0] for c in cluster) / len(cluster)
        my = sum(c[1] for c in cluster) / len(cluster)
        return self._map_to_world(mx, my)

    # ---------------- Goal helpers ----------------
    def _offset_towards_robot(self, goal_xy, robot_xy):
        gx, gy = goal_xy
        rx, ry = robot_xy
        off = float(self.get_parameter("goal_offset_m").value)

        dx = rx - gx
        dy = ry - gy
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return goal_xy

        gx2 = gx + (dx / n) * off
        gy2 = gy + (dy / n) * off

        if not self._in_bounds_world(gx2, gy2):
            return goal_xy
        return (gx2, gy2)

    def _choose_best(self, frontiers: List[Frontier], robot_xy: Tuple[float, float]) -> Optional[Frontier]:
        w_dist = float(self.get_parameter("w_dist").value)
        w_size = float(self.get_parameter("w_size").value)
        rx, ry = robot_xy

        best = None
        best_score = float("inf")

        for f in frontiers:
            gx, gy = f.centroid_world
            if self._is_blacklisted(gx, gy) or self._is_visited(gx, gy):
                continue

            dist = math.hypot(gx - rx, gy - ry)
            score = w_dist * dist + w_size * (1.0 / max(1, f.size))
            if score < best_score:
                best_score = score
                best = f

        return best

    def _is_blacklisted(self, gx, gy) -> bool:
        r = float(self.get_parameter("blacklist_radius_m").value)
        for bx, by in self.blacklist:
            if math.hypot(gx - bx, gy - by) <= r:
                return True
        return False

    def _is_visited(self, gx, gy) -> bool:
        r = float(self.get_parameter("visited_radius_m").value)
        for vx, vy in self.visited:
            if math.hypot(gx - vx, gy - vy) <= r:
                return True
        return False

    # ---------------- Planner reachability filter ----------------
    def _planner_ready(self) -> bool:
        return self.plan_client.wait_for_server(timeout_sec=0.1)

    def _is_reachable(self, start_xy, goal_xy) -> bool:
        start = PoseStamped()
        start.header.frame_id = self.get_parameter("goal_frame").value
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = float(start_xy[0])
        start.pose.position.y = float(start_xy[1])
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = self.get_parameter("goal_frame").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(goal_xy[0])
        goal.pose.position.y = float(goal_xy[1])
        goal.pose.orientation.w = 1.0

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal

        if hasattr(goal_msg, "planner_id"):
            goal_msg.planner_id = ""

        send_future = self.plan_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=1.0)
        if not send_future.done() or send_future.result() is None:
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=1.5)
        if not result_future.done() or result_future.result() is None:
            return False

        res = result_future.result().result
        path = getattr(res, "path", None)
        if path is None or len(path.poses) < 2:
            return False

        length = 0.0
        for i in range(1, len(path.poses)):
            x0 = path.poses[i-1].pose.position.x
            y0 = path.poses[i-1].pose.position.y
            x1 = path.poses[i].pose.position.x
            y1 = path.poses[i].pose.position.y
            length += math.hypot(x1 - x0, y1 - y0)

        return length >= float(self.get_parameter("min_path_length_m").value)

    # ---------------- Nav2 goal sending ----------------
    def _send_nav_goal(self, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float]):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 NavigateToPose server not ready.")
            return

        rx, ry = robot_xy
        gx, gy = goal_xy
        yaw = math.atan2(gy - ry, gx - rx)

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
            self.goal_in_flight = False

            if self.last_goal_xy:
                self.visited.append(self.last_goal_xy)

            if self.current_goal_kind == "APPROACH" and self.pending_goal_xy is not None:
                self.mode = "FINAL"
                self.final_retries_left = int(self.get_parameter("final_retry_count").value)
                self.last_goal_xy = None
                return

            if self.current_goal_kind == "FINAL":
                self.mode = "EXPLORE"
                self.pending_goal_xy = None

            self.last_goal_xy = None
            return

        self.get_logger().warn(f"Goal failed (status={status}) kind={self.current_goal_kind}")
        self.goal_in_flight = False
        self.last_goal_xy = None

        if not bool(self.get_parameter("enable_approach").value):
            return

        if self.current_goal_kind == "APPROACH":
            return

        if self.current_goal_kind == "FINAL":
            self.final_retries_left -= 1
            if self.final_retries_left <= 0:
                self.get_logger().warn("Final goal failed too many times; blacklisting.")
                if self.pending_goal_xy:
                    self.blacklist.append(self.pending_goal_xy)
                self.mode = "EXPLORE"
                self.pending_goal_xy = None
                return

            self.mode = "APPROACH"
            self.approach_round = 0
            robot_xy = self._get_robot_xy()
            if robot_xy is None or self.pending_goal_xy is None:
                return
            r = float(self.get_parameter("approach_radius_m").value)
            self.approach_queue = self._make_approach_candidates(self.pending_goal_xy, robot_xy, r)
            return

        # Normal explore failure: start approach around last goal (if we have it)
        failed_goal = self.last_goal_xy or self.pending_goal_xy
        if failed_goal is None:
            return

        self.pending_goal_xy = failed_goal
        self.mode = "APPROACH"
        self.approach_round = 0
        self.final_retries_left = int(self.get_parameter("final_retry_count").value)

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            return
        r = float(self.get_parameter("approach_radius_m").value)
        self.approach_queue = self._make_approach_candidates(self.pending_goal_xy, robot_xy, r)
        self.get_logger().info(f"Entering APPROACH mode. queue={len(self.approach_queue)} r={r:.2f}")

    # ---------------- TF / Map helpers ----------------
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

    def _map_to_world(self, mx: float, my: float) -> Tuple[float, float]:
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        x = origin.x + (mx + 0.5) * res
        y = origin.y + (my + 0.5) * res
        return x, y

    def _in_bounds_world(self, x: float, y: float) -> bool:
        return (
            float(self.get_parameter("min_x").value) <= x <= float(self.get_parameter("max_x").value)
            and float(self.get_parameter("min_y").value) <= y <= float(self.get_parameter("max_y").value)
        )

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


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
