#!/usr/bin/env python3
"""
Frontier exploration (ROS 2 / Nav2) using a *low-cost frontier point* as the goal.

- Detects frontier cells from /map (free cell adjacent to unknown).
- Clusters frontier cells.
- For each cluster, selects the *frontier cell with the lowest global costmap cost*.
- Chooses the best cluster primarily by (lowest cost, then nearest).
- Sends NavigateToPose goals to Nav2.

Notes:
- Global costmap cost: lower => farther from obstacles (after inflation). This naturally prefers
  “safer / more central” frontier points instead of hugging walls.
- Optional goal_backoff_m moves the goal slightly from the frontier toward the robot (helps reachability).
"""

import math
from dataclasses import dataclass
from collections import deque
from typing import List, Optional, Tuple, Set

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

# Nav2 global costmap message
from nav2_msgs.msg import Costmap


@dataclass
class FrontierCluster:
    cells: List[Tuple[int, int]]                 # list of (mx,my) frontier cells
    best_cell: Tuple[int, int]                   # (mx,my) chosen by lowest cost
    best_cost: int                               # costmap cost at best_cell
    best_world: Tuple[float, float]              # (x,y) best_cell center in world


class FrontierLowCostExplorer(Node):
    def __init__(self):
        super().__init__("frontier_low_cost_explorer")

        # ---------------- Parameters ----------------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("navigate_action", "/navigate_to_pose")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")

        # Frontier classification
        self.declare_parameter("occ_free_max", 50)          # <= this is considered free
        self.declare_parameter("min_frontier_cells", 8)     # cluster size threshold

        # Exploration bounds (optional: keep finite arena)
        self.declare_parameter("use_bounds", True)
        self.declare_parameter("min_x", -0.55)
        self.declare_parameter("max_x", 5.35)
        self.declare_parameter("min_y", -0.55)
        self.declare_parameter("max_y", 5.35)

        # Goal behavior
        self.declare_parameter("tick_period_s", 1.0)
        self.declare_parameter("min_goal_separation_m", 0.6)
        self.declare_parameter("min_repeat_goal_m", 0.8)
        self.declare_parameter("goal_backoff_m", 0.35)      # move slightly *toward robot* from frontier point
        self.declare_parameter("blacklist_radius_m", 0.8)

        # Stop condition (optional)
        self.declare_parameter("stop_unknown_fraction", 0.03)

        # Costmap filters (Nav2 cost conventions)
        self.declare_parameter("reject_unknown_cost", True)   # reject cost=255
        self.declare_parameter("reject_inscribed", True)      # reject cost=253
        self.declare_parameter("reject_lethal", True)         # reject cost=254

        # ---------------- State ----------------
        self.map: Optional[OccupancyGrid] = None
        self.costmap: Optional[Costmap] = None

        self.goal_in_flight = False
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self.blacklist_xy: List[Tuple[float, float]] = []

        # ---------------- QoS ----------------
        # slam_toolbox / map server often use transient local
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cm_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            Costmap,
            self.get_parameter("costmap_topic").value,
            self._on_costmap,
            cm_qos,
        )

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("navigate_action").value,
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer
        self.timer = self.create_timer(
            float(self.get_parameter("tick_period_s").value),
            self._tick,
        )

        self.get_logger().info("FrontierLowCostExplorer started.")

    # ---------------- Callbacks ----------------
    def _on_map(self, msg: OccupancyGrid):
        self.map = msg

    def _on_costmap(self, msg: Costmap):
        self.costmap = msg

    # ---------------- Main loop ----------------
    def _tick(self):
        if self.map is None:
            return
        if self.costmap is None:
            self.get_logger().warn("Waiting for /global_costmap/costmap ...")
            return
        if self.goal_in_flight:
            return

        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for NavigateToPose action server...")
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn("Waiting for TF map->robot...")
            return

        # Optional stop condition
        stop_frac = float(self.get_parameter("stop_unknown_fraction").value)
        unk = self._unknown_fraction_in_bounds()
        if unk is not None and unk <= stop_frac:
            self.get_logger().info(f"Exploration complete (unknown fraction={unk:.3f}).")
            self.get_logger().info("Save map: ros2 run nav2_map_server map_saver_cli -f ~/arena_map")
            rclpy.shutdown()
            return

        clusters = self._detect_frontier_clusters()
        self.get_logger().info(
            f"Frontier clusters: {len(clusters)} | blacklist: {len(self.blacklist_xy)} | unknown_frac: {unk}"
        )
        if not clusters:
            self.get_logger().info("No frontiers found right now. Retrying...")
            return

        cluster = self._choose_best_cluster(clusters, robot_xy)
        if cluster is None:
            self.get_logger().warn("No valid frontier cluster after filtering. Retrying...")
            return

        gx, gy = cluster.best_world

        # Optional backoff: move slightly toward robot so goal is easier to reach
        gx, gy = self._backoff_towards_robot((gx, gy), robot_xy, float(self.get_parameter("goal_backoff_m").value))

        # Validate final goal
        if self._dist((gx, gy), robot_xy) < float(self.get_parameter("min_goal_separation_m").value):
            self.get_logger().info(f"Skipping goal too close to robot: ({gx:.2f},{gy:.2f})")
            self._blacklist((gx, gy))
            return

        if self.last_goal_xy is not None and self._dist((gx, gy), self.last_goal_xy) < float(self.get_parameter("min_repeat_goal_m").value):
            self.get_logger().info(f"Skipping repeated/near goal: ({gx:.2f},{gy:.2f})")
            self._blacklist((gx, gy))
            return

        # Send goal
        self._send_goal(robot_xy, (gx, gy), cluster.best_cost)

    # ---------------- Frontier detection ----------------
    def _detect_frontier_clusters(self) -> List[FrontierCluster]:
        m = self.map
        w, h = m.info.width, m.info.height
        grid = m.data
        occ_free_max = int(self.get_parameter("occ_free_max").value)
        min_cells = int(self.get_parameter("min_frontier_cells").value)

        visited: Set[int] = set()
        clusters: List[FrontierCluster] = []

        for my in range(h):
            for mx in range(w):
                idx = my * w + mx
                if idx in visited:
                    continue
                if not self._is_frontier_cell(mx, my, grid, w, h, occ_free_max):
                    continue

                # BFS cluster
                q = deque([(mx, my)])
                visited.add(idx)
                cells: List[Tuple[int, int]] = []

                while q:
                    cx, cy = q.popleft()
                    cells.append((cx, cy))
                    for nx, ny in ((cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)):
                        if nx < 0 or ny < 0 or nx >= w or ny >= h:
                            continue
                        nidx = ny * w + nx
                        if nidx in visited:
                            continue
                        if self._is_frontier_cell(nx, ny, grid, w, h, occ_free_max):
                            visited.add(nidx)
                            q.append((nx, ny))

                if len(cells) < min_cells:
                    continue

                # Pick frontier cell with lowest costmap cost
                best = self._best_low_cost_frontier_cell(cells)
                if best is None:
                    continue

                best_cell, best_cost, best_world = best

                # Bounds + blacklist filtering
                if self._use_bounds() and not self._in_bounds_world(best_world[0], best_world[1]):
                    continue
                if self._is_blacklisted(best_world):
                    continue

                clusters.append(
                    FrontierCluster(
                        cells=cells,
                        best_cell=best_cell,
                        best_cost=best_cost,
                        best_world=best_world,
                    )
                )

        return clusters

    def _best_low_cost_frontier_cell(self, cells: List[Tuple[int, int]]) -> Optional[Tuple[Tuple[int, int], int, Tuple[float, float]]]:
        best_cell = None
        best_cost = 10**9
        best_world = None

        for mx, my in cells:
            wx, wy = self._map_cell_center_to_world(mx, my)

            # Ignore outside arena bounds early
            if self._use_bounds() and not self._in_bounds_world(wx, wy):
                continue

            cost = self._cost_at_world(wx, wy)
            if cost is None:
                continue
            if not self._cost_is_acceptable(cost):
                continue

            if cost < best_cost:
                best_cost = cost
                best_cell = (mx, my)
                best_world = (wx, wy)

        if best_cell is None:
            return None
        return best_cell, int(best_cost), best_world

    def _is_frontier_cell(self, mx: int, my: int, grid, w: int, h: int, occ_free_max: int) -> bool:
        v = grid[my * w + mx]
        if v < 0:                 # unknown
            return False
        if v > occ_free_max:      # occupied-ish
            return False

        # free cell adjacent to unknown
        for nx, ny in ((mx+1, my), (mx-1, my), (mx, my+1), (mx, my-1)):
            if 0 <= nx < w and 0 <= ny < h:
                if grid[ny * w + nx] == -1:
                    return True
        return False

    # ---------------- Cluster selection ----------------
    def _choose_best_cluster(self, clusters: List[FrontierCluster], robot_xy: Tuple[float, float]) -> Optional[FrontierCluster]:
        # Primary objective: LOWEST COST frontier point.
        # Tie-breakers: nearest to robot, then larger cluster.
        best = None
        best_key = None

        for c in clusters:
            gx, gy = c.best_world

            if self._is_blacklisted((gx, gy)):
                continue

            d = self._dist(robot_xy, (gx, gy))
            key = (c.best_cost, d, -len(c.cells))  # lower cost first, then closer, then bigger
            if best_key is None or key < best_key:
                best_key = key
                best = c

        if best:
            self.get_logger().info(
                f"Selected frontier point ({best.best_world[0]:.2f},{best.best_world[1]:.2f}) "
                f"cost={best.best_cost} size={len(best.cells)}"
            )
        return best

    # ---------------- Nav2 goal sending ----------------
    def _send_goal(self, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float], cost: int):
        rx, ry = robot_xy
        gx, gy = goal_xy
        yaw = math.atan2(gy - ry, gx - rx)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.get_parameter("global_frame").value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"Nav goal -> ({gx:.2f},{gy:.2f}) yaw={yaw:.2f} frontier_cost={cost}")
        self.goal_in_flight = True
        self.last_goal_xy = (gx, gy)

        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected. Blacklisting.")
            if self.last_goal_xy:
                self._blacklist(self.last_goal_xy)
            self.goal_in_flight = False
            self.last_goal_xy = None
            return

        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
        else:
            self.get_logger().warn(f"Goal failed (status={status}). Blacklisting goal region.")
            if self.last_goal_xy:
                self._blacklist(self.last_goal_xy)

        self.goal_in_flight = False
        self.last_goal_xy = None

    # ---------------- TF helpers ----------------
    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter("global_frame").value,
                self.get_parameter("robot_frame").value,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    # ---------------- Costmap helpers ----------------
    def _cost_at_world(self, x: float, y: float) -> Optional[int]:
        cm = self.costmap
        res = cm.metadata.resolution
        ox = cm.metadata.origin.position.x
        oy = cm.metadata.origin.position.y
        sx = cm.metadata.size_x
        sy = cm.metadata.size_y

        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= sx or my >= sy:
            return None
        return int(cm.data[my * sx + mx])

    def _cost_is_acceptable(self, cost: int) -> bool:
        # Nav2 costmap2d typical:
        # 255 = NO_INFORMATION
        # 254 = LETHAL_OBSTACLE
        # 253 = INSCRIBED_INFLATED_OBSTACLE
        if bool(self.get_parameter("reject_unknown_cost").value) and cost == 255:
            return False
        if bool(self.get_parameter("reject_lethal").value) and cost == 254:
            return False
        if bool(self.get_parameter("reject_inscribed").value) and cost == 253:
            return False
        return True

    # ---------------- Map coordinate helpers ----------------
    def _map_cell_center_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        m = self.map
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        return (ox + (mx + 0.5) * res, oy + (my + 0.5) * res)

    def _use_bounds(self) -> bool:
        return bool(self.get_parameter("use_bounds").value)

    def _in_bounds_world(self, x: float, y: float) -> bool:
        return (
            float(self.get_parameter("min_x").value) <= x <= float(self.get_parameter("max_x").value)
            and float(self.get_parameter("min_y").value) <= y <= float(self.get_parameter("max_y").value)
        )

    # ---------------- Blacklist helpers ----------------
    def _blacklist(self, xy: Tuple[float, float]):
        self.blacklist_xy.append(xy)

    def _is_blacklisted(self, xy: Tuple[float, float]) -> bool:
        r = float(self.get_parameter("blacklist_radius_m").value)
        for bx, by in self.blacklist_xy:
            if math.hypot(xy[0] - bx, xy[1] - by) <= r:
                return True
        return False

    # ---------------- Utility ----------------
    def _dist(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _backoff_towards_robot(self, goal_xy: Tuple[float, float], robot_xy: Tuple[float, float], backoff_m: float) -> Tuple[float, float]:
        if backoff_m <= 1e-6:
            return goal_xy
        gx, gy = goal_xy
        rx, ry = robot_xy
        dx, dy = rx - gx, ry - gy
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return goal_xy
        return (gx + (dx / n) * backoff_m, gy + (dy / n) * backoff_m)

    def _unknown_fraction_in_bounds(self) -> Optional[float]:
        m = self.map
        w, h = m.info.width, m.info.height
        total = 0
        unk = 0

        for my in range(h):
            for mx in range(w):
                x, y = self._map_cell_center_to_world(mx, my)
                if self._use_bounds() and not self._in_bounds_world(x, y):
                    continue
                total += 1
                if m.data[my * w + mx] == -1:
                    unk += 1

        if total == 0:
            return None
        return float(unk) / float(total)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = FrontierLowCostExplorer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
