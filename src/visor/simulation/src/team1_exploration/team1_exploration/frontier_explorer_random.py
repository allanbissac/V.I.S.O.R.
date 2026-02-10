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
    cells: List[Tuple[int, int, int]]          # (mx, my)
    centroid_world: Tuple[float, float]   # (x, y) in map frame
    min_cost_cells: List[Tuple[int, int]] # (x, y) in map frame
    size: int

class FrontierExplorerRandom(Node):
    # 8-neighbour offsets
    NEIGH8 = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    ]

    def __init__(self):
        super().__init__("frontier_explorer_random")

        # -------- Parameters --------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
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

        # Timing / retries
        self.declare_parameter("tick_period_s", 1.0)

        # Robot Start coordinates
        self.declare_parameter("robot_start_x", 0.0)
        self.declare_parameter("robot_start_y", 0.0)

        # -------- Runtime state --------
        self.map: Optional[OccupancyGrid] = None
        self.costmap: Optional[OccupancyGrid] = None
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

        # -------- Costmap subscription --------
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("costmap_topic").value,
            self._on_costmap,
            map_qos
        )

        # -------- Nav client --------
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(
            float(self.get_parameter("tick_period_s").value),
            self._tick,
        )
 
        self.get_logger().info("FrontierExplorer started.")

    def _on_map(self, msg: OccupancyGrid):
        self.map = msg

    def _on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg

        # grid = msg.data
        # w = msg.info.width
        # h = msg.info.height
        # occ_free_max = int(self.get_parameter("occ_free_max").value)

        # for y in range(h):
        #     for x in range(w):
        #         idx = y * w + x
        #         if self._is_frontier_cell(x, y, grid, w, h, occ_free_max):
        #             print('o', end='')
        #         else:
        #             print('-', end='')
        #     print('')

        # print('\n')

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
        
        if not self.spin_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for Nav2 Spin action server...")
            return
        
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for Nav2 NavigateToPose action server...")
            return
        
        frontiers = self._detect_frontiers()
        self.get_logger().info(
            f"Frontiers found: {len(frontiers)} | blacklist: {len(self.blacklist)} | unknown_frac: {unknown_frac}"
        )

        if not frontiers:
            self.get_logger().info("No frontiers found right now. Finishing...")
            self.timer.cancel()
            return
        
        candidate_frontier, candidate_target = self._choose_best(frontiers, robot_xy)
        if candidate_frontier is None:
            self.get_logger().info("No valid frontier after blacklist/bounds. Stopping.")
            rclpy.shutdown()
            return

        # print(candidate_frontier.cells)
        
        goal_xy = candidate_target
        self._send_nav_goal(robot_xy, goal_xy)
    
    # --------------- Frontier Generation --------------
    def _detect_frontiers(self) -> List[Frontier]:
        m = self.costmap
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
                cluster: List[Tuple[int, int, int]] = []

                while q:
                    cx, cy = q.popleft()
                    cluster.append((cx, cy, grid[cy * w + cx]))

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
                    min_cost_cells = self._get_min_cost_cells(cluster)
                    if (
                        self._in_bounds_world(gx, gy) 
                        and self._dist((gx, gy), (self.get_parameter("robot_start_x").value, self.get_parameter("robot_start_x").value)) > 1.0
                    ):
                        frontiers.append(Frontier(cluster, (gx, gy), min_cost_cells, len(cluster)))

        # for f in frontiers:
        #     cluster = f.cells
        #     min_cost_cells = f.min_cost_cells
        #     mx = int(sum(c[0] for c in cluster) / len(cluster))
        #     my = int(sum(c[1] for c in cluster) / len(cluster))

        #     c = [(cell[0], cell[1]) for cell in cluster]
        #     for y in range(h):
        #         for x in range(w):
        #             if x == mx and y == my:
        #                 print('0', end='')
        #             else:
        #                 print('-', end='')
        #         print('')
            
        #     print('\n')

        return frontiers
    
    # ---------------- Nav2 goal sending ----------------
    def _send_nav_goal(self, robot_xy: Tuple[float, float], goal_xy: Tuple[float, float], yaw: float = -1.0):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 NavigateToPose server not ready.")
            return

        rx, ry = robot_xy
        gx, gy = goal_xy
        if yaw == -1.0:
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

            self.last_goal_xy = None
            return

        self.get_logger().warn(f"Goal failed (status={status})")
        self.goal_in_flight = False
        self.last_goal_xy = None

    def _send_spin_goal(self, radians: float = 2.0 * math.pi):
        if not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 Spin server not ready.")
            return

        goal = Spin.Goal()
        goal.target_yaw = float(radians)

        self.get_logger().info("Spinning 360°...")
        self.goal_in_flight = True
        send_goal_future = self.spin_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_spin_goal_response)

    def _on_spin_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Spin rejected.")
            self.goal_in_flight = False
            self.spin = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_spin_result)

    def _on_spin_result(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Spin succeeded.")
            self.goal_in_flight = False
            self.spin = False
            return

        self.get_logger().warn(f"Spin failed (status={status})")
        self.goal_in_flight = False
        self.spin = False

    # ---------------- Helper Functions ----------------
    def _dist(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])
    
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
    
    def _cluster_centroid_world(self, cluster: List[Tuple[int, int, int]]) -> Tuple[float, float]:
        mx = sum(c[0] for c in cluster) / len(cluster)
        my = sum(c[1] for c in cluster) / len(cluster)
        return self._map_to_world(mx, my)

    def _get_min_cost_cells(self, cluster: List[Tuple[int, int, int]]):
        min_cost = min([cell[2] for cell in cluster])
        # return [(cell[0], cell[1]) for cell in cluster if cell[2] == min_cost]
        return [self._map_to_world(cell[0], cell[1]) for cell in cluster if cell[2] == min_cost]
    
    def _choose_best(self, frontiers: List[Frontier], robot_xy: Tuple[float, float]) -> Tuple[Frontier, Tuple[float, float]]:
        w_dist = float(self.get_parameter("w_dist").value)
        w_size = float(self.get_parameter("w_size").value)

        best_frontier = None
        best_target = None
        best_score = float("inf")

        for f in frontiers:
            target_cell = self._min_cost_cell_closest_to_centroid(f)
        
            dist = self._dist(target_cell, robot_xy)
            score = w_dist * dist + w_size * (1.0 / max(1, f.size))
            if score < best_score:
                best_score = score
                best_frontier = f
                best_target = target_cell

        return best_frontier, best_target
        
    def _min_cost_cell_closest_to_centroid(self, frontier: Frontier) -> Tuple[float, float]:
        distances = [self._dist(frontier.centroid_world, cell) for cell in frontier.min_cost_cells]
        closest_index = distances.index(min(distances))
        return frontier.min_cost_cells[closest_index]


def main():
    rclpy.init()
    node = FrontierExplorerRandom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()

