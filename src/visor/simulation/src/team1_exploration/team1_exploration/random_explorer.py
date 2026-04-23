#!/usr/bin/env python3
import math
import random
from collections import deque
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import ComputePathToPose
from action_msgs.msg import GoalStatus

import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class RandomSeenAreaExplorer(Node):
    # 8-connected neighbourhood
    NEIGH8 = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1),
    ]

    def __init__(self):
        super().__init__("random_explorer")

        # -------- Parameters --------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")

        # Occupancy interpretation
        self.declare_parameter("occ_free_max", 40)

        # Arena bounds
        self.declare_parameter("min_x", -0.55)
        self.declare_parameter("max_x", 5.35)
        self.declare_parameter("min_y", -0.55)
        self.declare_parameter("max_y", 5.35)

        # Goal selection
        self.declare_parameter("tick_period_s", 1.0)
        self.declare_parameter("min_goal_separation_m", 0.8)
        self.declare_parameter("min_last_goal_separation_m", 0.8)
        self.declare_parameter("visited_radius_m", 0.8)
        self.declare_parameter("blacklist_radius_m", 1.0)
        self.declare_parameter("goal_clearance_cells", 1)      # goal must have this many free cells around it
        self.declare_parameter("snap_radius_m", 0.8)

        # Randomisation
        self.declare_parameter("max_candidate_pool", 500)      # subsample connected free space to this many
        self.declare_parameter("max_goal_samples_per_tick", 30)
        self.declare_parameter("random_seed", -1)              # -1 => non-deterministic

        # Planner reachability filter
        self.declare_parameter("use_planner_filter", False)
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("min_path_length_m", 0.3)

        # Bookkeeping
        self.declare_parameter("visited_memory_size", 200)

        # -------- Runtime state --------
        self.map: Optional[OccupancyGrid] = None
        self.goal_in_flight = False
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self.blacklist: List[Tuple[float, float]] = []
        self.visited: List[Tuple[float, float]] = []

        seed = int(self.get_parameter("random_seed").value)
        self.rng = random.Random(None if seed < 0 else seed)

        # -------- QoS --------
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

        self.get_logger().info("RandomSeenAreaExplorer started.")

    def _on_map(self, msg: OccupancyGrid):
        self.map = msg

    # ---------------- Main loop ----------------
    def _tick(self):
        if self.map is None:
            self.get_logger().info('No map received. Waiting...')
            return

        if self.goal_in_flight:
            return
        
        self.blacklist = []

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn("Waiting for TF map->robot...")
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for Nav2 NavigateToPose action server...")
            return

        candidates = self._collect_connected_free_candidates(robot_xy)

        self.get_logger().info(
            f"Random free-space candidates: {len(candidates)} | "
            f"visited: {len(self.visited)} | blacklist: {len(self.blacklist)}"
        )

        if not candidates:
            self.get_logger().info("No valid random goals in known free space right now. Retrying...")
            return

        self.rng.shuffle(candidates)
        max_trials = min(
            len(candidates),
            int(self.get_parameter("max_goal_samples_per_tick").value)
        )

        for goal_xy in candidates[:max_trials]:
            snapped = self._snap_to_nearest_free(
                goal_xy[0], goal_xy[1],
                radius_m=float(self.get_parameter("snap_radius_m").value)
            )
            if snapped is None:
                continue

            if self._dist(robot_xy, snapped) < float(self.get_parameter("min_goal_separation_m").value):
                continue

            if self.last_goal_xy is not None and \
               self._dist(self.last_goal_xy, snapped) < float(self.get_parameter("min_last_goal_separation_m").value):
                continue

            if self._is_blacklisted(snapped[0], snapped[1]) or self._is_visited(snapped[0], snapped[1]):
                continue

            if bool(self.get_parameter("use_planner_filter").value):
                if not self._planner_ready():
                    self.get_logger().warn("Planner not ready; sending goal without reachability filter.")
                else:
                    if not self._is_reachable(robot_xy, snapped):
                        self.blacklist.append(snapped)
                        continue

            self._send_nav_goal(robot_xy, snapped)
            return

        self.get_logger().warn("Could not find a usable random goal this cycle.")

    # ---------------- Candidate generation ----------------
    def _collect_connected_free_candidates(self, robot_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Flood-fill the connected free region around the robot, then randomly pick
        from those free seen cells only.
        """
        start_ij = self._world_to_map(robot_xy[0], robot_xy[1])
        if start_ij is None:
            return []

        # If robot pose lies in non-free cell because of map discretisation/inflation,
        # snap the start to the nearest free cell first.
        if not self._cell_is_free(start_ij[0], start_ij[1]):
            snapped_start = self._snap_to_nearest_free(
                robot_xy[0], robot_xy[1],
                radius_m=float(self.get_parameter("snap_radius_m").value)
            )
            if snapped_start is None:
                return []
            start_ij = self._world_to_map(snapped_start[0], snapped_start[1])
            if start_ij is None:
                return []

        m = self.map
        w = m.info.width
        h = m.info.height
        clearance = int(self.get_parameter("goal_clearance_cells").value)

        q = deque([start_ij])
        visited_cells = {start_ij[1] * w + start_ij[0]}
        candidates: List[Tuple[float, float]] = []

        while q:
            cx, cy = q.popleft()

            wx, wy = self._map_to_world_cell_center(cx, cy)

            if self._in_bounds_world(wx, wy):
                if self._cell_has_clearance(cx, cy, clearance):
                    if self._dist((wx, wy), robot_xy) >= float(self.get_parameter("min_goal_separation_m").value):
                        if self.last_goal_xy is None or \
                           self._dist((wx, wy), self.last_goal_xy) >= float(self.get_parameter("min_last_goal_separation_m").value):
                            if not self._is_blacklisted(wx, wy) and not self._is_visited(wx, wy):
                                candidates.append((wx, wy))

            for dx, dy in self.NEIGH8:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue

                nidx = ny * w + nx
                if nidx in visited_cells:
                    continue

                if not self._cell_is_free(nx, ny):
                    continue

                nwx, nwy = self._map_to_world_cell_center(nx, ny)
                if not self._in_bounds_world(nwx, nwy):
                    continue

                visited_cells.add(nidx)
                q.append((nx, ny))

        max_pool = int(self.get_parameter("max_candidate_pool").value)
        if len(candidates) > max_pool:
            candidates = self.rng.sample(candidates, max_pool)

        return candidates

    def _cell_has_clearance(self, mx: int, my: int, clearance_cells: int) -> bool:
        """
        Require a small free neighbourhood around the selected goal cell.
        Helps avoid picking cells right against walls/obstacles.
        """
        m = self.map
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                nx, ny = mx + dx, my + dy
                if nx < 0 or ny < 0 or nx >= m.info.width or ny >= m.info.height:
                    return False
                if not self._cell_is_free(nx, ny):
                    return False
        return True

    # ---------------- Coordinate helpers ----------------
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

    def _map_to_world(self, mx: float, my: float) -> Tuple[float, float]:
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        x = origin.x + (mx + 0.5) * res
        y = origin.y + (my + 0.5) * res
        return x, y

    def _map_to_world_cell_center(self, mx: int, my: int) -> Tuple[float, float]:
        return self._map_to_world(float(mx), float(my))

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
            return self._map_to_world_cell_center(mx0, my0)

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
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _in_bounds_world(self, x: float, y: float) -> bool:
        return (
            float(self.get_parameter("min_x").value) <= x <= float(self.get_parameter("max_x").value)
            and float(self.get_parameter("min_y").value) <= y <= float(self.get_parameter("max_y").value)
        )

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
            x0 = path.poses[i - 1].pose.position.x
            y0 = path.poses[i - 1].pose.position.y
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

        self.get_logger().info(f"Random goal -> ({gx:.2f}, {gy:.2f}), yaw={yaw:.2f}")
        self.goal_in_flight = True
        self.last_goal_xy = goal_xy

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"Goal send failed: {e}")
            self.goal_in_flight = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected, blacklisting.")
            if self.last_goal_xy is not None:
                self.blacklist.append(self.last_goal_xy)
            self.goal_in_flight = False
            self.last_goal_xy = None
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        failed_goal = self.last_goal_xy  # preserve before clearing

        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().warn(f"Failed to get Nav2 result: {e}")
            self.goal_in_flight = False
            if failed_goal is not None:
                self.blacklist.append(failed_goal)
            self.last_goal_xy = None
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded.")
            self.goal_in_flight = False

            if failed_goal is not None:
                self.visited.append(failed_goal)
                max_mem = int(self.get_parameter("visited_memory_size").value)
                if len(self.visited) > max_mem:
                    self.visited = self.visited[-max_mem:]

            self.last_goal_xy = None
            return

        self.get_logger().warn(f"Goal failed (status={status}).")
        self.goal_in_flight = False

        if failed_goal is not None:
            self.blacklist.append(failed_goal)

        self.last_goal_xy = None

    # ---------------- TF ----------------
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


def main():
    try:
        rclpy.init()
        node = RandomSeenAreaExplorer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()