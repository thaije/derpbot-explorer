"""
FrontierExplorer — BFS-based frontier detection on the Nav2 /map, with
Nav2 NavigateToPose goal dispatch.

Runs as a component of agent_node; call spin() to start the exploration loop.

Frontier lifecycle:
  subscribe /map → BFS detect frontiers → cluster → score → send NavigateToPose
  → on success: recompute
  → on failure / stuck: blacklist centroid → recompute
  → no frontiers: signal DONE
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# Grid cell values from Nav2 OccupancyGrid
FREE = 0
UNKNOWN = -1
# occupied: any value > 50

# Frontier scoring weights
# Score = W_SIZE * cluster.size - W_DIST * dist
W_SIZE = 1.0    # prefer large frontier clusters (larger cluster = more open frontier edge)
W_DIST = 1.5    # penalise distance from robot (lowered from 2.0 to push the robot
                # toward distant unexplored areas; 4.0 is too local, 0.5 causes thrashing)

# Info-gain flood fill: disabled (replaced scoring approach; see git history)
# The flood fill caused all frontiers to hit the cap early in exploration,
# making scores equal and causing micro-stepping + Nav2 path failures.
# Left in codebase for future investigation with proper connected-components normalization.
MAX_FLOOD_CELLS = 5000  # kept for _flood_unknown method signature compatibility

# Stuck detection
STUCK_DIST_THRESHOLD = 0.10   # metres — robot must move this far
STUCK_TIME_THRESHOLD = 30.0   # seconds (Nav2 rotates first; give it time to start translating)

# Blacklist radius: frontiers within this distance of a failed goal are blacklisted
BLACKLIST_RADIUS = 0.5        # metres

# Patrol mode: kicks in after all LIDAR frontiers are exhausted.
# Selects free cells that are far from any previously visited goal (LIDAR coverage
# does not imply camera coverage; the robot must physically visit all regions).
PATROL_MIN_DIST = 3.0     # metres — patrol target must be > this far from all visited goals
PATROL_STEP_M = 1.0       # metres — coarse grid sampling resolution for patrol targets

# Minimum cluster size to be considered a meaningful frontier
MIN_FRONTIER_SIZE = 5         # cells

# Goal clearance: nav goals must be this far from any occupied cell.
# robot_radius(0.22) + inflation(0.12) = 0.34m → 7 cells at 5cm.
# Frontier cells adjacent to furniture (desks, cabinets) are filtered out;
# if no frontier cell passes, the best (minimum centroid-distance) cell is used.
GOAL_CLEARANCE_M = 0.36       # metres


@dataclass
class FrontierCluster:
    cells: list[tuple[int, int]]    # (row, col) in grid frame
    centroid_world: tuple[float, float]  # (x, y) in map frame (metres)
    reachable_unknown: int = 0      # flood-fill info gain (unknown cells accessible through frontier)

    @property
    def size(self) -> int:
        return len(self.cells)


class FrontierExplorer:
    """
    Frontier-based exploration using Nav2 NavigateToPose.

    Parameters
    ----------
    node : rclpy.Node
        The parent ROS 2 node to attach subscriptions/clients to.
    done_callback : Callable
        Called with no arguments when exploration is complete (no frontiers left).
    """

    def __init__(self, node: Node, done_callback: Callable[[], None]):
        self._node = node
        self._done_callback = done_callback
        self._logger = node.get_logger()

        self._map: Optional[OccupancyGrid] = None
        self._map_lock = threading.Lock()

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._odom_lock = threading.Lock()

        # Last position used for stuck detection
        self._last_moved_x: float = 0.0
        self._last_moved_y: float = 0.0
        self._last_move_time: float = 0.0  # set to sim time when each goal starts

        self._blacklist: list[tuple[float, float]] = []  # world coords of failed goals
        self._visited_goals: list[tuple[float, float]] = []  # all goal positions attempted
        self._active_goal_handle = None
        self._exploring = False

        # Subscriptions
        # /map is published TRANSIENT_LOCAL — subscriber must also use TRANSIENT_LOCAL
        # to receive the held last message immediately on subscribe.
        # ReentrantCallbackGroup ensures the map callback is never blocked by other
        # callbacks in the node's default MutuallyExclusiveCallbackGroup (e.g. TF
        # listener callbacks that fire at high frequency).
        map_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        reentrant = rclpy.callback_groups.ReentrantCallbackGroup()
        node.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos,
                                 callback_group=reentrant)
        node.create_subscription(
            Odometry, "/derpbot_0/odom", self._odom_cb,
            rclpy.qos.QoSProfile(depth=10),
        )

        # Nav2 action client
        self._nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        with self._map_lock:
            self._map = msg

    def _odom_cb(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start exploration in a background thread."""
        self._exploring = True
        t = threading.Thread(target=self._explore_loop, daemon=True)
        t.start()

    def stop(self) -> None:
        self._exploring = False
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # Exploration loop
    # ------------------------------------------------------------------

    def _explore_loop(self) -> None:
        self._logger.info("FrontierExplorer: waiting for Nav2 action server…")
        if not self._nav_client.wait_for_server(timeout_sec=60.0):
            self._logger.error("NavigateToPose action server not available — aborting.")
            self._done_callback()
            return

        self._logger.info("FrontierExplorer: Nav2 ready. Starting exploration.")

        _t_last_goal_end: float = self._sim_time()  # sim time when last goal cycle ended
        _goal_num: int = 0

        while self._exploring and rclpy.ok():
            with self._map_lock:
                current_map = self._map

            if current_map is None:
                self._logger.info("FrontierExplorer: waiting for /map…")
                time.sleep(1.0)
                continue

            with self._odom_lock:
                rx, ry = self._robot_x, self._robot_y

            frontiers = self._detect_frontiers(current_map)
            best = self._select_best_frontier(frontiers, rx, ry) if frontiers else None

            is_patrol = False
            if best is not None:
                cx, cy = best.centroid_world  # used for scoring/blacklisting

                # Navigate to the cluster cell closest to the centroid, not to the robot.
                # - All frontier cells are free (value == 0), so this is always navigable.
                # - Centroid can land outside free cells (average may fall in unknown/
                #   inflated space). Closest-to-centroid picks a real free cell near the
                #   geometric interior of the frontier.
                # - Unlike closest-to-robot, this requires the robot to actually travel
                #   toward the unexplored area.
                res = current_map.info.resolution
                ox = current_map.info.origin.position.x
                oy = current_map.info.origin.position.y
                centroid_r = (cy - oy) / res - 0.5
                centroid_c = (cx - ox) / res - 0.5
                goal_x, goal_y = cx, cy  # fallback
                min_d = math.inf
                min_d_any = math.inf  # best without clearance check (fallback)
                goal_x_any, goal_y_any = cx, cy
                clearance_cells = max(1, round(GOAL_CLEARANCE_M / res))
                _map_data = np.array(
                    current_map.data, dtype=np.int8
                ).reshape((current_map.info.height, current_map.info.width))
                _height, _width = _map_data.shape
                for r, c in best.cells:
                    d = math.hypot(r - centroid_r, c - centroid_c)
                    # Track best without clearance (fallback)
                    if d < min_d_any:
                        min_d_any = d
                        goal_x_any = ox + (c + 0.5) * res
                        goal_y_any = oy + (r + 0.5) * res
                    # Clearance check: no occupied cell within clearance_cells
                    r_lo = max(0, r - clearance_cells)
                    r_hi = min(_height, r + clearance_cells + 1)
                    c_lo = max(0, c - clearance_cells)
                    c_hi = min(_width, c + clearance_cells + 1)
                    if np.any(_map_data[r_lo:r_hi, c_lo:c_hi] > 50):
                        continue  # too close to obstacle — skip
                    if d < min_d:
                        min_d = d
                        goal_x = ox + (c + 0.5) * res
                        goal_y = oy + (r + 0.5) * res
                # If no cell passed clearance, fall back to best without check
                if min_d == math.inf:
                    goal_x, goal_y = goal_x_any, goal_y_any
            else:
                # Frontier exhausted — switch to patrol mode.
                # LIDAR coverage (98%+) does NOT mean camera coverage: the robot must
                # physically visit all map regions so the detector can see objects that
                # were only LIDAR-scanned from afar.
                patrol = self._select_patrol_target(current_map)
                if patrol is None:
                    self._logger.info("FrontierExplorer: exploration and patrol DONE.")
                    self._done_callback()
                    return
                goal_x, goal_y = patrol
                cx, cy = patrol
                is_patrol = True

            _goal_num += 1
            _t_idle = self._sim_time() - _t_last_goal_end  # BFS + selection overhead
            _t_goal_start = self._sim_time()
            if not is_patrol:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({goal_x:.2f}, {goal_y:.2f})"
                    f" [centroid ({cx:.2f}, {cy:.2f})],"
                    f" size={best.size}, reachable_unknown={best.reachable_unknown},"
                    f" idle_since_last={_t_idle:.1f}s"
                )
            else:
                self._logger.info(
                    f"FrontierExplorer: PATROL#{_goal_num} ({goal_x:.2f}, {goal_y:.2f})"
                    f" idle_since_last={_t_idle:.1f}s"
                )

            result = self._send_goal_and_wait(goal_x, goal_y, current_map.header.frame_id)

            _t_nav = self._sim_time() - _t_goal_start
            _t_last_goal_end = self._sim_time()
            self._visited_goals.append((cx, cy))

            if result is True:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f}) SUCCEEDED"
                    f" nav_time={_t_nav:.1f}s — blacklisting."
                )
                # Blacklist centroid on success. If frontier cells persist after the
                # robot visited (unknown area is behind a wall), blacklisting prevents
                # the robot from repeatedly returning to the same inaccessible spot.
                self._blacklist.append((cx, cy))
            elif result is False:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f}) FAILED"
                    f" nav_time={_t_nav:.1f}s — blacklisting."
                )
                self._blacklist.append((cx, cy))
                # Brief pause — give Nav2 time to finish cancellation/cleanup
                # before we send the next goal (avoids acceptance future delays).
                time.sleep(3.0)
            elif result is None:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} TIMEOUT (acceptance)"
                    f" nav_time={_t_nav:.1f}s — not blacklisting, retrying."
                )
                # Acceptance timed out — Nav2 busy, don't blacklist, just retry.
                time.sleep(3.0)

    # ------------------------------------------------------------------
    # Frontier detection — BFS
    # ------------------------------------------------------------------

    def _detect_frontiers(self, grid: OccupancyGrid) -> list[FrontierCluster]:
        """
        Return list of frontier clusters.
        A frontier cell is a FREE cell with at least one UNKNOWN neighbour.
        """
        width = grid.info.width
        height = grid.info.height
        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        # Mark frontier cells
        frontier_mask = np.zeros((height, width), dtype=bool)
        # Free cells (== 0) adjacent to unknown cells (== -1)
        free = data == 0
        # Shift in 4 directions and check for unknown neighbours
        unknown = data == UNKNOWN
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(unknown, (dr, dc), axis=(0, 1))
            # Zero out wrapped edges
            if dr == -1:
                shifted[-1, :] = False
            elif dr == 1:
                shifted[0, :] = False
            if dc == -1:
                shifted[:, -1] = False
            elif dc == 1:
                shifted[:, 0] = False
            frontier_mask |= (free & shifted)

        # BFS-cluster the frontier cells
        visited = np.zeros((height, width), dtype=bool)
        clusters: list[FrontierCluster] = []

        rows, cols = np.where(frontier_mask)
        for start_r, start_c in zip(rows, cols):
            if visited[start_r, start_c]:
                continue
            # BFS from this seed
            q: deque[tuple[int, int]] = deque()
            q.append((start_r, start_c))
            visited[start_r, start_c] = True
            cells: list[tuple[int, int]] = []
            while q:
                r, c = q.popleft()
                cells.append((r, c))
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < height and 0 <= nc < width:
                        if frontier_mask[nr, nc] and not visited[nr, nc]:
                            visited[nr, nc] = True
                            q.append((nr, nc))

            if len(cells) < MIN_FRONTIER_SIZE:
                continue

            # Compute centroid in world frame
            res = grid.info.resolution
            ox = grid.info.origin.position.x
            oy = grid.info.origin.position.y
            mean_r = sum(r for r, _ in cells) / len(cells)
            mean_c = sum(c for _, c in cells) / len(cells)
            wx = ox + (mean_c + 0.5) * res
            wy = oy + (mean_r + 0.5) * res

            cluster = FrontierCluster(cells=cells, centroid_world=(wx, wy))
            cluster.reachable_unknown = self._flood_unknown(cluster, data, height, width)
            clusters.append(cluster)

        return clusters

    def _flood_unknown(
        self,
        cluster: FrontierCluster,
        data: np.ndarray,
        height: int,
        width: int,
    ) -> int:
        """
        Flood-fill through unknown cells reachable from the unexplored side of
        this frontier cluster.  Returns the count of reachable unknown cells,
        capped at MAX_FLOOD_CELLS to bound computation time.

        This is the information-gain estimate: a frontier along a thin wall edge
        scores near 0; a frontier through a doorway into an unmapped room scores
        proportional to room size.
        """
        unknown = data == UNKNOWN

        # Seeds: unknown cells immediately adjacent to frontier cells
        visited_local = np.zeros((height, width), dtype=bool)
        q: deque[tuple[int, int]] = deque()
        for r, c in cluster.cells:
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width:
                    if unknown[nr, nc] and not visited_local[nr, nc]:
                        visited_local[nr, nc] = True
                        q.append((nr, nc))

        if not q:
            return cluster.size  # fallback: no unknown neighbours (shouldn't happen)

        # BFS through unknown cells only
        count = 0
        while q and count < MAX_FLOOD_CELLS:
            r, c = q.popleft()
            count += 1
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width:
                    if unknown[nr, nc] and not visited_local[nr, nc]:
                        visited_local[nr, nc] = True
                        q.append((nr, nc))

        return count

    # ------------------------------------------------------------------
    # Frontier selection — scoring
    # ------------------------------------------------------------------

    def _select_best_frontier(
        self,
        clusters: list[FrontierCluster],
        robot_x: float,
        robot_y: float,
    ) -> Optional[FrontierCluster]:
        best_score = -math.inf
        best_cluster = None

        for cluster in clusters:
            cx, cy = cluster.centroid_world
            if self._is_blacklisted(cx, cy):
                continue
            dist = math.hypot(cx - robot_x, cy - robot_y)
            score = W_SIZE * cluster.size - W_DIST * dist
            if score > best_score:
                best_score = score
                best_cluster = cluster

        return best_cluster

    def _is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by in self._blacklist:
            if math.hypot(x - bx, y - by) < BLACKLIST_RADIUS:
                return True
        return False

    def _select_patrol_target(
        self, grid: OccupancyGrid
    ) -> Optional[tuple[float, float]]:
        """
        Find the free cell farthest from all previously visited goal positions.
        Samples on a coarse grid (PATROL_STEP_M resolution) for efficiency.
        Returns world (x, y) of the best unvisited cell, or None if all free
        cells are within PATROL_MIN_DIST of a visited goal.
        """
        data = np.array(grid.data, dtype=np.int8).reshape(
            (grid.info.height, grid.info.width)
        )
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        step = max(1, round(PATROL_STEP_M / res))

        best_pos: Optional[tuple[float, float]] = None
        best_dist = PATROL_MIN_DIST  # must exceed threshold to qualify

        for r in range(0, grid.info.height, step):
            for c in range(0, grid.info.width, step):
                if data[r, c] != FREE:
                    continue
                wx = ox + (c + 0.5) * res
                wy = oy + (r + 0.5) * res
                if self._is_blacklisted(wx, wy):
                    continue
                if not self._visited_goals:
                    with self._odom_lock:
                        rx, ry = self._robot_x, self._robot_y
                    min_d = math.hypot(wx - rx, wy - ry)
                else:
                    min_d = min(
                        math.hypot(wx - px, wy - py)
                        for px, py in self._visited_goals
                    )
                if min_d > best_dist:
                    best_dist = min_d
                    best_pos = (wx, wy)

        if best_pos is not None:
            self._logger.info(
                f"FrontierExplorer: patrol target selected ({best_pos[0]:.2f}, {best_pos[1]:.2f})"
                f" min_dist_from_visited={best_dist:.1f}m"
            )
        return best_pos

    # ------------------------------------------------------------------
    # Nav2 goal dispatch
    # ------------------------------------------------------------------

    def _sim_time(self) -> float:
        """Return current ROS clock time in seconds (sim time when use_sim_time=True)."""
        return self._node.get_clock().now().nanoseconds / 1e9

    def _send_goal_and_wait(
        self, goal_x: float, goal_y: float, frame_id: str
    ) -> Optional[bool]:
        """
        Send a NavigateToPose goal and block until it succeeds, fails, or the
        robot gets stuck.
        Returns True on success, False on failure/stuck (blacklist), None on
        acceptance timeout (skip without blacklisting).
        """
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        _t_send = self._sim_time()
        future = self._nav_client.send_goal_async(goal_msg)
        # Poll — do NOT call spin_until_future_complete here; the node is already
        # spinning in a MultiThreadedExecutor in agent_node.py.
        deadline = time.time() + 90.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            self._logger.warning("FrontierExplorer: timeout waiting for goal acceptance — skipping (not blacklisting).")
            return None
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            # Rejection is NOT the same as an unreachable frontier — Nav2 may be
            # temporarily busy (e.g. finishing a cancel/preempt cycle). Never
            # blacklist on rejection; just pause and let the explore loop retry.
            self._logger.warning(
                "FrontierExplorer: goal rejected by Nav2 — not blacklisting, waiting to retry."
            )
            time.sleep(5.0)   # give Nav2 time to finish any cleanup
            return None

        _t_accept = self._sim_time()
        self._logger.debug(f"FrontierExplorer: TIMING accept={_t_accept - _t_send:.1f}s")

        self._active_goal_handle = goal_handle
        self._last_move_time = self._sim_time()

        with self._odom_lock:
            self._last_moved_x = self._robot_x
            self._last_moved_y = self._robot_y

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            time.sleep(0.2)

            # Stuck detection — use sim clock so RTF oscillations don't cause
            # false positives (wall-clock 30 s at RTF=0.1 = only 3 sim seconds).
            with self._odom_lock:
                rx, ry = self._robot_x, self._robot_y
            moved = math.hypot(rx - self._last_moved_x, ry - self._last_moved_y)
            if moved > STUCK_DIST_THRESHOLD:
                self._last_moved_x = rx
                self._last_moved_y = ry
                self._last_move_time = self._sim_time()
            elif self._sim_time() - self._last_move_time > STUCK_TIME_THRESHOLD:
                self._logger.warning(
                    "FrontierExplorer: stuck detected — cancelling goal."
                )
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cancel_deadline:
                    time.sleep(0.1)
                return False

            if not self._exploring:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cancel_deadline:
                    time.sleep(0.1)
                return False

        result = result_future.result()
        self._active_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True

        self._logger.info(
            f"FrontierExplorer: goal finished with status {result.status}."
        )
        return False
