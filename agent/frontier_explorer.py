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
W_SIZE = 1.0    # prefer large frontier clusters
W_DIST = 0.5    # penalise distance from robot

# Stuck detection
STUCK_DIST_THRESHOLD = 0.10   # metres — robot must move this far
STUCK_TIME_THRESHOLD = 30.0   # seconds (Nav2 rotates first; give it time to start translating)

# Blacklist radius: frontiers within this distance of a failed goal are blacklisted
BLACKLIST_RADIUS = 0.5        # metres

# Minimum cluster size to be considered a meaningful frontier
MIN_FRONTIER_SIZE = 5         # cells


@dataclass
class FrontierCluster:
    cells: list[tuple[int, int]]    # (row, col) in grid frame
    centroid_world: tuple[float, float]  # (x, y) in map frame (metres)

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
        self._last_move_time: float = time.time()

        self._blacklist: list[tuple[float, float]] = []  # world coords of failed goals
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
            return

        self._logger.info("FrontierExplorer: Nav2 ready. Starting exploration.")

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
            if not frontiers:
                self._logger.info("FrontierExplorer: no frontiers remain — exploration DONE.")
                self._done_callback()
                return

            best = self._select_best_frontier(frontiers, rx, ry)
            if best is None:
                self._logger.info("FrontierExplorer: all frontiers blacklisted — DONE.")
                self._done_callback()
                return

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
            for r, c in best.cells:
                d = math.hypot(r - centroid_r, c - centroid_c)
                if d < min_d:
                    min_d = d
                    goal_x = ox + (c + 0.5) * res
                    goal_y = oy + (r + 0.5) * res

            self._logger.info(
                f"FrontierExplorer: goal ({goal_x:.2f}, {goal_y:.2f})"
                f" [centroid ({cx:.2f}, {cy:.2f})], cluster size={best.size}"
            )

            result = self._send_goal_and_wait(goal_x, goal_y, current_map.header.frame_id)

            if result is True:
                # Blacklist centroid on success. If frontier cells persist after the
                # robot visited (unknown area is behind a wall), blacklisting prevents
                # the robot from repeatedly returning to the same inaccessible spot.
                self._blacklist.append((cx, cy))
            elif result is False:
                self._logger.info(
                    f"FrontierExplorer: goal ({cx:.2f}, {cy:.2f}) failed — blacklisting."
                )
                self._blacklist.append((cx, cy))
                # Brief pause — give Nav2 time to finish cancellation/cleanup
                # before we send the next goal (avoids acceptance future delays).
                time.sleep(3.0)
            elif result is None:
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

            clusters.append(FrontierCluster(cells=cells, centroid_world=(wx, wy)))

        return clusters

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

    # ------------------------------------------------------------------
    # Nav2 goal dispatch
    # ------------------------------------------------------------------

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
            self._logger.warning("FrontierExplorer: goal rejected by Nav2.")
            return False

        self._active_goal_handle = goal_handle
        self._last_move_time = time.time()

        with self._odom_lock:
            self._last_moved_x = self._robot_x
            self._last_moved_y = self._robot_y

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            time.sleep(0.2)

            # Stuck detection
            with self._odom_lock:
                rx, ry = self._robot_x, self._robot_y
            moved = math.hypot(rx - self._last_moved_x, ry - self._last_moved_y)
            if moved > STUCK_DIST_THRESHOLD:
                self._last_moved_x = rx
                self._last_moved_y = ry
                self._last_move_time = time.time()
            elif time.time() - self._last_move_time > STUCK_TIME_THRESHOLD:
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
