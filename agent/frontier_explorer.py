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
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
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
PATROL_MIN_DIST = 2.0     # metres — patrol target must be > this far from all visited goals
                          # 3.0m excluded fire_ext#2 at (1.7,4.7) which is ~2.7m from robot start
PATROL_STEP_M = 1.0       # metres — coarse grid sampling resolution for patrol targets

# Minimum cluster size to be considered a meaningful frontier
MIN_FRONTIER_SIZE = 5         # cells


@dataclass
class GoalStats:
    """Per-goal timing breakdown. All times in sim-seconds; NaN = not applicable."""
    goal_num: int
    is_patrol: bool
    result: Optional[bool]          # True=success, False=fail/stuck, None=rejected
    bfs_and_selection_s: float      # BFS + scoring + inter-goal overhead
    accept_latency_s: float         # send_goal_async → goal accepted (NaN if not accepted)
    time_to_first_move_s: float     # accepted → first 0.15 m displacement (NaN if never moved)
    nav_time_s: float               # accepted → result (0.0 if never accepted)
    post_goal_pause_s: float        # recovery sleeps in explore_loop after dispatch returns


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
        self._goal_stats: list[GoalStats] = []

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

        # Service client to clear global costmap once after a failed goal.
        # Prevents "Start occupied" cascade: robot arrives at frontier → SLAM
        # maps new walls → inflation marks robot's cell as LETHAL → planner
        # refuses every subsequent goal. One-shot clear per failure (not a loop).
        self._clear_global_costmap = node.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )

        # Backup action: physically moves robot ~0.3m backward after a failure
        # so its cell is no longer inside an inflation zone before replanning.
        self._backup_client = ActionClient(node, BackUp, "backup")

        # Local costmap clear — safe to call after failure (rebuilds from LiDAR
        # within one update cycle). Clears stale obstacle data so MPPI gets a
        # fresh view and can generate forward commands.
        self._clear_local_costmap = node.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )

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
        _reject_streak: int = 0          # consecutive rejections of the same centroid
        _last_reject_cx: float = math.nan
        _last_reject_cy: float = math.nan
        MAX_REJECT_STREAK: int = 3       # blacklist after this many consecutive rejections

        while self._exploring and rclpy.ok():
            with self._map_lock:
                current_map = self._map

            if current_map is None:
                self._logger.info("FrontierExplorer: waiting for /map…")
                time.sleep(1.0)
                continue

            with self._odom_lock:
                rx, ry = self._robot_x, self._robot_y

            _t_bfs_start = self._sim_time()
            frontiers = self._detect_frontiers(current_map)
            _t_bfs = self._sim_time() - _t_bfs_start
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
                for r, c in best.cells:
                    d = math.hypot(r - centroid_r, c - centroid_c)
                    if d < min_d:
                        min_d = d
                        goal_x = ox + (c + 0.5) * res
                        goal_y = oy + (r + 0.5) * res
            else:
                # Frontier exhausted — switch to patrol mode.
                # LIDAR coverage (98%+) does NOT mean camera coverage: the robot must
                # physically visit all map regions so the detector can see objects that
                # were only LIDAR-scanned from afar.
                patrol = self._select_patrol_target(current_map)
                if patrol is None:
                    self._logger.info("FrontierExplorer: exploration and patrol DONE.")
                    break
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
                    f" size={best.size},"
                    f" idle_since_last={_t_idle:.1f}s bfs={_t_bfs:.2f}s"
                )
            else:
                self._logger.info(
                    f"FrontierExplorer: PATROL#{_goal_num} ({goal_x:.2f}, {goal_y:.2f})"
                    f" idle_since_last={_t_idle:.1f}s bfs={_t_bfs:.2f}s"
                )

            result, _t_accept_lat, _t_first_move, _t_nav = self._send_goal_and_wait(
                goal_x, goal_y, current_map.header.frame_id
            )

            _t_after_nav = self._sim_time()
            self._visited_goals.append((cx, cy))

            _accept_str = f"{_t_accept_lat:.1f}s" if not math.isnan(_t_accept_lat) else "n/a"
            _move_str = f"{_t_first_move:.1f}s" if not math.isnan(_t_first_move) else "n/a"

            if result is True:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f}) SUCCEEDED"
                    f" accept={_accept_str} first_move={_move_str} nav={_t_nav:.1f}s — blacklisting."
                )
                # Blacklist centroid on success. If frontier cells persist after the
                # robot visited (unknown area is behind a wall), blacklisting prevents
                # the robot from repeatedly returning to the same inaccessible spot.
                self._blacklist.append((cx, cy))
            elif result is False:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f}) FAILED"
                    f" accept={_accept_str} first_move={_move_str} nav={_t_nav:.1f}s — blacklisting."
                )
                self._blacklist.append((cx, cy))
                # Back up + clear costmap after failure to prevent "Start occupied"
                # cascade: robot may be inside an inflation zone after a stuck/abort;
                # backing up first moves it to free space before the costmap rebuilds.
                self._recover_from_occupied_start()
                # Brief pause — give Nav2 time to finish cancellation/cleanup
                # before we send the next goal (avoids acceptance future delays).
                time.sleep(3.0)
            elif result is None:
                # Track consecutive rejections of the same centroid.
                if math.hypot(cx - _last_reject_cx, cy - _last_reject_cy) < BLACKLIST_RADIUS:
                    _reject_streak += 1
                else:
                    _reject_streak = 1
                _last_reject_cx, _last_reject_cy = cx, cy

                if _reject_streak >= MAX_REJECT_STREAK:
                    self._logger.warning(
                        f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f})"
                        f" rejected {_reject_streak}× in a row — blacklisting."
                    )
                    self._blacklist.append((cx, cy))
                    _reject_streak = 0
                else:
                    self._logger.info(
                        f"FrontierExplorer: goal#{_goal_num} TIMEOUT/REJECT"
                        f" nav={_t_nav:.1f}s — streak={_reject_streak}/{MAX_REJECT_STREAK}, retrying."
                    )
                # Nav2 busy or acceptance timed out — brief pause before retry.
                time.sleep(5.0)

            _t_last_goal_end = self._sim_time()
            self._goal_stats.append(GoalStats(
                goal_num=_goal_num,
                is_patrol=is_patrol,
                result=result,
                bfs_and_selection_s=_t_idle,
                accept_latency_s=_t_accept_lat,
                time_to_first_move_s=_t_first_move,
                nav_time_s=_t_nav,
                post_goal_pause_s=_t_last_goal_end - _t_after_nav,
            ))

        # Always log timing table on exit (time-limit, natural completion, or stop)
        self._log_timing_table()
        # Only signal done if exploration finished naturally (not stopped externally)
        if self._exploring:
            self._done_callback()

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

    def _recover_from_occupied_start(self) -> None:
        """
        Recovery after a failed goal when the robot may be in an inflation zone.
        1. Back up 0.3 m so the robot's cell is no longer LETHAL.
        2. Clear the global costmap so the planner sees the updated free space.
        Without the backup, costmap clear is ineffective: SLAM immediately
        re-inflates the same cell from the current LiDAR scan.
        """
        # Step 1: back up 0.3 m
        if self._backup_client.wait_for_server(timeout_sec=1.0):
            goal = BackUp.Goal()
            goal.target = Point(x=0.30, y=0.0, z=0.0)  # back up 0.3 m
            goal.speed = 0.10                            # m/s (slow, safe)
            goal.time_allowance.sec = 10
            future = self._backup_client.send_goal_async(goal)
            deadline = time.time() + 12.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.1)
            if future.done() and future.result() and future.result().accepted:
                result_future = future.result().get_result_async()
                deadline = time.time() + 10.0
                while not result_future.done() and time.time() < deadline:
                    time.sleep(0.1)
            self._logger.info("FrontierExplorer: backup complete.")
        else:
            self._logger.warning("FrontierExplorer: backup server not available — skipping backup.")

        # Step 2: clear both costmaps
        for svc, name in [
            (self._clear_local_costmap, "local"),
            (self._clear_global_costmap, "global"),
        ]:
            if svc.wait_for_service(timeout_sec=1.0):
                future = svc.call_async(ClearEntireCostmap.Request())
                deadline = time.time() + 3.0
                while not future.done() and time.time() < deadline:
                    time.sleep(0.05)
        self._logger.info("FrontierExplorer: local+global costmaps cleared.")

    def _log_timing_table(self) -> None:
        """Log a per-phase timing summary for the completed exploration run."""
        stats = self._goal_stats
        if not stats:
            return

        def _vals(attr: str) -> list[float]:
            return [getattr(s, attr) for s in stats if not math.isnan(getattr(s, attr))]

        nav_stats = [s for s in stats if not math.isnan(s.accept_latency_s)]
        travel_vals = [
            s.nav_time_s - s.time_to_first_move_s
            for s in nav_stats
            if not math.isnan(s.time_to_first_move_s)
        ]

        phases: list[tuple[str, list[float]]] = [
            ("BFS + selection",   _vals("bfs_and_selection_s")),
            ("Nav2 accept",       _vals("accept_latency_s")),
            ("Rotation / spin-up", _vals("time_to_first_move_s")),
            ("Travel to goal",    travel_vals),
            ("Post-goal pause",   _vals("post_goal_pause_s")),
        ]

        total_s = sum(
            s.bfs_and_selection_s + s.nav_time_s + s.post_goal_pause_s
            for s in stats
        )
        n_ok = sum(1 for s in stats if s.result is True)
        n_fail = sum(1 for s in stats if s.result is False)
        n_rej = sum(1 for s in stats if s.result is None)

        sep = "=" * 72
        lines = [
            "",
            sep,
            "TIMING TABLE — per-goal navigation stack breakdown (sim-seconds)",
            sep,
            f"{'Phase':<24} {'N':>4} {'Mean':>7} {'Median':>7} {'Max':>7} {'% total':>8}",
            "-" * 56,
        ]
        for name, vals in phases:
            if not vals:
                lines.append(f"  {name:<22} {'—':>4}")
                continue
            mean   = float(np.mean(vals))
            median = float(np.median(vals))
            mx     = float(np.max(vals))
            pct    = 100.0 * sum(vals) / total_s if total_s > 0 else 0.0
            lines.append(
                f"  {name:<22} {len(vals):>4} {mean:>6.1f}s {median:>6.1f}s"
                f" {mx:>6.1f}s {pct:>7.1f}%"
            )
        lines += [
            "-" * 56,
            f"  Goals dispatched: {len(stats)} | success={n_ok} fail={n_fail} rejected={n_rej}",
            f"  Total tracked sim-time: {total_s:.0f}s",
            sep,
        ]
        for line in lines:
            self._logger.info(line)

    def _sim_time(self) -> float:
        """Return current ROS clock time in seconds (sim time when use_sim_time=True)."""
        return self._node.get_clock().now().nanoseconds / 1e9

    def _send_goal_and_wait(
        self, goal_x: float, goal_y: float, frame_id: str
    ) -> tuple[Optional[bool], float, float, float]:
        """
        Send a NavigateToPose goal and block until it succeeds, fails, or the
        robot gets stuck.

        Returns (result, accept_latency_s, time_to_first_move_s, nav_time_s):
        - result: True=success, False=fail/stuck, None=rejected/timed-out
        - accept_latency_s: sim-seconds from send to accepted (NaN if not accepted)
        - time_to_first_move_s: sim-seconds from accepted to first 0.15 m displacement
                                (NaN if the robot never moved)
        - nav_time_s: sim-seconds from accepted to result (0.0 if never accepted)

        NOTE: the explore loop is responsible for any post-return recovery sleeps.
        """
        _nan = float("nan")

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
            return None, _nan, _nan, 0.0
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            # Rejection is NOT the same as an unreachable frontier — Nav2 may be
            # temporarily busy (e.g. finishing a cancel/preempt cycle). Never
            # blacklist on rejection; just return and let the explore loop retry.
            self._logger.warning(
                "FrontierExplorer: goal rejected by Nav2 — not blacklisting, waiting to retry."
            )
            return None, _nan, _nan, 0.0

        _t_accept = self._sim_time()
        accept_latency = _t_accept - _t_send
        self._logger.info(f"FrontierExplorer: TIMING accept={accept_latency:.1f}s")

        self._active_goal_handle = goal_handle
        self._last_move_time = _t_accept

        with self._odom_lock:
            self._last_moved_x = self._robot_x
            self._last_moved_y = self._robot_y
            # Snapshot position at accept time to track first-move milestone.
            _accepted_x, _accepted_y = self._robot_x, self._robot_y

        result_future = goal_handle.get_result_async()
        _first_move_t = _nan  # sim-seconds from accept to first 0.15 m displacement

        while not result_future.done():
            time.sleep(0.2)

            # Re-check immediately after sleep — goal may have completed while we
            # were sleeping. Doing this BEFORE the stuck check avoids a race where
            # the stuck timer fires in the same iteration the goal succeeds, causing
            # a false-positive FAILED result (and spurious blacklist).
            if result_future.done():
                break

            # Stuck detection — use sim clock so RTF oscillations don't cause
            # false positives (wall-clock 30 s at RTF=0.1 = only 3 sim seconds).
            with self._odom_lock:
                rx, ry = self._robot_x, self._robot_y

            # Track first significant displacement from accept position.
            if math.isnan(_first_move_t):
                if math.hypot(rx - _accepted_x, ry - _accepted_y) > 0.15:
                    _first_move_t = self._sim_time() - _t_accept

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
                return False, accept_latency, _first_move_t, self._sim_time() - _t_accept

            if not self._exploring:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cancel_deadline:
                    time.sleep(0.1)
                return False, accept_latency, _first_move_t, self._sim_time() - _t_accept

        result = result_future.result()
        nav_time = self._sim_time() - _t_accept
        self._active_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True, accept_latency, _first_move_t, nav_time

        self._logger.info(
            f"FrontierExplorer: goal finished with status {result.status}."
        )
        return False, accept_latency, _first_move_t, nav_time
