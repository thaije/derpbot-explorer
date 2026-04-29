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
import os
import threading
import time
from collections import defaultdict, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

_WALL_T0 = time.time()
from typing import Callable, Optional

import numpy as np
import rclpy
from profiler import GoalStats, log_timing_table, write_timeline
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Twist
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node

# Debug: dump selected frontier + /map + global costmap to /tmp for offline analysis.
# Gated on env var so normal runs aren't polluted. See issue #10.
_FRONTIER_DEBUG = os.environ.get("DERPBOT_FRONTIER_DEBUG") == "1"
_FRONTIER_DEBUG_DIR = Path(os.environ.get("DERPBOT_FRONTIER_DEBUG_DIR", "/tmp/derpbot_frontier_dumps"))

# Grid cell values from Nav2 OccupancyGrid
FREE = 0
UNKNOWN = -1
# occupied: any value > 50

# Frontier scoring weights
# Score = W_SIZE * cluster.size - W_DIST * dist
W_SIZE = (
    1.0  # prefer large frontier clusters (larger cluster = more open frontier edge)
)
W_DIST = 1.5  # penalise distance from robot (lowered from 2.0 to push the robot
# toward distant unexplored areas; 4.0 is too local, 0.5 causes thrashing)

# Stuck detection
STUCK_DIST_THRESHOLD = 0.10  # metres — robot must move this far
STUCK_TIME_THRESHOLD = (
    30.0  # seconds (Nav2 rotates first; give it time to start translating)
)

# Blacklist radius: frontiers within this distance of a failed goal are blacklisted
BLACKLIST_RADIUS = 0.5  # metres

# Patrol mode: kicks in after all LIDAR frontiers are exhausted.
# Selects free cells that are far from any previously visited goal (LIDAR coverage
# does not imply camera coverage; the robot must physically visit all regions).
PATROL_MIN_DIST = (
    2.0  # metres — patrol target must be > this far from all visited goals
)
# 3.0m excluded fire_ext#2 at (1.7,4.7) which is ~2.7m from robot start
PATROL_STEP_M = 1.0  # metres — coarse grid sampling resolution for patrol targets

# Minimum cluster size to be considered a meaningful frontier
MIN_FRONTIER_SIZE = 5  # cells

# Frontier cells with global_costmap >= this value are rejected: they sit inside
# the inflation layer's inscribed-radius zone, so Nav2 cannot position the robot
# there even though SLAM marks them free. Without this filter, the BFS picks
# wall-hugging frontier ribbons (SLAM-free, costmap-lethal), the goal lands in
# inflation, MPPI can't complete, stuck → blacklist → next wall-hugging cluster.
# See issue #10.
#
# Published Nav2 costmap uses 0..100 + -1 (unknown). Default inflation layer
# scaling: 99 = inscribed radius, 100 = LETHAL. Accepting -1 (unknown in costmap,
# e.g. right after ClearEntireCostmap) is required so recovery doesn't
# starve the explorer while the costmap repopulates from sensors.
COSTMAP_LETHAL_THRESHOLD = 99  # reject cells with costmap value >= this


@dataclass
class FrontierCluster:
    cells: list[tuple[int, int]]  # (row, col) in grid frame
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

        self._bfs_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="bfs")
        self._map: Optional[OccupancyGrid] = None
        self._map_lock = threading.Lock()

        # Global costmap snapshot: used to filter wall-hugging frontier cells
        # whose goal would land in the inflation layer (issue #10).
        self._global_costmap: Optional[OccupancyGrid] = None
        self._costmap_lock = threading.Lock()

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_vx: float = 0.0
        self._robot_wz: float = 0.0
        self._odom_lock = threading.Lock()
        self._odom_initialized: bool = False
        self._meters_traveled: float = 0.0
        # Per-phase motion accumulators — odom callback buckets distance and
        # rotation into whichever phase `_tl` last set as current. Gives true
        # time-weighted avg speeds rather than phase-start velocity snapshots.
        self._current_phase: str = "init"
        self._phase_dist: dict[str, float] = defaultdict(float)  # ∫|vx|·dt per phase
        self._phase_rot: dict[str, float] = defaultdict(float)  # ∫|wz|·dt per phase
        self._phase_sample_time: dict[str, float] = defaultdict(float)  # Σdt per phase
        self._last_odom_sim_t: float = 0.0

        # Last position used for stuck detection
        self._last_moved_x: float = 0.0
        self._last_moved_y: float = 0.0
        self._last_move_time: float = 0.0  # set to sim time when each goal starts

        self._blacklist: list[tuple[float, float]] = []  # world coords of failed goals
        self._visited_goals: list[
            tuple[float, float]
        ] = []  # all goal positions attempted
        self._active_goal_handle = None
        self._exploring = False
        self._goal_stats: list[GoalStats] = []
        self._timeline: list[dict] = []

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
        node.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos, callback_group=reentrant
        )
        node.create_subscription(
            Odometry,
            "/derpbot_0/odom",
            self._odom_cb,
            rclpy.qos.QoSProfile(depth=10),
        )

        # Subscribe to global costmap (used both in production for frontier
        # filtering, and by debug dumps). Nav2 publishes TRANSIENT_LOCAL when
        # always_send_full_costmap is true.
        node.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._global_costmap_cb,
            map_qos,
            callback_group=reentrant,
        )

        if _FRONTIER_DEBUG:
            _FRONTIER_DEBUG_DIR.mkdir(parents=True, exist_ok=True)
            self._logger.info(
                f"FrontierExplorer: DEBUG dumps enabled → {_FRONTIER_DEBUG_DIR}"
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

        # Cmd_vel publisher for spinning during startup (accelerates slam_toolbox mapping)
        self._cmd_vel_pub = node.create_publisher(Twist, "/derpbot_0/cmd_vel", 10)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        with self._map_lock:
            self._map = msg

    def _global_costmap_cb(self, msg: OccupancyGrid) -> None:
        with self._costmap_lock:
            self._global_costmap = msg

    def _dump_frontier_debug(
        self,
        goal_num: int,
        cluster: Optional[FrontierCluster],
        cx: float,
        cy: float,
        goal_x: float,
        goal_y: float,
        robot_x: float,
        robot_y: float,
        is_patrol: bool,
    ) -> None:
        """Snapshot /map + global costmap + cluster for offline analysis (issue #10)."""
        with self._map_lock:
            m = self._map
        with self._costmap_lock:
            cm = self._global_costmap
        if m is None:
            return
        map_data = np.array(m.data, dtype=np.int8).reshape((m.info.height, m.info.width))
        cm_data = None
        cm_info = None
        if cm is not None:
            cm_data = np.array(cm.data, dtype=np.int8).reshape(
                (cm.info.height, cm.info.width)
            )
            cm_info = {
                "width": cm.info.width,
                "height": cm.info.height,
                "resolution": cm.info.resolution,
                "origin_x": cm.info.origin.position.x,
                "origin_y": cm.info.origin.position.y,
            }
        cells_arr = (
            np.array(cluster.cells, dtype=np.int32)
            if cluster is not None
            else np.zeros((0, 2), dtype=np.int32)
        )
        path = _FRONTIER_DEBUG_DIR / f"goal_{goal_num:03d}.npz"
        np.savez_compressed(
            path,
            map_data=map_data,
            map_info=np.array(
                [
                    m.info.width,
                    m.info.height,
                    m.info.resolution,
                    m.info.origin.position.x,
                    m.info.origin.position.y,
                ],
                dtype=np.float64,
            ),
            costmap_data=cm_data if cm_data is not None else np.zeros((0, 0), dtype=np.int8),
            costmap_info=np.array(
                [
                    cm_info["width"],
                    cm_info["height"],
                    cm_info["resolution"],
                    cm_info["origin_x"],
                    cm_info["origin_y"],
                ]
                if cm_info
                else [],
                dtype=np.float64,
            ),
            cluster_cells=cells_arr,
            meta=np.array(
                [cx, cy, goal_x, goal_y, robot_x, robot_y, 1.0 if is_patrol else 0.0],
                dtype=np.float64,
            ),
        )
        self._logger.info(
            f"FrontierExplorer: DEBUG dumped goal#{goal_num} → {path}"
            f" (costmap={'yes' if cm_data is not None else 'NO'})"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        # dt from node sim-clock — header.stamp on derpbot_0/odom is unreliable
        # (observed zero). Looser dt bound (< 5s) tolerates GIL latency spikes.
        now = self._sim_time()
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        with self._odom_lock:
            if self._odom_initialized:
                dx = msg.pose.pose.position.x - self._robot_x
                dy = msg.pose.pose.position.y - self._robot_y
                self._meters_traveled += math.hypot(dx, dy)
                # Per-phase integrators: ∫|v|·dt over all odom samples within
                # the phase. Skip dt=0 (same clock tick) and dt>5s (big gap).
                dt = now - self._last_odom_sim_t
                if 0.0 < dt < 5.0:
                    self._phase_dist[self._current_phase] += abs(vx) * dt
                    self._phase_rot[self._current_phase] += abs(wz) * dt
                    self._phase_sample_time[self._current_phase] += dt
            self._last_odom_sim_t = now
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y
            self._robot_vx = vx
            self._robot_wz = wz
            self._odom_initialized = True

    def _tl(self, phase: str, goal_num: int = 0, notes: str = "") -> None:
        """Append a timeline entry and switch the current phase bucket."""
        with self._odom_lock:
            vx, wz = self._robot_vx, self._robot_wz
            self._current_phase = phase
        self._timeline.append(
            {
                "t": self._sim_time(),
                "phase": phase,
                "goal": goal_num,
                "vx": vx,
                "wz": wz,
                "notes": notes,
            }
        )

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
        # Shutdown BFS executor
        self._bfs_executor.shutdown(wait=False)
        # Write profile even on abnormal exit (sim died, time-limit, etc.)
        self._write_timeline()

    # ------------------------------------------------------------------
    # Exploration loop
    # ------------------------------------------------------------------

    def _explore_loop(self) -> None:
        # Mark startup: captures Nav2 action-server wait + first /map wait.
        # Time spent BEFORE _explore_loop runs (mission fetch, detector spawn,
        # AgentNode init) is tracked in agent_node.py. Here we track from
        # wait_for_server until first bfs_detect fires.
        t0 = self._sim_time()
        t0_wall = time.time()
        self._tl("startup", 0, "nav2 wait + map wait")

        # #18 fix: signal the launcher that wall-time init is done so it can
        # unpause Gazebo. All heavy work (imports, OWLv2 load, Nav2 client
        # construction) has already happened in AgentNode.__init__ by the
        # time this thread starts. See scripts/start_stack.sh.
        ready_flag = os.environ.get("DERPBOT_READY_FLAG", "/tmp/derpbot_agent_ready")
        try:
            Path(ready_flag).touch()
        except OSError as exc:
            self._logger.warning(f"Could not touch ready flag {ready_flag}: {exc}")

        self._logger.info("FrontierExplorer: waiting for Nav2 action server…")
        t_nav2_start = self._sim_time()
        if not self._nav_client.wait_for_server(timeout_sec=60.0):
            self._logger.error("NavigateToPose action server not available — aborting.")
            self._done_callback()
            return
        t_nav2_ready = self._sim_time()
        self._logger.info(
            f"[STARTUP TIMING] Nav2 wait: {t_nav2_ready - t_nav2_start:.2f}s"
        )

        # Wait for first /map to arrive (slam_toolbox must finish initial scan)
        # Move robot in small circles to accelerate map building — slam_toolbox needs
        # scan overlap from translation (not just rotation) to build a coherent map.
        t_map_wait_start = self._sim_time()
        t_map_wait_start_wall = time.time()
        spin_msg = Twist()
        spin_msg.linear.x = 0.1  # forward velocity
        spin_msg.angular.z = 0.5  # rad/s — gentle arc
        _publish_count = 0
        while self._map is None and self._exploring and rclpy.ok():
            self._cmd_vel_pub.publish(spin_msg)
            _publish_count += 1
            time.sleep(0.05)  # 20 Hz
        t_map_ready = self._sim_time()
        t_map_ready_wall = time.time()

        self._logger.info(
            f"[STARTUP] Moved during /map wait: {_publish_count} cmd_vel publishes"
        )

        # Stop moving
        stop_msg = Twist()
        self._cmd_vel_pub.publish(stop_msg)

        if self._map is not None:
            self._logger.info(
                f"[STARTUP TIMING] first /map: {t_map_ready - t_map_wait_start:.2f}s sim, {t_map_ready_wall - t_map_wait_start_wall:.2f}s wall"
            )
        else:
            self._logger.warning(
                "[STARTUP TIMING] no /map received before exploration start"
            )

        self._logger.info(
            f"[STARTUP TIMING] total startup phase: {t_map_ready - t0:.2f}s sim, {t_map_ready_wall - t0_wall:.2f}s wall"
        )
        self._logger.info("FrontierExplorer: Nav2 ready. Starting exploration.")

        _t_last_goal_end: float = (
            self._sim_time()
        )  # sim time when last goal cycle ended
        _goal_num: int = 0
        _reject_streak: int = 0  # consecutive rejections of the same centroid
        _last_reject_cx: float = math.nan
        _last_reject_cy: float = math.nan
        MAX_REJECT_STREAK: int = 3  # blacklist after this many consecutive rejections

        while self._exploring and rclpy.ok():
            with self._map_lock:
                current_map = self._map

            if current_map is None:
                self._logger.info("FrontierExplorer: waiting for /map…")
                time.sleep(0.1)
                continue

            with self._odom_lock:
                rx, ry = self._robot_x, self._robot_y

            self._tl("bfs_detect", _goal_num + 1)
            _t_bfs_start = self._sim_time()
            # Run BFS in separate thread to allow camera callbacks to run concurrently
            # (numpy releases GIL during computation)
            future = self._bfs_executor.submit(self._detect_frontiers, current_map)
            frontiers = future.result()  # Blocks until BFS completes
            _t_bfs = self._sim_time() - _t_bfs_start
            best = self._select_best_frontier(frontiers, rx, ry) if frontiers else None

            is_patrol = False
            if best is not None:
                cx, cy = best.centroid_world  # used for scoring/blacklisting
                goal_x, goal_y = self._goal_cell_from_cluster(best, current_map.info)
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
                self._tl(
                    "frontier_select",
                    _goal_num,
                    f"({cx:.1f},{cy:.1f}) size={best.size} dist={math.hypot(cx - rx, cy - ry):.1f}",
                )
            else:
                self._tl("frontier_select", _goal_num, f"PATROL ({cx:.1f},{cy:.1f})")
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

            if _FRONTIER_DEBUG:
                self._dump_frontier_debug(
                    _goal_num,
                    best,
                    cx,
                    cy,
                    goal_x,
                    goal_y,
                    rx,
                    ry,
                    is_patrol,
                )

            self._tl("nav2_send", _goal_num)
            result, _t_accept_lat, _t_first_move, _t_nav = self._send_goal_and_wait(
                goal_x, goal_y, current_map.header.frame_id, _goal_num
            )

            _t_after_nav = self._sim_time()
            self._visited_goals.append((cx, cy))

            _accept_str = (
                f"{_t_accept_lat:.1f}s" if not math.isnan(_t_accept_lat) else "n/a"
            )
            _move_str = (
                f"{_t_first_move:.1f}s" if not math.isnan(_t_first_move) else "n/a"
            )

            if result is True:
                self._tl("goal_reached", _goal_num, f"nav={_t_nav:.1f}s")
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f}) SUCCEEDED"
                    f" accept={_accept_str} first_move={_move_str} nav={_t_nav:.1f}s — blacklisting."
                )
                # Blacklist centroid on success. If frontier cells persist after the
                # robot visited (unknown area is behind a wall), blacklisting prevents
                # the robot from repeatedly returning to the same inaccessible spot.
                self._blacklist.append((cx, cy))
            elif result is False:
                self._tl("goal_failed", _goal_num, f"nav={_t_nav:.1f}s")
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f}) FAILED"
                    f" accept={_accept_str} first_move={_move_str} nav={_t_nav:.1f}s — blacklisting."
                )
                self._blacklist.append((cx, cy))
                # Back up + clear costmap after failure to prevent "Start occupied"
                # cascade: robot may be inside an inflation zone after a stuck/abort;
                # backing up first moves it to free space before the costmap rebuilds.
                self._tl("recovery", _goal_num, "backup+clear")
                self._recover_from_occupied_start()
                # Brief pause — give Nav2 time to finish cancellation/cleanup
                # before we send the next goal (avoids acceptance future delays).
                self._tl("post_goal_sleep", _goal_num, "1.0s sim")
                self._sim_sleep(1.0)
            elif result is None:
                # Track consecutive rejections of the same centroid.
                if (
                    math.hypot(cx - _last_reject_cx, cy - _last_reject_cy)
                    < BLACKLIST_RADIUS
                ):
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
                self._tl("rejection_sleep", _goal_num, "1.0s sim")
                self._sim_sleep(1.0)

            _t_last_goal_end = self._sim_time()
            self._goal_stats.append(
                GoalStats(
                    goal_num=_goal_num,
                    is_patrol=is_patrol,
                    result=result,
                    bfs_and_selection_s=_t_idle,
                    accept_latency_s=_t_accept_lat,
                    time_to_first_move_s=_t_first_move,
                    nav_time_s=_t_nav,
                    post_goal_pause_s=_t_last_goal_end - _t_after_nav,
                )
            )

        # Always log timing table + profile on exit (time-limit, natural completion, or stop)
        self._log_timing_table()
        self._write_timeline()
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
            frontier_mask |= free & shifted

        # #10 clip frontier mask to observed extent (cells with data >= 0).
        # Unknown cells beyond the known map bounds cause the costmap to expand
        # indefinitely, creating phantom frontiers at the edge. Reject any cell
        # outside the axis-aligned bbox of observed cells (free + occupied).
        observed = data >= 0
        if np.any(observed):
            rows, cols = np.where(observed)
            rmin, rmax = rows.min(), rows.max()
            cmin, cmax = cols.min(), cols.max()
            frontier_mask = frontier_mask & (
                (np.arange(height)[:, None] >= rmin)
                & (np.arange(height)[:, None] <= rmax)
                & (np.arange(width)[None, :] >= cmin)
                & (np.arange(width)[None, :] <= cmax)
            )

        # #10 drop frontier cells inside the global costmap inflation layer.
        # SLAM marks a cell FREE as soon as a LiDAR beam reaches it, including
        # cells right next to walls. The BFS then picks these wall-hugging
        # ribbons — but Nav2's inflation layer (inscribed_radius=robot_radius)
        # marks them unreachable. Filtering here prevents the stuck→blacklist→
        # next-wall-hugger cascade and keeps goal cells in traversable space.
        #
        # Cells where costmap == -1 (unknown, e.g. right after ClearEntireCostmap)
        # are kept; otherwise recovery would temporarily starve the explorer.
        self._apply_costmap_filter(frontier_mask, grid)

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

    def _apply_costmap_filter(
        self, frontier_mask: np.ndarray, grid: OccupancyGrid
    ) -> None:
        """
        Clear bits in `frontier_mask` whose world location has global costmap
        value >= COSTMAP_LETHAL_THRESHOLD. Mutates frontier_mask in place.
        No-op when the costmap snapshot is unavailable (first few seconds of run).

        See issue #10: wall-hugging frontier ribbons look reachable in /map but
        are inside Nav2's inflation-layer inscribed radius.
        """
        with self._costmap_lock:
            cm = self._global_costmap
        if cm is None:
            return
        cm_data = np.asarray(cm.data, dtype=np.int16).reshape(
            (cm.info.height, cm.info.width)
        )
        # World coords of frontier cells → costmap grid indices. Vectorised over
        # the (usually small) set of remaining frontier candidates.
        rs, cs = np.where(frontier_mask)
        if rs.size == 0:
            return
        map_res = grid.info.resolution
        map_ox = grid.info.origin.position.x
        map_oy = grid.info.origin.position.y
        cm_res = cm.info.resolution
        cm_ox = cm.info.origin.position.x
        cm_oy = cm.info.origin.position.y
        wx = map_ox + (cs + 0.5) * map_res
        wy = map_oy + (rs + 0.5) * map_res
        cm_c = np.floor((wx - cm_ox) / cm_res).astype(np.int64)
        cm_r = np.floor((wy - cm_oy) / cm_res).astype(np.int64)
        in_cm = (
            (cm_r >= 0) & (cm_r < cm.info.height) & (cm_c >= 0) & (cm_c < cm.info.width)
        )
        lethal = np.zeros(rs.size, dtype=bool)
        if np.any(in_cm):
            idx_r = cm_r[in_cm]
            idx_c = cm_c[in_cm]
            values = cm_data[idx_r, idx_c]
            lethal[in_cm] = values >= COSTMAP_LETHAL_THRESHOLD
        dropped = int(lethal.sum())
        if dropped > 0:
            frontier_mask[rs[lethal], cs[lethal]] = False
            self._logger.debug(
                f"_apply_costmap_filter: dropped {dropped}/{rs.size} frontier cells"
                f" with costmap >= {COSTMAP_LETHAL_THRESHOLD}"
            )

    def _costmap_value_for_cell(
        self, r: int, c: int, grid_info
    ) -> int:
        """
        Look up the global costmap value for a /map cell (r, c). Returns -1 when
        the costmap snapshot is unavailable or the cell falls outside its extent
        (match OccupancyGrid's `unknown` convention). Used by goal-cell selection
        to steer goals away from inflation even inside an otherwise-free cluster.
        """
        with self._costmap_lock:
            cm = self._global_costmap
        if cm is None:
            return -1
        map_res = grid_info.resolution
        map_ox = grid_info.origin.position.x
        map_oy = grid_info.origin.position.y
        wx = map_ox + (c + 0.5) * map_res
        wy = map_oy + (r + 0.5) * map_res
        cm_c = int(math.floor((wx - cm.info.origin.position.x) / cm.info.resolution))
        cm_r = int(math.floor((wy - cm.info.origin.position.y) / cm.info.resolution))
        if not (0 <= cm_r < cm.info.height and 0 <= cm_c < cm.info.width):
            return -1
        return int(cm.data[cm_r * cm.info.width + cm_c])

    def _goal_cell_from_cluster(
        self, cluster: FrontierCluster, grid_info
    ) -> tuple[float, float]:
        """
        Pick the best cluster cell to navigate to and return its world (x, y).

        Selection rule: among cluster cells with non-lethal costmap value
        (< COSTMAP_LETHAL_THRESHOLD, or -1 "unknown" when the costmap lags),
        minimise a combined score of `costmap_value + DIST_WEIGHT * grid_distance_to_centroid`.
        This steers the goal toward the interior of the cluster (away from walls)
        while still requiring actual travel rather than picking a cell the robot
        already satisfies within xy_goal_tolerance.

        Fallback: if the costmap snapshot is unavailable or all cluster cells
        were filtered, fall back to pure closest-to-centroid (legacy behaviour).
        """
        res = grid_info.resolution
        ox = grid_info.origin.position.x
        oy = grid_info.origin.position.y
        cx, cy = cluster.centroid_world
        centroid_r = (cy - oy) / res - 0.5
        centroid_c = (cx - ox) / res - 0.5

        # Score weight: centroid-distance in cells is ~0–30 for typical clusters;
        # costmap values are 0–99. Scale distance up so it can tie-break among
        # equally low-cost cells without being drowned out by costmap noise.
        DIST_WEIGHT = 2.0

        best_score = math.inf
        best_r, best_c = cluster.cells[0]
        fallback_min_d = math.inf
        fallback_r, fallback_c = cluster.cells[0]
        any_passed = False

        for r, c in cluster.cells:
            d = math.hypot(r - centroid_r, c - centroid_c)
            if d < fallback_min_d:
                fallback_min_d = d
                fallback_r, fallback_c = r, c

            cost = self._costmap_value_for_cell(r, c, grid_info)
            if cost >= COSTMAP_LETHAL_THRESHOLD:
                continue
            # Treat unknown (-1) as cost 0 — assume traversable until proven otherwise.
            effective_cost = 0 if cost < 0 else cost
            score = effective_cost + DIST_WEIGHT * d
            if score < best_score:
                best_score = score
                best_r, best_c = r, c
                any_passed = True

        if not any_passed:
            # Cluster contains only lethal cells per current costmap snapshot —
            # should be rare because _apply_costmap_filter drops these earlier.
            # Can happen if the costmap advanced (more inflation) between BFS
            # and goal selection. Fall back to legacy closest-to-centroid so
            # we still attempt something rather than stalling silently.
            best_r, best_c = fallback_r, fallback_c
            self._logger.debug(
                "_goal_cell_from_cluster: all cluster cells lethal per current costmap; "
                "falling back to closest-to-centroid"
            )

        goal_x = ox + (best_c + 0.5) * res
        goal_y = oy + (best_r + 0.5) * res
        return goal_x, goal_y

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
                        math.hypot(wx - px, wy - py) for px, py in self._visited_goals
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
            goal.speed = 0.10  # m/s (slow, safe)
            goal.time_allowance.sec = 10
            future = self._backup_client.send_goal_async(goal)
            deadline = self._sim_time() + 12.0
            while not future.done() and self._sim_time() < deadline:
                time.sleep(0.1)
            if future.done() and future.result() and future.result().accepted:
                result_future = future.result().get_result_async()
                deadline = self._sim_time() + 10.0
                while not result_future.done() and self._sim_time() < deadline:
                    time.sleep(0.1)
            self._logger.info("FrontierExplorer: backup complete.")
        else:
            self._logger.warning(
                "FrontierExplorer: backup server not available — skipping backup."
            )

        # Step 2: clear both costmaps
        for svc, name in [
            (self._clear_local_costmap, "local"),
            (self._clear_global_costmap, "global"),
        ]:
            if svc.wait_for_service(timeout_sec=1.0):
                future = svc.call_async(ClearEntireCostmap.Request())
                deadline = self._sim_time() + 3.0
                while not future.done() and self._sim_time() < deadline:
                    time.sleep(0.05)
        self._logger.info("FrontierExplorer: local+global costmaps cleared.")

    def _log_timing_table(self) -> None:
        log_timing_table(self._logger, self._goal_stats)

    def _write_timeline(self) -> None:
        with self._odom_lock:
            meters = self._meters_traveled
            phase_dist = dict(self._phase_dist)
            phase_rot = dict(self._phase_rot)
            phase_sample_time = dict(self._phase_sample_time)
        write_timeline(
            self._logger,
            self._timeline,
            self._sim_time(),
            meters,
            phase_dist,
            phase_rot,
            phase_sample_time,
        )

    def _sim_time(self) -> float:
        """Return current ROS clock time in seconds (sim time when use_sim_time=True)."""
        return self._node.get_clock().now().nanoseconds / 1e9

    def _sim_sleep(self, sim_seconds: float, max_wall: float = 30.0) -> None:
        """Sleep until sim_seconds of sim-time elapse (wall-clock safety ceiling)."""
        t0_sim = self._sim_time()
        t0_wall = time.time()
        while (
            self._sim_time() - t0_sim < sim_seconds and time.time() - t0_wall < max_wall
        ):
            time.sleep(0.1)

    def _send_goal_and_wait(
        self, goal_x: float, goal_y: float, frame_id: str, goal_num: int = 0
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
            self._logger.warning(
                "FrontierExplorer: timeout waiting for goal acceptance — skipping (not blacklisting)."
            )
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
        self._tl("nav2_accepted", goal_num, f"accept_lat={accept_latency:.1f}s")

        self._active_goal_handle = goal_handle
        self._last_move_time = _t_accept

        with self._odom_lock:
            self._last_moved_x = self._robot_x
            self._last_moved_y = self._robot_y
            # Snapshot position at accept time to track first-move milestone.
            _accepted_x, _accepted_y = self._robot_x, self._robot_y

        result_future = goal_handle.get_result_async()
        _first_move_t = _nan  # sim-seconds from accept to first 0.15 m displacement
        _sub = "waiting"  # nav sub-phase for timeline profiling
        _wall_accept = time.time()  # wall-clock escape: catch dead sim (clock frozen)

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
                vx, wz = self._robot_vx, self._robot_wz

            # Sub-phase detection for timeline profiling
            if abs(vx) >= 0.05:
                new_sub = "traveling"
            elif abs(wz) >= 0.1:
                new_sub = "rotating"
            else:
                new_sub = "waiting"
            if new_sub != _sub:
                self._tl(new_sub, goal_num)
                _sub = new_sub

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
                self._tl("goal_stuck", goal_num)
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cancel_deadline:
                    time.sleep(0.1)
                return (
                    False,
                    accept_latency,
                    _first_move_t,
                    self._sim_time() - _t_accept,
                )

            if not self._exploring:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < cancel_deadline:
                    time.sleep(0.1)
                return (
                    False,
                    accept_latency,
                    _first_move_t,
                    self._sim_time() - _t_accept,
                )

            # Wall-clock escape: if a single goal takes > 180 wall-seconds,
            # the sim is likely dead (clock frozen, Nav2 unresponsive).
            if time.time() - _wall_accept > 180.0:
                self._logger.warning(
                    "FrontierExplorer: wall-clock timeout (180s) — sim likely dead, bailing out."
                )
                cancel_future = goal_handle.cancel_goal_async()
                cancel_deadline = time.time() + 3.0
                while not cancel_future.done() and time.time() < cancel_deadline:
                    time.sleep(0.1)
                return (
                    False,
                    accept_latency,
                    _first_move_t,
                    self._sim_time() - _t_accept,
                )

        result = result_future.result()
        nav_time = self._sim_time() - _t_accept
        self._active_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True, accept_latency, _first_move_t, nav_time

        self._logger.info(
            f"FrontierExplorer: goal finished with status {result.status}."
        )
        return False, accept_latency, _first_move_t, nav_time
