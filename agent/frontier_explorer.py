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
from concurrent.futures import ProcessPoolExecutor
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
from ros_gz_interfaces.msg import Contacts
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
# W_DIST is dynamic: scales with the largest frontier in the current frame.
# Large frontiers → area is fresh → stay local (high W_DIST).
# Small frontiers → area is nearly exhausted → look farther (low W_DIST).
# Generic proxy — no room/class/phase awareness, works on any map.
W_SIZE = (
    1.0  # prefer large frontier clusters (larger cluster = more open frontier edge)
)
W_DIST_MAX = 3.5  # when frontiers are large (new area) — stay local
W_DIST_MIN = 1.5  # when frontiers are tiny (area exhausted) — look farther
W_DIST_SIZE_NORM = 150.0  # max cluster size that saturates W_DIST at MAX

# Stuck detection
STUCK_DIST_THRESHOLD = 0.10  # metres — robot must move this far
STUCK_TIME_THRESHOLD = (
    30.0  # seconds (Nav2 rotates first; give it time to start translating)
)
BUMPER_STUCK_TIME_THRESHOLD = 3.0  # seconds — fast stuck trigger on bumper contact
GROUND_PLANE_NORMAL_Z = 0.9  # contacts with |normal.z| >= this are ground plane

# Blacklist radius: frontiers within this distance of a failed goal are blacklisted
BLACKLIST_RADIUS = 0.5  # metres

# Succeeded goals are soft-excluded for this many sim-seconds so SLAM has time to
# mark the area as explored. Short enough to allow revisits if the frontier genuinely
# persists; does NOT permanently block valid frontiers. Issue #27.
SUCCESS_EXCLUSION_TTL = 45.0  # sim-seconds

# Patrol mode: kicks in after all LIDAR frontiers are exhausted.
# Selects free cells that are far from any previously visited goal (LIDAR coverage
# does not imply camera coverage; the robot must physically visit all regions).
PATROL_MIN_DIST = (
    1.0  # metres — patrol target must be > this far from all visited goals.
    # 2.0m excluded rooms adjacent to corridor goals (e.g. Meeting Room at 1.2m from
    # corridor frontier centroid); 1.0m allows entering them while still skipping
    # cells the robot was essentially on top of. See issue #32.
    # Note: 3.0m excluded fire_ext#2 at (1.7,4.7) which is ~2.7m from robot start.
)
PATROL_STEP_M = 1.0  # metres — coarse grid sampling resolution for patrol targets

# Minimum cluster size to be considered a meaningful frontier
MIN_FRONTIER_SIZE = 5  # cells

# --- Detection-aware exploration (Task 5 / #8) ---
# When the tracker reports a pending candidate (1-sighting object), the explorer
# creates a "detection frontier" at the candidate's world position. These get
# score_override so they are preferred over geographic frontiers, forcing the
# robot to detour toward partially-detected objects for re-detection.
# Only one detour per candidate (deduplicated by track_id).

CANDIDATE_SCORE_MULTIPLIER = 10.0  # override = max_frontier_score * this
CANDIDATE_BLACKLIST_RADIUS = 0.5    # metres — ignore candidates near blacklisted points

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


def bfs_worker(
    grid_data: list,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    costmap_data: Optional[list] = None,
    costmap_width: Optional[int] = None,
    costmap_height: Optional[int] = None,
    costmap_resolution: Optional[float] = None,
    costmap_origin_x: Optional[float] = None,
    costmap_origin_y: Optional[float] = None,
) -> list:
    """
    Top-level function to run BFS in a separate process.
    Returns list of (cells, centroid_world) tuples.
    """
    import numpy as np
    from collections import deque

    FREE = 0
    UNKNOWN = -1
    MIN_FRONTIER_SIZE = 5

    data = np.array(grid_data, dtype=np.int8).reshape((height, width))

    # Mark frontier cells
    frontier_mask = np.zeros((height, width), dtype=bool)
    free = data == FREE
    unknown = data == UNKNOWN
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        shifted = np.roll(unknown, (dr, dc), axis=(0, 1))
        if dr == -1:
            shifted[-1, :] = False
        elif dr == 1:
            shifted[0, :] = False
        if dc == -1:
            shifted[:, -1] = False
        elif dc == 1:
            shifted[:, 0] = False
        frontier_mask |= free & shifted

    # Clip frontier mask to observed extent
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

    # Apply costmap filter
    if costmap_data is not None and costmap_width is not None:
        cm = np.asarray(costmap_data, dtype=np.int16).reshape(
            (costmap_height, costmap_width)
        )
        rs, cs = np.where(frontier_mask)
        if rs.size > 0:
            map_res = resolution
            map_ox = origin_x
            map_oy = origin_y
            cm_res = costmap_resolution
            cm_ox = costmap_origin_x
            cm_oy = costmap_origin_y
            wx = map_ox + (cs + 0.5) * map_res
            wy = map_oy + (rs + 0.5) * map_res
            cm_c = np.floor((wx - cm_ox) / cm_res).astype(np.int64)
            cm_r = np.floor((wy - cm_oy) / cm_res).astype(np.int64)
            in_cm = (
                (cm_r >= 0)
                & (cm_r < costmap_height)
                & (cm_c >= 0)
                & (cm_c < costmap_width)
            )
            if np.any(in_cm):
                idx_r = cm_r[in_cm]
                idx_c = cm_c[in_cm]
                values = cm[idx_r, idx_c]
                lethal = np.zeros(rs.size, dtype=bool)
                lethal[in_cm] = values >= COSTMAP_LETHAL_THRESHOLD
                frontier_mask[rs[lethal], cs[lethal]] = False

    # BFS-cluster the frontier cells
    visited = np.zeros((height, width), dtype=bool)
    clusters = []

    rows, cols = np.where(frontier_mask)
    for start_r, start_c in zip(rows, cols):
        if visited[start_r, start_c]:
            continue
        q = deque()
        q.append((start_r, start_c))
        visited[start_r, start_c] = True
        cells = []
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

        mean_r = sum(r for r, _ in cells) / len(cells)
        mean_c = sum(c for _, c in cells) / len(cells)
        wx = origin_x + (mean_c + 0.5) * resolution
        wy = origin_y + (mean_r + 0.5) * resolution
        clusters.append((cells, (wx, wy)))

    return clusters


@dataclass
class FrontierCluster:
    cells: list[tuple[int, int]]  # (row, col) in grid frame
    centroid_world: tuple[float, float]  # (x, y) in map frame (metres)

    @property
    def size(self) -> int:
        return len(self.cells)


@dataclass
class DetectionFrontier:
    """Pseudo-frontier for a pending detection candidate (1-sighting object)."""
    track_id: str
    class_name: str
    world_x: float
    world_y: float
    score_override: float  # assigned at creation; always beats geographic frontiers


# Type alias: a navigation target is either a geographic frontier or a detection
ExplorationTarget = FrontierCluster | DetectionFrontier


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

    def __init__(self, node: Node, done_callback: Callable[[], None], tracker=None):
        self._node = node
        self._done_callback = done_callback
        self._logger = node.get_logger()
        self._tracker = tracker  # tracker.Tracker or None

        self._bfs_executor = ProcessPoolExecutor(max_workers=1)
        self._map: Optional[OccupancyGrid] = None
        self._map_lock = threading.Lock()

        # Callback-based goal tracking (replaces polling in _send_goal_and_wait)
        self._goal_response_event = threading.Event()
        self._goal_result_event = threading.Event()
        self._goal_accepted: Optional[bool] = None
        self._goal_handle = None
        self._goal_result = None

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

        # Bumper contact: set when a non-ground-plane collision is detected.
        # Used to trigger faster stuck detection when the robot physically
        # contacts an obstacle the LiDAR can't see (e.g. low chair wheels).
        self._bumper_contact: bool = False
        self._bumper_lock = threading.Lock()

        self._blacklist: list[tuple[float, float]] = []  # failed goal centroids (permanent)
        self._success_exclusion: list[tuple[float, float, float]] = []  # (x, y, expire_t)
        self._visited_goals: list[
            tuple[float, float]
        ] = []  # all goal positions attempted
        self._active_goal_handle = None
        self._exploring = False
        self._goal_stats: list[GoalStats] = []
        self._timeline: list[dict] = []

        # Detection-aware exploration (Task 5 / #8)
        # Set of track_ids that have already been visited as detection frontiers.
        # Each candidate is visited once; after the detour, SLAM may have updated
        # and the object may be confirmed or the candidate expired.
        self._candidate_visited: set[str] = set()

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
            callback_group=reentrant,
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

        # Bumper contact sensor: detects physical collisions with obstacles
        # (including low obstacles the LiDAR misses, e.g. chair wheels).
        node.create_subscription(
            Contacts,
            "/derpbot_0/bumper_contact",
            self._bumper_cb,
            rclpy.qos.QoSProfile(depth=10),
            callback_group=reentrant,
        )

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

    def _bumper_cb(self, msg: Contacts) -> None:
        """Process bumper contact events. Set flag when a non-ground-plane
        collision is detected, enabling faster stuck detection for obstacles
        the LiDAR can't see (e.g. low chair wheels)."""
        for contact in msg.contacts:
            collision1_name = contact.collision1.name.lower()
            collision2_name = contact.collision2.name.lower()
            # Filter ground-plane contacts: entity names containing "ground" or "plane"
            # are the floor. The robot's own links contain "derpbot" or "caster".
            is_ground = (
                "ground" in collision1_name or "plane" in collision1_name
                or "ground" in collision2_name or "plane" in collision2_name
            )
            # Also filter by contact normal: ground contacts have nearly vertical normals
            is_vertical_normal = False
            if contact.normals:
                for normal in contact.normals:
                    if abs(normal.z) >= GROUND_PLANE_NORMAL_Z:
                        is_vertical_normal = True
                        break
            if is_ground or is_vertical_normal:
                continue
            # Non-ground contact detected — robot is touching something
            with self._bumper_lock:
                self._bumper_contact = True
            return  # one valid contact is enough to set the flag

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

    # ------------------------------------------------------------------
    # Nav2 goal callbacks (replaces polling in _send_goal_and_wait)
    # ------------------------------------------------------------------

    def _goal_response_callback(self, future) -> None:
        """Called when Nav2 responds to goal request (accept/reject)."""
        goal_handle = future.result()
        self._goal_accepted = goal_handle.accepted
        self._goal_handle = goal_handle
        self._goal_response_event.set()

    def _goal_result_callback(self, future) -> None:
        """Called when Nav2 goal completes (success/failure)."""
        self._goal_result = future.result()
        self._goal_result_event.set()

    def stop(self) -> None:
        self._exploring = False
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        # Shutdown BFS process pool
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
            frontiers = self._detect_frontiers(current_map)
            _t_bfs = self._sim_time() - _t_bfs_start

            # Build detection frontiers from pending tracker candidates
            detection_frontiers = self._build_detection_frontiers(rx, ry)

            best = self._select_best_frontier(
                frontiers, rx, ry, detection_frontiers
            ) if (frontiers or detection_frontiers) else None

            is_detection = isinstance(best, DetectionFrontier)
            is_patrol = False
            if is_detection:
                # Re-validate: candidate may have been confirmed since
                # _build_detection_frontiers ran (tracker worker runs async).
                # Skip detour if already confirmed — object is already found.
                if self._tracker and not self._tracker.is_still_pending(best.track_id):
                    self._logger.info(
                        f"FrontierExplorer: detection candidate {best.track_id} "
                        f"already confirmed — skipping detour."
                    )
                    self._candidate_visited.add(best.track_id)
                    continue
                cx, cy = best.world_x, best.world_y
                goal_x, goal_y = best.world_x, best.world_y
            elif best is not None:
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
            if is_detection:
                self._tl(
                    "detection_select",
                    _goal_num,
                    f"candidate {best.class_name} ({cx:.1f},{cy:.1f}) dist={math.hypot(cx - rx, cy - ry):.1f}",
                )
                self._logger.info(
                    f"FrontierExplorer: DETECTION#{_goal_num} {best.class_name} ({goal_x:.2f}, {goal_y:.2f})"
                    f" [track {best.track_id}]"
                    f" idle_since_last={_t_idle:.1f}s bfs={_t_bfs:.2f}s"
                )
            elif not is_patrol:
                self._tl(
                    "frontier_select",
                    _goal_num,
                    f"({cx:.1f},{cy:.1f}) goal=({goal_x:.1f},{goal_y:.1f}) size={best.size} dist={math.hypot(cx - rx, cy - ry):.1f}",
                )
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({goal_x:.2f}, {goal_y:.2f})"
                    f" [centroid ({cx:.2f}, {cy:.2f})],"
                    f" size={best.size},"
                    f" idle_since_last={_t_idle:.1f}s bfs={_t_bfs:.2f}s"
                )
            else:
                self._tl("frontier_select", _goal_num, f"PATROL ({cx:.1f},{cy:.1f}) goal=({goal_x:.1f},{goal_y:.1f})")
                self._logger.info(
                    f"FrontierExplorer: PATROL#{_goal_num} ({goal_x:.2f}, {goal_y:.2f})"
                    f" idle_since_last={_t_idle:.1f}s bfs={_t_bfs:.2f}s"
                )

            if _FRONTIER_DEBUG:
                debug_cluster = best if isinstance(best, FrontierCluster) else None
                self._dump_frontier_debug(
                    _goal_num,
                    debug_cluster,
                    cx,
                    cy,
                    goal_x,
                    goal_y,
                    rx,
                    ry,
                    is_patrol or is_detection,
                )

            self._tl("nav2_send", _goal_num)
            # Geographic goals are preemptable: if a detection candidate
            # appears mid-navigation, cancel and detour toward it.
            # Detection detours are NOT preemptable — we commit to avoid oscillation,
            # but cancel if the object gets confirmed en route.
            result, _t_accept_lat, _t_first_move, _t_nav = self._send_goal_and_wait(
                goal_x, goal_y, current_map.header.frame_id, _goal_num,
                preemptable=(not is_detection and not is_patrol),
                detection_track_id=(best.track_id if is_detection else None),
            )

            # Handle preemption: geographic goal cancelled because a detection
            # candidate appeared. Don't count as a goal; loop back to re-select.
            if result is self.PREEMPTED:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} PREEMPTED by detection candidate"
                    f" — re-evaluating frontiers."
                )
                # Don't increment goal_num or add to stats — preemption is not a goal outcome.
                _goal_num -= 1  # undo the increment above
                _t_last_goal_end = self._sim_time()
                self._sim_sleep(0.5)  # brief pause for Nav2 cleanup after cancel
                continue

            # Handle confirmed en route: detection detour cancelled because
            # the object was already confirmed by the tracker — no need to visit.
            if result is self.CONFIRMED_EN_ROUTE:
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} CANCELLED — detection "
                    f"candidate {best.track_id} confirmed en route."
                )
                self._candidate_visited.add(best.track_id)
                _goal_num -= 1  # not a real goal outcome
                _t_last_goal_end = self._sim_time()
                self._sim_sleep(0.5)  # brief pause for Nav2 cleanup after cancel
                continue

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
                    f" accept={_accept_str} first_move={_move_str} nav={_t_nav:.1f}s"
                    f" — soft-excluding for {SUCCESS_EXCLUSION_TTL:.0f}s."
                )
                # Soft-exclude centroid for SUCCESS_EXCLUSION_TTL sim-seconds so SLAM
                # has time to mark the area explored before it can be re-selected.
                # Permanent blacklisting on success caused premature frontier exhaustion
                # in low-frontier maps (issue #27).
                self._success_exclusion.append(
                    (cx, cy, self._sim_time() + SUCCESS_EXCLUSION_TTL)
                )
                # Detection frontier: mark as visited (one detour per candidate)
                if is_detection:
                    self._candidate_visited.add(best.track_id)
                    self._logger.info(
                        f"FrontierExplorer: detection candidate {best.track_id} visited, will not retry."
                    )
            elif result is False:
                self._tl("goal_failed", _goal_num, f"nav={_t_nav:.1f}s")
                self._logger.info(
                    f"FrontierExplorer: goal#{_goal_num} ({cx:.2f}, {cy:.2f}) FAILED"
                    f" accept={_accept_str} first_move={_move_str} nav={_t_nav:.1f}s — blacklisting."
                )
                self._blacklist.append((cx, cy))
                # Detection frontier: only mark visited on SUCCESS, not on failure.
                # On Nav2 failure/abort the candidate may be in an unreachable area —
                # leave it eligible for retry. The blacklist radius (0.5m) prevents
                # infinite loops for truly unreachable locations.
                if is_detection:
                    self._logger.info(
                        f"FrontierExplorer: detection candidate {best.track_id} detour failed — NOT marking visited (eligible for retry)."
                    )
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
        Runs BFS in a separate process to bypass GIL contention.
        """
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        # Get costmap data if available
        with self._costmap_lock:
            cm = self._global_costmap
        costmap_args = {}
        if cm is not None:
            costmap_args = {
                "costmap_data": list(cm.data),
                "costmap_width": cm.info.width,
                "costmap_height": cm.info.height,
                "costmap_resolution": cm.info.resolution,
                "costmap_origin_x": cm.info.origin.position.x,
                "costmap_origin_y": cm.info.origin.position.y,
            }

        # Submit to process pool
        future = self._bfs_executor.submit(
            bfs_worker,
            list(grid.data),
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            **costmap_args,
        )
        results = future.result()  # Wait for result

        # Convert results to FrontierCluster objects
        clusters = [
            FrontierCluster(cells=cells, centroid_world=centroid)
            for cells, centroid in results
        ]
        return clusters

    # ------------------------------------------------------------------
    # Detection-aware exploration — candidate frontier injection
    # ------------------------------------------------------------------

    def _build_detection_frontiers(
        self, robot_x: float, robot_y: float
    ) -> list[DetectionFrontier]:
        """
        Query tracker for pending candidates (1-sighting objects) and convert
        them into DetectionFrontier pseudo-frontiers. Filters out candidates
        that are already visited, blacklisted, or too close to the robot
        (already being seen — no detour needed).
        """
        if self._tracker is None:
            return []

        sim_time = self._sim_time()
        candidates = self._tracker.get_pending_candidates(sim_time)
        if not candidates:
            return []

        frontiers = []
        for cand in candidates:
            tid = cand["track_id"]
            if tid in self._candidate_visited:
                continue
            wx, wy = cand["world_x"], cand["world_y"]

            # Skip candidates near blacklisted locations
            if self._is_blacklisted(wx, wy):
                continue
            # Skip candidates too close to robot (already seeing it — let tracker confirm)
            if math.hypot(wx - robot_x, wy - robot_y) < 1.0:
                continue

            frontiers.append(DetectionFrontier(
                track_id=tid,
                class_name=cand["class_name"],
                world_x=wx,
                world_y=wy,
                score_override=0.0,  # assigned later by _select_best_frontier
            ))

        if frontiers:
            det_info = ", ".join(
                f"{f.class_name}@({f.world_x:.1f},{f.world_y:.1f})"
                for f in frontiers
            )
            self._logger.info(
                f"FrontierExplorer: {len(frontiers)} detection candidates "
                f"({len(candidates)} total, {len(self._candidate_visited)} visited): [{det_info}]"
            )
        return frontiers

    # ------------------------------------------------------------------
    # Frontier selection — scoring
    # ------------------------------------------------------------------

    def _select_best_frontier(
        self,
        clusters: list[FrontierCluster],
        robot_x: float,
        robot_y: float,
        detection_frontiers: Optional[list[DetectionFrontier]] = None,
    ) -> Optional[FrontierCluster | DetectionFrontier]:
        # Dynamic W_DIST: scales with the largest frontier in this BFS round.
        # Large frontier nearby → new unexplored area → stay local (high W_DIST).
        # All frontiers tiny → area nearly exhausted → look farther (low W_DIST).
        if clusters:
            eligible_sizes = [
                c.size for c in clusters
                if not self._is_blacklisted(*c.centroid_world)
            ]
            max_size = max(eligible_sizes) if eligible_sizes else 0
            if max_size > 0:
                scale = min(1.0, max_size / W_DIST_SIZE_NORM)
                w_dist = W_DIST_MIN + (W_DIST_MAX - W_DIST_MIN) * scale
            else:
                w_dist = W_DIST_MIN
        else:
            w_dist = W_DIST_MIN
            max_size = 0

        best_score = -math.inf
        best_target: Optional[FrontierCluster | DetectionFrontier] = None
        scored: list[tuple[float, FrontierCluster]] = []

        for cluster in clusters:
            cx, cy = cluster.centroid_world
            if self._is_blacklisted(cx, cy):
                continue
            dist = math.hypot(cx - robot_x, cy - robot_y)
            score = W_SIZE * cluster.size - w_dist * dist
            scored.append((score, cluster))
            if score > best_score:
                best_score = score
                best_target = cluster

        # Detection frontiers always beat geographic frontiers by score override.
        # Pick the closest unvisited, un-blacklisted candidate.
        if detection_frontiers:
            best_cand_dist = math.inf
            best_cand = None
            for df in detection_frontiers:
                if self._is_blacklisted(df.world_x, df.world_y):
                    continue
                d = math.hypot(df.world_x - robot_x, df.world_y - robot_y)
                if d < best_cand_dist:
                    best_cand_dist = d
                    best_cand = df
            if best_cand is not None:
                # Override score so detection frontier always beats geographic frontiers.
                # If no geographic frontiers scored, use 0 as baseline.
                override = max(best_score, 0.0) + 1.0
                best_cand.score_override = override
                best_target = best_cand

        n_bl = len(clusters) - len(scored)
        scored.sort(key=lambda t: t[0], reverse=True)
        top5 = " | ".join(
            f"({c.centroid_world[0]:.1f},{c.centroid_world[1]:.1f}) sz={c.size} sc={s:.0f}"
            for s, c in scored[:5]
        )
        target_type = "GEOGRAPHIC"
        if isinstance(best_target, DetectionFrontier):
            target_type = f"DETECTION({best_target.class_name}@{best_target.world_x:.1f},{best_target.world_y:.1f})"
        elif best_target is None:
            target_type = "NONE"
        cand_info = ""
        if detection_frontiers:
            cand_info = f", {len(detection_frontiers)} candidates=[{', '.join(f'{df.class_name}@({df.world_x:.1f},{df.world_y:.1f})' for df in detection_frontiers)}]"
        self._logger.info(
            f"FrontierExplorer: {len(clusters)} clusters "
            f"({n_bl} BL, {len(scored)} eligible), max_sz={max_size if clusters else 0}, W_DIST={w_dist:.2f}.{cand_info} "
            f"SELECTED={target_type} Top5: [{top5}]"
        )

        return best_target

    def _is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by in self._blacklist:
            if math.hypot(x - bx, y - by) < BLACKLIST_RADIUS:
                return True
        now = self._sim_time()
        for bx, by, expire_t in self._success_exclusion:
            if expire_t > now and math.hypot(x - bx, y - by) < BLACKLIST_RADIUS:
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

    # Sentinels returned by _send_goal_and_wait
    PREEMPTED = "preempted"          # geographic goal cancelled for detection candidate
    CONFIRMED_EN_ROUTE = "confirmed" # detection detour cancelled: object already confirmed

    def _send_goal_and_wait(
        self, goal_x: float, goal_y: float, frame_id: str, goal_num: int = 0,
        preemptable: bool = False,
        detection_track_id: Optional[str] = None,
    ) -> tuple[Optional[bool] | str, float, float, float]:
        """
        Send a NavigateToPose goal and block until it succeeds, fails, or the
        robot gets stuck.

        If preemptable=True (geographic goal), polls the tracker for pending
        detection candidates every ~5 sim-seconds. If a new candidate appears,
        cancels Nav2 and returns (PREEMPTED, ...). Detection detours use
        preemptable=False — once started, we commit unless the object gets
        confirmed mid-navigation, in which case we cancel and return
        CONFIRMED_EN_ROUTE.

        If detection_track_id is set, polls the tracker to check whether the
        object has been confirmed during navigation. Stops early if so — no
        need to visit an already-confirmed object.

        Returns (result, accept_latency_s, time_to_first_move_s, nav_time_s):
        - result: True=success, False=fail/stuck, None=rejected/timed-out,
                  PREEMPTED=cancelled because a detection candidate appeared,
                  CONFIRMED_EN_ROUTE=detection detour cancelled: object confirmed en route
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

        # Clear events and send goal with response callback
        self._goal_response_event.clear()
        self._goal_result_event.clear()
        self._goal_accepted = None
        self._goal_handle = None

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

        # Wait for goal response (accept/reject) via callback + event
        if not self._goal_response_event.wait(timeout=90.0):
            self._logger.warning(
                "FrontierExplorer: timeout waiting for goal acceptance — skipping (not blacklisting)."
            )
            return None, _nan, _nan, 0.0

        goal_handle = self._goal_handle

        if not goal_handle or not self._goal_accepted:
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

        self._goal_handle = goal_handle
        self._active_goal_handle = goal_handle  # Keep for compatibility
        self._last_move_time = _t_accept
        # Reset bumper flag for new goal — stale contact from previous goal
        # shouldn't trigger stuck detection in the next one.
        with self._bumper_lock:
            self._bumper_contact = False

        with self._odom_lock:
            self._last_moved_x = self._robot_x
            self._last_moved_y = self._robot_y
            # Snapshot position at accept time to track first-move milestone.
            _accepted_x, _accepted_y = self._robot_x, self._robot_y

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

        _first_move_t = _nan  # sim-seconds from accepted to first 0.15 m displacement
        _sub = "waiting"  # nav sub-phase for timeline profiling
        _wall_accept = time.time()  # wall-clock escape: catch dead sim (clock frozen)
        _last_preempt_check = _t_accept  # last sim-time we checked for detection candidates

        while not self._goal_result_event.is_set():
            time.sleep(0.2)

            # Detection preemption: for geographic goals (preemptable=True),
            # poll tracker for pending candidates every ~5 sim-seconds. If a
            # new candidate appears, cancel Nav2 and return PREEMPTED so the
            # explore loop can detour toward it. Detection detours are NOT
            # preemptable — we commit once started to avoid oscillation.
            if preemptable and self._tracker is not None:
                _now = self._sim_time()
                if _now - _last_preempt_check >= 5.0:
                    _last_preempt_check = _now
                    with self._odom_lock:
                        _rx, _ry = self._robot_x, self._robot_y
                    candidates = self._tracker.get_pending_candidates(_now)
                    for cand in candidates:
                        if cand["track_id"] in self._candidate_visited:
                            continue
                        _wx, _wy = cand["world_x"], cand["world_y"]
                        if self._is_blacklisted(_wx, _wy):
                            continue
                        if math.hypot(_wx - _rx, _wy - _ry) < 1.0:
                            continue
                        # New candidate found — cancel current geographic goal
                        self._logger.info(
                            f"FrontierExplorer: PREEMPT goal#{goal_num} by detection candidate "
                            f"{cand['class_name']} ({_wx:.1f},{_wy:.1f}) "
                            f"[track {cand['track_id']}]"
                        )
                        self._tl("goal_preempted", goal_num, f"by {cand['class_name']} track {cand['track_id']}")
                        cancel_future = goal_handle.cancel_goal_async()
                        cancel_deadline = time.time() + 5.0
                        while not cancel_future.done() and time.time() < cancel_deadline:
                            time.sleep(0.1)
                        # Wait for result event so we don't leave a dangling callback
                        self._goal_result_event.wait(timeout=10.0)
                        return (
                            self.PREEMPTED,
                            accept_latency,
                            _first_move_t,
                            self._sim_time() - _t_accept,
                        )

            # Detection detour: check if the object has been confirmed while
            # navigating toward it. If so, cancel — no need to visit an
            # already-confirmed object.
            if detection_track_id and self._tracker is not None:
                if not self._tracker.is_still_pending(detection_track_id):
                    self._logger.info(
                        f"FrontierExplorer: detection candidate {detection_track_id} "
                        f"confirmed en route — cancelling detour."
                    )
                    self._tl("detection_confirmed_en_route", goal_num)
                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_deadline = time.time() + 5.0
                    while not cancel_future.done() and time.time() < cancel_deadline:
                        time.sleep(0.1)
                    self._goal_result_event.wait(timeout=10.0)
                    return (
                        self.CONFIRMED_EN_ROUTE,
                        accept_latency,
                        _first_move_t,
                        self._sim_time() - _t_accept,
                    )

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
                # Moving — clear bumper contact flag (obstacle was cleared or transient)
                with self._bumper_lock:
                    self._bumper_contact = False
            else:
                # Not moving — check stuck threshold based on bumper state
                with self._bumper_lock:
                    bumper_hit = self._bumper_contact
                stuck_threshold = (
                    BUMPER_STUCK_TIME_THRESHOLD if bumper_hit
                    else STUCK_TIME_THRESHOLD
                )
                if self._sim_time() - self._last_move_time > stuck_threshold:
                    reason = "bumper contact" if bumper_hit else "no movement"
                    self._logger.warning(
                        f"FrontierExplorer: stuck detected ({reason}) — cancelling goal."
                    )
                    self._tl("goal_stuck", goal_num, reason)
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

        result = self._goal_result
        nav_time = self._sim_time() - _t_accept
        self._active_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True, accept_latency, _first_move_t, nav_time

        self._logger.info(
            f"FrontierExplorer: goal finished with status {result.status}."
        )
        return False, accept_latency, _first_move_t, nav_time
