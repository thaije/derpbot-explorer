# Agent Handoff — autonomous derpbot explorer

Plan for current Nav2 ROS2 approach: [`docs/approach1_classical_pipeline.md`](approach1_classical_pipeline.md)
Architecture survey (all 5 approaches): [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)  
Task + robot spec: [`docs/AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md)

---

## What works (verified)

- **slam_toolbox**: publishing `/map` (TRANSIENT_LOCAL, 5 cm res, `base_footprint` frame)
- **Nav2 stack**: all nodes active — SmacPlanner2D + MPPI controller + recovery behaviours + collision_monitor
- **Mission client**: fetches targets from `http://localhost:7400/mission`; handles dict targets `{"type": "..."}` correctly
- **Detector**: Single-stage OWLv2 (`google/owlv2-base-patch16-ensemble`). Takes mission target names as text queries (underscores→spaces), returns bboxes + confidence directly. Runs in a separate subprocess (spawn context) to avoid GIL contention. Verified detecting fire_extinguisher and first_aid_kit reliably at conf=0.20. Includes 90s watchdog that restarts the subprocess on CUDA hang.
- **FrontierExplorer**: BFS frontiers + patrol mode. 98% coverage with W_DIST=1.5. Best score: 66.1 (C), 4/6 found, 1 collision (seed=42, easy scenario).
- **cmd_vel pipeline**: controller_server → velocity_smoother → collision_monitor → `/derpbot_0/cmd_vel` ✓

---

## Key Gotchas

- **`/map` subscriber must use `TRANSIENT_LOCAL` QoS** — slam_toolbox publishes with TRANSIENT_LOCAL; a VOLATILE subscriber misses the held message and waits forever.
- **Mission targets are dicts** — the HTTP endpoint returns `{"type": "fire_extinguisher", ...}` objects, not plain strings. `mission_client.py` extracts `t["type"]`.
- **ROS 2 logger is single-arg** — `get_logger().info("msg %s", val)` crashes. Use f-strings.
- **Nav2 Jazzy `navigation_launch.py` includes `docking_server`** — fails without dock plugin config. Use our trimmed `launch/navigation_launch.py` (excludes docking + route server).
- **slam_toolbox launch arg is `slam_params_file`**, not `params_file`.
- **MPPI `consider_footprint: true` crashes** without a polygon footprint. Set `consider_footprint: false` when using `robot_radius`.
- **numpy must be `<2`** — cv_bridge was compiled against numpy 1.x; numpy 2.x causes `ImportError` in image callbacks. Pin with `uv pip install "numpy<2"`.
- **slam_toolbox `base_frame: base_footprint`** — odom's `child_frame_id` is `base_footprint`, not `base_link`.
- **Nav2 stuck detection threshold**: 8 s too short (Nav2 rotates in place first). Set to 30 sim-seconds. **Must use sim clock** (`node.get_clock().now().nanoseconds / 1e9`), NOT `time.time()` (wall clock). At RTF=0.16, wall-clock 30 s = only 5 sim-seconds; stuck detection would fire while the robot is still turning.
- **Goal acceptance future can take 60+ s** after a goal cancel — bt_navigator finishes its current BT tick before processing the preemption. Acceptance timeout set to 90 s; returns `None` (skip, no blacklist) on timeout.
- **`/map` TRANSIENT_LOCAL delivery**: with FastDDS discovery server, map arrives in <30s. Without it, DDS peer discovery takes ~2 min. Always start with FastDDS (see `scripts/ros_env.sh`).
- **Frontier goal point must be closest-to-centroid cell** — centroid can land in unknown/inflated space. "Closest-to-robot" causes immediate Nav2 success (robot already within tolerance) with no map update. "Closest-to-centroid" picks a free cell in the interior of the frontier, requiring actual navigation.
- **Blacklist frontier centroids on success too** — with `xy_goal_tolerance=0.5 m`, Nav2 succeeds when robot is within 0.5 m. If the frontier cells still exist (unknown area behind a wall), the same cluster is re-selected every loop. Blacklisting on success prevents this. Legitimate frontiers disappear naturally once the map updates from the robot's new scans.
- **SLAM smearing near furniture** — raise `link_match_minimum_response_fine` (now 0.35) to reject weak scan matches, and `minimum_travel_distance` (now 0.2 m) to skip map updates on jitter. Loop closure (`do_loop_closing: true`) corrects accumulated drift when the robot revisits known areas.
- **Always restart slam_toolbox between scenario runs** — it keeps its old map after a scenario ends. If reused, FrontierExplorer sees an already-explored map with no frontiers and blacklists all goals within ~2 min.
- **PyTorch/CUDA deadlocks in ROS2 executor threads** — calling `model.predict()` from any ROS2 executor callback or executor-managed thread causes an indefinite hang due to Python GIL starvation (8 executor threads starve inference thread; `time.sleep(0.01)` took ~10 s). Fix: run inference in a separate subprocess (`multiprocessing`, spawn context). Never call YOLOE/PyTorch from executor threads.
- **Nav2 lifecycle DDS timeout at startup** — on first launch, `smoother_server` occasionally times out during lifecycle transition with "failed to send response to /smoother_server/change_state". Kill Nav2 and restart; second launch succeeds.
- **inflation=0.25 empirically optimal despite theoretically blocking 1m doors** — OfficeA door (1m wide) is theoretically blocked at 0.25m inflation, but recovery behaviors (spin+backup) push the robot through. High inflation = fast goal rejection (planner fails immediately, robot moves on); low inflation (0.12) = slow rejection (MPPI execution fails) → blacklist fills → 33-43% coverage. Use 0.25 for both global and local.
- **Goal clearance code changes path divergence — do NOT use** — Clearance code (prefer cells ≥0.25m from obstacles) changes which frontier cells are selected as goals, causing path divergence: finds fire_ext#2 (1.7,4.7) but misses fire_ext#1 (1.4,7.5) + person, net loss vs 66.1 baseline. Also caused 5 collisions (100% coverage sends robot into unmapped meeting room furniture). Pushing was even worse (3 collisions + 5 FPs). REVERTED: use pure closest-to-centroid cell selection (the 66.1 baseline). See commits 8b2c54e→73bd0f1 for history.
- **OWL_CONF_THRESHOLD: currently 0.15** — Tuning table: 0.10=3FP/3real; 0.12=3FP/3real (no improvement vs 0.10); 0.15=1FP/4real (best). Use 0.15. fire_ext#2 detection at 0.15 is marginal (confidence near threshold) but found when coverage is good.
- **collision_monitor `scan: min_height` must be ≤ 0.10m** — DerpBot's LiDAR is at ~0.10m above ground. The default `min_height: 0.15` silently filters ALL scan data, making the collision monitor completely blind. This caused 277 physics collisions in one run. Set to `0.05`.
- **Local costmap window must cover the nearest obstacles** — window_radius (= width/2) is the maximum distance at which MPPI sees obstacles. A 3m window (1.5m radius) missed desk furniture at ~3.08m → 27 near-miss oscillations → Nav2 lifecycle failure (heartbeat timeout). Enlarging to 5m/5cm/3Hz tripled CPU → same failure. Fix: 6m/10cm/2Hz — same 3600-cell count as original, desk visible from start, `obstacle_max_range ≥ 3.5m`. CPU tuning order: resolution first (10cm→15cm = 2.25× fewer cells), then window size; keep frequency as high as feasible since MPPI plans against the costmap (stale costmap = bad avoidance of moving obstacles).
- **`max_planning_time` must be ≥ 5.0 sim-seconds** — the default 2.0 s causes cross-building path plans to immediately timeout with `NO_PATH_FOUND` and abort the goal.
- **`transform_tolerance: 0.3` everywhere** — set in controller_server, local_costmap, global_costmap, behavior_server, and collision_monitor. Lower values cause TF timestamp age failures at non-nominal RTF.
- **Nav2 goal rejection ≠ unreachable frontier** — `goal_handle.accepted == False` means Nav2 is temporarily busy (cancel/preempt in progress), not that the frontier is unreachable. Return `None` (retry, no blacklist) on rejection; only return `False` (blacklist) on `STATUS_ABORTED` or stuck detection.
- **OWLv2 and Gazebo share GPU 0 (single-GPU hardware)** — previously `CUDA_VISIBLE_DEVICES=1` kept PyTorch off Gazebo's rendering GPU; now only GPU 0 exists so both share it (`CUDA_VISIBLE_DEVICES=0`). Expect lower RTF vs. 3-GPU baseline. If RTF collapses: consider reducing OWLv2 inference rate or switching to a lighter model.
- **Ghost TF from previous run corrupts new run** — when a sim is killed and a new one started at t=0, DDS delivers cached TF messages from the old run (which had timestamps 800–900+s) before the new sim's data. The new run's TF buffer sees these future-timestamped transforms first; current transforms then look "old". Result: all Nav2 goals immediately abort (status 6), robot never moves. Fix: kill the ROS2 daemon (`ros2 daemon stop`) and restart it (`ros2 daemon start`) between runs, and kill ALL gz/ros2 processes by PID before restarting.
- **RViz degrades RTF** — opening RViz subscribes to costmap, /map, and TF topics, spiking CPU. Can cause temporary RTF dips to ~1.25 at speed=2. Close RViz before measuring performance.
- **Frontier W_DIST balance** — W_DIST=1.5 gives 98% coverage (current). Higher (4.0) keeps robot too local; lower (0.5) causes cross-map thrashing. Never go below 1.0 without testing.
- **Starting SLAM/Nav2/agent late into a running sim causes TF flood** — if the stack starts 30+ sim-seconds after the sim, every Nav2 node gets flooded with TF_OLD_DATA warnings for wheel joint frames at ~56 Hz. This saturates CPU through the ROS2 DDS logging layer and drags RTF from ~2.0 to ~0.4. Always start SLAM → Nav2 → agent within 5 wall-seconds of the sim reporting ready. Never resume a session where the sim has been idle for minutes without restarting the entire stack from scratch.
- **98% LIDAR coverage ≠ 98% camera coverage** — LIDAR range (10-20m) can map remote areas before the camera ever visits them. Fix: patrol mode navigates to physically unvisited free cells after frontier exhaustion.
- **Info-gain frontier scoring fails early in exploration** — flood-filling unknown cells through frontiers gives all frontiers the same score (cap=5000 cells) when the map is mostly unexplored (all unknown regions are one connected blob). W_DIST then dominates, robot picks tiny nearby clusters → micro-stepping → Nav2 path failures → blacklist fills → robot stuck. Coverage collapsed from 84% → 32%. Fix: revert to cluster.size scoring. Future work: scipy connected-components normalization so frontiers in separate unknown pockets get different scores. Never re-enable without connected-components normalization.
- **FastDDS discovery server required for monitoring scripts** — after adding `ROS_DISCOVERY_SERVER`, all `ros2` CLI subprocesses (including inside `rtf_monitor.py`) must also have the var set or they see no topics. Source `scripts/ros_env.sh` before any ROS2 command outside the stack sessions. The discovery server runs in tmux session `fds`; kill it with `pkill -f "fastdds discovery"`. `ROS_SUPER_CLIENT=1` required for `ros2 node/topic list` to show the full graph.
- **Robot avg_speed is ~0.022 m/s (mostly idle)** — most time is spent waiting on Nav2 goal acceptance and rotating at waypoints. Coverage comes from LiDAR sweeps during rotation, not path length. Reducing inter-goal idle time is the primary win for speed score.
- **Nav2 timing benchmark (seed=42, easy, 2026-04-06)** — 15 goals, 4 accepted, 1 success. Phase breakdown: BFS+selection 2.6%, Nav2 accept 7.4% (mean 3.8s), rotation/spin-up 30.1% (median 22s **per waypoint** — Nav2 rotates to face goal direction before every translate), travel 45.4%. Two dominant failures: (a) rejected 11× in a row on same frontier `(16.88, 3.89)` — no blacklist on rejection so selector re-picks it indefinitely, burned ~160/300 sim-s; (b) 22s pre-travel rotation per goal.
- **Repeated rejection = blacklist bug** — `result=None` (Nav2 rejected) never blacklists the frontier. Same bad frontier can loop forever consuming the entire run. Fix: blacklist after N consecutive rejections of the same goal position.
- **Default Nav2 BT kills `collision_monitor` via costmap-clear storm** — `navigate_to_pose_w_replanning_and_recovery.xml` enters a ClearGlobalCostmap→replan loop on planning failure; rapid DDS flooding starves `collision_monitor` bond heartbeat → lifecycle_manager shuts down entire Nav2 stack → "Action server is inactive" forever. Fix: use `navigate_w_replanning_only_if_path_becomes_invalid.xml` (aborts cleanly on failure, no recovery loop). Set in `derpbot_nav2_params.yaml` as `default_nav_to_pose_bt_xml`.
- **`collision_monitor` bond starvation** — starves when CPU/DDS load is high during long navigations; `bond_timeout` raised to 60s in `launch/navigation_launch.py`; `attempt_respawn_on_failure: True` added so lifecycle_manager restarts dead nodes instead of killing the whole stack.
- **"Start occupied" cascade after frontier arrival** — robot reaches frontier → SLAM maps new nearby walls → inflation marks robot's cell LETHAL → SmacPlanner refuses every subsequent goal with "Start occupied". Old BT recovered via ClearGlobalCostmap loop (caused heartbeat storm); new fix: call `clear_global_costmap` once (fire-and-forget) after each failed goal in `frontier_explorer.py`.
- **Stuck detection race condition** — if goal succeeds during `time.sleep(0.2)`, the stuck-check fires before the while-loop condition re-evaluates, falsely reporting SUCCESS as FAILED and blacklisting a good frontier. Fixed: `if result_future.done(): break` immediately after sleep.

---

## Architecture

**Key design decisions:**
- slam_toolbox async mode — drops scans under load, never blocks
- Nav2 SmacPlanner2D + MPPI — `allow_unknown: true` so robot plans into unexplored space
- `/map` subscriber uses TRANSIENT_LOCAL QoS — gets map immediately on subscribe
- FrontierExplorer polls goal future (not `spin_until_future_complete`) — node already spins in MultiThreadedExecutor
- Detector uses OWLv2 text queries — mission targets become text prompts, no retraining
- Tracker requires ≥2 sightings from diverse poses (>0.2 m apart) before publishing — reduces false positives; MATCH_RADIUS=2.0m merges depth-noise duplicates

## Key files

```
slam_toolbox (async online)
  scan: /derpbot_0/scan  →  /map (OccupancyGrid, 5 cm res, TRANSIENT_LOCAL)

Nav2 stack (launch/navigation_launch.py — trimmed, no docking/route server)
  config/derpbot_nav2_params.yaml
  global planner:  SmacPlanner2D
  local controller: MPPI (DiffDrive)
  safety:          collision_monitor → /derpbot_0/cmd_vel

agent/
  mission_client.py    — GET http://localhost:7400/mission → targets[]
  frontier_explorer.py — BFS /map frontiers → NavigateToPose goals, stuck detection
  detector.py          — OWLv2 (google/owlv2-base-patch16-ensemble) on GPU 0 (shared w/ Gazebo), subprocess, 5 Hz, conf=0.20; 90s watchdog
  depth_projector.py   — bbox centre + depth image + TF2 → world (x, y)
  tracker.py           — multi-sighting fusion, persistent IDs, publishes /derpbot_0/detections
  agent_node.py        — INIT → EXPLORE → DONE state machine, MultiThreadedExecutor

config/
  slam_toolbox_params.yaml   — base_frame: base_footprint, scan_topic: /derpbot_0/scan
  derpbot_nav2_params.yaml   — robot_radius: 0.22, inflation=0.25 (both global+local), cost_scaling=3.0
```

## TF frames (confirmed)
`map → odom → base_footprint → base_link → camera_link / lidar_link`

## How to run

Delegate to the `arst-runner` subagent — it handles atomic startup, RTF/TF-flood checks, monitoring, and score reporting. Never start the stack manually; timing constraints make that unreliable.

For interactive observation during a run, `/arst-test` skill has the monitoring commands.

### Agent (manual launch, debug only)

```bash
# From derpbot-explorer root:
python3.12 agent/agent_node.py

# SLAM:
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml use_sim_time:=true

# Nav2:
ros2 launch $(pwd)/launch/navigation_launch.py \
    params_file:=$(pwd)/config/derpbot_nav2_params.yaml use_sim_time:=true
```

All three must start within 5 wall-seconds of sim ready. Use `scripts/start_stack.sh` to handle this automatically.

Results land in `~/Projects/robot-sandbox/results/`. Key fields: `overall_score`, `overall_grade`, `raw_metrics.found_ratio`, `raw_metrics.exploration_coverage`, `raw_metrics.collision_count`.

---

## Development guidelines

Test new features before marking complete. Prefer Python unit tests where possible.
