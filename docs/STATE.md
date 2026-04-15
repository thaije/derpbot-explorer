# STATE — derpbot-explorer

Classical modular pipeline: slam_toolbox + Nav2 + frontier explorer + OWLv2 detector.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits + [`benchmark_results.md`](benchmark_results.md).

Project spec: [`AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md) · Architecture survey: [`five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md) · Deep research: [`deep_research_robust_nav2.md`](deep_research_robust_nav2.md)

---

## Current performance

| Run | Score | Coverage | Found | Collisions | Notes |
|---|---|---|---|---|---|
| Best easy (full perception) | 66.1 C | 98% | 4/6 | 1 | seed=42, patrol mode, OWL_CONF=0.15, W_DIST=1.5 |
| Task 4 nav baseline (`--no-perception`) | 54.9 D | 71.3% | — | 0 | seed=42, footprint-aware MPPI |

**Target:** ≥ 70 (B) on easy before starting medium tier. Score is currently gated by perception, not nav. See [#8](https://github.com/thaije/derpbot-explorer/issues/8) (Task 5 — detection-aware exploration).

---

## What's running

```
slam_toolbox (async online)
  /derpbot_0/scan  →  /map (OccupancyGrid, 5 cm, TRANSIENT_LOCAL, base_footprint frame)

Nav2 stack — launch/navigation_launch.py (trimmed, no docking/route server)
  global:   SmacPlanner2D (allow_unknown: true)
  local:    RotationShimController → MPPIController (DiffDrive)
            consider_footprint: true with explicit polygon (32×22 cm)
            vx_std: 0.15, wz_std: 0.3
  safety:   collision_monitor → /derpbot_0/cmd_vel
            own lifecycle_manager_safety (isolated from navigation group)
  BT:       launch/behavior_trees/navigate_backup_clear_replan.xml
            START_OCCUPIED recovery: BackUp → ClearGlobalCostmap → retry (no gate)
  progress: PoseProgressChecker (counts angular motion — required by the shim)

agent/
  mission_client.py    HTTP → targets[]  (dicts like {"type": "fire_extinguisher"})
  frontier_explorer.py BFS /map frontiers → NavigateToPose, stuck detection, patrol mode
  detector.py          OWLv2 in subprocess (spawn), 5 Hz, conf=0.15, 90 s watchdog
  depth_projector.py   bbox → depth → world (x, y) via TF2
  tracker.py           multi-sighting fusion (≥2, >0.2 m apart), MATCH_RADIUS=2.0 m
  agent_node.py        INIT → EXPLORE → DONE, MultiThreadedExecutor

config/
  slam_toolbox_params.yaml   base_frame: base_footprint, link_match_minimum_response_fine: 0.35
  derpbot_nav2_params.yaml   robot_radius: 0.22, global inflation=0.55/csf=3.0, local=0.35/csf=5.0
```

TF chain: `map → odom → base_footprint → base_link → camera_link / lidar_link`

---

## Invariants (will bite again — keep in context)

Anything in committed config/code is omitted. Only things a fresh agent would rediscover the hard way.

### ROS 2 / Nav2
- **`/map` subscribers must use `TRANSIENT_LOCAL` QoS.** slam_toolbox publishes with TRANSIENT_LOCAL; a VOLATILE subscriber misses the held message and waits forever.
- **ROS 2 logger is single-arg.** `get_logger().info("msg %s", val)` crashes — use f-strings.
- **Always use sim clock, not wall clock**, for any timeout/stuck detection: `node.get_clock().now().nanoseconds / 1e9`. At RTF=0.16, wall-clock 30 s = only 5 sim-seconds.
- **Goal acceptance future can take 60+ s after a cancel.** `bt_navigator` finishes its current BT tick before processing preemption. Acceptance timeout 90 s; return `None` (retry, no blacklist) on timeout.
- **Goal rejection ≠ unreachable frontier.** `accepted == False` means Nav2 is temporarily busy. Return `None` on rejection; only return `False` (blacklist) on `STATUS_ABORTED` or stuck detection.
- **RotationShimController MUST pair with `PoseProgressChecker`.** `SimpleProgressChecker` treats in-place rotation as "no progress" → 10 s timeout → cascades. See [#2](https://github.com/thaije/derpbot-explorer/issues/2).
- **Stuck detection race:** if goal succeeds during `time.sleep(0.2)`, the stuck-check fires before the while-loop re-evaluates and falsely blacklists a good frontier. Fix: `if result_future.done(): break` immediately after sleep.

### Runtime / environment
- **Gazebo must be paused during stack bringup.** `start_stack.sh` pauses the sim right after "Simulation ready" and unpauses once the agent touches `/tmp/derpbot_agent_ready` (set at `_explore_loop` entry, after OWLv2 load + Nav2 action-client construction). Without the pause, ~20 wall-s of slam+nav2+agent cold-launch × RTF consumes ~40 sim-s of the 300 s mission budget and floods Nav2 with TF_OLD_DATA. Never start the stack manually — use `start_stack.sh` or delegate to the `arst-runner` agent. See [#18](https://github.com/thaije/derpbot-explorer/issues/18).
- **Ghost TF from previous run corrupts new run.** Always `ros2 daemon stop && ros2 daemon start` and kill all `gz`/`ros2` processes between runs.
- **Always restart slam_toolbox between scenario runs** — it retains its old map; reusing makes FrontierExplorer think the map is already explored and blacklist every goal within ~2 min.
- **FastDDS discovery server required for monitoring scripts.** All `ros2` CLI subprocesses need `ROS_DISCOVERY_SERVER` set. Source `scripts/ros_env.sh` before any out-of-stack ROS2 command. `ROS_SUPER_CLIENT=1` required for `ros2 node/topic list` to see the full graph.
- **RViz degrades RTF.** Close it before measuring performance.
- **Never call PyTorch / OWLv2 from a ROS2 executor thread.** Python GIL contention with the 8-thread executor causes indefinite hangs (`time.sleep(0.01)` → ~10 s). Run inference in a `multiprocessing` subprocess (spawn context). `detector.py` already does this.

### Frontier exploration
- **Pick closest-to-centroid cell, not closest-to-robot.** Centroid can land in unknown/inflated space; "closest-to-robot" causes immediate Nav2 success with no map update. Closest-to-centroid picks a free interior cell that actually requires navigation.
- **Blacklist frontier centroids on success too.** With `xy_goal_tolerance=0.5 m`, Nav2 succeeds when the robot is within 0.5 m. Without success-blacklist, the same cluster gets re-selected every loop; legitimate frontiers disappear naturally once the map updates.
- **Frontier W_DIST=1.5** balances coverage. ≥4.0 keeps the robot local; ≤0.5 causes cross-map thrashing. Never go below 1.0 without testing.
- **OWL_CONF_THRESHOLD=0.15** is the tuned value. 0.10=3FP/3real, 0.12=3FP/3real, 0.15=1FP/4real. Don't lower without a new mechanism.
- **Robot avg_speed is ~0.022 m/s (mostly idle).** Most time is spent waiting on Nav2 goal acceptance and rotating at waypoints. Coverage comes from LiDAR sweeps during rotation, not path length. Reducing inter-goal idle time is the primary speed-score lever.
- **LIDAR coverage ≠ camera coverage.** LIDAR (10–20 m) maps remote areas before the camera visits them. Patrol mode navigates to physically unvisited free cells after frontier exhaustion.

---

## How to run

Delegate to the `arst-runner` subagent for scored runs — it handles atomic startup, RTF/TF-flood checks, monitoring, and score reporting. For interactive observation use the `arst-test` skill.

Manual debug-only:
```bash
python3.12 agent/agent_node.py                  # full stack
python3.12 agent/agent_node.py --no-perception  # nav-only, found_ratio=0
./scripts/start_stack.sh [--no-perception]      # sim + slam + nav2 + agent
```

Results: `~/Projects/robot-sandbox/results/`. Key fields: `overall_score`, `overall_grade`, `raw_metrics.found_ratio`, `raw_metrics.exploration_coverage`, `raw_metrics.collision_count`.

**Per-run timeline profile** (auto-written every run, no flag needed): `results/profile_<UTC>.md`. Phase budget table + per-goal breakdown + raw timeline. Use when diagnosing where sim-time goes (waiting / rotating / traveling / nav2_send / recovery). Summarise with `python3.12 scripts/profile_run.py [path]` (defaults to most recent; pass globs to compare runs).

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues, not this doc.

- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before proposing a major change, check closed dead-ends:**
  `gh issue list --state closed --label dead-end --search <topic>`
- **Before retrying a previously-done task:**
  `gh issue list --state closed --label task --search <topic>`
- **Backlog / known bugs:**
  `gh issue list --state open --label backlog`
- **New finding during a task:** file it immediately with the right label (`task`, `bug`, `dead-end`, `backlog`, `upstream`), don't add it to this file. Invariants (things that apply across all future work) do go here — keep them ≤ 3 lines each.
