# Agent Handoff — autonomous derpbot explorer

Plan for current Nav2 ROS2 approach: [`docs/approach1_classical_pipeline.md`](approach1_classical_pipeline.md)
Architecture survey (all 5 approaches): [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)  
Task + robot spec: [`docs/AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md)

---

## What works (verified)

- **slam_toolbox**: publishing `/map` (TRANSIENT_LOCAL, 5 cm res, `base_footprint` frame)
- **Nav2 stack**: all nodes active — SmacPlanner2D + MPPI controller + recovery behaviours + collision_monitor
- **Mission client**: fetches targets from `http://localhost:7400/mission`; handles dict targets `{"type": "..."}` correctly
- **Detector**: Single-stage OWLv2 (`google/owlv2-base-patch16-ensemble`). Takes mission target names as text queries (underscores→spaces), returns bboxes + confidence directly. Runs in a separate subprocess (spawn context) to avoid GIL contention. Verified detecting fire_extinguisher and first_aid_kit reliably at conf=0.20. exit_sign is NOT reliably detected (see gotchas). Includes 90s watchdog that restarts the subprocess on CUDA hang.
- **FrontierExplorer**: BFS frontiers detected on `/map`, goals sent to Nav2, robot navigating. ~55% coverage currently — Meeting Room and deep Office B not consistently reached.
- **cmd_vel pipeline**: controller_server → velocity_smoother → collision_monitor → `/derpbot_0/cmd_vel` ✓

Not yet verified: depth projection, tracker publishing confirmed detections, end-to-end detection score.

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
- **`/map` TRANSIENT_LOCAL delivery takes ~2 min** after agent start — DDS peer discovery is slow when many entities exist. Normal; agent polls until map arrives.
- **Frontier goal point must be closest-to-centroid cell** — centroid can land in unknown/inflated space. "Closest-to-robot" causes immediate Nav2 success (robot already within tolerance) with no map update. "Closest-to-centroid" picks a free cell in the interior of the frontier, requiring actual navigation.
- **Blacklist frontier centroids on success too** — with `xy_goal_tolerance=0.5 m`, Nav2 succeeds when robot is within 0.5 m. If the frontier cells still exist (unknown area behind a wall), the same cluster is re-selected every loop. Blacklisting on success prevents this. Legitimate frontiers disappear naturally once the map updates from the robot's new scans.
- **SLAM smearing near furniture** — raise `link_match_minimum_response_fine` (now 0.35) to reject weak scan matches, and `minimum_travel_distance` (now 0.2 m) to skip map updates on jitter. Loop closure (`do_loop_closing: true`) corrects accumulated drift when the robot revisits known areas.
- **Always restart slam_toolbox between scenario runs** — it keeps its old map after a scenario ends. If reused, FrontierExplorer sees an already-explored map with no frontiers and blacklists all goals within ~2 min.
- **PyTorch/CUDA deadlocks in ROS2 executor threads** — calling `model.predict()` from any ROS2 executor callback or executor-managed thread causes an indefinite hang due to Python GIL starvation (8 executor threads starve inference thread; `time.sleep(0.01)` took ~10 s). Fix: run inference in a separate subprocess (`multiprocessing`, spawn context). Never call YOLOE/PyTorch from executor threads.
- **Nav2 lifecycle DDS timeout at startup** — on first launch, `smoother_server` occasionally times out during lifecycle transition with "failed to send response to /smoother_server/change_state". Kill Nav2 and restart; second launch succeeds.
- **inflation=0.25 empirically optimal despite theoretically blocking 1m doors** — With inflation=0.25: OfficeA door (1m wide) is theoretically blocked (1.0 - 2*(0.10+0.22+0.25) = -0.14m), but in practice the robot navigates through via recovery behaviors (spin+backup) which inadvertently push it through. Result: 0 near-misses, 97.8% coverage. With inflation=0.12: door always open in costmap but MPPI execution fails in tight areas → more slow ABORTS → blacklist fills → 33-43% coverage. Root cause: high inflation causes FAST rejection of bad goals (planner fails immediately) while low inflation causes SLOW rejection (MPPI execution fails). Fast rejection = robot moves on quickly = better coverage. Use inflation=0.25 for both global and local. GOAL_CLEARANCE_M=0.50m to match (robot_radius+inflation=0.47m).
- **Frontier goals adjacent to furniture: soft clearance only** — GOAL_CLEARANCE_M=0.25m prefers cells ≥0.25m from obstacles when available. If no cell passes, falls back to closest-to-centroid (no push). CRITICAL: do NOT push goals into unknown territory — SLAM hasn't mapped furniture there, causing collisions. Pushing with 0.50m step gave 3 collisions + 5 FPs vs 1 collision + 1 FP without push. Current setting: 0.25m soft preference, no push fallback.
- **OWL_CONF_THRESHOLD: currently 0.15** — Tuning table: 0.10=3FP/3real; 0.12=3FP/3real (no improvement vs 0.10); 0.15=1FP/4real (best). Use 0.15. fire_ext#2 detection at 0.15 is marginal (confidence near threshold) but found when coverage is good.
- **exit_sign is not detectable by OWLv2 in this sim** — the model is a realistic 3D mesh (black housing, teal face, white "EXIT" text) mounted high on walls. DerpBot's camera is at ground level; the sign appears as a tiny dark rectangle near the top of the image. OWLv2 produces near-zero confidence at any reasonable threshold. Even with threshold=0.10, zero exit signs were found across multiple runs. The 4 exit signs account for 4/9 targets in the easy scenario — this is the primary bottleneck to a good score.
- **collision_monitor `scan: min_height` must be ≤ 0.10m** — DerpBot's LiDAR is at ~0.10m above ground. The default `min_height: 0.15` silently filters ALL scan data, making the collision monitor completely blind. This caused 277 physics collisions in one run. Set to `0.05`.
- **`max_planning_time` must be ≥ 5.0 sim-seconds** — the default 2.0 s causes cross-building path plans to immediately timeout with `NO_PATH_FOUND` and abort the goal.
- **`transform_tolerance: 0.3` everywhere** — set in controller_server, local_costmap, global_costmap, behavior_server, and collision_monitor. Lower values cause TF timestamp age failures at non-nominal RTF.
- **Nav2 goal rejection ≠ unreachable frontier** — `goal_handle.accepted == False` means Nav2 is temporarily busy (cancel/preempt in progress), not that the frontier is unreachable. Return `None` (retry, no blacklist) on rejection; only return `False` (blacklist) on `STATUS_ABORTED` or stuck detection.
- **OWLv2 must not touch GPU 0 (Gazebo's rendering device)** — PyTorch initialising on GPU 0 starves Gazebo's Ogre2 renderer, collapsing RTF. Fix: `CUDA_VISIBLE_DEVICES=1` set in `detector.py`'s subprocess worker *before* torch is imported, so PyTorch only ever sees GPU 1 (RTX 2070 SUPER). This is already in the code; don't remove it.
- **numactl required for ALL stack components** — NUMA node 0 (cores 0–9, 20–29) is thermally throttled to ~230 MHz; node 1 (cores 10–19, 30–39) runs at 2400–3100 MHz. Without numactl, sim-only RTF at speed=2 drops to 0.78. Fix: `numactl --cpunodebind=1 --membind=1` applied in `scripts/start_stack.sh` for sim, SLAM, Nav2, and agent. Both GPUs are on node 1 as well. RTF with full stack at speed=2 = 1.89 (stable).
- **Ghost TF from previous run corrupts new run** — when a sim is killed and a new one started at t=0, DDS delivers cached TF messages from the old run (which had timestamps 800–900+s) before the new sim's data. The new run's TF buffer sees these future-timestamped transforms first; current transforms then look "old". Result: all Nav2 goals immediately abort (status 6), robot never moves. Fix: kill the ROS2 daemon (`ros2 daemon stop`) and restart it (`ros2 daemon start`) between runs, and kill ALL gz/ros2 processes by PID before restarting.
- **RViz degrades RTF** — opening RViz subscribes to costmap, /map, and TF topics, spiking CPU. Can cause temporary RTF dips to ~1.25 at speed=2. Close RViz before measuring performance.
- **Frontier W_DIST balance** — W_DIST=4.0 keeps the robot too local (62% coverage); W_DIST=0.5 causes cross-map thrashing. W_DIST=2.0 gave 84% coverage (with FastDDS). Now trying W_DIST=1.5 to push toward 90%+. Never go below 1.0 without testing.
- **Starting SLAM/Nav2/agent late into a running sim causes TF flood** — if the stack starts 30+ sim-seconds after the sim, every Nav2 node gets flooded with TF_OLD_DATA warnings for wheel joint frames at ~56 Hz. This saturates CPU through the ROS2 DDS logging layer and drags RTF from ~2.0 to ~0.4. Always start SLAM → Nav2 → agent within 5 wall-seconds of the sim reporting ready. Never resume a session where the sim has been idle for minutes without restarting the entire stack from scratch.
- **Upper half of map (y > 8) not reliably reached within 900 sim-seconds** — at W_DIST=2.0 the robot systematically explores the entire lower half (Offices A/B + lower corridors) before pushing through the doorways into the Meeting Room area. A number of objects are in the upper half and are consistently missed. Coverage stalls at ~52%. Known issue; tuning needed.
- **first_aid_kit #2 at (6.437, 11.69) is persistently undetectable** — robot navigates to within ~2m (goals at y~10-11) but OWLv2 never triggers. No elevated surface obstacle near it (conf_chairs, no table). Possibly a visual appearance issue or the object faces away from typical nav paths. Not yet solved; may need a specific vantage point or lower threshold.
- **98% LIDAR coverage ≠ 98% camera coverage** — LIDAR has 10-20m range; robot at (14,3) can LIDAR-scan the upper section (y>7) without the camera ever pointing there. All 3 missed objects (y=7.55, 8.46, 11.69) were in LIDAR-covered area but the camera never got close. Fix: patrol mode navigates the robot to physically unvisited regions after frontier exhaustion.
- **Info-gain frontier scoring fails early in exploration** — flood-filling unknown cells through frontiers gives all frontiers the same score (cap=5000 cells) when the map is mostly unexplored (all unknown regions are one connected blob). W_DIST then dominates, robot picks tiny nearby clusters → micro-stepping → Nav2 path failures → blacklist fills → robot stuck. Coverage collapsed from 84% → 32%. Fix: revert to cluster.size scoring. Future work: scipy connected-components normalization so frontiers in separate unknown pockets get different scores. Never re-enable without connected-components normalization.
- **FastDDS discovery server required for monitoring scripts** — after adding `ROS_DISCOVERY_SERVER`, all `ros2` CLI subprocesses (including inside `rtf_monitor.py`) must also have the var set or they see no topics. Source `scripts/ros_env.sh` before any ROS2 command outside the stack sessions. The discovery server runs in tmux session `fds`; kill it with `pkill -f "fastdds discovery"`. `ROS_SUPER_CLIENT=1` required for `ros2 node/topic list` to show the full graph.
- **Robot avg_speed is ~0.022 m/s (mostly idle)** — robot's actual travel speed averaged across full 900s run is ~0.022 m/s vs max 0.5 m/s. Robot spends most time waiting on Nav2 goal acceptance, cleanup between goals, and rotation. 19.72m path in 900s. Coverage (84%) comes from LiDAR sweeps during rotation at waypoints, not from path length. Reducing idle time is the primary win for higher coverage (Task 3/4a).
- **seed=42 has no exit_sign as mission targets** — the 4 exit_sign objects in the world are labelled but NOT in the mission target list for seed=42. Mission targets are fire_extinguisher(3), first_aid_kit(2), person(1) = 6 total. exit_sign concern may be seed-specific.

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
  detector.py          — OWLv2 (google/owlv2-base-patch16-ensemble) on GPU 1 via numactl, subprocess, 5 Hz, conf=0.20; 90s watchdog
  depth_projector.py   — bbox centre + depth image + TF2 → world (x, y)
  tracker.py           — multi-sighting fusion, persistent IDs, publishes /derpbot_0/detections
  agent_node.py        — INIT → EXPLORE → DONE state machine, MultiThreadedExecutor

config/
  slam_toolbox_params.yaml   — base_frame: base_footprint, scan_topic: /derpbot_0/scan
  derpbot_nav2_params.yaml   — robot_radius: 0.22, inflation=0.25 (both global+local), cost_scaling=3.0. GOAL_CLEARANCE_M=0.25m (soft preference, no push) in frontier_explorer.py
```

## TF frames (confirmed)
`map → odom → base_footprint → base_link → camera_link / lidar_link`

## How to run

Delegate to the `arst-runner` subagent — it handles atomic startup, RTF/TF-flood checks, monitoring, and score reporting. Never start the stack manually; timing constraints make that unreliable.

For interactive observation during a run, `/arst-test` skill has the monitoring commands.

### Agent (manual launch, debug only)

```bash
# From derpbot-explorer root:
numactl --cpunodebind=1 --membind=1 python3.12 agent/agent_node.py

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

## Current performance (easy scenario, speed=2)

| Run | Seed | Score | Found | Coverage | RTF | Notes |
|-----|------|-------|-------|----------|-----|-------|
| Best (prev session) | 42 | 52.1 (D) | 4/6 | 52% | ~1.97 | 1 collision, 2 FP |
| 2026-03-21 FastDDS baseline | 42 | **55.4 (C)** | 3/6 | **84%** | ~1.89 | 10 near-miss, 2 collision, 1 FP. FastDDS = +32% coverage vs old multicast. avg_speed=0.022 m/s (mostly idle). Targets: fire_ext(3), first_aid(2), person(1); seed=42 has NO exit_signs as mission targets |
| 2026-03-21 info-gain scoring (FAILED) | 42 | 41.3 (D) | 3/6 | 32% | ~1.89 | 12 collision, 7 FP, 0.674m travel. Info-gain scoring reverted (see gotcha below) |
| 2026-03-21 W_DIST=1.5 | 42 | 57.6 (C) | 3/6 | **98%** | ~1.97 | 1 collision, 3 FP. Coverage solved (W_DIST=1.5 works). Detection bottleneck: all 3 real detections at t<55s near start. 3 missing objects (fire_ext#1 at y=7.55, first_aid#2 at y=11.69, person at y=8.46) never physically visited by camera. Robot ran out of frontiers after 6 goals, idled rest of scenario. 3 FPs from OWL_CONF=0.10 |
| 2026-03-21 patrol+OWL=0.15 | 42 | **66.1 (C)** | 4/6 | 98% | ~1.97 | 1 FP. Patrol+W_DIST=1.5. fire_ext#1(y=7.55)+person(y=8.46) found via upper-map goal early. fire_ext#2(y=4.70) missed (OWL=0.15 too high for weak detection). first_aid#2(y=11.69) still missed (structural issue). 1 collision. |
| 2026-03-21 OWL=0.12 run1 | 42 | 50.5 (D) | 3/6 | **29%** | ~1.97 | 3 FP, 5 collisions (t=99-109s). Nav failure: robot explored only east side, never reached y>7. OWL=0.12 is NOT better than 0.15 — same FP rate 0.10 level. Reverted to 0.15. |
| 2026-03-21 infl=0.20 | 42 | 29.1 (F) | 2/6 | 28% | ~1.97 | 13 collisions. Root cause: infl=0.20 still blocks 1m OfficeA door → planner reroutes through OfficeB (x=14-15) → long detour → collisions. |
| 2026-03-21 infl=0.12 | 42 | 45.2 (D) | 3/6 | 43% | ~1.97 | 0 collisions! 49 near-misses (recovery spins). Found lower section only. OfficeA door theoretically open but robot kept trying east section (x=9 wall). Goal clearance fix not yet active. |
| 2026-03-21 infl=0.12+0.05+clearance | 42 | 56.8 (C) | 4/6 | 33% | ~1.97 | 1 collision, 11 near-misses, 1 FP. Coverage REGRESSION: low inflation = slow Nav2 rejection = blacklist fills at 33% coverage. Reverted to 0.25/0.25. |
| 2026-03-21 infl=0.25+clearance=0.50m+push | 42 | 42.2 (D) | 3/6 | 90% | ~1.97 | 3 collisions, 0 near-misses, 5 FPs! Coverage OK (90%) but push fallback sends robot into unmapped furniture → collisions. Removed push, reduced clearance to 0.25m soft preference. |

## RTF benchmarking (2026-03-20, RESOLVED)

**Root cause**: NUMA node 0 (cores 0–9, 20–29) thermally throttled to ~230 MHz; node 1 runs at 2400–3100 MHz. Fix: `numactl --cpunodebind=1 --membind=1` applied to sim, SLAM, Nav2, and agent in `scripts/start_stack.sh`.

| Configuration | Speed setting | RTF avg | Range |
|---------------|--------------|---------|-------|
| Sim only — no numactl | 2 | 0.78 | 0.49–1.27 |
| Sim only — numactl node 1 | 2 | 2.07 | 1.63–2.53 |
| Sim + SLAM + Nav2 | 2 | 2.01 | 1.52–2.45 |
| Full stack (+ agent) | 2 | 1.89 | 1.28–2.29 |
| Sim only — numactl node 1 | 3 | 2.91 | 2.36–3.68 |
| Sim only — numactl node 1 | 4 | 3.06 | 2.78–3.41 |

**RTF ceiling: ~3× real-time** — the sim tops out at ~3.0 regardless of speed setting (speed=3 and speed=4 give near-identical RTF). SLAM+Nav2 and sensor subscriptions add negligible overhead. Full stack at speed=3 is impractical: DDS peer discovery takes ~2 wall-minutes = 6 sim-minutes wasted waiting for `/map`, leaving too little of the 900 sim-second budget. **Speed=2 is the recommended setting** (1.89 RTF, stable, full budget available).

**FastDDS note**: with discovery server active, speed=3 may now be viable (map arrives in <30s instead of ~4 sim-min). Test after a few speed=2 baselines establish the new baseline score.

---

## Questions for Tjalling (2026-03-21)

1. **exit_sign detection**: OWLv2 gives near-zero confidence on exit signs in this sim (tiny dark rectangle high on walls). 4 out of 9 targets in easy are exit signs. Options: (a) OCR/template match on "EXIT" text in RGB image, (b) train/fine-tune a small detector, (c) treat exit signs as out-of-scope and target B-grade on remaining 5 objects. Recommendation: (a) — EasyOCR or PaddleOCR on cropped top-of-frame region, no GPU needed. **Decision needed before Task 5.**

2. **W_DIST=1.5 resolved**: 98% coverage confirmed. Coverage is solved. Primary bottleneck is now physical camera coverage of the upper map section (y>7.5). Patrol mode deployed (run in progress).

3. **Task ordering after Task 4**: easy scenario coverage now at 98% but detection is 3/6. Patrol mode should find remaining objects (y>7.5 area). After patrol run confirms 5-6/6 found, assess whether to tackle medium scenario or stay on easy. Recommendation: verify easy=6/6 first.

---

## Development guidelines

Test new features before marking complete. Prefer Python unit tests where possible.
