# Agent Handoff — autonomous derpbot explorer

Plan for current Nav2 ROS2 approach: [`docs/approach1_classical_pipeline.md`](approach1_classical_pipeline.md)
Architecture survey (all 5 approaches): [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)  
Task + robot spec: [`docs/AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md)

---

## What works (verified)

- **slam_toolbox**: publishing `/map` (TRANSIENT_LOCAL, 5 cm res, `base_footprint` frame)
- **Nav2 stack**: all nodes active — SmacPlanner2D + MPPI controller + recovery behaviours + collision_monitor
- **Mission client**: fetches targets from `http://localhost:7400/mission`; handles dict targets `{"type": "..."}` correctly
- **Detector**: Two-stage pipeline — YOLOE-11s region proposals (broad class set, conf=0.01) → CLIP ViT-B/32 re-classification. Runs in subprocess (spawn context). Verified detecting fire extinguisher + first aid kit on live sim frames. Root issue: YOLOE text embeddings don't match low-poly sim meshes; CLIP reranking fixes this generically without per-class prompt tuning. CLIP sim threshold=0.20.
- **FrontierExplorer**: BFS frontiers detected on `/map`, goals sent to Nav2, robot navigating. 93-97% coverage per run.
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
- **Nav2 stuck detection threshold**: 8 s too short (Nav2 rotates in place first). Set to 30 s.
- **Goal acceptance future can take 60+ s** after a goal cancel — bt_navigator finishes its current BT tick before processing the preemption. Acceptance timeout set to 90 s; returns `None` (skip, no blacklist) on timeout.
- **`/map` TRANSIENT_LOCAL delivery takes ~2 min** after agent start — DDS peer discovery is slow when many entities exist. Normal; agent polls until map arrives.
- **Frontier goal point must be closest-to-centroid cell** — centroid can land in unknown/inflated space. "Closest-to-robot" causes immediate Nav2 success (robot already within tolerance) with no map update. "Closest-to-centroid" picks a free cell in the interior of the frontier, requiring actual navigation.
- **Blacklist frontier centroids on success too** — with `xy_goal_tolerance=0.5 m`, Nav2 succeeds when robot is within 0.5 m. If the frontier cells still exist (unknown area behind a wall), the same cluster is re-selected every loop. Blacklisting on success prevents this. Legitimate frontiers disappear naturally once the map updates from the robot's new scans.
- **SLAM smearing near furniture** — raise `link_match_minimum_response_fine` (now 0.35) to reject weak scan matches, and `minimum_travel_distance` (now 0.2 m) to skip map updates on jitter. Loop closure (`do_loop_closing: true`) corrects accumulated drift when the robot revisits known areas.
- **Always restart slam_toolbox between scenario runs** — it keeps its old map after a scenario ends. If reused, FrontierExplorer sees an already-explored map with no frontiers and blacklists all goals within ~2 min.
- **PyTorch/CUDA deadlocks in ROS2 executor threads** — calling `model.predict()` from any ROS2 executor callback or executor-managed thread causes an indefinite hang due to Python GIL starvation (8 executor threads starve inference thread; `time.sleep(0.01)` took ~10 s). Fix: run inference in a separate subprocess (`multiprocessing`, spawn context). Never call YOLOE/PyTorch from executor threads.
- **Nav2 lifecycle DDS timeout at startup** — on first launch, `smoother_server` occasionally times out during lifecycle transition with "failed to send response to /smoother_server/change_state". Kill Nav2 and restart; second launch succeeds.
- **inflation_radius 0.35 m blocks narrow corridors** — the office has corridors < 0.70 m wide; inflation ≥ 0.35 m makes them impassable and all goals get ABORTED. Keep at 0.25 m.
- **CLIP threshold must stay at 0.20** — raising it (e.g. to 0.28) eliminates fire extinguisher detections entirely. The red cylinder mesh in sim scores barely above 0.20 against "fire extinguisher"; the threshold is already at the floor. Prompts are `t.replace("_", " ")` (e.g. "fire extinguisher", "first aid kit", "hazard sign") — already optimized for low-poly sim meshes; more specific prompts were tested and did not improve recall.
- **Frontier W_DIST must be high (≥ 4.0)** — at W_DIST=0.5 the robot picks the largest cluster regardless of distance, causing cross-map thrashing. This wastes time and means the robot visits each area only once, preventing tracker from getting the 2 diverse sightings needed to confirm objects. W_DIST=4.0 keeps the robot local.

---

## Architecture

**Key design decisions:**
- slam_toolbox async mode — drops scans under load, never blocks
- Nav2 SmacPlanner2D + MPPI — `allow_unknown: true` so robot plans into unexplored space
- `/map` subscriber uses TRANSIENT_LOCAL QoS — gets map immediately on subscribe
- FrontierExplorer polls goal future (not `spin_until_future_complete`) — node already spins in MultiThreadedExecutor
- Detector uses `model.set_classes(targets)` — mission targets are text prompts, no retraining
- Tracker requires ≥2 sightings from diverse poses (>0.5 m apart) before publishing — reduces false positives

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
  detector.py          — YOLOE26-S on GPU 0 (~1-2 GB VRAM, 5 Hz)
  depth_projector.py   — bbox centre + depth image + TF2 → world (x, y)
  tracker.py           — multi-sighting fusion, persistent IDs, publishes /derpbot_0/detections
  agent_node.py        — INIT → EXPLORE → DONE state machine, MultiThreadedExecutor

config/
  slam_toolbox_params.yaml   — base_frame: base_footprint, scan_topic: /derpbot_0/scan
  derpbot_nav2_params.yaml   — robot_radius: 0.22, inflation_radius: 0.25
```

## TF frames (confirmed)
`map → odom → base_footprint → base_link → camera_link / lidar_link`

## How to run (debug, components separately)

```bash
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml use_sim_time:=true

ros2 launch $(pwd)/launch/navigation_launch.py \
    params_file:=$(pwd)/config/derpbot_nav2_params.yaml use_sim_time:=true

cd agent && python3 agent_node.py
```

## Development guidelines

Test new features before marking complete. Prefer Python unit tests where possible.
