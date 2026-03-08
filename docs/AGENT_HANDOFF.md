# Agent Handoff — autonomous derpbot explorer

Plan for current Nav2 ROS2 approach: [`docs/approach1_classical_pipeline.md`](approach1_classical_pipeline.md)
Architecture survey (all 5 approaches): [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)  
Task + robot spec: [`docs/AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md)

---

## What works (verified)

- **slam_toolbox**: publishing `/map` (TRANSIENT_LOCAL, 5 cm res, `base_footprint` frame)
- **Nav2 stack**: all nodes active — SmacPlanner2D + MPPI controller + recovery behaviours + collision_monitor
- **Mission client**: fetches targets from `http://localhost:7400/mission`; handles dict targets `{"type": "..."}` correctly
- **Detector**: YOLOE26-S loaded on GPU 0 (~2 s from cache), running at ~5 Hz on RGB stream
- **FrontierExplorer**: BFS frontiers detected on `/map`, goals sent to Nav2, robot navigating
- **cmd_vel pipeline**: controller_server → velocity_smoother → collision_monitor → `/derpbot_0/cmd_vel` ✓

Not yet verified: depth projection, tracker publishing detections, end-to-end scenario score.

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
