# DerpBot — Approach 1: Classical Modular Pipeline

> **Current implementation plan.** Supersedes `simple_autonomy.md` (deleted).
> Full architecture survey: [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)

---

## Architecture

```
slam_toolbox (async online)
  scan: /derpbot_0/scan  →  /map (OccupancyGrid, 5cm res)

Nav2 stack
  global planner:  SmacPlanner2D (A*, obstacle-inflated, allow_unknown=true)
  local controller: MPPI (DiffDrive, 100 Hz)
  recovery:        Spin → BackUp → Wait (RoundRobin, max 6 retries)
  costmap:         track_unknown_space=true, robot_radius=0.22 m

agent/
  mission_client.py    — GET http://localhost:7400/mission → targets[]
  frontier_explorer.py — BFS /map frontiers → NavigateToPose goals
  detector.py          — YOLOE26-S on GPU 0 (~1-2 GB VRAM, 5 Hz)
  depth_projector.py   — bbox + depth + TF2 → world (x,y)
  tracker.py           — multi-sighting fusion, ByteTrack IDs, publishes /derpbot_0/detections
  agent_node.py        — INIT → EXPLORE → DONE state machine

config/
  slam_toolbox_params.yaml
  derpbot_nav2_params.yaml

launch/
  derpbot_autonomy.launch.py
```

---

## Running

```bash
# With launch file (recommended — starts slam_toolbox + Nav2 + agent)
ros2 launch launch/derpbot_autonomy.launch.py

# Components separately for debugging:
ros2 launch slam_toolbox online_async_launch.py \
    params_file:=config/slam_toolbox_params.yaml use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py \
    params_file:=config/derpbot_nav2_params.yaml use_sim_time:=true

python3 agent/agent_node.py
```

---

## Component Notes

### slam_toolbox
- Mode: **async** — drops scans gracefully under CPU load (never falls behind)
- Publishes `/map` + TF `map → odom`
- Map frame: `map`; base frame: `base_link`
- Scan topic remapped to `/derpbot_0/scan`

### Nav2
- `cmd_vel` output: via `collision_monitor` → `/derpbot_0/cmd_vel`
- Odom topic: `/derpbot_0/odom` (set in params)
- `allow_unknown: true` in SmacPlanner2D — robot can plan into unexplored space
- `track_unknown_space: true` in global costmap — unknown ≠ free

### FrontierExplorer
- BFS on raw `/map` data; frontier = free cell (0) adjacent to unknown cell (-1)
- Clusters frontier cells; scores by `W_SIZE * size - W_DIST * dist`
- Blacklists failed/stuck goals (radius 0.5 m); stuck = <10 cm movement in 8 s
- Calls `NavigateToPose` action server directly

### Detector
- YOLOE26-S via `ultralytics`; auto-falls back to `yolov8s-worldv2.pt`
- `model.set_classes(targets)` — mission targets are text prompts
- Processes every 2nd frame at 10 Hz → ~5 Hz effective
- Confidence threshold: 0.30

### DepthProjector
- Camera intrinsics: `fx = fy ≈ 320`, `cx=320`, `cy=240` (90° HFOV, 640×480)
- Samples 5×5 median patch at bbox centre; rejects NaN / <0.3 m / >3.0 m
- TF2 lookup: `camera_link → map`
- **TODO:** confirm `camera_link` TF frame name from sim (`ros2 run tf2_tools view_frames`)

### Tracker
- Match radius: 1.0 m (same class + world pos)
- Publishes only after ≥ 2 sightings from robot poses ≥ 0.5 m apart
- Detection format: `results[0].hypothesis.class_id`, `id="track_N"`, world pos in `results[0].pose.pose.position.x/y`

---

## Verification Checklist

| Step | Command | Pass criteria |
|------|---------|---------------|
| 1. slam_toolbox | `ros2 topic echo /map --once` | OccupancyGrid publishes |
| 2. Nav2 | `ros2 action send_goal /navigate_to_pose …` | Robot drives to goal |
| 3. Frontiers | watch `/map` while agent runs | Coverage grows, robot moves |
| 4. Detector | `ros2 topic echo /derpbot_0/detections` | class_id, track ID present |
| 5. End-to-end | `easy.yaml --seed 42 --timeout 300` | ≥80% coverage, 0 collisions |

---

## Known Issues / TODOs

- `camera_link` TF frame name must be confirmed from sim environment
- YOLOE26 model name may vary across ultralytics versions — check `ultralytics` changelog if load fails
- Nav2 `collision_monitor` remaps `cmd_vel_smoothed → /derpbot_0/cmd_vel`; confirm topic names match sim
- For `medium`/`hard` tiers: consider adding SLAM loop-closure tuning and frontier prioritisation toward detected objects

---

## Dependency Install

```bash
# ROS 2 Jazzy packages
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-mppi-controller ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-vision-msgs

# Python
pip install ultralytics  # YOLOE26 / YOLO-World
```
