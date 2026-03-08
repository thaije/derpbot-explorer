# DerpBot Autonomous Agent — Implementation Plan

## Overview

A ROS 2 (Jazzy) Python node that autonomously explores an unknown indoor environment, builds a map, detects target objects via camera, and reports detections — all within a time limit. Built incrementally across 5 phases, each producing a runnable agent.

**Single node architecture.** Everything runs in one Python node (`agent_node.py`) to minimize complexity. Internal modules handle mapping, planning, control, and detection.

---

## Sensors

DerpBot's full sensor suite (from AUTONOMOUS_AGENT_GUIDE.md) plus the added RGBD camera:

| Sensor | Topic | Rate | Details |
|---|---|---|---|
| 2-D LiDAR | `/derpbot_0/scan` | ~10 Hz | 360°, 720 rays, 0.5° res, 0.15–12 m, `sensor_msgs/LaserScan` |
| IMU | `/derpbot_0/imu` | 100 Hz | `sensor_msgs/Imu`, **BEST_EFFORT QoS** |
| RGBD camera (RGB) | `/derpbot_0/rgbd/image` | 10 Hz | 640×480, 90° H FOV, forward-facing, `sensor_msgs/Image` |
| RGBD camera (depth) | `/derpbot_0/rgbd/depth_image` | 10 Hz | 640×480, 90° H FOV, 0.3–10 m range (reliable ≤3 m, noisy beyond), `sensor_msgs/Image` 32FC1 (metres) |
| RGBD camera (pointcloud) | `/derpbot_0/rgbd/points` | 10 Hz | Optional. `sensor_msgs/PointCloud2` |
| Odometry | `/derpbot_0/odom` | — | `nav_msgs/Odometry`, wheel-encoder dead-reckoning |

**Compute:** 2× Nvidia GPUs, 8 GB VRAM each. Available for ML inference (object detection).

---

## Requirements (MoSCoW)

### Must Have
- Obstacle avoidance using LiDAR (no collisions)
- Occupancy grid mapping from LiDAR + odometry
- Systematic exploration (frontier-based) to maximize coverage
- Object detection from RGBD camera using open-vocabulary model (GPU-accelerated)
- Object world-position estimation from depth channel + robot pose
- Detection publishing (`vision_msgs/Detection2DArray`) with class, tracking ID, world position
- Mission endpoint fetch and target-aware behaviour
- `use_sim_time=True`, BEST_EFFORT QoS for IMU

### Should Have
- Persistent tracking IDs (same object across frames → same ID)
- Smooth velocity control (acceleration limiting) for safety score
- Re-observation: revisit uncertain detections to improve confidence
- Frontier prioritisation (prefer large, nearby frontiers)
- Multi-sighting position fusion (average world position across observations)

### Could Have
- SLAM (e.g. scan matching) to correct odometry drift
- Multi-pass detection fusion (average position across sightings)
- Adaptive speed (slow near obstacles, fast in open space)
- Door detection and passage planning

### Won't Have (this iteration)
- Multi-robot coordination
- 3D mapping / multi-floor navigation
- Manipulation or interaction with objects
- Custom-trained or fine-tuned ML models (use pre-trained open-vocabulary models)

---

## Architecture

```
agent_node.py
├── MissionClient         — GET /mission, parse targets
├── OccupancyGrid         — 2D grid from LiDAR scans + robot pose
├── FrontierPlanner       — Find frontier cells, select next goal
├── Navigator             — Path planning (A*) + path following
├── ObstacleAvoider       — Reactive safety layer on cmd_vel
├── Detector              — RGB frame → open-vocabulary object detections (GPU)
├── Tracker               — Merge detections, assign persistent IDs, depth→world projection
└── Main loop             — State machine: INIT → EXPLORE → NAVIGATE → DETECT → DONE
```

### Data flow
1. LiDAR + odom → OccupancyGrid (updated every scan)
2. FrontierPlanner queries grid → selects goal
3. Navigator plans path on grid → produces cmd_vel
4. ObstacleAvoider wraps cmd_vel → publishes to `/derpbot_0/cmd_vel`
5. RGB + depth frames (synced) → Detector → Tracker → publishes to `/derpbot_0/detections`
6. Depth at detection bounding box centre + robot pose → world position estimate

---

## Phases

### Phase 1 — Drive and Survive

**Goal:** Robot moves without crashing. Validates sensor subscriptions and control.

**Approach:**
- Subscribe to `/derpbot_0/scan`, `/derpbot_0/odom`, `/derpbot_0/imu` (BEST_EFFORT), `/derpbot_0/rgbd/image`, `/derpbot_0/rgbd/depth_image`
- Implement reactive obstacle avoidance: read LiDAR, compute a "safe" twist
- Algorithm: Virtual Force Field (VFF) — repulsive vectors from nearby obstacles, attractive vector forward. Simple, no map needed.
- Publish `Twist` to `/derpbot_0/cmd_vel`
- Wander: default forward motion with random turns when blocked

**Output:** Robot drives continuously without collision. No mapping, no detection.

**Test:** Run `easy.yaml --seed 42 --timeout 120`. Robot should survive with zero collisions. Check Safety score.

---

### Phase 2 — Mapping

**Goal:** Build an occupancy grid from LiDAR.

**Approach:**
- Maintain a 2D numpy array (e.g. 20m × 20m at 5 cm resolution = 400×400 cells)
- On each LiDAR scan: ray-cast from robot pose, mark free cells along ray, occupied at endpoint
- Use log-odds update for probabilistic occupancy (standard approach, avoids flip-flopping)
- Robot pose from `/derpbot_0/odom` (sufficient for easy/medium; drift is a known risk for hard+)
- Coordinate frame: grid origin = robot start position

**Output:** Live occupancy grid that can be queried and optionally saved as PNG for debugging.

**Test:** Run `easy.yaml --seed 42`. After run, dump grid as image. Visually verify it matches the environment layout. Compare against `world_state.py` PNG if available.

---

### Phase 3 — Exploration

**Goal:** Systematic coverage of the full environment using frontier-based exploration.

**Approach:**
- **Frontier detection:** Scan occupancy grid for free cells adjacent to unknown cells. Cluster adjacent frontier cells into frontier regions.
- **Frontier selection:** Score each frontier by: size (prefer large openings), distance from robot (prefer nearby), direction (prefer continuing forward). Weighted combination.
- **Path planning:** A* on the occupancy grid from robot cell to selected frontier centroid. Inflate obstacles by robot radius (~20 cm) before planning to avoid tight squeezes.
- **Path following:** Pure pursuit or simple waypoint following — head toward the next waypoint on the path, skip waypoints within a threshold distance.
- **Stuck detection:** If robot hasn't moved >10 cm in 5 seconds, mark current frontier as unreachable, pick next.

**State machine:**
```
EXPLORE: find frontiers → select best → plan path → NAVIGATE
NAVIGATE: follow path → if reached goal → EXPLORE
                       → if stuck → blacklist frontier → EXPLORE
                       → if no frontiers remain → DONE
```

**Output:** Robot systematically covers the environment. Coverage % and Efficiency scores should be meaningful now.

**Test:** Run `easy.yaml --seed 42`. Measure coverage % from scorecard. Target: >80% coverage. Visualise path overlaid on grid.

---

### Phase 4 — Detection (Oracle First)

**Goal:** Wire up detection publishing using the ground-truth oracle. Validates the full scoring pipeline.

**Approach:**
- Subscribe to `/derpbot_0/detections` (oracle) — this gives ground-truth `Detection2DArray`
- Re-publish with proper formatting: class_id, tracking ID, world position
- Fetch mission from `http://localhost:7400/mission` at startup, parse `targets`
- Only publish detections for target types listed in mission
- Assign stable tracking IDs: maintain a dict of known objects keyed by (class, approximate position). If a new detection is within 1.0m of an existing tracked object of the same class, reuse the ID.

**Output:** Full end-to-end run with scoring. Should achieve passing grades on easy.

**Test:** Run `easy.yaml` with 3+ different seeds. Target: SUCCESS outcome on all. Review full JSON scorecard — all categories should be D or above, most B+.

---

### Phase 5 — Vision Pipeline

**Goal:** Replace oracle with real camera-based detection using RGBD + GPU.

**Approach:**
- Subscribe to `/derpbot_0/rgbd/image` and `/derpbot_0/rgbd/depth_image` (both 10 Hz, `sensor_msgs/Image`). Synchronise using `message_filters.ApproximateTimeSynchronizer`.
- **Object detection model — open-vocabulary, GPU-accelerated:**
  - Primary: **YOLO-World** (ultralytics) or **GroundingDINO** — text-prompted object detection. Feed target type strings directly from the mission endpoint (e.g. `"fire extinguisher"`, `"hazard sign"`) as text prompts. No retraining needed; any new target type the mission specifies is automatically supported.
  - Fallback: **YOLOv8n** with COCO classes — covers common objects like fire extinguishers. Less flexible but simpler to deploy.
  - Runs on one GPU. Model loaded once at startup. Inference at ~5 Hz (process every 2nd frame) to keep latency manageable.
- **World position from depth:** For each detection bounding box, sample the depth image at the box centre (median of a small patch to reject noise). Combined with pixel coordinates, camera intrinsics (computed from 90° FOV + 640×480 resolution), and robot pose from odom, project to world XY coordinates. Reject detections where depth is NaN, < 0.3 m, or > 3 m (unreliable range).
- **Confidence filtering:** Only accept detections above a confidence threshold (start at 0.3, tune per tier). Require ≥2 sightings from different robot poses before publishing to reduce false positives.
- Tracker from Phase 4 continues to handle ID assignment and deduplication.

**Output:** Fully autonomous agent with no oracle dependency. General-purpose: adapts to any target type via mission endpoint.

**Test:** Run easy and medium with multiple seeds. Compare accuracy scores vs Phase 4 oracle baseline. Target: precision >0.7, recall >0.6 on easy.

---

## Testing Strategy

### Per-phase validation (run after each phase)
| Phase | Test | Pass criteria |
|---|---|---|
| 1 | `easy.yaml --seed 42 --timeout 120` | 0 collisions, robot continuously moving |
| 2 | Above + dump grid PNG | Grid resembles room layout |
| 3 | `easy.yaml --seed 42` | Coverage >80%, no stuck loops |
| 4 | `easy.yaml` × 3 seeds | SUCCESS on all, all scores ≥ D |
| 5 | `easy.yaml` + `medium.yaml` × 3 seeds | SUCCESS on easy, precision >0.7 |

### Regression testing
After each phase, re-run Phase 1 collision test to ensure safety hasn't regressed.

### Difficulty progression
Only move to the next difficulty tier after consistently passing the current one:
- `easy` → Phase 4 complete
- `medium` → Phase 5 complete and tuned
- `hard` → After adding odometry drift correction
- `brutal` / `perception_stress` → Stretch goals

### Debug tooling
- Grid visualisation: save occupancy grid as PNG each run (`debug/grid_SEED.png`)
- Path visualisation: overlay planned path on grid
- Detection log: CSV of all detections with timestamp, class, position, confidence
- Use `scripts/world_state.py` to compare against ground truth during development

---

## Key Technical Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Language | Python | ROS 2 Jazzy native support, fast iteration, OpenCV bindings |
| Mapping | Custom occupancy grid (numpy) | Simple, full control, no external SLAM dependency |
| Exploration | Frontier-based | Well-proven for unknown indoor environments |
| Path planning | A* on inflated grid | Simple, optimal on grid, fast enough for this scale |
| Detection | Open-vocabulary model (YOLO-World / GroundingDINO) on GPU | General-purpose: any target type via text prompt, no retraining. GPU available. |
| Depth sensing | RGBD camera (hardware depth) | Direct depth per pixel, no monocular estimation needed. Reliable ≤3 m. |
| Localisation | Odometry only (Phase 1–5) | Sufficient for easy/medium. SLAM is a could-have. |
| Node structure | Single node, modular classes | Simpler lifecycle, easier debugging |

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| Odometry drift on longer runs | Detections map to wrong world position | Keep paths short via nearest-frontier selection. Add scan matching later. |
| Open-vocab model fails on unusual target names | Low recall for unknown object types | Fall back to YOLOv8 COCO classes. Tune text prompts. Test with varied target types. |
| Depth noise beyond 3 m | Inaccurate world position estimates | Reject detections with depth > 3 m. Only publish when object is within reliable range. |
| Robot gets stuck in narrow doorways | Exploration stalls, timeout | Inflate obstacles conservatively. Add stuck detection + blacklisting. |
| False positive detections | Precision penalty | Require ≥2 sightings from different poses before publishing. Confidence threshold. |
| GPU model loading time | Delayed start | Load model during INIT phase while fetching mission. Budget ~10 s startup. |
| LiDAR misses glass/thin objects | Collision | Use depth camera as secondary obstacle cue. Keep speed low near unknowns. |

---

## File Structure

```
agent/
├── agent_node.py          — Entry point, state machine, ROS2 node
├── mission_client.py      — HTTP fetch + parse mission JSON
├── occupancy_grid.py      — Grid creation, ray-casting, log-odds update
├── frontier_planner.py    — Frontier detection, clustering, selection
├── navigator.py           — A* path planning, path following
├── obstacle_avoider.py    — Reactive safety layer
├── detector.py            — Open-vocab GPU model, RGB frame → bounding boxes + classes
├── depth_projector.py     — Depth image + bbox + robot pose → world XY position
├── tracker.py             — Detection merging, persistent IDs, multi-sighting fusion
└── utils.py               — Coordinate transforms, angle math, camera intrinsics
```

---

## Implementation Notes for Developer

- All ROS2 subscribers must use `use_sim_time=True` parameter on the node
- IMU subscriber needs `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)`
- `cmd_vel` max values: linear.x ∈ [-0.5, 0.5], angular.z ∈ [-2.0, 2.0]
- Detection `id` field must be a string (e.g. `"track_1"`)
- Detection position is in world frame (odom frame), not robot frame
- Grid resolution of 5 cm balances detail vs computation
- Run with: `ros2 run agent_pkg agent_node` or `python3 agent_node.py`
- The `AGENT_HANDOFF.md` doc (referenced in guide) may contain additional architecture notes — read it if available in the sim environment
