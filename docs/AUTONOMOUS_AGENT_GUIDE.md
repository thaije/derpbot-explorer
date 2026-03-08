# Autonomous Agent Guide — ARST

Gazebo Harmonic + ROS 2 Jazzy simulation testbed. Your task: deploy a fully autonomous agent and complete the scenario.

---

## 1. Task

You will be placed in an unknown indoor environment. Before each run you receive a **mission description** with the mission goal listing for example the target object types to locate. Your agent must explore the environment, detect all instances of every target type, and report detections — all within a **time limit**.

Scenarios come in multiple **difficulty tiers**. The environment may challenge your agent in various ways — expect conditions to differ between tiers and runs. Object placement and spawn position are **randomised each run** unless a seed is fixed. Doors are passable or permanently locked; no unlocking or interaction is required.

> **Design advice:** Build a general-purpose agent. Tiers, object types, and environment conditions may be extended or changed at any time. Agents that adapt to what they observe will hold up far better than those tuned to specific scenarios.

**Success** = all required instances detected before the time limit. Thorough exploration of the full environment earns bonus points regardless of outcome.

---

## 2. Grading

Runs are scored 0–100 across five categories, combined as a weighted sum.

| Category | What it measures |
|---|---|
| **Speed** | Task completion time relative to par (par → B grade) |
| **Accuracy** | `found_ratio` (recall, 0.60) + `precision` (0.40) |
| **Safety** | Collisions and near-misses |
| **Efficiency** | Coverage-per-metre (0.60) + path length vs par (0.40) |
| **Effectiveness** | Detection completeness (0.65) + exploration coverage % (0.35) |

Grade thresholds: **S** ≥ 95 · **A** ≥ 85 · **B** ≥ 70 · **C** ≥ 55 · **D** ≥ 40 · **F** < 40.
A B-grade means *competent performance for that difficulty level*.

Full score breakdown is written to `results/` as JSON after each run.

---

## 3. Platform — DerpBot

Ground robot (differential drive — currently the only supported model).

| Property | Value |
|---|---|
| Body | 30 × 20 × 10 cm |
| Max linear speed | 0.5 m/s |
| Max angular speed | 2.0 rad/s |
| Drive command | `geometry_msgs/Twist` |

### Sensors

| Sensor | Topic | Rate | Details |
|---|---|---|---|
| 2-D LiDAR | `/derpbot_0/scan` | ~10 Hz | 360°, 720 rays, 0.5° resolution, 0.15–12 m range, `sensor_msgs/LaserScan` |
| IMU | `/derpbot_0/imu` | 100 Hz | `sensor_msgs/Imu`, **BEST_EFFORT QoS** |
| RGBD — RGB | `/derpbot_0/rgbd/image` | 10 Hz | 640 × 480, 90° H-FOV, forward-facing, `sensor_msgs/Image` |
| RGBD — depth | `/derpbot_0/rgbd/depth_image` | 10 Hz | float32 metres, 0.15–6.0 m range, Gaussian noise σ=0.01 m, `sensor_msgs/Image` |
| RGBD — intrinsics | `/derpbot_0/rgbd/camera_info` | 10 Hz | `sensor_msgs/CameraInfo` — required for 3-D back-projection |
| RGBD — point cloud | `/derpbot_0/rgbd/points` | 10 Hz | `sensor_msgs/PointCloud2` — **off by default**, enable with `--enable-pointcloud` |
| Odometry | `/derpbot_0/odom` | — | `nav_msgs/Odometry`, wheel-encoder dead-reckoning |

### Control & TF

| Topic/Frame | Details |
|---|---|
| `/derpbot_0/cmd_vel` | `geometry_msgs/Twist` — drive command |
| `/derpbot_0/joint_states` | `sensor_msgs/JointState` |
| TF tree | `odom → base_footprint → base_link` |

### Detection output

Publish your detections on `/derpbot_0/detections` as `vision_msgs/Detection2DArray`. The scoring system validates each detection against the ground truth.

**Required fields per `Detection2D`:**

| Field | Content |
|---|---|
| `results[0].hypothesis.class_id` | Object type string, e.g. `"fire_extinguisher"` |
| `id` | Persistent per-instance tracking ID (e.g. `"track_23"`). Use a stable ID for the same physical object across frames. |
| `results[0].pose.pose.position.{x,y}` | Estimated world position in metres (from robot pose + depth estimate) |

**Validation logic:** a detection is a **true positive** if the class matches a known target type, the claimed position is within 1.5 m of a real object of that type, and there is line of sight. Detections of unknown classes are silently ignored (no penalty). False positives, duplicate positives and localization error reduce your score.

**Safety threshold:** a near-miss is any obstacle within **≤ 0.3 m** of the robot (as measured by LiDAR). Collisions and near-misses both reduce the Safety score. The robot body is 30 × 20 cm (LiDAR-to-surface offset ≈ 0.15 m front/back).

**Note:** a ground-truth oracle can be enabled with `--enable-oracle` (see §4). When active, the simulator bridges its bbox camera directly to `/derpbot_0/detections`, replacing the need for your own vision pipeline. **Off by default** — for scored runs you must publish your own detections using `/derpbot_0/rgbd` and `/derpbot_0/rgbd/depth_image`.

---

## 4. Running a scenario

**Requirements:** ROS 2 Jazzy, Gazebo Harmonic, Python 3.12, a sourced workspace.

```bash
# Single run, headless, random seed
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless

# Pin layout for reproducibility
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless --seed 42

# Shorten timeout for iteration
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --timeout 300

# Enable oracle detections (dev/cheat — bbox camera feeds /detections directly)
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --enable-oracle

# Enable RGBD point cloud bridge (high bandwidth — off by default)
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless --enable-pointcloud
```

Available difficulty tiers (ascending): `easy` · `medium` · `hard` · `brutal` · `perception_stress`
All YAML files are under `config/scenarios/office_explore_detect/`.

Startup takes ~5 s. The scenario ends on `SUCCESS` or `TIME_LIMIT`; scorecard prints to stdout and JSON is written to `results/`.

### Mission description endpoint

Once the scenario is running, fetch the mission description from:

```
GET http://localhost:7400/mission
```

The response is JSON with both a **human-readable** `description` field and **structured** `targets` for rule-based consumption:

```json
{
  "scenario": "office_medium_001",
  "goal": "Explore the environment and locate all required target objects. Report detections before the time limit expires. Thorough exploration earns bonus points.",
  "time_limit_seconds": 600,
  "targets": [
    {"type": "fire_extinguisher", "count": 3,   "count_exact": true},
    {"type": "first_aid_kit",     "count": 2,   "count_exact": true},
    {"type": "hazard_sign",       "count_exact": false, "min_count": 1}
  ],
  "status": "running",
  "description": "Explore the environment and locate all target objects within 600s. Targets: exactly 3 fire extinguisher(s), exactly 2 first aid kit(s), at least 1 hazard sign(s) (exact count unknown). Status: running."
}
```

`count_exact: false` means the scenario intentionally withholds the exact count — treat `min_count` as a lower bound. `status` transitions to `"completed"` with an `"outcome"` field (`SUCCESS` / `TIME_LIMIT`) after the run ends; the server stays up until the process exits.

A mission brief is also printed to stdout at scenario start and again just before the scorecard.

### ROS 2 node requirements

- Create your node with `use_sim_time=True` or sensor messages will be silently dropped.
- Subscribe to IMU with `ReliabilityPolicy.BEST_EFFORT`.
- Isolated parallel runs: prefix with `ROS_DOMAIN_ID=N`.

---

## 5. What your agent must do

1. **Receive the mission** — fetch the mission description (see section 4) before or at run start.
2. **Explore** the environment — no map is provided; build it from LiDAR and odometry.
3. **Detect objects** — publish `vision_msgs/Detection2DArray` on `/derpbot_0/detections` with class, tracking ID, and world position (see §3).
4. **Navigate safely** — collisions and near-misses reduce your Safety score.
5. **Stop or let the runner end the episode** — the runner polls detections and terminates automatically.

---

## 6. Development tips

- Start with `easy.yaml` for a more forgiving baseline.
- Use `--seed N` to freeze the layout while iterating on your algorithm.
- `scripts/robot_control.py` — manual drive / camera snapshot (dev/cheat tool).
- `scripts/world_state.py` — PNG map + object ground-truth positions (dev/cheat tool).
- Reference docs: [`docs/AGENT_HANDOFF.md`](AGENT_HANDOFF.md) (architecture, gotchas).
