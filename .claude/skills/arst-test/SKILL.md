---
name: arst-test
description: Test and evaluate an autonomous robot running an ARST scenario. Use when you are the observer/tester — starting the sim, watching a robot agent run, monitoring ground-truth progress, and reading final scores. NOT for driving the robot yourself (use /arst-nav for that, if available).
user_invocable: true
---

# arst-test — ARST Autonomous Robot Test Skill

You are the **observer/tester**, not the driver. A separate autonomy stack controls DerpBot.
Your job: start the sim, start the agent, monitor from the outside, and interpret results.

All paths: `~/Projects/derpbot-explorer` (explorer root), `~/Projects/robot-sandbox` (sandbox root).
Always use `python3.12`.

---

## Environment setup (REQUIRED before any ROS2 command)

The stack uses a FastDDS discovery server. All ROS2 CLI commands and monitoring scripts
**must** have these env vars set, or they will see no topics/nodes:

```bash
. ~/Projects/derpbot-explorer/scripts/ros_env.sh
```

This sets `ROS_DISCOVERY_SERVER=127.0.0.1:11811`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
and `ROS_SUPER_CLIENT=1`. Source it at the start of each bash block that uses ros2 or
calls rtf_monitor.py / world_state.py / robot_control.py.

---

## Scenario tiers

See the AUTONOMOUS_AGENT_GUIDE.md file.

---

## 1. Start the stack

**Use `start_stack.sh`** (from the derpbot-explorer repo). It kills stale processes, restarts the
ROS2 daemon, and starts sim → SLAM → Nav2 → agent in the correct order within 5s of sim ready.

```bash
cd ~/Projects/derpbot-explorer

# Full stack (sim + SLAM + Nav2 + agent)
./scripts/start_stack.sh --speed 2 --seed 42 --scenario easy

# Without agent (benchmarking)
./scripts/start_stack.sh --speed 2 --seed 42 --no-agent

# Nav-only mode — agent runs without OWLv2 detector/tracker (found_ratio=0; isolates Nav2)
./scripts/start_stack.sh --speed 2 --seed 42 --no-perception

# Options: --speed N (default 2), --seed N (default 42), --scenario TIER (default easy),
#          --no-agent, --no-perception
# Scenario tiers: easy, medium, hard, brutal, perception_stress
```

Creates tmux sessions: `sim`, `slam`, `nav2`, `agent`. Wait for `=== Stack launched ===`.

---

## 2. Check RTF immediately after launch

```bash
. ~/Projects/derpbot-explorer/scripts/ros_env.sh
cd ~/Projects/robot-sandbox && python3.12 scripts/rtf_monitor.py --once
```

Run this 3 times over ~30s. Expected: ~1.9 at speed=2.

**Abort if RTF < 1.0 sustained** — autonomy is too heavy. Kill stack, report to main agent.

---

## 3. Check for TF flood (first 60s wall-time after launch)

```bash
tmux capture-pane -t agent -p -S -50
```

**TF flood symptom**: agent logs show Nav2 goals immediately aborting (status 6), robot never moves,
and Nav2/SLAM logs flooded with `TF_OLD_DATA` or timestamp warnings.

**If detected**: kill stack (`./scripts/start_stack.sh` will clean up on next call), report to main agent.
Do NOT retry more than once — this is a known hard failure, not a transient issue.

---

## 4. Monitor ground-truth progress

Primary observability tool — call every 30s wall-time until scenario ends.

```bash
# Default: writes arst_world_map.png in repo root, prints full status
python3.12 scripts/world_state.py
# Then: Read arst_world_map.png
```

**Output includes:**
- Object list with status: `not found`, `FOUND ✓`, or `not found  [visible]`
- Robot pose (world frame, yaw, facing direction)
- Collision status: `COLLISION` or `no collision`
- Elapsed time and remaining time (when scenario is active)

**Map legend:**
- Blue circle + arrow = robot; RED ring = currently colliding
- White ring on unfound object = currently visible in camera (LOS-checked)
- Coloured circles = unfound objects (red=fire_ext, green=first_aid, yellow=hazard)
- Grey = found, Brown = furniture, Dark = walls

**Save to a custom path** (useful for comparing across time):
```bash
python3.12 scripts/world_state.py --png /tmp/map_t0.png
```

**Post-run — render map from a results file** (no sim needed):
```bash
python3.12 scripts/world_state.py --results results/<file>.json --no-ros
```

---

## 5. Check robot pose and camera

```bash
# Print current pose (world frame, yaw):
python3.12 scripts/robot_control.py status

# Take a camera snapshot (what the robot currently sees):
python3.12 scripts/robot_control.py snapshot
# Default output: /tmp/robot_snapshot.png — then: Read /tmp/robot_snapshot.png
```

---

## 6. Read live tmux output

```bash
# Sim logs (last 30 lines):
tmux capture-pane -t sim -p -S -30

# Agent logs (last 50 lines):
tmux capture-pane -t agent -p -S -50
```

The sim prints a scorecard + `SUCCESS` or `TIME_LIMIT` when the scenario ends.
The results JSON is written to `results/` automatically.

---

## 7. Interpret the results JSON

Results land in `results/<scenario_name>_<timestamp>.json`.

```bash
ls -t results/ | head -5   # most recent first
```

Key fields:

```
status              — "SUCCESS" or "TIME_LIMIT"
elapsed_seconds     — sim-seconds taken
overall_score       — 0–100
overall_grade       — S / A / B / C / D / F
categories          — per-category scores: speed, accuracy, safety, efficiency, effectiveness
raw_metrics:
  found_ratio           — 0.0–1.0 (1.0 = all 9 objects found)
  precision             — true positives / all detections (low = many false positives)
  duplicate_rate        — fraction of detections that are duplicates
  collision_count       — number of collision events
  near_miss_count       — passes within 20 cm of obstacle
  meters_traveled       — total odometry distance
  exploration_coverage  — % of free space covered by LiDAR
```

Grade thresholds: S ≥ 95, A ≥ 85, B ≥ 70, C ≥ 55, D ≥ 40, F < 40.

Category weights: speed 20%, accuracy 30%, safety 25%, efficiency 10%, effectiveness 15%.

---

## 8. Monitoring loop (while agent runs)

Repeat every 30s wall-time until scenario ends or time runs out:

```bash
. ~/Projects/derpbot-explorer/scripts/ros_env.sh

# Ground truth status
cd ~/Projects/robot-sandbox && python3.12 scripts/world_state.py

# Agent logs
tmux capture-pane -t agent -p -S -50

# Sim logs (check for SUCCESS / TIME_LIMIT)
tmux capture-pane -t sim -p -S -20
```

Read the map PNG after each `world_state.py` call.

---

## Abort conditions — kill stack and report immediately

| Condition | How to detect |
|---|---|
| RTF < 1.0 for >60s wall-time | `rtf_monitor.py --once` repeatedly low |
| Robot not moving for >120s wall-time | `robot_control.py status` pose unchanged across 2+ checks |
| Agent process crashed | `tmux capture-pane -t agent` shows traceback + no new output |
| Nav2 lifecycle startup failure | Nav2 logs: "failed to send response to /smoother_server/change_state" |
| TF flood | Agent logs show Nav2 goals immediately aborting (status 6), TF_OLD_DATA warnings |

For Nav2 lifecycle failure: kill stack and restart once. If it fails again, report to main agent.

---

## 9. Scenario end

Scenario ends when sim prints `SUCCESS` or `TIME_LIMIT`.

```bash
# Get most recent results file
ls -t ~/Projects/robot-sandbox/results/ | head -1

# Read it
cat ~/Projects/robot-sandbox/results/<filename>
```

Report back:
- `overall_score` + `overall_grade`
- `raw_metrics.found_ratio`, `exploration_coverage`, `collision_count`, `near_miss_count`
- Any abort conditions that fired during the run
- Final world_state map (run `world_state.py --no-ros --results <file>` and read the PNG)
- If there were multiple scenarios run, for instance to retry after an issue, and which one is the correct one.

---

## Kill stack (cleanup)

```bash
for sess in agent nav2 slam sim fds; do tmux kill-session -t $sess 2>/dev/null || true; done
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "scenario_runner" 2>/dev/null || true
pkill -9 -f "fastdds discovery" 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
```

Always clean up before reporting, even on abort.

---

## Key gotchas

- **Python**: always `python3.12`. `python3` may resolve to a different venv.
- **`world_state.py` requires a running sim** unless you pass `--no-ros --results <file>`.
- **`robot_control.py status/snapshot` requires a running sim** (needs ROS topics).
- **Timeout is in sim-seconds** — `timeout_seconds: 900` in the scenario config means 900 sim-seconds. At `--speed 2`, that's ~450 wall-clock seconds. At `--speed 3`, ~300 wall-clock seconds.
- **Robot autonomy can significantly impact sim RTF**: minimum RTF is 1.0. Anything lower and the autonomy is too heavy, and needs to be made more efficient.
- **RTF should be checked immediately after launch** and monitored throughout — sustained drop below 0.5× target means something is wrong.
