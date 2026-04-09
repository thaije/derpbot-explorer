---
name: arst-test
description: Test and evaluate an autonomous robot running an ARST scenario. Use when you are the observer/tester — starting the sim, watching a robot agent run, monitoring ground-truth progress, and reading final scores. NOT for driving the robot yourself (use /arst-nav for that, if available).
user_invocable: true
---

# arst-test — ARST Autonomous Robot Test Skill

You are the **observer/tester**, not the driver. A separate autonomy stack controls DerpBot.
Your job: start the sim, start the agent, monitor from the outside, and interpret results.

All commands run from the ~/Projects/robot-sandbox repo root. PYTHONPATH and ROS env are set by hooks — no prefix needed.

---

## Scenario tiers

See the AUTONOMOUS_AGENT_GUIDE.md file.
---

## 1. Start the simulation

**Recommended: use `start_stack.sh`** (from the derpbot-explorer repo). It kills stale processes, restarts the ROS2 daemon, and starts sim → SLAM → Nav2 → agent in the correct order within 5s of sim ready.

```bash
cd ~/Projects/derpbot-explorer

# Full stack (sim + SLAM + Nav2 + agent)
./scripts/start_stack.sh --speed 2 --seed 42

# Without agent (benchmarking)
./scripts/start_stack.sh --speed 2 --seed 42 --no-agent

# Nav-only mode — agent runs without OWLv2 detector/tracker (found_ratio=0; isolates Nav2)
./scripts/start_stack.sh --speed 2 --seed 42 --no-perception

# Options: --speed N (default 2), --seed N (default 42), --scenario TIER (default easy),
#          --no-agent, --no-perception
```

Creates tmux sessions: `sim`, `slam`, `nav2`, `agent`. Then switch to `~/Projects/robot-sandbox` for monitoring.

**Manual sim-only** (if needed):
```bash
cd ~/Projects/robot-sandbox
tmux new -s sim -d './scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --seed 42 --speed 2'
tmux capture-pane -t sim -p -S -20   # check startup
```

Wait ~5 s. Sim is ready when output shows "Simulation ready".

**Check RTF — run synchronously from `~/Projects/robot-sandbox`:**
```bash
python3.12 scripts/rtf_monitor.py --samples 5
python3.12 scripts/rtf_monitor.py --once   # single value
```
RTF should be near `--speed N`. Full stack at speed=2 achieves ~1.9. Sim ceiling is ~3× regardless of speed setting.

> **Warning**: `rtf_monitor.py` produces empty output when run as a background shell task (Python stdout buffering). Always run it synchronously or use `--once` in a loop.

After starting the agent, check RTF again — a sustained drop below 0.5× target means something is wrong.

---

## 2. Start the autonomous agent

Run the robot's agent script in a separate tmux session:
```bash
tmux new -s agent -d 'python3.12 <path/to/agent_script.py>'
tmux capture-pane -t agent -p -S -30   # watch agent output
```

The agent controls the robot via ROS topics or `robot_control.py`. You do not drive.

---

## 3. Monitor ground-truth progress (world_state.py)

This is your primary observability tool. Call it any time to get a full status snapshot.

```bash
# Default: writes arst_world_map.png in repo root, prints full status
python3.12 scripts/world_state.py
# Then: Read arst_world_map.png
```

**Output includes:**
- Object list with status: `not found`, `FOUND ✓`, or `not found  [visible 👁]`
- Robot pose (world frame, yaw, facing direction)
- Collision status: `⚡ COLLISION` or `no collision`
- Elapsed time and remaining time (when scenario is active)

**Map legend:**
- Blue circle + arrow = robot; RED ring = currently colliding
- White ring on unfound object = currently visible in camera (LOS-checked)
- Coloured circles = unfound objects (red=fire_ext, green=first_aid, yellow=hazard)
- Grey = found, Brown = furniture, Dark = walls

**Save to a custom path** (useful for comparing across time):
```bash
python3.12 scripts/world_state.py --png /tmp/map_t0.png
# ... later ...
python3.12 scripts/world_state.py --png /tmp/map_t1.png
```

**Post-run — render map from a results file** (no sim needed):
```bash
python3.12 scripts/world_state.py --results results/office_medium_001_<timestamp>.json --no-ros
```

---

## 4. Check robot pose and camera

```bash
# Print current pose (world frame, yaw):
python3.12 scripts/robot_control.py status

# Take a camera snapshot (what the robot currently sees):
python3.12 scripts/robot_control.py snapshot
# Default output: /tmp/robot_snapshot.png — then: Read /tmp/robot_snapshot.png

# Save to a named path:
python3.12 scripts/robot_control.py snapshot --output /tmp/snap_t1.png
```

Use these to diagnose agent behaviour:
- Robot stuck? `status` to get pose, compare across calls.
- Agent claiming to see objects? `snapshot` to verify the camera view.
- Collision event? `world_state.py` shows collision ring on robot in map.

---

## 5. Read live tmux output

```bash
# Sim logs (last 30 lines):
tmux capture-pane -t sim -p -S -30

# Agent logs (last 50 lines):
tmux capture-pane -t agent -p -S -50
```

The sim prints a scorecard + `SUCCESS` or `TIME_LIMIT` when the scenario ends.
The results JSON is written to `results/` automatically.

---

## 6. Interpret the results JSON

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

## 7. Monitoring loop (while agent runs)

Repeat until scenario ends or time runs out:

```
world_state.py  →  Read map  →  check found count + collision status
robot_control.py status  →  check pose if robot seems stuck
tmux capture-pane -t agent  →  watch for errors or stalls
```

Polling interval: every 15–30 s wall-time is enough unless debugging a specific issue.
At `--speed 3`, 30 s wall = 90 s sim. Check more often for short timeouts.

---

## Key gotchas

- **Python**: always `python3.12`. `python3` may resolve to a different venv.
- **`world_state.py` requires a running sim** unless you pass `--no-ros --results <file>`.
- **`robot_control.py status/snapshot` requires a running sim** (needs ROS topics).
- **Timeout is in sim-seconds** — `timeout_seconds: 900` in the scenario config means 900 sim-seconds. At `--speed 2`, that's ~450 wall-clock seconds. At `--speed 3`, ~300 wall-clock seconds.
- **Robot autonomy can significantly impact sim RTF**: minimum RTF is 1.0. Anything lower and the autonomy is too heavy, and needs to be made more efficient.

