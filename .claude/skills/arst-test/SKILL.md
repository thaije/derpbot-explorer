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

Run in a tmux session so it stays alive:
```bash
tmux new -s sim -d './scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless [--seed N] [--speed N] [--timeout N]'
tmux capture-pane -t sim -p -S -20   # check startup output
```

Wait ~5 s. Sim is ready when `world_state.py` returns without error.

**Check RTF after sim starts (before agent):**
```bash
gz topic -e -t /stats | head -20
```
Look for `real_time_factor`. Should be at or near `1.0` (or near `--speed N` if set).
A RTF of 1.0 is a minimum. If it is lower than that there is something still running on the computer hogging resources.

After starting the agent, **check RTF again**. The sim and autonomy stack share the same hardware. A drop in RTF at this point means the agent is too compute-heavy; the sim will slow down, timing-dependent behaviour will break, and metrics will be skewed.

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
- **Elapsed time is in sim-seconds** — at `--speed 3`, wall time is ~3× faster.
- **Robot autonomy can significantly impact sim RTF**: minimum RTF is 1.0. Anything lower and the autonomy is too heavy, and needs to be made more efficient.

