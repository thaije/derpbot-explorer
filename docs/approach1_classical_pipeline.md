# DerpBot — Approach 1: Classical Modular Pipeline

> Current implementation. Full architecture survey: [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)

## Status

Core pipeline is built and running. Remaining work:

| Item | Status | Notes |
|------|--------|-------|
| End-to-end easy scenario score | ⚠️ in progress | Best: 66.1 (C), 4/6, 98% coverage (patrol+OWL=0.15). Target: ≥70 (B). |
| Reduce inter-goal idle time | ⚠️ in progress | Goal orientation → travel dir + yaw_goal_tolerance=3.14 + removed _flood_unknown. Need run to verify. |
| Detection-aware exploration | ❌ not started | Task 5 |
| `medium`/`hard` tier | ❌ not started | After easy scenario ≥ B |

---

## Work queue

Tasks in priority order. Each must be completed and verified before moving to the next.

---

### Task 3 — Reduce inter-goal idle time

**Goal:** Speed score is currently grade F (avg 0.022 m/s over full run). Robot spends most time waiting on Nav2 goal acceptance and rotating at waypoints, not moving. Improving this raises the speed score and leaves more time budget for detections.

**Plan:**
- Instrument `frontier_explorer.py`: log timestamps at goal selected / sent / accepted / completed. Compute avg time-to-accept, avg navigation time, avg idle between goals.
- Identify dominant source (acceptance latency, inter-goal pause, BFS scan frequency) and implement the minimal fix.

**Definition of done:** 
- A table with average metrics per step in the navigation stack, and what constitutes the main bottleneck for improving robot speed. 
- Measurable improvement in `avg_speed_kmh` in results JSON without coverage regression.

---

### Task 5 — Detection-aware exploration

**Goal:** As a developer, I want the robot to revisit areas where objects were partially detected but not yet confirmed, so that detection rate improves beyond what pure coverage achieves.

**Plan:**
- Extend `tracker.py` to export a list of "pending candidates" — objects with 1 sighting (below MIN_SIGHTINGS) and their estimated world position.
- In `frontier_explorer.py`, inject pending candidates as high-priority pseudo-frontiers: give them a score override above any geographic frontier, so the robot immediately detours toward them when a candidate appears.
- Add a recency timeout: a candidate that has been pending for > N sim-seconds without a second sighting is discarded (object was likely a false positive).
- Tune the priority weight so the robot doesn't thrash between candidates and coverage (one detour per candidate, then resume coverage).

**Definition of done:**
- Easy-scenario run shows an increase in confirmed detections relative to Task 4 baseline, with no increase in false positive rate.
- `AGENT_HANDOFF.md` updated with the candidate injection design and any threshold values tuned.

---

## Potential future extensions


### IMU-fused odometry (robot_localization EKF)
If odometry angular accuracy degrades again (e.g. different sim versions or real hardware), add a `robot_localization` EKF node that fuses `/derpbot_0/imu` (100 Hz gyro, accurate angular rate) with `/derpbot_0/odom` (accurate linear, unreliable angular). The EKF publishes `/odom_fused`; point `slam_toolbox`'s `odom_frame` at it. This eliminates systematic wheel-baseline angular error at the source without touching SLAM params. Config: ~20 lines of YAML (`ekf.yaml`) + one launch node.

