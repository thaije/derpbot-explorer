# DerpBot — Approach 1: Classical Modular Pipeline

> Current implementation. Full architecture survey: [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)

## Status

Core pipeline is built and running. Remaining work:

| Item | Status | Notes |
|------|--------|-------|
| End-to-end easy scenario score | ⚠️ in progress | Best: 66.1 (C), 4/6, 98% coverage (patrol+OWL=0.15). Nav fix (infl=0.12/0.05 + goal clearance+push) running 2026-03-21. |
| RTF stability | ✅ resolved | Root cause: NUMA node 0 thermal throttling. Fix: numactl applied in start_stack.sh |
| FastDDS discovery | ✅ resolved | /map now arrives in <30s instead of ~4 sim-min; agent navigates immediately |
| Exploration coverage | ✅ resolved | 98.12% with W_DIST=1.5. Root cause was LIDAR coverage ≠ camera coverage |
| Physical camera coverage | ✅ resolved | Patrol mode navigates to physically unvisited regions after frontier exhaustion. Person+fire_ext found at y>7.5 in 66.1 run. |
| Nav2 door passability | ✅ resolved | global infl=0.12/local=0.05, GOAL_CLEARANCE_M=0.36+push fallback in frontier_explorer.py |
| Detection-aware exploration | ❌ not started | Task 5 — after nav fix verified |
| `medium`/`hard` tier | ❌ not started | After easy scenario is solved |

---

## Work queue

Tasks in priority order. Each must be completed and verified before moving to the next.

---

### Task 1 — FastDDS discovery server ✅ DONE

**Result:** `fastdds discovery -i 0 -l 127.0.0.1 -p 11811` started in `fds` tmux session before ROS2 daemon. All sessions get `ROS_DISCOVERY_SERVER=127.0.0.1:11811 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_SUPER_CLIENT=1`. Agent started navigating "almost immediately" on first test run (verified 2026-03-21). Source `scripts/ros_env.sh` before any `ros2` CLI/monitoring command.

---

### Task 2 — Clean baseline run ✅ DONE

**Result:** Multiple complete runs on file; best: **66.1 (C), 4/6 found, 98% coverage** (seed=42, speed=2, RTF≈1.97, 2026-03-21). FastDDS baseline was 55.4 (C). Patrol + OWL=0.15 brought score to 66.1. RTF stable throughout at 1.89–1.97. Startup sequence fully automated in `scripts/start_stack.sh`. Key finding: seed=42 has 6 mission targets (fire_ext×3, first_aid×2, person×1); no exit_signs. `AGENT_HANDOFF.md` performance table has all runs.

---

### Task 3 — Exploration idle-time profiling

**Goal:** As a developer, I want to know what fraction of sim-time the robot spends actually moving vs. waiting on Nav2 overhead, so I can decide whether to fix the scoring or reduce the overhead.

**Plan:**
- Instrument `frontier_explorer.py` to log timestamps at: goal selected, goal sent, goal accepted, goal completed. Compute per-goal: time-to-accept, navigation time, time-between-goals (idle between SUCCEEDED and next goal sent).
- Run easy scenario to completion. Parse the structured log to compute: % time moving (velocity > 0.1 m/s), average idle time between goals, average navigation time per goal, number of goals per 100 sim-seconds.
- Compare against the theoretical minimum: if the robot moved at 0.5 m/s for the full 900s, what coverage would it achieve? Use this to set a realistic target.
- Compare against the par score defined in the scenario file.

**Definition of done:**
- Metrics table recorded: % moving, avg idle, avg nav time, goals/100s.
- Clear statement of what the primary bottleneck is (scoring quality, Nav2 overhead, or stuck/recovery time).
- If idle time > 20% of total sim time, proceed to Task 4a. If frontier scoring is picking poor goals (robot reaches goal but gains minimal new area), proceed to Task 4b.

---

### Task 4 — Exploration coverage fix

**Goal:** As a developer, I want the robot to reach ≥ 90% map coverage within the 900s time limit, so that all target areas are visited at least once.

**Plan (depends on Task 3 outcome):**

**4a — Reduce idle time (if overhead is the bottleneck):**
- Identify the dominant idle source (goal acceptance delay, Nav2 cleanup between goals, BFS frontier scan frequency). Implement the minimal fix — e.g. reduce inter-goal pause, cache frontier clusters between ticks, pipeline goal sending with map update.

**4b — Improve frontier scoring (if poor goal selection is the bottleneck):** ✅ DEPLOYED + REVERTED
- Info-gain scoring failed (all frontiers hit MAX_FLOOD_CELLS=5000 cap early → equal scores → micro-stepping). Reverted to cluster.size. W_DIST lowered 2.0→1.5.
- **Root problem was different**: 98% LIDAR coverage achieved but 3 mission objects (y>7.5) never visited by camera. Fix: patrol mode (navigate to physically unvisited free cells after frontier exhaustion).

**Definition of done:**
- A complete easy-scenario run achieves ≥ 90% exploration coverage within 900 sim-seconds.
- `AGENT_HANDOFF.md` updated with new W_DIST / scoring approach and coverage achieved.

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

### Other fixes
- map takes 2min to become available. Find a way to speed this up.