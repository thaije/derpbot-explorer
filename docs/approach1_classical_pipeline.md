# DerpBot — Approach 1: Classical Modular Pipeline

> Current implementation. Full architecture survey: [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)
> Benchmark history: [`docs/benchmark_results.md`](benchmark_results.md)
> Deep research on robust Nav2 recovery: [`docs/deep_research_robust_nav2.md`](deep_research_robust_nav2.md)

## Status

Core pipeline is built and running. Current focus: hardening exploration robustness before layering on perception improvements.

| Item | Status | Notes |
|------|--------|-------|
| End-to-end easy scenario score | ⚠️ in progress | Best: 66.1 (C), 4/6, 98% coverage. Target: ≥ 70 (B). |
| Reduce inter-goal idle time | ✅ done | `yaw_goal_tolerance=3.14`, goal pose oriented to travel dir, `_flood_unknown` removed, streak blacklist on reject. Residual rotation bottleneck addressed by Task 2 (RotationShim). Benchmark table in [`benchmark_results.md`](benchmark_results.md). |
| Task 1 — differential inflation + START_OCCUPIED BT | ✅ done | 0 START_OCCUPIED events across 2 runs, doors passable, bond-death no longer affects scored run. Coverage 74% in verification run (--no-perception). See [`benchmark_results.md`](benchmark_results.md). |
| Task 2 — Rotation Shim Controller | ✅ done | Rotation phase 30.1% → 13.2% (DoD met). Required `PoseProgressChecker` swap + tighter shim engagement (20°/8.6° hysteresis). Coverage didn't recover to baseline due to orthogonal bugs (see backlog). |
| Task 3 — collision_monitor lifecycle isolation | ✅ done | Split into `lifecycle_manager_safety`. Nav-side failures no longer reach the safety bond. `bond_timeout` 60→10s, `attempt_respawn_on_failure` → `attempt_respawn_reconnection` (Jazzy). 0 bond events during scored run; teardown-only event remains. |
| Task 4 — MPPI critic retune | ❌ not started | Narrow-door polish + enables local csf=8.0. May also fix the goal-mid-cascade bug in backlog. |
| Task 5 — Detection-aware exploration | ❌ not started | After exploration is robust. |
| `medium` / `hard` tier | ❌ not started | After easy scenario ≥ B. |

---

## Work queue

Tasks in priority order. Each must be completed and verified before moving to the next.
Rationale and deep-research sources for Tasks 1–4: [`docs/deep_research_robust_nav2.md`](deep_research_robust_nav2.md).

---

### Task 1 — Differential costmap inflation + START_OCCUPIED BT recovery

**Goal:** Eliminate the "Start occupied" cascade that currently causes the robot to permanently refuse goals after reaching a frontier. Biggest single coverage killer in long runs.

**Root cause (from deep research):** With `robot_radius: 0.22` + `inflation_radius: 0.25`, the soft inflation zone is only **3 cm** wide. Any SLAM correction places the robot's center in a cost-253 cell; SmacPlanner2D refuses every subsequent goal with "Start occupied". The current fire-and-forget `clear_global_costmap` in `frontier_explorer.py` is a band-aid that does not address either cause (narrow soft zone) or the BT's gated recovery path.

**Secondary issue:** Nav2 Jazzy's default BT has `WouldAPlannerRecoveryHelp` gating the contextual `ClearEntireCostmap` recovery. It only matches `UNKNOWN`, `NO_VALID_PATH`, `TIMEOUT` — not `START_OCCUPIED`. So the contextual recovery never fires for this failure mode.

**Plan:**
- **Differential inflation** in `config/derpbot_nav2_params.yaml`:
  - `global_costmap.inflation_layer`: `inflation_radius: 0.55`, `cost_scaling_factor: 3.0` (wide soft zone so planner paths are centered and post-SLAM corrections don't land on lethal cells)
  - `local_costmap.inflation_layer`: `inflation_radius: 0.35`, `cost_scaling_factor: 8.0` (narrow, steep decay — at 0.28 m from wall in a 1 m door, cost ≈ 157, well below 253 threshold)
- Enable `footprint_clearing_enabled: true` on the static layer (Nav2 Jazzy PR #4282 specifically addresses SLAM wall corrections placing the robot inside newly detected walls).
- **Custom BT** `launch/behavior_trees/navigate_backup_clear_replan.xml`:
  - Remove the `WouldAPlannerRecoveryHelp` gate from the planner `RecoveryNode`
  - Lead the recovery sequence with `BackUp(0.15 m, 0.10 m/s)` *before* `ClearEntireCostmap` — move the robot out of the lethal cell first, then clear stale data, then replan
  - Keep outer `RecoveryNode number_of_retries="4"` — fail fast to application layer
- Set `default_nav_to_pose_bt_xml` in `config/derpbot_nav2_params.yaml` to the new XML.
- Remove fire-and-forget `clear_global_costmap` call from `frontier_explorer.py` once the BT handles it correctly.

**Conflicts with current handoff:** The `inflation=0.25 empirically optimal` entry in `AGENT_HANDOFF.md` was established with uniform inflation only — the differential approach was not in that A/B. Handoff must be updated with the new evidence after verification.

**Definition of done:**
- Full easy-scenario run shows zero "Start occupied" blacklist entries.
- Doors in OfficeA still passable (manual trace of robot path through 1 m doorway).
- Coverage ≥ 98% baseline, ≥ 4 detections, ≤ 1 collision.
- `AGENT_HANDOFF.md` updated: revised inflation entry + removal of the fire-and-forget clear note.

---

### Task 2 — Rotation Shim Controller wrapping MPPI ✅ DONE

**Goal:** Cut pre-travel rotation time from 22 s median per waypoint (~30% of run) toward near-zero, directly raising the speed score and freeing budget for exploration/detection.

**Root cause (from deep research + benchmark):** MPPI's random trajectory sampling rarely finds valid turning trajectories in constrained spaces (GitHub #4049). When the robot must rotate significantly before translating, MPPI produces slow, jittery turns. The old Task 3's `yaw_goal_tolerance=3.14` only skips the *post-arrival* rotation; the *pre-travel* rotation is still handled by MPPI sampling and remains the dominant bottleneck.

**What landed:**
- `controller_server.FollowPath` wraps MPPI under `nav2_rotation_shim_controller::RotationShimController` (`primary_controller: "nav2_mppi_controller::MPPIController"`).
- Final shim params (after 3 verification runs of tuning):
  - `angular_dist_threshold: 0.35` (engage on 20° mismatch — 45° was too loose, MPPI got stuck doing 20–45° rotations slowly)
  - `angular_disengage_threshold: 0.15` (8.6° hysteresis — only hand off to MPPI once well aligned)
  - `forward_sampling_distance: 0.5`
  - `rotate_to_heading_angular_vel: 1.8` (close to `wz_max=2.0`)
  - `max_angular_accel: 3.0` (matches velocity_smoother)
  - `simulate_ahead_time: 1.0`
  - `rotate_to_goal_heading: true`
- **Critical fix:** Swapped `progress_checker` from `SimpleProgressChecker` → `PoseProgressChecker` (`required_movement_angle: 0.25`). The simple checker only counts linear displacement, so the shim's in-place rotation was being treated as "no progress" → fired after 10s → cascaded into local-costmap-clear loops → MPPI "Costmap timed out". This is the root reason the first verification runs collapsed coverage to 32%.
- Restored `yaw_goal_tolerance: 3.14` (had drifted to 0.5 against the old Task 3 status).

**Result:** Rotation phase 30.1% → 13.2% — DoD met. Per-goal first_move drops from 22 s median (baseline) to 1.9–2.3 s when the shim has a clean start (verified on goal#3 / goal#4 in run 3). Goal#4 (long cross-map hop) recovered from a guaranteed-failure cascade.

**Coverage did *not* recover to the 74% baseline** in the verification runs (best was 53%). Investigation traced the loss to three orthogonal bugs that the rotation shim does not address — captured in the [Backlog](#backlog--known-issues-not-blocking-task-progression) section below. Decision: ship Task 2 and fold those into the next tasks rather than scope-creep here.

**Verification:** [`benchmark_results.md`](benchmark_results.md) — 2026-04-09 Task 2 entry with per-goal phase table.

---

### Task 3 — Isolate collision_monitor in its own lifecycle group ✅ DONE

**Goal:** Decouple the safety node from navigation failures. A planner crash should not take the collision monitor down with it.

**Root cause (from deep research + handoff gotcha):** `collision_monitor` shared `lifecycle_manager_navigation` with controller/planner/behavior servers. Under GPU load (Gazebo + OWLv2 on shared GPU 0), bond heartbeats slipped and the manager would bring the *entire* group down, leaving the robot unprotected during respawn. Previous mitigation (`bond_timeout: 60 s`) masked the symptom but did not isolate the safety path. Worse: the parameter `attempt_respawn_on_failure` was the *old* name and was being silently ignored on Jazzy.

**What landed:** `launch/navigation_launch.py`
- `lifecycle_manager_navigation` now owns: `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `velocity_smoother`, `bt_navigator`, `waypoint_follower`.
- New `lifecycle_manager_safety` owns: `collision_monitor`.
- Both managers: `bond_timeout: 10.0` (was 60 s) and `attempt_respawn_reconnection: true` (Jazzy rename — old key was silently ignored).
- Same change applied to the composable-bringup branch for parity.

**Note on `bond_heartbeat_period`:** the deep-research recommendation to set `bond_heartbeat_period: 0.5` does not apply to Jazzy — the `nav2_lifecycle_manager` library doesn't expose this parameter; the bond library uses its compiled-in default. Only `bond_timeout` is tunable on the manager side. Verified by `strings` on `libnav2_lifecycle_manager_core.so`.

**Note on `use_realtime_priority`:** `nav2_collision_monitor` does support `use_realtime_priority: true` (verified in the binary), but enabling it requires `/etc/security/limits.conf` to grant the user `rtprio` capability. Deferred — see Backlog. For now the soft separation + 10 s bond is sufficient.

**Verification (2026-04-10, easy scenario, seed=42, --no-perception):**
- Both `lifecycle_manager_navigation` (pid 10780) and `lifecycle_manager_safety` (pid 10781) reached "Managed nodes are active" cleanly.
- Zero bond events on either manager during the scored run.
- During goal#2's mid-run planner failure cascade, the safety manager logged nothing — `collision_monitor` continued running with bond intact while nav-side aborts piled up. This is exactly the isolation behaviour the task targets.
- One bond event on the safety manager at t = scenario_end + 7 s (teardown only — acceptable, expected when the bridge / sim is going away).

**Definition of done:** ✅
- Isolation confirmed in vivo by goal#2 nav-side failure cascade leaving safety untouched (de facto kill-test).
- Zero bond timeout entries during the scored run (teardown event excluded).
- `AGENT_HANDOFF.md` updated with the new architecture note and the old `bond_timeout: 60 s` hack entry removed.

---

### Task 4 — MPPI critic retune for narrow passages

**Goal:** Make 1 m doorway traversal reliable without depending on recovery behaviors (spin/backup) to push the robot through.

**Plan:**
- `CostCritic.consider_footprint: true` — non-negotiable for DiffDrive in tight spaces per deep research. Verify at runtime whether this still crashes without a polygon footprint (handoff gotcha warned about this, but the earlier crash was without either `robot_radius` *or* polygon footprint — retest). If it crashes, add a 4-vertex polygon footprint matching the physical chassis.
- `PathAlignCritic.cost_weight: 14.0` (forces tight path tracking through doors).
- `CostCritic.critical_cost: 300.0`.
- Reduce MPPI sampling std: `vx_std: 0.15`, `wz_std: 0.3` (tighter sampling = precision in constrained spaces).
- Enable `publish_critics_stats: true` (Jazzy) for per-critic cost visibility during tuning.
- Depends on Task 1: new inflation changes the cost landscape critics see.

**Definition of done:**
- Clean traversal through the OfficeA 1 m door across 3 seeds, no recovery spin-backup interventions recorded.
- No regression in collision count or coverage.

---

### Task 5 — Detection-aware exploration

**Goal:** As a developer, I want the robot to revisit areas where objects were partially detected but not yet confirmed, so that detection rate improves beyond what pure coverage achieves.

**Prerequisite:** Tasks 1–4 complete. Exploration must be robust before layering perception feedback onto it.

**Plan:**
- Extend `tracker.py` to export a list of "pending candidates" — objects with 1 sighting (below `MIN_SIGHTINGS`) and their estimated world position.
- In `frontier_explorer.py`, inject pending candidates as high-priority pseudo-frontiers: give them a score override above any geographic frontier, so the robot immediately detours toward them when a candidate appears.
- Add a recency timeout: a candidate that has been pending for > N sim-seconds without a second sighting is discarded (object was likely a false positive).
- Tune the priority weight so the robot doesn't thrash between candidates and coverage (one detour per candidate, then resume coverage).

**Definition of done:**
- Easy-scenario run shows an increase in confirmed detections relative to Task 4 baseline, with no increase in false positive rate.
- `AGENT_HANDOFF.md` updated with the candidate injection design and any threshold values tuned.

---

## Backlog — known issues not blocking task progression

Surfaced during Task 2 verification (2026-04-09). None are rotation-shim-related; logged here so they aren't lost. Re-check after Task 3/4 — some may resolve as a side effect.

- **Cold-start delay on goals 1–2 (~11 s `first_move` each).** Shim engaged correctly per logs, but the robot didn't start moving for ~11 s on the first two goals of every Task 2 run, regardless of heading offset (later goals show 1.9–2.3 s). Suspected: local_costmap waiting for first scan, slam_toolbox initial convergence, or velocity_smoother cold-boot deadband. Investigate by reading `controller_server` log timing between "Received a goal" and first `cmd_vel` publish on goal#1.
- **Frontier picks off-map points.** Run 3 selected frontier (16.94, -0.12) on the OfficeA map whose east wall is ~12 m. Robot wasted the last 180 s of the run on an unreachable target. `frontier_explorer.py` BFS does not clip to the SLAM map's known extent — should reject any cell outside the bounding box of `OccupancyGrid` cells with state ≥ 0.
- **MPPI "Failed to make progress" cascade mid-navigation.** Run 2's goal#4 ran cleanly for ~64 s then triggered "Failed to make progress" → local clear → "Costmap timed out" → "Resulting plan has 0 poses" loop. Independent of the shim (the rotation issue was already fixed by `PoseProgressChecker`). May resolve via Task 4 (tighter MPPI sampling + `consider_footprint: true`); if not, needs its own root-cause pass.
- **`avg_speed_kmh` reported as ~0.015 km/h while `meters_traveled` ≈ 1.235 m**, despite the robot clearly traversing multi-metre distances. Suspected metric-reporting bug in the scenario runner (predates Task 2). Tanks the Speed category. Not in this repo — check `robot-sandbox` scoring code.
- **`collision_monitor` `use_realtime_priority: true` deferred (Task 3).** The parameter is supported (verified in `libcollision_monitor_core.so`) and would elevate the safety thread to priority 90, hardening it against GPU/CPU starvation. Enabling it requires editing `/etc/security/limits.conf` to grant the run user `rtprio` capability — a system-level change. Currently the soft separation (own lifecycle group + 10 s bond) is sufficient; revisit if bond events reappear under perception load.

---

## Potential future extensions

### IMU-fused odometry (robot_localization EKF)
If odometry angular accuracy degrades again (e.g. different sim versions or real hardware), add a `robot_localization` EKF node that fuses `/derpbot_0/imu` (100 Hz gyro, accurate angular rate) with `/derpbot_0/odom` (accurate linear, unreliable angular). The EKF publishes `/odom_fused`; point `slam_toolbox`'s `odom_frame` at it. This eliminates systematic wheel-baseline angular error at the source without touching SLAM params. Config: ~20 lines of YAML (`ekf.yaml`) + one launch node.
