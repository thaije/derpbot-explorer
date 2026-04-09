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
| Task 2 — Rotation Shim Controller | ❌ not started | Primary speed-score fix. |
| Task 3 — collision_monitor lifecycle isolation | ❌ not started | Fixes residual bond death. |
| Task 4 — MPPI critic retune | ❌ not started | Narrow-door polish + enables local csf=8.0. |
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

### Task 2 — Rotation Shim Controller wrapping MPPI

**Goal:** Cut pre-travel rotation time from 22 s median per waypoint (~30% of run) to near-zero, directly raising the speed score and freeing budget for exploration/detection.

**Root cause (from deep research + benchmark):** MPPI's random trajectory sampling rarely finds valid turning trajectories in constrained spaces (GitHub #4049). When the robot must rotate significantly before translating, MPPI produces slow, jittery turns. The old Task 3's `yaw_goal_tolerance=3.14` only skips the *post-arrival* rotation; the *pre-travel* rotation is still handled by MPPI sampling and remains the dominant bottleneck.

**Plan:**
- Wrap existing MPPI plugin as `primary_controller` under `nav2_rotation_shim_controller::RotationShimController` in `config/derpbot_nav2_params.yaml` → `controller_server.FollowPath`.
- Params:
  - `angular_dist_threshold: 0.785` (engage shim when path heading off by > 45°)
  - `angular_disengage_threshold: 0.3925` (Jazzy hysteresis: disengage at < 22.5°)
  - `forward_sampling_distance: 0.5`
  - `rotate_to_heading_angular_vel: 1.5`
  - `max_angular_accel: 2.5`
  - `simulate_ahead_time: 1.0`
  - `rotate_to_goal_heading: true`
- Keep `yaw_goal_tolerance: 3.14` (goal-arrival rotation stays suppressed).
- Re-run timing benchmark (same seed, same phase table as [`benchmark_results.md`](benchmark_results.md)).

**Definition of done:**
- Rotation phase drops from 30% → < 15% of total run time in the timing table.
- New timing table appended to `benchmark_results.md`.
- `avg_speed_kmh` in results JSON measurably higher, no coverage regression.

---

### Task 3 — Isolate collision_monitor in its own lifecycle group

**Goal:** Decouple the safety node from navigation failures. A planner crash should not take the collision monitor down with it.

**Root cause (from deep research + handoff gotcha):** `collision_monitor` currently shares `lifecycle_manager_navigation` with controller/planner/behavior servers. Under GPU load (Gazebo + OWLv2 on shared GPU 0), bond heartbeats are delayed; a failure in any managed node triggers the lifecycle manager to bring *all* nodes down, leaving the robot unprotected during respawn. Current mitigation (`bond_timeout: 60 s`) masks the symptom but does not isolate the safety path.

**Plan:**
- Split `launch/navigation_launch.py` lifecycle managers:
  - `lifecycle_manager_navigation`: `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `smoother_server`, `velocity_smoother`, `waypoint_follower`
  - `lifecycle_manager_safety`: `collision_monitor` only
- Align bond timing on both managers: `bond_timeout: 10.0`, `bond_heartbeat_period: 0.5` (drop from 60 s — properly-aligned heartbeat is the real fix, not the oversized timeout).
- Add `use_realtime_priority: true` on `collision_monitor` — elevates its thread to priority 90. **Requires** `/etc/security/limits.conf` to grant `rtprio` permissions — flag to Tjalling before enabling.
- Rename `attempt_respawn_on_failure` → `attempt_respawn_reconnection` on both managers (Nav2 Jazzy parameter rename).

**Definition of done:**
- Manual test: kill `planner_server` mid-run. `collision_monitor` continues running without bond starvation. `lifecycle_manager_navigation` respawns the dead node without touching the safety group.
- Full easy-scenario run with zero bond timeout entries in logs.
- `AGENT_HANDOFF.md` updated: new lifecycle architecture note, remove the `bond_timeout: 60 s` hack entry.

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

## Potential future extensions

### IMU-fused odometry (robot_localization EKF)
If odometry angular accuracy degrades again (e.g. different sim versions or real hardware), add a `robot_localization` EKF node that fuses `/derpbot_0/imu` (100 Hz gyro, accurate angular rate) with `/derpbot_0/odom` (accurate linear, unreliable angular). The EKF publishes `/odom_fused`; point `slam_toolbox`'s `odom_frame` at it. This eliminates systematic wheel-baseline angular error at the source without touching SLAM params. Config: ~20 lines of YAML (`ekf.yaml`) + one launch node.
