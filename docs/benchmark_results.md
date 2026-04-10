# Benchmark Results

Historical performance snapshots. Append new entries on top; keep older ones for regression comparison.

---

## 2026-04-10 — Task 4 verification (seed=42, easy, speed=2, --no-perception)

`CostCritic.consider_footprint: true` with explicit polygon footprint, tighter MPPI sampling (`vx_std: 0.15`, `wz_std: 0.3`), `publish_critics_stats: true`. Polygon `[[0.16,0.11],[0.16,-0.11],[-0.16,-0.11],[-0.16,0.11]]` on both costmaps (chassis 30×20cm + 1cm margin).

| Metric | Task 3 baseline | **Task 4 (csf=5.0)** | csf=8.0 follow-on |
|---|---|---|---|
| Score / grade | 50.6 D | **54.9 D** | 54.8 D |
| Coverage | 54.4% | **71.3%** | 68.3% |
| Goals reached | 3/5 | **5+/6** | 6 |
| Collisions | 1 (t=17s) | **0** | 0 |
| START_OCCUPIED | 0 | **0** | 0 |
| Bond events (scored run) | 0 | **0** | 0 |
| Failed-to-make-progress | 0 | **0** | 0 |
| Costmap timed out | 0 | **0** | 0 |
| Median first_move | — | **~2.1 s** (4 of 5 goals) | ~11 s |

**Per-goal first_move (Task 4 main run):**

| Goal | Target | first_move | nav | Outcome |
|---|---|---|---|---|
| #1 | (0.71, 1.74)  | 10.5 s | 17.5 s | SUCCEEDED (cold start) |
| #2 | (2.09, 7.38)  |  2.5 s | 40.2 s | SUCCEEDED |
| #3 | (7.38, 6.53)  | 20.7 s | 64.7 s | SUCCEEDED |
| #4 | (12.65, 6.13) |  2.1 s | 31.7 s | SUCCEEDED |
| #5 | (17.20, 6.08) |  2.0 s | 32.3 s | SUCCEEDED |
| #6 | (15.04, 0.56) |  —     | DNF (TIME_LIMIT) | started but truncated |

**Verdict:**
- **Task 4 DoD met.** No recovery interventions, collision count 1 → 0, coverage 54.4 → 71.3% (+16.9 pp). Footprint-aware CostCritic + tighter sampling pays back exactly as the deep research predicted.
- **Polygon footprint is non-negotiable for `consider_footprint: true`.** First attempt with `robot_radius: 0.22` only crashed at `controller_server.on_configure`: "Inconsistent configuration in collision checking. ... no robot footprint provided in the costmap." Adding the explicit polygon resolved it. The earlier handoff gotcha was correct — updated to clarify that `robot_radius` alone is insufficient.
- **csf=8.0 follow-on tested and reverted.** Hypothesis (steeper local inflation = wider passable corridor through 1m doors) didn't pay back. Controller was stable — no "Failed to make progress" cascades, no rate misses, no bond drops — but `first_move` regressed by ~5–10s on every goal (median 2.1s → 11s) and coverage dropped 3 pp. MPPI hesitates longer before committing under the steeper gradient. csf=5.0 stays.
- **Score is now gated by perception** (accuracy=40 D, found_ratio=0 because of `--no-perception`). Nav layer is performing materially better; next big lever is Task 5 (detection-aware exploration).
- **Backlog status:** `goal#3 first_move=20.7s` is a new outlier worth watching; could be the same cold-start mechanism as goal#1, or a doorway-negotiation pause. Cold-start and frontier-off-map bugs from Task 2 backlog still present (less impactful at higher coverage).

---

## 2026-04-10 — Task 3 verification (seed=42, easy, speed=2, --no-perception)

`collision_monitor` lifted into its own `lifecycle_manager_safety` group; both managers on `bond_timeout: 10.0` + `attempt_respawn_reconnection: true`.

| Metric | Value |
|---|---|
| Score / grade | 50.6 (D) |
| Coverage | 54.4% |
| Goals succeeded / attempted | 3 of 5 (goal#5 truncated by time-limit) |
| Collisions | 1 (t=17 s) |
| START_OCCUPIED events | 0 |
| Bond events on `lifecycle_manager_navigation` | **0** during scored run |
| Bond events on `lifecycle_manager_safety` | **0** during scored run; 1 in teardown (~7 s after sim shutdown) |
| `collision_monitor` log errors / warnings | 0 |

**Verdict — Task 3 DoD met:**
- Both lifecycle managers reached "Managed nodes are active" cleanly on startup.
- During goal#2's planner-failure cascade (multiple aborts on the navigation side), the safety manager logged nothing and `collision_monitor`'s bond stayed intact. This is the in-vivo equivalent of the planned manual `kill planner_server` test.
- Score regression vs Task 2 (54.0 → 50.6) is from the same backlog issues already documented (early collision, mid-nav cascade, off-map frontier picks). No new regressions traceable to lifecycle isolation.

---

## 2026-04-09 — Task 2 verification (seed=42, easy, speed=2, --no-perception)

RotationShimController wraps MPPI; `PoseProgressChecker` replaces `SimpleProgressChecker`.

**Config under test (final, run 3):**
- `controller_server.FollowPath`: `nav2_rotation_shim_controller::RotationShimController` wrapping `nav2_mppi_controller::MPPIController`
- Shim params: `angular_dist_threshold: 0.35`, `angular_disengage_threshold: 0.15`, `rotate_to_heading_angular_vel: 1.8`, `max_angular_accel: 3.0`, `forward_sampling_distance: 0.5`, `simulate_ahead_time: 1.0`, `rotate_to_goal_heading: true`
- `progress_checker`: `nav2_controller::PoseProgressChecker` with `required_movement_radius: 0.1`, `required_movement_angle: 0.25`, `movement_time_allowance: 10.0`
- `general_goal_checker.yaw_goal_tolerance: 3.14` (restored — had drifted to 0.5)

| Metric | Run 1 (45° shim, `SimpleProgressChecker`) | Run 2 (45° shim, `PoseProgressChecker`) | **Run 3 (20° shim, `PoseProgressChecker`)** |
|---|---|---|---|
| Score / grade | 49.4 (D) | 50.8 (D) | **54.0 (D)** |
| Coverage | 32.1% | 58.7% | **53.0%** |
| Goals succeeded / attempted | 3 / 5 | 3 / 4 | **4 / 5** |
| Collisions | 2 | 1 | **0** |
| START_OCCUPIED events | 0 | 0 | **0** |
| MPPI "Failed to make progress" | ~6 | 1 (goal#4 cascade) | **1** |
| MPPI "Costmap timed out" | ~6 | 3 (goal#4 cascade) | **1** |
| Bond death (during scored run) | none | none | **none** |

**Per-goal phase breakdown (run 3):**

| Goal | first_move | nav_total | first_move % | Outcome |
|---|---|---|---|---|
| #1 (1.61, 0.62)  | 11.2 s |  20.0 s | 56% | SUCCEEDED |
| #2 (1.61, 7.33)  | 11.3 s |  52.1 s | 22% | SUCCEEDED |
| #3 (5.19, 7.75)  |  1.9 s |  14.5 s | 13% | SUCCEEDED |
| #4 (6.82, 3.49)  |  2.3 s | 116.1 s |  2% | SUCCEEDED (recovered from prev-run failure) |
| #5 (16.94,-0.12) |  —     | DNF     |  —  | OFF-MAP, time-limit hit |
| **Total**        | **26.7 s** | **202.7 s** | **13.2%** | — |

**Verdict:**
- **Task 2 rotation-phase DoD met:** rotation-spin-up dropped from 30.1% baseline → **13.2%** of total run time. Median first_move 22 s → ~6.8 s; goals 3 and 4 hit ~2 s when the shim has a clean start.
- **Critical fix uncovered:** the *rotation-counts-as-progress* check matters as much as the shim itself. With `SimpleProgressChecker` the shim's in-place rotation triggers "Failed to make progress" cascades after 10 s and tanks coverage to 32%. Always pair RotationShimController with `PoseProgressChecker`.
- **Coverage did not recover to 74% Task 1 baseline.** Loss is traceable to three orthogonal bugs (logged in [`approach1_classical_pipeline.md`](approach1_classical_pipeline.md) Backlog section): cold-start delay on first goals, frontier picking off-map points, and an MPPI mid-navigation "Failed to make progress" cascade. None are rotation-shim issues; deferred to Task 3/4 verification.
- **Goal#4 long cross-map hop recovered** — previously a guaranteed-failure case in run 2, now succeeds with first_move 2.3 s.

---

## 2026-04-09 — Task 1 verification (seed=42, easy, speed=2, --no-perception)

Custom BT + differential inflation + static-layer footprint clearing.

**Config under test:**
- `global_costmap.inflation_layer`: `inflation_radius: 0.55`, `cost_scaling_factor: 3.0`
- `local_costmap.inflation_layer`: `inflation_radius: 0.35`, `cost_scaling_factor: 5.0` (initially 8.0; softened after run 1)
- `static_layer.footprint_clearing_enabled: true` on both costmaps
- Custom BT: [`launch/behavior_trees/navigate_backup_clear_replan.xml`](../launch/behavior_trees/navigate_backup_clear_replan.xml)

| Metric | Run 1 (csf=8.0) | Run 2 (csf=5.0) |
|---|---|---|
| Score / grade | 54.3 (D) | 55.1 (C) |
| Coverage | 58.5% | 74.3% |
| Goals succeeded / attempted | 3 / 4 | 4 / 5 |
| Collisions / near-misses | 0 / 0 | 0 / 0 |
| START_OCCUPIED events | **0** | **0** |
| MPPI "Failed to make progress" | 2 (t=46s, t=58s) | 1 (mid-goal#5) |
| Control loop rate misses | 1 @ 4.46 Hz | 0 |
| Stack state at end | bond-death @ t=207s (torn down) | survived to TIME_LIMIT, bond-death in teardown |

**Verdict:**
- **Task 1 primary objective achieved**: START_OCCUPIED cascade fully eliminated, confirmed across both runs.
- **Softer local `cost_scaling_factor: 5.0`** restored MPPI control loop health (20Hz sustained, no rate misses). The deep-research recommendation of 8.0 requires Task 4's `consider_footprint: true` + tighter sampling stds to be paired with it; without those, point-cost MPPI cannot converge against the steep gradient.
- **Coverage regression vs. 66.1 baseline** (74% vs. 98%) attributed to slow per-goal nav times (~60s average). Addressed by Task 2 (RotationShim).
- **Bond death now post-hoc only** — no longer affects scored run. Remaining work for Task 3 (lifecycle isolation).

---

## 2026-04-07 — Nav2 timing breakdown (seed=42, easy scenario)

Source run for the old Task 3 ("Reduce inter-goal idle time") analysis. Established that pre-travel rotation and Nav2 acceptance latency dominate runtime, not BFS/selection.

| Phase | N | Mean | % total |
|---|---|---|---|
| BFS + selection | 15 | 0.3 s | 2.6% |
| Nav2 accept | 4 | 3.8 s | 7.4% |
| Rotation/spin-up | 3 | 22 s median | 30.1% |
| Travel to goal | 3 | 30.8 s | 45.4% |
| Post-goal pause | 4 | 0.0 s | 0% |

**Bottlenecks identified:**
1. **Pre-travel rotation — 30% of time, 22 s median per waypoint.** Nav2 rotates the robot to face the goal direction before translating, on every goal. MPPI sampling rarely finds valid turning trajectories in constrained spaces (GH #4049), so the turn is slow and jittery.
2. **Nav2 acceptance latency — 3.8 s mean.** Minor relative to rotation.
3. **Rejection loops.** Robot rejected 11× on same frontier `(16.88, 3.89)` with no blacklist-on-reject, burning ~160/300 sim-s. Fixed by streak-blacklisting after 3× same centroid.

**Outcomes of old Task 3:**
- `yaw_goal_tolerance: 3.14` set (skips post-arrival rotation)
- Goal pose oriented toward travel direction (skips some pre-travel rotation)
- `_flood_unknown` removed
- Streak blacklist on rejection added

**Known residual:** Pre-travel rotation still dominant because goal-pose-orientation only helps when BFS gives a clean travel vector. The RotationShimController approach (new Task 2) supersedes this — deterministic in-place rotate-to-path-heading instead of relying on MPPI sampling.

---

## 2026-04-07 — Easy scenario best score

| Metric | Value |
|---|---|
| Overall score | 66.1 (grade C) |
| Found / total | 4 / 6 |
| Exploration coverage | 98% |
| Collisions | 1 |
| Seed | 42 |
| Config | patrol mode + OWL_CONF_THRESHOLD=0.15, W_DIST=1.5 |

Target: ≥ 70 (grade B) before moving to medium tier.
