# Benchmark Results

Historical performance snapshots. Append new entries on top; keep older ones for regression comparison.

---

## 2026-04-29 — Perception vs speed tradeoff (seed 42, easy, speed=2 vs 1)

Four runs isolating the effect of perception on Task 6 speed progress. The April 23 baseline showed no-perception mean 0.314 km/h, closing toward the 0.50 DoD target. These runs show **perception overhead negates ~60% of that speed gain**.

| Config | Score | Grade | avg_speed_kmh | Coverage | Found | Collisions | Meters |
|---|---|---|---|---|---|---|---|
| No-perc, spd=2 (Apr 23 baseline) | 52.5 | D | 0.327 | 62.3% | 0/6 | 0 | 27.4 |
| No-perc, spd=2 (today) | 47.8 | D | **0.4** | 64.9% | 0/6 | 1 | 33.4 |
| **Perc ON, spd=2** | **62.9** | **C** | **0.197** | 51.2% | **2/6** | 0 | 16.6 |
| **Perc ON, spd=1** | **63.1** | **C** | **0.155** | 50.0% | **2/6** | 0 | 12.9 |

**Key findings:**
- **Speed progress wiped by perception:** no-perception runs hit 0.4 km/h (+29% vs Apr 23 baseline), but enabling perception drops to 0.155–0.197 km/h — back to square one for Task 6.
- **Perception still raises overall score** (+15 points, 47.8→63.1) despite halved travel distance — finding 2/6 objects outweighs coverage loss.
- **RTF (speed=1 vs 2) makes no meaningful difference** with perception (63.1 vs 62.9) — bottleneck is exploration logic, not sim compute.
- **Perception cuts meters by ~60%** (33.4→12.9 m) — detector/confirmation loops dominate cycle time.
- **Task 6 DoD (≥0.50 km/h) impossible with current perception pipeline.** Need to optimize detection-aware exploration or run perception asynchronously.

**Verdict:** The 0.314→0.4 km/h speed progress (no-perception) is real, but the perception pipeline's time cost (~0.16 km/h) negates it entirely. See #22 for GIL probe results.

**GIL Probes (#22) completed:**
- Probe 1: GIL confirmed — BFS 79s (25.9%) → 3.1s (1.0%) with subscribers disabled
- Probe 2: ProcessPoolExecutor — BFS 15.1s → 5.7s mean (partial fix, IPC overhead)
- Probe 3: Rate limiter (5 Hz) — modest help (15.1s → 12.5s)
- Probe 4: Callback-based nav2_send — 15% → 9.2% budget (28.3s total)

**Best combo:** ProcessPoolExecutor + callbacks → BFS ~5.7s + nav2_send ~7.1s per goal

---

## 2026-04-29 — GIL Probes Summary (#22)

| Config | BFS total | BFS mean | nav2_send total | Score | Speed (km/h) |
|---|---|---|---|---|---|
| No-perception (Apr 23) | 2.7s (0.9%) | 0.4s | ~3s (1%) | 52.5 D | 0.327 |
| Perception ON (before probes) | 79s (25.9%) | 15.1s | ~50s (15%) | 62.9 C | 0.197 |
| **No-subscribers (Probe 1)** | **3.1s (1.0%)** | **0.6s** | ~3s (1%) | 52.0 D | **0.404** |
| **+ ProcessPoolExecutor (Probe 2)** | **22.9s (7.5%)** | **5.7s** | ~50s (15%) | 59.0 C | 0.155 |
| **+ Callback nav2_send (Probe 4)** | 11.6s (3.8%) | 2.9s | **28.3s (9.2%)** | 66.1 C | 0.077 |

**Key finding:** GIL contention confirmed as primary bottleneck. Best speed (0.404 km/h) achieved with subscribers disabled. Combining ProcessPoolExecutor + callbacks should give ~5-7s BFS + ~7s nav2_send per goal.

---

## 2026-04-23 — Task 6 speed baseline (current master, #17, easy, 3 seeds, --no-perception)

Establishes the Task 6 baseline after #17 (sleep tightening), #18 (startup pause), #10 (lethal costmap filter), #19 (profiler race fix) all landed. Profile writing fixed — cleanup.sh now sends SIGINT before tmux kill so Python finally block runs.

| Seed | Score | avg_speed_kmh | Coverage | Collisions | Meters | Profile |
|---|---|---|---|---|---|---|
| 42 | 52.5 D | 0.327 | 62.3% | 0 | 27.4 m | `profile_20260423T222714.md` |
| 55 | 51.2 D | 0.246 | 52.3% | 0 | 20.6 m | `profile_20260423T224509.md` |
| 99 | 51.4 D | 0.368 | 60.9% | 0 | 31.0 m | `profile_20260423T224944.md` |
| **mean** | **51.7 D** | **0.314** | **58.5%** | **0** | **26.3 m** | |

**DoD: avg_speed_kmh ≥ 0.50 on ≥ 3 seeds. Current mean: 0.314 → need +59%.**

**Time budget across seeds (profiles):**

| Phase | seed=42 | seed=55 | seed=99 | mean % |
|---|---|---|---|---|
| traveling | 61.0% | 46.8% | 67.5% | 58.4% |
| **rotating** | **26.3%** | **25.2%** | **18.6%** | **23.4%** |
| startup | 5.4% | 5.7% | 5.2% | 5.4% |
| waiting | 4.6% | 13.4% | 4.3% | 7.4% |
| bfs_detect | 0.2% | 3.6% | 0.9% | 1.6% |
| other | 2.5% | 5.3% | 4.1% | 3.9% |

**Moving speed: 0.134–0.150 m/s mean** across seeds (well below 0.5 m/s max).

**Key findings:**
- `rotating` is the consistently dominant non-travel overhead (18–26% across all seeds) — primary lever for Task 6.
- seed=55 was badly hurt by 2/4 goal failures (stuck-detect) producing 13.4% waiting; #9 first-move pattern still present (goal#1: 33.3 s rotating, goal#2: 27.2 s rotating, both failed).
- Moving speed uniformly low (~0.147 m/s). MPPI not using the 0.5 m/s max — secondary lever.
- Startup 5.2–5.7% (~16 s) still visible; the #18 fix shifted it from ~57 s but bringup settle still costs sim-time.

**Levers ranked by impact (from profiles):**
1. Fix `rotating` overhead (23% mean → target ≤ 10%): eliminate #9 first-move stalls; tune RotationShim thresholds
2. Increase moving speed (0.147 → 0.20+ m/s): tune MPPI temperature / reduce CostCritic weight in open corridors
3. Reduce goal failures on bad seeds (seed=55 had 2/4 failed, eating waiting budget)

---

## 2026-04-14 — Task 6 sleep-tightening validation (#17, easy, speed=2, perception ON)

Per-seed 2×2 A/B measuring inter-goal sleep reductions in `frontier_explorer.py`:
- `post_goal_sleep` (after failure/recovery): 3.0 → 1.0 sim-s
- `rejection_sleep` (after Nav2 reject): 5.0 → 1.0 sim-s
- first-map wall-poll: 1.0 → 0.1 s

| Seed | Variant | Score | avg_speed_kmh | Coverage | Found | Collisions | Meters |
|---|---|---|---|---|---|---|---|
| 42 | baseline (8ddc165) | 51.2 D | 0.194 | 34.2% | 2/6 | 0 | 16.3 |
| 42 | change | 53.6 D | **0.247** (+27%) | 39.4% | 3/6 | 1 | 20.8 |
| 55 | baseline (8ddc165) | 59.0 C | 0.166 | 47.8% | 2/6 | 0 | 13.9 |
| 55 | change | 51.2 D | **0.246** (+48%) | 62.9% | 1/6 | 0 | 20.7 |

**Task 6 DoD metric (`avg_speed_kmh`):** 0.180 → 0.247 mean (+37%), improved on both seeds. Still below 0.50 target.
**Collateral:** coverage +10pp, meters +37%, no collision regression in aggregate. Overall score dipped −2.7 mean, driven entirely by accuracy/detection variance (2/6 → 2/6 mean, but swings ±1 across seeds) — orthogonal to the timing changes.

**Context:** Current master baseline is roughly ~55 D mean, far from the historical 66.1 C (2026-04-07, seed=42, 98% coverage). Something between then and now regressed coverage sharply (34–48% vs 98%); the sleep-tightening is a small step, not a return to baseline. Investigation of rotation/startup/goal-2-failure still owed.

---

## 2026-04-12 — Task 6a profiling run (seed=55, easy, speed=2, perception ON)

First run with timeline profiler instrumentation (#16). Validates profiling infrastructure and establishes the time-budget baseline for Task 6 (#17).

| Metric | Value |
|---|---|
| Score / grade | **63.8 C** |
| Found | 2/6 — FE#2 @ t=123.6 s, Person @ t=141.4 s |
| Coverage | 64.7% |
| Collisions | 0 |
| `meters_traveled` | 20.6 m |
| `avg_speed_kmh` | 0.244 |

**Time budget (259.4 sim-s tracked, 100% accounted):**

| Phase | % | Time (s) | Notes |
|---|---|---|---|
| **traveling** (linear motion) | **49.8%** | 129.3 | avg vx=0.174 m/s — well below vx_max 0.5 |
| nav2_send (dispatch → accept) | 13.6% | 35.2 | 7.0 s mean/goal; includes GIL stalls |
| goal_reached (result → next BFS) | 11.9% | 30.8 | pure loop overhead + GIL contention |
| **rotating** (RotationShim) | **9.4%** | 24.3 | goal#2: 20.4 s rotation; goal#3: 4.0 s |
| recovery (backup + clear) | 5.4% | 14.0 | 1 failure; wall-clock deadlines inflate at RTF > 1 |
| nav2_accepted → first motion | 2.9% | 7.5 | 1.5 s mean — planner + BT startup |
| BFS + frontier selection | 4.7% | 12.4 | reasonable |
| other (stuck, sleep, failed) | 2.3% | 6.0 | — |

**Key findings:**
- **Robot moving only 49.8% of the time.** The other 50% is overhead: goal dispatch (16.5%), inter-goal loop stalls (11.9%), rotation (9.4%), recovery (5.4%).
- **Moving speed 0.154 m/s** — MPPI is conservative. CostCritic + corridor inflation keep average well below 0.5 m/s max.
- **GIL contention is a real tax.** The 4-thread MultiThreadedExecutor processing high-frequency callbacks (odom 20 Hz, TF 30+ Hz) preempts the explorer thread. `nav2_send` and `goal_reached` phases include 5–10 s of pure scheduling waste per occurrence.
- **post_goal_sleep shows 0.0 s** — the `time.sleep → _sim_sleep` fix is working; 3 sim-s sleep completed in ~1.5 wall-s at RTF 2.0.

**Levers for Task 6 (2× speed target):**
1. Reduce nav2 dispatch overhead (35.2 → <10 s) — investigate bt_navigator acceptance latency
2. Reduce inter-goal loop stalls (30.8 → <5 s) — GIL optimization or reduce executor thread count
3. Faster rotation (24.3 → <10 s) — tune `rotate_to_heading_angular_vel` or reduce `angular_dist_threshold`
4. Increase moving speed (0.154 → 0.20+ m/s) — tune MPPI temperature, reduce CostCritic weight in open areas

Profile: `results/profile_20260412T173435.md` · Results: `robot-sandbox/results/office_easy_001_20260412T173145.json`

---

## 2026-04-10 — Fresh-seed perception runs (seeds 123, 7, 99, easy, speed=2, perception ON)

Three scored runs with full perception after all Task 1–4 fixes landed, to test generalization beyond seed=42. seed=99 also verifies the issue #12 fix (`meters_traveled` / `avg_speed_kmh`).

### Four-seed summary

| Seed | Score | Grade | Found | Coverage | Collisions | `m_traveled` | `avg_kmh` | Notes |
|---|---|---|---|---|---|---|---|---|
| 42 (old) | 66.1 | C | 4/6 | 98% | 1 | — | — | historical best |
| **123** | **43.0** | **D** | 2/6 | 48.0% | 2 | 0.0 ❌ | 0.0 ❌ | stuck in (2.5, 7.1) corridor, 2 collisions there |
| **7**   | **57.3** | **C** | 1/6 | 34.8% | 0 | 4.9 ❌ | 0.058 ❌ | stuck goals #2 & #3 early; safety=S rescued score |
| **99**  | **66.6** | **C** | 4/6 | 44.6% | 0 | **21.7 ✅** | **0.26 ✅** | issue #12 fix verified; ties seed=42 best |

Four-seed averages: score ≈ 58.3, found ≈ 2.75/6 (46%), coverage ≈ 56%, collisions ≈ 0.75.
**seed=42's score is no longer an outlier — seed=99 matched it.** The spread (43.0 to 66.6) is driven by early stuck events on "bad" seeds, not by a ceiling regression. Any Task 5 delta must still be measured across ≥ 3 seeds.

### seed=99 run (office_easy_001_20260410T212000.json) — issue #12 fix verified

| Metric | Value |
|---|---|
| Score / grade | **66.6 C** |
| Found | **4/6** — FA#1 @ t=33.1 s, FE#2 @ t=58.0 s, FA#2 @ t=134.8 s, Person @ t=137.7 s. Missed: FE#1 (far-right Office B), FE#3 (top-right meeting room) |
| Coverage | 44.6% |
| Collisions | **0** |
| False positives / duplicates | 2 / 1 |
| Mean loc. error | 0.28 m |
| Completion time | 300.6 s (TIME_LIMIT) |
| Category — speed / accuracy / safety / efficiency / effectiveness | 21.2 F / 66.7 C / 100 S / 78.6 B / 63.3 C |
| **`meters_traveled`** | **21.684 m ✅** (was ~0–5 m on prior seeds) |
| **`avg_speed_kmh`** | **0.26 km/h ✅** (was ~0–0.06 km/h) |

**Issue #12 fix verification:** cross-checked against 12 pose snapshots from `world_state.py` during the monitoring loop. Straight-line sum between consecutive snapshots ≈ 18.2 m (lower bound — misses curves, rotations, backups). Reported 21.7 m is fully consistent with that. Fix confirmed — close #12.

**Behavior:** RTF 1.73–1.99 stable. No stuck events, no bond deaths, no lifecycle cascades, no TF flood, no aborts. Two first_move outliers: goal#2 @ 15.7 s, goal#3 @ 26.2 s — same pattern as prior runs, worth noting as a residual lever for Task 5 or a separate backlog issue.


### seed=7 run (office_easy_001_20260410T205244.json)

| Metric | Value |
|---|---|
| Score / grade | **57.3 C** |
| Found | **1/6** (fire_extinguisher #3 @ t=54 s, loc_err 1.10 m) |
| Coverage | **34.8%** |
| Collisions | **0** |
| False positives | 0 (precision 1.0) |
| Mean loc. error | 1.10 m |
| Completion time | 303.3 s (TIME_LIMIT) |
| Category — speed | 21.0 F |
| Category — accuracy | 50.0 D |
| Category — safety | 100.0 S |
| Category — efficiency | 100.0 S |
| Category — effectiveness | 20.8 F |
| `meters_traveled` (reported) | 4.921 — **clearly wrong**, ground-truth path > 10 m |
| `avg_speed_kmh` (reported) | 0.058 — **clearly wrong**, issue #12 |

**Failure mode:** robot stuck twice very early — goal#2 (1.41, 6.58) cancelled at ~37 s into nav, goal#3 (6.64, -0.61) cancelled at ~20 s into nav. Both triggered backup + clear-costmap recovery. ~70 s of the 300 s budget burned on recovery, leaving only ~150 s to explore; goal#5 dispatched at t=162 but never finished. Also 3× `first_move=n/a` (goals 1/2/3) and 1 first_move=12.2 s outlier on goal#4.

**Tracker noise:** persistent pending fire_extinguisher sightings at phantom locations (5.4, 4.1) and (3.3, 1.6) that never crossed the confirmation threshold. Detector firing on office clutter; projection/tracker didn't lock. Worth investigating as part of Task 5.

**Stack health:** 0 bond deaths, 0 lifecycle crashes, 0 collisions. Task 1–4 safety fixes held perfectly (safety S, efficiency S) — the bottleneck is entirely exploration velocity and early-mission stalls.

### seed=123 run (office_easy_001_20260410T203458.json)

| Metric | seed=42 baseline (old) | **seed=123** |
|---|---|---|
| Score / grade | 66.1 C | **43.0 D** |
| Found | 4 / 6 | **2 / 6** |
| Coverage | 98% | **48.0%** |
| Collisions | 1 | **2** (t=69.9 s, t=77.2 s) |
| False positives | — | 3 (precision 0.40) |
| Mean loc. error | — | 0.33 m |
| Completion time | — | 301.3 s (TIME_LIMIT) |
| Category — speed | — | 21.2 F |
| Category — accuracy | — | 36.0 F |
| Category — safety | — | 86.0 A |
| Category — efficiency | — | 0.0 F |
| Category — effectiveness | — | 42.8 D |

**Confirmed detections (scored):** person #1 @ t=34.5 s, first_aid_kit #2 @ t=51.8 s.
**Confirmed by tracker but NOT scored:** 3× fire_extinguisher at (5.2, 5.0), (5.4, 7.4), (8.8, 8.3) — likely gated by localization error (0.33 m) + MATCH_RADIUS. Worth investigating as its own lever.

**Stuck events / failure timeline:**
- goal#2 (1.40, 7.25): `first_move = 22.4 s` despite coming straight off a completed goal#1 (not a cold start). Controller silent for 22 s with an active goal. Then 114 s of nav ending in stuck-detect + backup recovery.
- Both collisions fired during/after the goal#2 stall in the (2.5, 7.1) doorway/corridor area — footprint-aware MPPI critic (Task 4) did not prevent them here.
- goal#3 (7.44, 5.35): never moved at all (`first_move = n/a`), cancelled after 34.8 s.
- Net effect: robot never reached the east/north half of the map; fire_extinguisher #2 at (12.9, 11.5) and #3 at (10.4, 1.7) never approached.

**Stack health (all ✅):**
- RTF sustained ~1.98, no TF flood.
- No bond deaths, no Nav2 lifecycle failures, no START_OCCUPIED, no BackUp cascades on the managed side.
- Safety lifecycle (Task 3) held through both collision events.

**Verdict:**
- **Regression on this seed, not the stack.** Task 1–4 fixes preserved lifecycle stability, but the failure mode is new: mid-mission first_move stall in a tight area, followed by collision-in-corridor. This is *not* the previously characterised goal#4 cold-start or START_OCCUPIED problem.
- **seed=42 baseline was optimistic.** First evidence that our "best easy" (66.1 C) may be seed-specific. Need 2–3 more seeds before trusting any Task 5 delta.
- **Confirmation gap is real.** 5 tracker-confirmed objects → only 2 scored. If that ratio holds on other seeds, detection-pipeline → scored-detection conversion is a bigger lever than raw exploration coverage for Task 5.

**Issue #12 (avg_speed_kmh bug) reproducer confirmed:** results JSON still reports `avg_speed_kmh: 0.0` and `meters_traveled: 0.0` this run — Tjalling monitoring.

**New findings to file (not filed yet, flagged for next pass):**
- Mid-mission goal#2 `first_move = 22.4 s` stall in tight corridor (distinct from #9 cold-start).
- 2 collisions in (2.5, 7.1) area despite footprint-aware MPPI — Task 4 doesn't fully cover every tight passage.
- `world_state.py --no-ros --results` crashes with `KeyError: 'class_id'` — minor tooling bug.
- Tracker-confirmed-but-unscored detection gap (localization-error gated?).

Results JSON: `~/Projects/robot-sandbox/results/office_easy_001_20260410T203458.json`

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
- **Coverage did not recover to 74% Task 1 baseline.** Loss is traceable to three orthogonal bugs: cold-start delay on first goals ([#9](https://github.com/thaije/derpbot-explorer/issues/9)), frontier picking off-map points ([#10](https://github.com/thaije/derpbot-explorer/issues/10)), and an MPPI mid-navigation "Failed to make progress" cascade ([#11](https://github.com/thaije/derpbot-explorer/issues/11)). None are rotation-shim issues.
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
