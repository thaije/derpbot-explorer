# Agent prompt — issue #23: detection/confirmation cycle too slow

**Task: fix the detection/confirmation bottleneck for Task 6 (issue #23)**

**Goal.** Minimize the speed penalty from running perception. The `--no-perception` (no-subscribers) baseline is **0.404 km/h** — that's the ceiling. Current perception-on best is **0.28 km/h**, so perception is costing ~31% of speed. The goal is to close that gap as much as possible without degrading detection quality. Pragmatic pass line: **≥ 0.35 km/h** on ≥ 3 seeds, which recovers roughly half the overhead. This is the primary remaining blocker for Task 6 (#17).

**Reference run.** The canonical profile is `results/profile_20260429T233217.md` (seed=42, 0.28 km/h). The budget breakdown is:

| Phase | Time | % budget |
|---|---|---|
| traveling | 119.5 s | 38.9% |
| waiting | 77.9 s | 25.3% |
| rotating | 29.0 s | 9.4% |
| nav2_send | 28.3 s | 9.2% |
| frontier_select | 19.9 s | 6.5% |

The `waiting` phase (25.3%) is the largest single lever. Goal 2 alone accounts for 63.8 s of it (42.7 s + 21.0 s in two back-to-back `waiting` stretches), and the robot is barely moving during these (vx ≈ 0.03–0.05 m/s). This is what issue #23 attributes to the detection/confirmation cycle. **Before implementing anything, diagnose what actually causes these `waiting` stretches.**

**Critical pre-work: find the actual cause.** Read `agent/frontier_explorer.py` (the `_send_goal_and_wait` method and its sub-phase classifier), `agent/tracker.py` (`_process_loop`, `_publish_confirmed`), and `agent/agent_node.py`. Note: the explore loop in `frontier_explorer.py` does not contain any explicit pause triggered by detections — the tracker runs in its own thread and publishes independently. If the `waiting` phase is not detection-driven, the issue title is misleading and the real cause (MPPI slow-path, obstacle avoidance, remaining GIL, something else) must be identified before picking an approach. Check closed issues (`gh issue list --state closed --label dead-end`) for prior attempts in the nav/detection area.

**Confirmed facts to build on:**
- Tracker: `MIN_SIGHTINGS = 2`, `MIN_POSE_DISTANCE = 0.2 m`. A second sighting only counts if the robot moves ≥ 0.2 m between frames. So while the robot is standing still (or nearly so), no new diverse sightings accumulate.
- Detector runs at 5 Hz in a `multiprocessing` subprocess (spawn). The main GIL fix from #22 was moving BFS to a `ProcessPoolExecutor`. The tracker's `_process_loop` still runs in a Python thread on the ROS executor and calls `queue.get(timeout=0.5)` in a loop — check whether this thread is still competing for the GIL with the explore loop during the `waiting` phases.
- `frontier_select` for goal 2 was 11.7 s (vs ≤ 5 s for goals 1, 3, 4). Investigate whether the BFS ProcessPoolExecutor is serialising correctly or whether goal 2's cluster was genuinely harder.

**Approaches (try in order of invasiveness):**
1. **Diagnose `waiting` first.** Add temporary logging to confirm whether `waiting` correlates with active detection callbacks. If GIL is still the culprit for `waiting`, move the tracker's `_process_loop` to its own daemon process (same subprocess pattern as the detector) rather than a thread. This is the least risky structural change.
2. **Confirm while moving.** If `waiting` is caused by something else and the robot genuinely pauses near detected objects, decouple detection acknowledgement from the nav loop. The explore loop should never yield for a detection event; the tracker already accumulates sightings passively. Check whether `_publish_confirmed` or anything downstream signals the explore loop to pause.
3. **Do not reduce `OWL_CONF_THRESHOLD` below 0.15.** That value is empirically tuned (0.15 = 1 FP / 4 real vs 0.12 = 3 FP / 3 real). Don't change it without a new mechanism to suppress FPs.
4. **Do not skip multi-sighting confirmation.** `MIN_SIGHTINGS = 2` filters single-frame noise — removing it risks FP inflation. `MIN_POSE_DISTANCE = 0.2 m` is already tight; don't lower it further without ≥ 3 seeds data.

**Verification.** Run with the `arst-runner` agent — never start the stack manually. Run ≥ 2 seeds (seed=42 + seed=43 minimum) against the pre-change baseline and the fix. For each run check:
1. `avg_speed_kmh` trend toward ≥ 0.35, and report the gap to the 0.404 km/h no-subscribers ceiling explicitly.
2. `collision_count = 0` — hard stop on any regression.
3. Profile (`python3.12 scripts/profile_run.py`) — confirm `waiting` phase has shrunk; report the new budget table.
4. `found_ratio` — ≥ 2/6 objects must still be found (detection must not regress).

**DoD.** `avg_speed_kmh ≥ 0.35` on ≥ 3 seeds, 0 collisions, ≥ 2/6 objects found. Report the remaining gap to the 0.404 km/h no-subscribers ceiling. Append results to `docs/benchmark_results.md`. Close issue #23 with a comment: root cause, fix summary, commit SHA. Commit referencing the issue: `perf(agent): <description> (#23)`.
