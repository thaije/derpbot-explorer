# Agent prompt — issue #21: tune goal-picker DIST_WEIGHT

**Task: tune goal-picker DIST_WEIGHT (issue #21)**

**Context.** `frontier_explorer.py` scores frontier clusters with a formula combining costmap cost and distance-to-centroid, weighted by `DIST_WEIGHT` (also referred to as `W_DIST` in docs). The current value was chosen by reasoning rather than empirical tuning. `STATE.md` documents that `W_DIST=1.5` balances coverage, ≥4.0 keeps the robot local, and ≤0.5 causes cross-map thrashing. There is also a `COSTMAP_LETHAL_THRESHOLD` (likely 99 — cells at or above this cost are excluded from frontier clusters). Both constants live in `agent/frontier_explorer.py` — check the current values before assuming anything, since the issue may have been filed before a prior change.

**Goal.** Confirm or update `DIST_WEIGHT` with multi-seed data, and check whether `COSTMAP_LETHAL_THRESHOLD=99` is ever rejecting valid frontier cells in narrow corridors. This feeds Task 6 (#17) by reducing inter-goal loop stalls from bad goal selection.

**Work.**
1. Read `agent/frontier_explorer.py` and confirm the current values of `DIST_WEIGHT` (or `W_DIST`) and `COSTMAP_LETHAL_THRESHOLD`.
2. Run a sweep of `DIST_WEIGHT` across [0.5, 1.0, 1.5, 2.0, 4.0] — or a subset if some values are already ruled out by the `STATE.md` bounds (never go below 1.0 without testing). Use seed=42 and seed=43 for each value. Use the `arst-runner` agent; never start the stack manually.
3. For each run record: `avg_speed_kmh`, coverage, goals reached, stuck events, and any logged "skipping inflated cell" or similar messages that suggest the threshold is rejecting valid cells.
4. Check whether `COSTMAP_LETHAL_THRESHOLD=99` is ever too aggressive: look for cases where the goal picker has few or no valid cells in a frontier cluster and falls back to a suboptimal choice.

**Constraints.**
- Do not go below `DIST_WEIGHT=1.0` without explicit justification from the data.
- Do not change `COSTMAP_LETHAL_THRESHOLD` unless the logs show it rejecting clearly valid cells — the 99 value is well-grounded (Nav2 inscribed-radius cost).
- ≥ 2 seeds per value before drawing conclusions.
- Check `gh issue list --state closed --label dead-end` for prior attempts in the goal-picker area (see dead-end #6 — do not re-introduce "prefer cells ≥0.25m from obstacles").

**DoD.** `DIST_WEIGHT` confirmed or updated with ≥ 3 seeds data. Add a short comment to the code constant with the tuned value and rationale. Close issue #21 with a comment: chosen value, data summary, commit SHA. Commit referencing the issue: `perf(explorer): tune DIST_WEIGHT to <value> (#21)`.
