# ROADMAP — derpbot-explorer

Table of contents for the issue tracker. Each active task gets a short entry here with a link to the GitHub issue where the full plan, root cause, and discussion live. Do not copy issue content into this file.

Current state lives in [`STATE.md`](STATE.md). History lives in closed issues + commits + [`benchmark_results.md`](benchmark_results.md).

---

## Ground rules (apply to all upcoming work)

- **Preserve Task 4's nav baseline:** 0 collisions, ≥ 70% coverage, 5+/6 goals on easy/seed=42/`--no-perception`. Any change that regresses these without a compensating gain gets reverted.
- **No dead-end retries.** `gh issue list --state closed --label dead-end` before proposing a change in an area that has prior attempts.
- **No hardcoded class names, scenario-specific logic, or oracle mode.** Generic robot goal — see [`../CLAUDE.md`](../CLAUDE.md).

---

## Next

### Task 6a — Profile per-goal time budget · [#16](https://github.com/thaije/derpbot-explorer/issues/16)
Timeline profiler in frontier_explorer: every sim-second accounted for, zero "untracked". **✅ Done** — profile infrastructure validated, seed=55 baseline captured. See [benchmark_results.md](benchmark_results.md).

### Task 6 — 2× average speed · [#17](https://github.com/thaije/derpbot-explorer/issues/17)
Double avg_speed from ~0.07 → ~0.14 m/s by reducing downtime. Profiling shows robot moving only 50% of the time; biggest levers: Nav2 dispatch overhead (13.6%), inter-goal loop stalls (11.9%), rotation (9.4%).
**Blocked on:** Task 6a ✅
**DoD:** `avg_speed_kmh` ≥ 0.50 on ≥ 3 seeds, 0 collision regression.

### Task 5 — Detection-aware exploration · [#8](https://github.com/thaije/derpbot-explorer/issues/8)
Revisit partially-detected areas so the detection rate exceeds the pure-coverage ceiling. Nav is now good enough (Task 4: 71% coverage, 0 collisions); score is gated by perception. Next big lever.
**Blocked on:** Tasks 1–4 ✅
**DoD:** confirmed detections up vs Task 4 baseline, no FP increase, target score ≥ 70 B.

---

## Later

Titles only. Expand when a task is promoted to "Next".

- **Medium tier scenario** — once easy ≥ 70 B.
- **Hard tier scenario** — once medium is stable.

---

## Open backlog

Known issues not currently prioritized. Full details in the linked issues; check before starting related work in case one is already tracked.

- [#9](https://github.com/thaije/derpbot-explorer/issues/9) — Cold-start delay ~11 s on goals 1–2 (first_move)
- [#10](https://github.com/thaije/derpbot-explorer/issues/10) — Frontier explorer picks off-map points
- [#11](https://github.com/thaije/derpbot-explorer/issues/11) — MPPI "Failed to make progress" cascade mid-nav (may be fixed by Task 4)
- [#13](https://github.com/thaije/derpbot-explorer/issues/13) — Enable `collision_monitor` realtime priority (needs rtprio grant)
- [#14](https://github.com/thaije/derpbot-explorer/issues/14) — Client-side goal pre-validation via `PyCostmap2D`
- [#15](https://github.com/thaije/derpbot-explorer/issues/15) — Use `ClearCostmapAroundPose` (Jazzy) for targeted clearing

Run `gh issue list --state open --label backlog` for the live list.

---

## Potential future extensions

Not tracked as issues — reconsider only if triggered.

- **IMU-fused odometry (robot_localization EKF).** If odometry angular accuracy degrades (different sim versions, real hardware), add an EKF fusing `/derpbot_0/imu` (100 Hz gyro) with `/derpbot_0/odom`. Publishes `/odom_fused`; point `slam_toolbox`'s `odom_frame` at it. ~20 lines of YAML + one launch node.

---

## Workflow

- **Starting a task:** read `STATE.md`, `ROADMAP.md`, and the task's issue. Check closed dead-ends in the same area.
- **During a task:** log findings and decisions as comments on the issue, not in this doc.
- **New finding:** `gh issue create` with `task` / `bug` / `dead-end` / `backlog` / `upstream` label. Cross-link related issues.
- **Completing a task:** close the issue with a final comment (outcome + commit SHA). Delete the task entry from the "Next" section here. Update `STATE.md` only if a new *invariant* came out of it.
- **Commits:** reference the issue, e.g. `feat(nav2): tighter MPPI sampling (#4)`. GitHub auto-links.
