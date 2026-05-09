# ROADMAP — derpbot-explorer

Table of contents for the issue tracker. Each active task gets a short entry here with a link to the GitHub issue where the full plan, root cause, and discussion live. Do not copy issue content into this file.

Current state lives in [`STATE.md`](STATE.md). History lives in closed issues + commits + [`benchmark_results.md`](benchmark_results.md).

---

## Ground rules (apply to all upcoming work)

- **Preserve Task 4's nav baseline:** 0 collisions, ≥ 70% coverage, 5+/6 goals on easy/seed=42/`--no-perception`. Any change that regresses these without a compensating gain gets reverted.
- **No dead-end retries.** `gh issue list --state closed --label dead-end` before proposing a change in an area that has prior attempts.
- **No hardcoded class names, scenario-specific logic, or oracle mode.** Generic robot goal — see [`../CLAUDE.md`](../CLAUDE.md).

---

## Completed

- **Task 8 — Automated benchmark submission · #33** — Benchmark v1 submitted and on leaderboard. Results: https://github.com/thaije/robot-sandbox/tree/main/results/submissions/derpbot-explorer-v1
- **Task 6 — 2× average speed · #17** — Closed as not feasible. Profiling and GIL probes (#22) brought speed from ~0.07 to ~0.24 km/h but 0.50 km/h DoD is structurally unachievable with current sim/architecture.

---

## Next

### Task 5 — Detection-aware exploration · [#8](https://github.com/thaije/derpbot-explorer/issues/8)
Revisit partially-detected areas so the detection rate exceeds the pure-coverage ceiling. Nav is now good enough (Task 4: 71% coverage, 0 collisions); score is gated by perception.
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

- [#30](https://github.com/thaije/derpbot-explorer/issues/30) — Reduce global inflation radius — patrol bot blocks corridor
- [#15](https://github.com/thaije/derpbot-explorer/issues/15) — Use ClearCostmapAroundPose (Jazzy) for targeted clearing
- [#14](https://github.com/thaije/derpbot-explorer/issues/14) — Client-side goal pre-validation via PyCostmap2D
- [#13](https://github.com/thaije/derpbot-explorer/issues/13) — Harden: enable collision_monitor use_realtime_priority

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