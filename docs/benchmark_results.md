# Benchmark Results

Historical performance snapshots. Append new entries on top; keep older ones for regression comparison.

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
