# Benchmark Results

Historical performance snapshots. Append new entries on top; keep older ones for regression comparison.



## 2026-05-16 — Easy-seed sweep: bumper sensor + detection skip + spin speedup (#8)

5-seed easy benchmark (1 run per seed) with bumper-based stuck detection, confirmed-detection skip, and doubled spin recovery speed. Agent: `derpbot-explorer-v3-test1`.

| Seed | Score | Grade | Cov% | Found/6 | FP | Collisions | avg_speed | Detections |
|------|-------|-------|------|---------|----|------------|-----------|------------|
| 1 | 63.5 | C | 75.6% | 5/6 | 3 | 1 | 0.291 km/h | FE=3/3 FA=1/2 Person=1/1 |
| 2 | 65.8 | C | 97.0% | 4/6 | 1 | 0 | 0.291 km/h | FE=3/3 FA=0/2 Person=1/1 |
| 3 | 46.2 | D | 98.5% | 0/6 | 0 | 2 | 0.284 km/h | (detector silent) |
| 4 | 42.3 | D | 20.5% | 0/6 | 0 | 0 | — | (nav failure, barely moved) |
| 5 | 61.2 | C | 4/6 | 4/6 | 2 | 1 | 0.272 km/h | FE=2/3 FA=1/2 Person=1/1 |
| **Mean** | **55.8** | **C** | **72.8%** | **2.6/6** | **1.2** | **0.8** | | |

v2 baseline (5 seeds × 3 runs): mean 54.7 D, 64.1% cov, 2.5/6 found, 0.8 coll.

### False positive analysis (from submission_log)

| Seed | FP Class | Reported Pos | Nearest GT | GT Dist | Track ID | Diagnosis |
|------|----------|-------------|-----------|---------|----------|-----------|
| 1 | fire_extinguisher | (13.2, 5.7) | GT#1 FE @ (14.0, 3.3) | 2.57m | track_2 | Hallucination — too far, likely clutter |
| 1 | fire_extinguisher | (17.1, 3.1) | GT#1 FE @ (14.0, 3.3) | 3.19m | track_4 | Hallucination — edge of map, no FE nearby |
| 1 | first_aid_kit | (4.4, 3.6) | GT#4 FA @ (2.6, 3.2) | 1.92m | track_6 | Depth projector error — right class, pos off |
| 2 | fire_extinguisher | (10.4, 10.4) | GT#3 FE @ (2.7, 7.6) | 2.89m | track_3 | Hallucination — mirror/double of GT#3 |
| 5 | person | (9.4, 8.9) | GT#13 Person @ (9.0, 7.6) | 1.70m | track_3 | Depth projector error — right class, pos off |
| 5 | person | (9.6, 8.3) | GT#13 Person @ (9.0, 7.6) | 1.53m | track_3 | Duplicate — same track_id, shifted pos |

**FP pattern:** Two distinct causes — (a) OWLv2 hallucination (3/6 FPs, >2.5m from GT, wrong location), (b) depth projector localisation error (3/6 FPs, right class but 1.5–1.9m off). Note: track_3 on seed=5 appears twice — tracker re-confirming with shifted position; possible tracker dedup bug.

### Seed failure analysis

- **Seed 3 (0/6, 98.5% cov):** Detector silent — 0 submissions despite 98.5% coverage. Same pattern as issue #26. Nav/coverage fine.
- **Seed 4 (0/6, 20.5% cov):** Navigation failure — robot barely moved. Unrelated to new changes.

Results: `robot-sandbox/results/submissions/derpbot-explorer-v3-test1/`

Note: An earlier manual test run (same code, seed=1, before the benchmark script run) achieved 74.9 B with 6/6 found — see the separate entry below. The benchmark re-ran seed=1 and got 63.5 C (5/6 found), illustrating typical run-to-run variance.

---

## 2026-05-16 — Bumper sensor + detection skip + spin speedup (seed=1, easy, #8)

First SUCCESS completion. Bumper-based stuck detection (3s threshold on physical contact), confirmed-detection skip (pre-commit + en-route), Spin recovery speed doubled (max 1.0→2.0 rad/s, min 0.4→1.0 rad/s).

| Metric | Value |
|---|---|
| **Status** | **SUCCESS** |
| **Overall score / grade** | **74.9 B** |
| **Seed** | 1 |
| **Elapsed time** | 390.1 s (of 900 s) |
| **Found ratio** | **1.0 (6/6)** — FE#2 @ 32.9s, FA#1 @ 39.6s, FE#3 @ 130.8s, FA#2 @ 132.7s, Person @ 136.0s, FE#1 @ 200.7s |
| **Coverage** | 55.5% |
| **Collisions** | 0 |
| **Near-misses** | 0 |
| **False positives** | 5 |
| **Precision** | 0.5455 |
| **Mean loc error** | 0.45 m |
| **Avg speed** | 0.291 km/h |
| **Meters traveled** | 31.5 m |
| **Speed / Accuracy / Safety / Efficiency / Effectiveness** | 30.0 F / 81.8 B / 100.0 S / 66.7 C / 84.4 B |

v2 baseline for comparison (5 seeds × 3 runs): mean 54.7 D, 64.1% cov, 2.5/6 found, 0.8 coll. Task 5 initial (3 seeds): mean 52.1 D, 66.5% cov, 2.3/6 found, 0.3 coll.

**Assessment:** Single run, high variance — cannot conclude yet. But 6/6 detections is a clear signal that detection-aware exploration is working. 5 false positives remain a concern (precision 0.55). Coverage lower than baseline (55.5% vs 64.1%) — detection detours may substitute breadth for targeted visits. Need multi-seed A/B comparison.

Results: `robot-sandbox/results/office_easy_001_20260516T140452.json`

---

## 2026-05-12 — Task 5 initial test: detection-aware exploration (seeds 1–3, easy, full perception #8)

First test of detection-aware exploration (commit `beeebf0`). Pending candidates (1-sighting objects) are injected as high-priority pseudo-frontiers, causing the robot to detour toward partially-detected objects for re-detection. Compared against recalibrated v2 baseline (5 seeds × 3 runs, 15 runs total).

| Seed | Score | Grade | Cov% | Found/6 | FP | Detections | Collisions |
|------|-------|-------|------|---------|----|------------|------------|
| 1 | 53.6 | D | 52.5% | 3/6 | 2 | 3 | 1 |
| 2 | 41.6 | D | 52.5% | 1/6 | 4 | 1 | 0 |
| 3 | 61.2 | C | 94.4% | 3/6 | 2 | 3 | 0 |
| **Mean** | **52.1** | **D** | **66.5%** | **2.3/6** | **2.7** | **2.3** | **0.3** |

v2 baseline for comparison (5 seeds × 3 runs): mean 54.7 D, 64.1% cov, 2.5/6 found, 0.8 coll.

**Assessment:** Too few runs to conclude — range overlaps baseline. Seed=2 had 4 false positives from candidate detours. Need A/B comparison with `--no-detect-explore` and ≥2 runs per seed.

Results: `robot-sandbox/results/office_easy_001_20260512T{214104,215324,220741}.json`

---
