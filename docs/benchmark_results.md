# Benchmark Results

Historical performance snapshots. Append new entries on top; keep older ones for regression comparison.

## 2026-10-10 — BISECT: conf-margin gate disabled (margin=0.0), easy, collapsed seeds 4,3,1 × 1 run

Agent: `derpbot-explorer-v4-bisect-nomargin` (commit 787736c — CONFIDENCE_MARGIN 0.05→0.0; STABILITY_RADIUS 1.2, VETO_RADIUS 0.8, AMBIGUITY_MARGIN 0.10, FAST_CONFIRM_* unchanged). Bisect of the v4-gated recall regression (t_e5736e7b: gated stack mean 3.1/6 vs pre-fix master 5.4/6; suspect: margin required conf ≥0.20, killing near-threshold real objects). Operator sequencing note: bisect verdict only — full verify batch (v5-nomargin 5×2) NOT run here; master-parity benchmark takes priority and the verify dispatches separately.

| Seed | Found/6 | TP-class recall (FE/FAK/person) | FP | Precision | Score | Grade | Cov% | Col |
|------|---------|-------------------------------|-----|-----------|-------|-------|------|-----|
| 4 | 4/6 | 1/3 + 2/2 + 1/1 | 1 | 0.80 | 64.5 | C | 98.7 | 0 |
| 3 | 2/6 | 0/3 + 1/2 + 1/1 | 3 | 0.40 | 42.1 | D | 82.2 | 3 |
| 1 | 4/6 | 1/3 + 2/2 + 1/1 | 1 | 0.80 | 58.7 | C | 62.2 | 1 |
| **Mean** | **3.33/6** | FE tp-recall 0.33 | **1.7** | **0.67** | **55.1** | **C** | **81.0%** | **1.3** |

Same-seed comparison vs the v4-antihallucination gated batch: seed4 r1 1/6→4/6, seed3 r2 1/6→(run r1 here) 2/6, seed1 4/6,4/6→4/6. Detected 10/18 total; fire_extinguisher remains the weakest class overall (detected 2/9 across the 3 runs, all misses; first_aid_kit 5/6, person 3/3).

**Verdict: FAIL the bisect threshold — mean found 3.33/6 < 4.5/6.** Margin gate removal recovers some recall vs the gated batch's collapsed seeds (gate-attributed r1s: seed4 1/6→4/6, seed3 1/6→2/6) but does NOT approach pre-fix master parity (5.4/6 mean): **stability/veto/fast-confirm gates also suppress recall** (fire_extinguisher tp-recall 0.33 across runs, seed3 0/3 — plausibly veto + stability holding near-threshold FE sightings). Per operator decision rule: STOP and report; next step is revert-gates-entirely vs geometry-based rework (operator decides; full nomargin verify not run).

Results: `robot-sandbox/results/office_easy_001_20261010T172006.json` (s4), `...T172831.json` (s3), `...T173653.json` (s1); copies under `results/submissions/derpbot-explorer-v4-bisect-nomargin/`.

---

## 2026-10-10 — Full re-benchmark: v4 anti-hallucination gate, easy, 5 seeds × 2 runs

Agent: `derpbot-explorer-v4-antihallucination` (commits a45b124→fbdc078: anti-hallucination gate + cross-class veto + fast-confirm + STABILITY_RADIUS 1.2m). Verification run for the ≥75%-completion gate (task t_e5736e7b); resumable re-run completed a batch reclaimed at 7/10 (seeds 1–3 identical data). Speed 2×, 900 s easy budget, all runs completed (0 failed, 0 DOA).

| Seed | Run | Score | Grade | Found/6 | Cov% | FP | Col |
|------|-----|-------|-------|---------|------|----|----|
| 1 | 1 | 53.0 | D | 4/6 | 70.9 | 2 | 3 |
| 1 | 2 | 58.6 | C | 4/6 | 96.8 | 2 | 2 |
| 2 | 1 | 56.5 | C | 3/6 | 98.7 | 2 | 0 |
| 2 | 2 | 55.3 | C | 4/6 | 98.9 | 2 | 4 |
| 3 | 1 | 51.5 | D | 4/6 | 100.0 | 2 | 6 |
| 3 | 2 | 35.5 | F | 1/6 | 99.3 | 4 | 4 |
| 4 | 1 | 39.6 | F | 1/6 | 94.5 | 4 | 0 |
| 4 | 2 | 55.2 | C | 3/6 | 98.7 | 0 | 1 |
| 5 | 1 | 66.5 | C | 5/6 | 98.7 | 2 | 0 |
| 5 | 2 | 49.6 | D | 2/6 | 98.3 | 3 | 0 |
| **Mean** | | **52.1** | **D** | **3.1/6** | **95.8%** | **2.3** | **2.0** |

**Verdict: FAIL — 0/10 runs reached found=6/6 (target ≥75%); mean found_ratio 0.517, per-seed any-run-completes 0/5 seeds.** 95% CI (bootstrap, run-level found_ratio, 10k resamples): [0.383, 0.650] — the ceiling is far below the target regardless of seed variance.

### Per-seed means (found / score)

| Seed | 1 | 2 | 3 | 4 | 5 |
|------|---|---|---|---|---|
| mean found | 0.67 | 0.58 | 0.42 | 0.33 | 0.58 |
| mean score | 55.8 | 55.9 | 43.5 | 47.4 | 58.0 |

### Failure-mode attribution (29 missing-target events across 10 runs)

| Class | Count | Share | Note |
|-------|-------|-------|------|
| unconfirmed-nearby (>90% coverage, target never confirmed/submitted) | 22 | 76% | dominant; tracker gate refuses to confirm despite detection |
| wrong-class (submission of another class within 2.0 m of GT) | 5 | 17% | cross-class confusion at close range |
| coverage_gap (target area never explored) | 2 | 7% | seed1 run1 only; run2 reached 96.8% on that seed |

FP-crowding: the two catastrophic runs (seed3 r2, seed4 r1 — found 1/6) each burned 4 FP submissions; in seed4 r1 FPs consumed the detection budget while true targets stayed unconfirmed. DOA runs: 0/10 (84–108 m traveled in every run).

### Reading

Coverage is essentially solved (95.8% mean; 8/10 runs ≥94.5%). The bottleneck is entirely detection-side confirmation: the anti-hallucination gate (conf margin +0.05, 1.2 m stability) still passes SE-corner hallucinations (known residual from t_3f2be52a validation), while simultaneously suppressing enough marginal true detections to produce 22 unconfirmed-nearby misses. Next levers to rank against this data: (1) conf margin 0.07–0.10, (2) bbox-depth plausibility check (generic), (3) wrong-class submissions (5 events) suggest the cross-class veto radius/margin needs tuning, not just the stability gate.

Results: `robot-sandbox/results/submissions/derpbot-explorer-v4-antihallucination/` (easy_seed{1..5}_run{1,2}.json)

---

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