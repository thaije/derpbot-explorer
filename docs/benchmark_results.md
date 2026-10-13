# Benchmark Results

Historical performance snapshots. Append new entries on top; keep older ones for regression comparison.

## 2026-10-13 — t_c6b546ac: event-triggered ROI second-pass detector recall — GATE REJECT (easy 5×1, mechanism reverted)

Agent: `derpbot-explorer-v7-roigate` @ `ef8cb50` (ROI mechanism) — v5-master semantics otherwise unchanged, tracker untouched. Mechanism (forensics category-c fix, 8/19 absence events): when the tracker reports a class pending (unconfirmed, ≥1 sighting) AND the full-frame pass yields no sighting of that class for K=10 relay cycles (~2 s at 5 Hz), ONE extra inference runs on a 320 px center crop around the class's last-known image position, upscaled 2×, conf 0.12; results tagged `source="roi_second_pass"`. Tracker boundary crossed read-only (`agent_node` polls `tracker.get_pending_candidates()`); cooldown 4 s/class keeps ROI ≤1 pass/class/4 s (≤6% extra inference), same subprocess worker (GIL invariant holds).

**REPLAY-A/B LIMITATION (stated explicitly, per pre-registered plan): detector-side changes CANNOT be replayed from recorded streams — no images were recorded, only projected sightings. The replay-first rule therefore yields to this small gate run.**

Unit tests (tests/test_detector_roi.py): 11/11 — trigger fires for silent pending class, skips recently-sighted/non-pending/no-anchor classes, cooldown rate-limit honored, `roi_second_pass` source marker present, ROI hit refreshes sighting clock + image anchor, crop→full-frame coordinate mapping. Regression at `ef8cb50`: watchdog 3/3, tracker gates 8/8, replay fixture 10/10.

Gate (easy seeds 1–5 × 1 run, speed 2, pinned pkg1, 5/5 completed, 0 failed):

| seed | found | FP/run | col | cov% |
|------|-------|--------|-----|------|
| 1 | 1/6 | 6.0 | 7.0 | 82.8 |
| 2 | 2/6 | 5.0 | 1.0 | 95.7 |
| 3 | 3/6 | 4.0 | 1.0 | 94.3 |
| 4 | 4/6 | 1.0 | 0.0 | 98.7 |
| 5 | 3/6 | 5.0 | 0.0 | 98.1 |
| **mean** | **2.60/6** | **4.2** | | |

**Pre-registered escalate bar: found mean ≥4.9 AND FP/run ≤5.0 AND ≥2 completions. Result: 2.60 found, 4.2 FP, 0 completions → REJECT.** Mechanism reverted at `a661c2c` (tests + diagnosis kept).

Operational notes (honest accounting):
- Seeds 1–2 ran with a worker-side ROI-drain starvation bug (requests only serviced on frame-queue timeout — effectively never at 5 Hz; 18+7 triggers, 0 hits). Fixed post-batch; seeds 3–5 ran with the drain active (114 triggers, 2 ROI hits total: fire_ext 0.13, first_aid 0.22 — both near-threshold; nothing converted to a found object).
- Even on the fixed slice (seeds 3–5), mean found 3.0 vs v5-master same-seed mean 4.67 — no evidence of benefit worth a 5×3 batch; the gate rule says stop.
- Attribution: ROI hits were rare AND low-confidence; the (c)-bucket misses need either higher-res inference than a 2× crop gives (far/small objects at 3–5 m occupy <30 px even upscaled) or a different recall surface (e.g. closer dwell), not more attempts with the same crop.
- s3r2 collapse diagnosis (separate deliverable, zero-sim, `results/s3r2_collapse_diagnosis.md`): slam_toolbox silent-activation startup abort (no /map within 120 s → STARTUP_MAP_TIMEOUT bail; robot never moved; mission timed out at 901.5 s). NOT watchdog / torch-oom / env. Filed as its own mechanism candidate: slam first-scan sentinel + restart-once.

Result JSONs: `robot-sandbox/results/submissions/derpbot-explorer-v7-roigate/easy_seed{1..5}_run1.json`; streams: `results/streams/stream_easy_seed*_run1_20261013*.jsonl`. Tree = v5-master semantics baseline 4/15 (no change).

---

## 2026-10-13 — t_6b03f03c: v6 batch 2 reverted to v5-master semantics; min-error publish REJECTED by zero-sim A/B (no sim spent)

Confirmation-policy experiment (bcfc, `eb59b96`) reverted at `dc3515d` — gates suite 8/8, replay fixture 10/10, v5-master semantics restored (STABILITY_RADIUS 1.2, VETO_RADIUS 0.8, margin 0.0, fast-confirm present). One new mechanism, publish-side min-error-per-sighting position (`78b6fb5`: corroborated sighting ≥2 same-track sightings within 0.3 m, else closest-to-mean; REPUBLISH_SHIFT_M kept; generic), evaluated by offline replay A/B on the 3-seed corpus streams (publish events unchanged; sanity anchor held — TP keys == live v5-master run sets {1,2,4}/{1,3,13}/{3}):

| seed | publish | found/16 | TP keys | FP | centroid-err mean/med/p95 |
|------|---------|----------|---------|----|---------------------------|
| 1 | running-mean | 3 | {1,2,4} | 3 | 0.158 / 0.213 / 0.230 |
| 1 | min-error    | 3 | {1,2,4} | 4 | 0.158 / 0.206 / 0.228 |
| 4 | running-mean | 3 | {1,3,13} | 0 | 0.285 / 0.241 / 0.476 |
| 4 | min-error    | 3 | {1,3,13} | 0 | 0.267 / 0.281 / 0.382 |
| 5 | running-mean | 1 | {3} | 0 | 0.271 / 0.271 / 0.271 |
| 5 | min-error    | 1 | {3} | 0 | 0.260 / 0.260 / 0.260 |

Pre-registered accept bar: found/18 ≥ running-mean AND FP/run not worse AND p95 drops ≥ 0.2 m. Result: found 7 vs 7 (equal), FP/run worse (seed1 3→4; 1.0→1.33 aggregate), p95 drops 0.002/0.094/0.000 m. **REJECT** → mechanism reverted (`af3790a`), tree = v5-master semantics + running-mean publish. No sim time spent: the zero-sim A/B caught a non-improving mechanism before the gate batch. Revert itself still restores the live-verifying 4/15 baseline for the next mechanism candidate.

---

## 2026-10-11 — v6-fastconfirm: bcfc policy adopted, decisive easy 5×3 batch — REGRESSION, HALTED

Agent: derpbot-explorer @ `eb59b96` (bcfc adopted: STABILITY_RADIUS 1.2→100.0, VETO_RADIUS 0.8→0.0, margin 0.0, fast-confirm untouched). Testbed unchanged (v1.0.3 tag, office scenario). Speed 2, pinned pkg1, 15/15 completed, 0 failed, 0 DOA.

**COMPLETIONS (found=6/6): 2/15 (13.3%). BAR WAS 11–12; PRE-REGISTERED RULE <8/15 → HALT, no submission, do not iterate.** Mean found 4.27/6 vs v5-master 4.87/6; completions halved (2 vs 4). The sweep's offline prediction (bcfc = strictly more recall: 13/18 replay corpus vs master-as-is 7/18) DID NOT TRANSFER to live runs.

| Seed | Run1 | Run2 | Run3 | Mean found |
|------|------|------|------|------------|
| 1 | 6/6 (68.6 C) | 5/6 (50.6 D) | 5/6 (60.8 C) | 5.33 |
| 2 | 3/6 (44.8 D) | 5/6 (60.5 C) | 6/6 (66.5 C) | 4.67 |
| 3 | 5/6 (65.4 C) | 0/6 (40.9 D) | 5/6 (62.5 C) | 3.33 |
| 4 | 4/6 (58.4 C) | 4/6 (56.4 C) | 5/6 (64.6 C) | 4.33 |
| 5 | 3/6 (52.0 D) | 5/6 (64.3 C) | 3/6 (55.5 C) | 3.67 |
| **Mean** | | | | **4.27/6 — 58.1, 2/15 completions (13.3%)** |

Per-class recall: fire_extinguisher 31/45 (0.69), first_aid_kit 20/30 (0.67), person 13/15 (0.87). Centroid-err TP: mean 0.442 m, med 0.266, p95 1.263, n=64; 8 above 1.0 m, 0 outside the 1.5 m window. FP mean 6.6/run (+0.3 FP_LOS, 0.3 DP); collisions mean 1.0; coverage mean 85.2%.

vs v5-master per-seed (same protocol): s1 −0.33, s2 0.00, s3 −0.56, s4 +0.22, s5 −0.61.

**Per-run attribution (26 missed mission-target objects over 13 non-6/6 runs, via per-run sighting streams + miss diagnosis):**
- 12 × detector-absence (GT object's class never sighted near it within 2 m; 2 of those zero sightings of the whole class in-stream) — s1r2 GT5, s1r3 GT5, s2r1 GT3+GT4+GT5, s2r2 GT4, s4r2 GT3, s5r1 GT1+GT5, s5r3 GT1+GT3+GT5, plus s3r2's five never-detected objects.
- 6 × confirmed-in-stream-but-unfound (nearest raw sighting 0.01–0.05 m, dozens of sightings, poses diverse — the tracker had the object and never published a submission that scored): s3r1 GT2, s3r3 GT2, s4r1 GT2+GT13, s4r2 GT2, s5r1 GT3. Not gate/veto (both inert), not the 1.5 m window (raw sightings land on the object) — points at centroid/MERGE contamination in live fusion (huge pose spans 5.5–12.6 m pull the running-mean centroid off the object; replay streams showed the same objects found, so live-only context — candidates/detours/exploration interplay — is implicated).
- 2 × diversity/fast-confirm gaps at single viewpoint (pose span 0.0 m, 3 or fewer sightings): s3r2 GT13 (also the DOA-adjacent 0/6 run), s4r3 GT3.
- s3r2 = the one catastrophic run: 0/6, only 131 sightings ALL class `person` (detector never fired on the other classes all run), coverage 24.2% — exploration/detector collapse, not tracker.
- DOA: 0 anywhere; no run died inside the 1.5 m near-miss margin on centroid error (p95 1.263 m).

**Interpretation for next iteration (evidence-backed candidates, NOT dispatched here):**
1. The 12 detector-absence misses are now the largest bucket — publish-side min-error-per-sighting candidates (sweep row `baseline-all-gates-off` predicted 15/18 but FP 8/run) and FA detector-side fixes were already queued from t_451895af; the live FP mean (6.6) is worse than sweep-predicted 2.7, so chain-merge into dense-view clusters is the FP mechanism to kill first.
2. The 6 "confirmable-unfound" misses say live centroid fusion (running mean across chained matches) diverges from what replay predicted — a publish-side min-error-per-sighting rule (publish the best-supported sighting position, not the running-mean centroid) addresses both buckets at once and was already queued as the next candidate.

Result JSONs: `robot-sandbox/results/submissions/derpbot-explorer-v6-fastconfirm/easy_seed{1..5}_run{1..3}.json`; per-run sighting streams: `results/streams/stream_easy_seed*_run*_<stamp>.jsonl`; batch table: `results/v6_fastconfirm_batch_rows.json`. NOT validated as a submission (decision rule). Tracker commit eb59b96 stands (offline regression exact); its LIVE effect is this regression — revert question is now open for the operator.

---

## 2026-10-10 — MASTER-PARITY: clean master, easy, 5 seeds × 3 runs (formal leaderboard entry `derpbot-explorer-v5-master`)

Agent: derpbot-explorer @ `24f7658` (= a45b124~1, pre-gate master; agent code commit-identical to the v4-bumper-fix era). Testbed: robot-sandbox @ `a53295a` (tagged `v1.0.3` locally; office scenario byte-identical to v1.0.2). Speed 2, pinned pkg1, 15/15 completed, 0 DOA. Full report: task t_96aad7bd attachment.

| Seed | Run1 | Run2 | Run3 | Mean found |
|------|------|------|------|------------|
| 1 | 6/6 (67.4) | 6/6 (77.6) | 5/6 (57.9) | 5.67 |
| 2 | 5/6 (59.0) | 4/6 (48.0) | 5/6 (62.8) | 4.67 |
| 3 | 4/6 (58.6) | 5/6 (67.0) | 6/6 (76.2) | 5.0 |
| 4 | 3/6 (53.4) | 3/6 (53.8) | 5/6 (61.8) | 3.67 |
| 5 | 6/6 (69.9) | 5/6 (63.4) | 5/6 (64.3) | 5.33 |
| **Mean** | | | | **4.87/6 — 63.7, 4/15 completions (26.7%)** |

FP mean 4.5/run (precision mean ~0.51); collisions mean 0.8.

**vs v4-bumper-fix (same protocol, May): 2.67/6, 0/15 completions, 56.6** — improvement REPRODUCES at protocol parity: Δ +2.20/6, bootstrap 95% CI [+1.33, +3.07]. Origin audit: **environment, not code** — zero agent code/config commits between eras; office testbed diff empty; May cohort shows thermal-handicap signatures (2 freeze-runs 0/6, 28% less travel, worse loc error 0.48 vs 0.38). Levers outside the repos: pkg1 CPU pin (2026-10-04) + gz-sim 8.10→8.11.

Submission `derpbot-explorer-v5-master` validated PASS; leaderboard.json/html regenerated (34 entries); sandbox stamping fixed in `962948b` (resolved release tag instead of `main-<sha>`).

---

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