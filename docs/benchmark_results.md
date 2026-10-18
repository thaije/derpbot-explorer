## 2026-10-18 — t_f4620454: matched-version pair completed (v12-v2 easy 5×3 @ sandbox v2.0.0 vs v10-assoc 5×3 @ v1.0.3) — SIM-EFFECT VERDICT: NULL on found (sim + mandatory intrinsics migration; see premise restatement below)

Par basis: **v1-human-derived** (easy par 6.00/6, v1 human teleop n=5/session) — **v2 human re-baseline PENDING (operator)**; the paired deltas below are therefore v1-human-derived-par-relative in spirit, and agent deltas are only interpretable at matched sandbox version, which is exactly what this entry holds fixed.

Agent: `derpbot-explorer-v12-v2` @ code `d2cd78d` (HEAD-at-batch `a1f14bd` is behavior-identical to it; DERPBOT_TILED_INFERENCE off). **Premise restated (review: the earlier "recorder-only / same-code-as-twin" wording was wrong): `git diff b4cd9f8..HEAD -- agent/` is NOT recorder-only — `agent/depth_projector.py` +108/−25 replaces the hardcoded 640×480 pinhole (fx=320) with `/derpbot_0/rgbd/camera_info`-driven intrinsics (fx≈960 @ 1080p) — the functional, mandatory adaptation the v2 camera requires. So this pair is "v2 sim + intrinsics-migrated agent" vs "v1 sim + pre-migration agent" (v10-assoc @ `b4cd9f8`), not "code held exactly matched"; the migrated projection was never run in-sim on the v1 camera. Its v1-equivalence is argued, not measured: `tests/depth_projector_intrinsics.py` asserts the migration's v1 fallback reproduces 640×480 / fx=320 exactly, and `agent/tracker.py` (+84) is stream-recorder header/keyframe gating only (default-off `DERPBOT_KEYFRAME_FULLRES`); `agent/detector.py` and `agent/frontier_explorer.py` are net-zero `b4cd9f8..HEAD`.** Easy (900 s) × 5 seeds × 3 runs, speed 2, pkg1-pinned tmux server (10-19,30-39), one sim at a time. v2 side: 15/15 written, 5 failed-then-retried to completion (task precondition listed 4 missing but was stale — 10/15 existed; the 10-18 08:56Z batch then hit a spawn-failure spiral: 5× "Sim did not become ready within 60s" (s3r3, s4r2, s4r3, s5r2, s5r3), closed completed=0 skipped=10 failed=5; the surviving resumable runner restarted 11:08:10Z, re-SKIPped the 10 present and ran exactly those five, finishing ~12:33Z = 13:08–14:33 CEST — the batch's 10-18 mtimes 13:25–14:33 are those five JSONs; t_91d433ec's batch, finished here); v1 side: 15/15 (t_3b4b2bd6 batch), 0 failed. Provenance: every one of the 15 v2 submission JSONs traces to a unique raw mission result (`robot-sandbox/results/office_easy_001_2026101[78]*.json`, exact signature match incl. submission timestamps); no duplicates, no re-runs overwriting.

### Paired table (per-seed means, 3 runs each; sandbox v2.0.0 minus v1.0.3, sim otherwise held fixed)

Par: **v1-human-derived** (easy par 6.00/6) + **v2 human re-baseline pending** — treat absolute v2 levels with that caveat.

| seed | found v2 | found v1 | Δfound | score v2 | score v1 | Δscore | v2 per-class (FE/FAK/per, of 9/6/3) | FP v2 | dup v2 | cov% v2 | coll v2 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 5.67 | 5.33 | +0.33 | 72.9 | 65.9 | +7.0 | FE 9, FAK 5, per 3 | 1.67 | 2.7 | 79.3 | 2.33 |
| 2 | 4.00 | 3.33 | +0.67 | 56.4 | 53.6 | +2.8 | FE 8, FAK 1, per 3 | 4.67 | 6.7 | 98.9 | 1.00 |
| 3 | 4.67 | 4.33 | +0.33 | 62.9 | 61.4 | +1.5 | FE 7, FAK 4, per 3 | 3.00 | 7.7 | 90.5 | 0.33 |
| 4 | 5.00 | 5.00 | 0.00 | 69.9 | 66.1 | +3.8 | FE 7, FAK 6, per 2 | 1.00 | 11.0 | 97.7 | 0.00 |
| 5 | 3.33 | 4.33 | −1.00 | 58.4 | 61.5 | −3.1 | FE 4, FAK 3, per 3 | 2.00 | 2.0 | 88.9 | 0.00 |
| **all** | **4.53/6** | **4.47/6** | **+0.07** | **64.1** | **61.7** | **+2.4** | FE 35/45 (.78) vs v1 34; FAK 19/30 (.63) vs 20; per 14/15 (.93) vs 13 | **2.47** (v1 2.93) | **6.0** (v1 7.1) | **91.1** (v1 88.3) | **0.73** (v1 1.00) |

### Paired-delta verdict (the deliverable)

- **Found/6, pooled Δ (v2−v1): +0.07/6, 95% bootstrap CI [−0.60, +0.67] — zero inside. Method (stated exactly, computed by the committed `robot-sandbox/scripts/paired_delta_v12v2.py`, 15/15 both sides): pooled paired-run bootstrap — resample the n=15 run-level paired deltas (Δᵢ = v2ᵢ − v1ᵢ over all 15 (seed, run) pairs) with replacement, 10 000 resamples, `numpy.random.default_rng(42)`, percentile CI (2.5/97.5).** Wins/ties/losses 6/6/3. VERDICT: the v2.0.0 Full-HD camera effect on found-mass — sim + mandatory intrinsics migration vs pre-migration agent at v1 (equivalence argued from `tests/depth_projector_intrinsics.py`, see premise above) — is NULL: indistinguishable from zero. Seed-level (paired-run bootstrap within each seed's 3 deltas, 10k resamples, rng seed 42+seed): s1 +0.33 (CI [+0.00,+1.00]), s2 +0.67 ([+0.00,+1.00]), s3 +0.33 ([+0.00,+1.00]), s4 0.00 ([−1.00,+1.00]), s5 −1.00 ([−3.00,+2.00]); n=3 per seed makes these CIs coarse, but no seed is individually decisive and seed5 (the noise-dominant seed, deltas −3/+2/−2) nets negative.
- **Score/grade: pooled Δscore +2.4 (same scheme: CI [−1.97, +6.87], zero inside — not significant). Grade hist v2 {A1 B3 C10 D1} vs v1 {B1 C11 D3}; completions (SUCCESS) 3/15 vs 1/15 — directionally more completions at 1080p but nowhere near the campaign goal-check target (12/15): **goal check 12/15: explicitly NOT MET (3/15)**.
- **Per-class recall: FE 35/45 (0.778 vs v1 0.756, +1 object), FAK 19/30 (0.633 vs 0.667, −1), person 14/15 (0.933 vs 0.867, +1).** No class moves enough to matter at n=15.
- **FAK conf band: pooled conf median 0.189 (v2) vs 0.190 (v1) [n=1036 vs 571 FAK detection confidence values — ALL detection events across the whole committed cohort, one record per detection per frame, not first-sighting or per-track values (first-per-run medians would be 0.175 vs 0.173; also inside the band)], both 0.15–0.37 (v2 max) / 0.41 (v1 max) — UNCHANGED, replicating the Phase-B finding that the low-conf band is a detector property that resolution does not lift.** FAK sighting mass, however, moved: per-seed paired mass mean v2 69.1/run vs v1 38.1/run (**×1.81 pooled; per-seed ×0.63 s1 / ×5.50 s2 / ×1.80 s3 / ×1.75 s4 / ×3.42 s5**; ≈767 vs 423 FAK detection records per 10 000 sim-s — pooled over the whole cohort, 1036 and 571 records over 15 × 900 s respectively, NOT first-sightings). The 1080p camera delivers its engineered evidence-density gain (this is Phase-A's offline bar, now confirmed live at 5×3) — but the mass converts to found objects only weakly and unevenly (s2: ×5.5 mass yet FAK recall 1/6; s1: mass ×0.63 yet FAK 5/6), because the confirm-gate/tracker, not evidence mass, is the binding constraint at matched code.
- **Stream cohort (named, so the mass/conf numbers are reproducible): v2 side = `derpbot-explorer/results/streams/` run1s `stream_easy_seed{1..5}_run1_20261017T15{40,51}../16{08,22,39}..Z` (the committed 5×1 gate cohort) + run2/3 from 10-17T19:16–20:36 (seeds 1–2 run2/3 + seed3 run2) and 10-18T11:08–12:16 (seed3 run3 + seeds 4–5 run2/3); the second, NON-committed 10-17T17:16–18:25 run1 stream set exists and is deliberately excluded (late discard re-runs, not the cohort). v1 side (v10-assoc) = run1s `stream_easy_seed{1..5}_run1_20261013T09{31,40,48}/09{56}/10{05}...Z` + run2/3 `..._run{2,3}_20261016T16:19–17:36Z`. All file names are pinned in the committed `paired_delta_v12v2.py` cohorts `V2_COHORT`/`V1_COHORT`.**
- **Dup-pressure (the 7.6 gate → answer): v2 5×3 dup/run mean 6.0 — BELOW the 7.6 v2 5×1 gate reference (and below the v1 twin's 7.1). The 1080p pixel-area duplicate pressure flagged at migration did not worsen in the full batch; seed4 remains the dup hotspot (11.0/run vs cohort 6.0), as in every prior cohort.**
- Secondary: precision mean v2 0.670 vs v1 0.614 (+0.056); FP/run 2.47 vs 2.93 (−0.46); collisions 0.73 vs 1.00. All small, same direction as the tile-gate attribution: v2 slightly safer/cleaner, not more effective.

**Bottom line: with the sim protocol held fixed and the only agent-side delta being the mandatory v2 camera intrinsics migration (v1-equivalence argued from the depth_projector test suite, not measured in-sim — see premise above), sandbox v2.0.0's camera bump buys sighting mass (×1.81 FAK), slightly cleaner FP/precision/collisions, and a couple more completions, but ZERO detectable change in the mission metric that defines the leaderboard (found/6, Δ +0.07 with CI [−0.60, +0.67] straddling zero), and the v2 par stays v1-human-derived pending the operator's human re-baseline. The earlier v12-v2 5×1 gate cohort (4.20/6, dup 7.6, prec 0.633 mean) sits inside this batch's run1s and replicates within noise in the 5×3 (4.53/6, dup 6.0, prec 0.670).**

Result JSONs: `robot-sandbox/results/submissions/derpbot-explorer-v12-v2/easy_seed{1..5}_run{1..3}.json` (raw sources include `robot-sandbox/results/office_easy_001_20261017T{175023,…}.json` — plain `…175023.json`, no `Z` suffix — plus the other committed raw mission results). Sighting streams: gate run1s `results/streams/stream_easy_seed*_run1_20261017T15-16??Z.jsonl` (committed cohort; the second non-committed 10-17T17:1x–18:2x run1 stream set is NOT part of it), batch stream evidence `…_20261017T19/20??Z.jsonl` + `…_20261018T11-12??Z.jsonl` (FAK masses above from these records; full pinned file list in `paired_delta_v12v2.py`). Leaderboard: both entries verified in `robot-sandbox/docs/leaderboard.json` (v12-v2 @ v2.0.0 total_score mean 0.756±0.188; v10-assoc @ v1.0.3 0.744±0.153), n=15 each.

---

## 2026-10-17 — t_ea3db560: tiled native-resolution inference (DERPBOT_TILED_INFERENCE=1) @1080p — 5×1 gate REJECT, mechanism reverted, agent-side inventory CLOSED (11/11)

Par basis: **v1-human-derived** (easy par 6.00/6, v1 human teleop n=5/session) — v2 human re-baseline PENDING (operator); all tile-vs-baseline comparisons here are **within-version** (same sandbox v2.0.0 camera, both cohorts v12-det lineage on 1080p), which per the campaign binding takes precedence.

Agent `derpbot-explorer-v12-tile` @ HEAD `bf4506b` (assoc-split lineage = v12-v2 baseline `1fcb550` + precision-doc fix `d2cd78d` + tiling enablement; mechanism first authored at `d5ef4ff` for v1 640×480, re-planned to v2 geometry per the resumption ruling). Diff from the v12-v2 baseline agent: DERPBOT_TILED_INFERENCE=1, DERPBOT_TILED_GRID=3,2 (6 tiles of 360×960 native scale), everything else byte-identical. Easy (900 s), speed 2, pkg1-pinned tmux server, 5/5 runs completed 0 failed.

### Go/no-go math (pre-sim, per the ruling: no full-sim probe)

v2-geometry microbench (offline, pkg1-pinned, RTX 2070 SUPER, noise frames; OWLv2's processor resize makes per-inference wall input-size-INDEPENDENT — full 1920×1080 ≈ per-tile ≈ 139 ms): full-frame 1080p = 139 ms; grid (2,2) = 4×139 ≈ 554 ms → 1.81 Hz FFE; (3,2) = 6×139 ≈ 837 ms → 1.20 Hz FFE; (3,3) = 9×139 ≈ 1261 ms → 0.79 Hz FFE. All three clear the 0.5 Hz FFE HALT bar offline → GO on the 5×1 gate with the 6-tile grid (more tiles = more native pixel area per frame at acceptable FFE). Committed enablement: `DERPBOT_TILED_GRID="<gy>,<gx>"` env (default (2,2)), marker `tile_q{q}` format-generated for grids > 2×2, forwarded through `start_stack.sh` (bf4506b).

Measured in-sim (pkg1, GPU shared with Gazebo): tiled_wall 1141–1597 ms/frame (≈ 1.40× the offline 837 ms) → FFE ≈ 0.86/1.14 Hz — above the pre-registered 0.5 Hz bar, gate allowed to proceed.

### Gate table (easy 5×1, sandbox v2.0.0)

| seed | status | score | found | FE | FAK | per | FP | dup | prec | cov% | dur (sim-s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | TIME_LIMIT | 61.9 C | 5/6 | 3/3 | 2/2 | 0/1 | 5 | 21 | 0.50 | 67 | 900 |
| 2 | SUCCESS | 72.3 B | 6/6 | 3/3 | 2/2 | 1/1 | 3 | 6 | 0.67 | 99 | 780 |
| 3 | TIME_LIMIT | 57.1 C | 4/6 | 3/3 | 0/2 | 1/1 | 5 | 17 | 0.44 | 57 | 901 |
| 4 | TIME_LIMIT | 59.7 C | 4/6 | 2/3 | 1/2 | 1/1 | 3 | 19 | 0.57 | 71 | 901 |
| 5 | TIME_LIMIT | 52.7 D | 3/6 | 1/3 | 1/2 | 1/1 | 4 | 3 | 0.43 | 74 | 901 |
| **mean** | | **60.7** | **4.40/6** | 12/15 | 6/10 | 4/5 | **4.00** | 13.2 | 0.522 | 74 | |

**Pre-registered escalation bar (found ≥ 4.9 AND FP ≤ 5.0 AND ≥ 2 completions): found 4.40 FAIL, FP 4.00 pass, completions 1/5 FAIL → REJECT.** No 5×3, no submission, no leaderboard entry.

### Per-class attribution vs the same-version baseline cohort (v12-v2 gate, c22fee3)

| metric | v12-v2 baseline (native full-frame) | v12-tile (3,2 native-res) | delta |
|---|---|---|---|
| FAK sightings/run (mean) | 62.2 | 27.6 | **−56%** |
| FAK sightings/k-sim-s (mean) | 72.6 | 31.0 | **−57%** |
| FAK conf median | 0.184 | 0.201 | unchanged (low band is a detector property, as Phase B already established) |
| FAK recall | 7/10 (0.70) | 6/10 (0.60) | −1 |
| FE recall | 9/15 (0.60) | 12/15 (0.80) | +3 |
| person recall | 5/5 (1.00) | 4/5 (0.80) | −1 |
| FP/run | 2.60 | 4.00 | +54% |
| dup/run | 7.6 | 13.2 | +74% (tile-edge duplicate pressure, as pre-flagged) |
| score mean | 62.0 | 60.7 | −1.3 |

**Mechanism verdict: measured-negative in-sim.** The causal chain the ruling proposed ("tiles raise per-frame FAK evidence density at same-position") did not survive contact with the live system: tiles cost **6.0× full-frame wall per frame** (837/139 ms — OWLv2's internal resize makes per-tile wall = full-frame wall), so the 3×2 frame's effective full-frame-equivalent cadence HALVED vs native full-frame (live ≈0.9 Hz vs baseline ≈1.5 Hz), and sighting mass — the constraint the offline bar cleared on — fell 57% per run. The tracker needs cadence more than it needs per-frame native-res coverage at 1080p: full-frame inference already delivers 1.5 Hz of 100%-coverage frames, while tiling bought resolution the low-conf FAK band (unchanged 0.184→0.201) never converts. The FE recall gain (+3/15) is real but is dwarfed in the same table by worse FP, worse dup, worse confirm throughput (1/5 completions), and lower score mean.

**Decision per pre-registration: REJECT → revert.** Reverts `bf4506b` (3cf784b) + `d5ef4ff` (a885611): agent/detector.py tiling code, tests/test_detector_tiled.py, and start_stack.sh forwarding removed; regression suite re-run green (keyframe tallied 42/0, tracker gates 8/8, slam sentinel 27/0, watchdog OK, detector-tiled pytest 29 passed pre-revert). Baseline returns to `1fcb550` + `d2cd78d` + reverts (HEAD `a885611`).

**Agent-side mechanism inventory: 11/11 formally resolved (10 prior + this one), all measured-resolved or resolved-negative by pre-registered bars.** The standing v2 baseline is v12-det-on-v2 (native full-frame OWLv2 @1080p, ≈1.5 Hz). Residual improvement paths are operator-side: (a) G-DINO-threshold re-cut (untested, t_b4361707 caveat), (b) detector-class swap if a real-time model wins a future bake-off, (c) compute (faster GPU raises cadence). Tiling would only re-enter if per-tile wall ever drops far below ~60 ms (9-tile frame < ~0.5 s at ≥10 tiles for full 1080p coverage at ≥0.5 Hz FFE with cadence headroom).

Files: results in `robot-sandbox/results/submissions/derpbot-explorer-v12-tile/`; run1 sighting streams `results/streams/stream_easy_seed*_run1_20261017T17{1655,3436}Z` … `T18{0737,2539}Z` (tile cohort) — NOTE: sighting records carry no per-detection `source` field (relay → stream does not forward it), so tile-attribution by marker must be re-derived from the per-frame `tiled_wall` cadence signature; counted FAK masses above come from the stream records as stored. Microbench script + gate aggregator preserved in the kanban workspace (`microbench_v2_tiling.py`, `tile_gate_table.py`).

---
## 2026-10-17 — t_b0b71441 Phase B: v12-det-on-v2 scored gate (easy 5×1) — REJECT, no escalation, no submission

Par basis: **v1-human-derived** (easy par 6.00/6, v1 human teleop n=5/session) — v2 human re-baseline PENDING (operator); v2 scores expected to shift vs a v1-camera par. The 5×3 escalation did NOT run (rule below), so no leaderboard effect, nothing to revert — baseline unchanged.

Agent `derpbot-explorer-v12-v2` @ HEAD `1fcb550` (assoc-split lineage: 7aedc20 + f8e3bd4 + v2 migration commits + fullres-flag `80d06e8`). Easy (900 s), speed 2, pkg1-pinned tmux server (10-19,30-39), one sim at a time, 5/5 completed 0 failed (results in `robot-sandbox/results/submissions/derpbot-explorer-v12-v2/`).

| seed | status | score | found | FE | FAK | per | FP | dup | coll | cov% | dur |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | SUCCESS | 74.6 B | 6/6 | 3/3 | 2/2 | 1/1 | 2 | 3 | 0 | 69.1 | 509 |
| 2 | TIME_LIMIT | 55.4 C | 4/6 | 2/3 | 1/2 | 1/1 | 5 | 5 | 1 | 98.7 | 901 |
| 3 | TIME_LIMIT | 58.3 C | 4/6 | 1/3 | 2/2 | 1/1 | 3 | 14 | 1 | 99.0 | 900 |
| 4 | TIME_LIMIT | 71.5 B | 5/6 | 2/3 | 2/2 | 1/1 | 0 | 15 | 0 | 95.1 | 901 |
| 5 | TIME_LIMIT | 50.1 D | 2/6 | 1/3 | 0/2 | 1/1 | 3 | 1 | 0 | 98.6 | 900 |
| **mean** | | **62.0** | **4.20/6** | 9/15 | 7/10 | 5/5 | **2.60** | 7.6 | 0.4 | **92.5** | |

**Pre-registered escalation bar (submit-side of the gate): found mean ≥ 4.9 AND FP/run ≤ 5.0 AND ≥ 2 completions. Result: found 4.20 (FAIL), FP 2.60 (pass, not attributable per rule), completions 1/5 (FAIL) → NO ESCALATION, no 5×3, no submission, no leaderboard entry. Goal 12/15: explicitly NOT MET (1/5 completions).**

Per-class recall (5 runs × GT 3 FE / 2 FAK / 1 person): fire_extinguisher 9/15 (0.60), first_aid_kit 7/10 (0.70), person 5/5 (1.00). Precision: per-run 0.75 / 0.44 / 0.57 / 1.00 / 0.40 — mean 0.633 (pooled found/(found+FP) = 21/34 = 0.617).

**FAK sighting-mass attribution (the campaign headline metric):** the v1 collapse (conf 0.15–0.24, 1–6 sightings/run on seed2, 10/30 FAK missed across the v1 5×3) did NOT reproduce as a FAK problem at 1080p — FAK recall 7/10 and live FAK sighting mass per run 21/29/69/177/15 (conf med 0.16–0.21, same low band as v1). The misses swung to **fire_extinguisher (6/15)** + seed5 (both FAK missed, 15 sightings only — the seed5 analogue of the old collapse, now on a different seed) + person confirmed 5/5 in every run (v1's timing lottery did not appear in the person slot here). Miss table: seed1 —, seed2 {FE2, FAK4}, seed3 {FE2, FE3}, seed4 {FE3}, seed5 {FE1, FE3, FAK4, FAK5}. Dup mean 7.6 (v1 5×3: 7.1) unchanged — the ~9×-pixel-area duplicate pressure flagged at migration persists (15 dups on seed4).
- FAK conf lift DID NOT materialize in live missions: detected-FAK conf median 0.19–0.21 (bake-off max 0.35), the low-conf band is a detector property, not a resolution property. 1080p bought sighting MASS (bake-off FAK instance-rate ×2.08) which converts to confirmations on 4/5 seeds — but not on the low-mass run (seed5, 15 sightings).

**Rule-branch decision: 4.20 < 4.9 → stop.** Everything kept (baseline unchanged, sandbox stays v2.0.0 operator-sanctioned, nothing reverted). The pre-registered Phase-B stop condition puts the operator decision back on: (a) G-DINO-threshold variant (the t_b4361707 bake-off's FAK-gain-inseparable-from-FP issue could in principle be re-cut at a higher threshold — untested), (b) camera fidelity beyond 1080p (diminishing: FAK conf unchanged v1→1080p, so resolution does not lift the low-conf tail), or (c) remain at v12-det-on-v2 as the standing v2 baseline. No next gate is pre-authorized by this campaign.

---
## 2026-10-17 — t_b0b71441 Phase A: v2 1080p OWLv2 probe — offline bar CLEARS (FAK ×2.08, FP-proxy ×1.34) → Phase B gate

Par basis for every table below: **v1-human-derived** (easy par 6.00/6 from v1 human teleop, n=5/session) — sandbox v2 human re-baseline PENDING (operator); v2 scores are expected to shift relative to a v1-camera par, and all v2-vs-v1 deltas here are CROSS-VERSION (640×480 → 1920×1080 camera), so they speak to the resolution hypothesis only, with within-version comparisons taking precedence.

Agent `derpbot-explorer-v2probe(-fr)` @ HEAD `80d06e8` (assoc-split: v12-det lineage = 7aedc20 + f8e3bd4 fixes + v2 migration + fullres-flag). Easy (900 s), speed 2, pkg1-pinned tmux server (10-19,30-39), one sim at a time, streams + keyframes recorded.

### Instrument upgrade (required before the bar could be measured)

The pre-registered instrument ("re-run `bakeoff_run.py` on the v2 frames") was **structurally blind to resolution**: `KeyframeRecorder` downscales every stored frame to `KEYFRAME_WIDTH=320` (agent/detector.py:122), so v1 640×480 → 320×240 JPEGs and v2 1920×1080 → **320×180** JPEGs — two ~320-wide image sets; the 1080p detail is discarded before the detector runs. Measured on the downscale corpus the bar "passed" (FAK 0.479 vs 0.319 = ×1.50 by a hair, FP-proxy 0.209 = ×0.77) — recorded as instrument-blindness, not as signal (`results/bakeoff_v2/`).

Fix (committed `80d06e8`): `DERPBOT_KEYFRAME_FULLRES=1` stores detector-input-resolution keyframes (forwarded via `start_stack.sh`; default off — v1 behavior byte-identical, keyframe tests 42/0 on ROS env). `run_benchmark.sh` wall timeout corrected for v2 RTF (sim×3/speed + 300; observed RTF 0.84–0.89 at speed 2 — the old /speed+300 formula timed out healthy missions twice before the fix). Both probe passes then re-run at full res: `results/bakeoff_v2_fr/` (+ `_tol2` sensitivity).

### Bake-off — OWLv2-on-1080p vs OWLv2-on-v1 (within-model, cross-camera)

| corpus | frames (paired) | FE rate | FAK rate | FAK conf med | person rate | FP-proxy/frame | lat med |
|---|---|---|---|---|---|---|---|
| v1 (320×240 downscale of 480p) | 1435 | 0.826 | 0.319 | 0.24 | 0.741 | 0.271 | 289 ms |
| v2 FR (320w downscale of 1080p — blind corpus) | 225 | 0.908 | 0.479 (×1.50) | 0.21 | 0.690 | 0.209 | 264 ms |
| **v2 true 1080p (1920×1080)** | 207 | 0.927 | **0.664 (×2.08)** | 0.20 | — (0 vis) | **0.362 (×1.34)** | 372 ms |
| sensitivity ±2.0 s tol | v1 2228 / v2 342 | 0.747 | 0.273 / v2 **0.548 (×2.01)** | — | — | 0.266 / **0.327 (×1.23)** | — |

**Pre-registered offline bar: FAK rate ≥ 1.5× v1 AND FP-proxy ≤ 1.5× v1. Result: ×2.08 (clears) AND ×1.34 (clears) — sensitivity tol 2.0 s: ×2.01 AND ×1.23 (both hold). Phase A verdict: BAR CLEARS → Phase B gate proceeds per pre-registration.**

Caveats on the offline numbers:
- person visible-instances = 0 in the FR corpus — a corpus-composition artifact (FR probe runs: person never in-frame during the candidate-dense keyframe windows; person sightings in-stream were 8 on seed1, 0 on seed4, vs 131 on the 320px seed4 run), NOT a model regression claim; person stays unmeasured by this offline instrument on v2.
- FAK confidence did NOT lift: detected-FAK conf median 0.20 (v1 0.24), max 0.35 (v1 0.46) — 1080p buys more FAK sightings at the same low-conf band, not higher conf. The confirm-gate question (tracker accepts 0.19–0.35 conf chains) is exactly what Phase B tests live.
- Cross-camera caveat: v1 corpus is 640×480-rendered; composition (which frames exist) is trigger-dependent and not seed-paired across versions.

### Live probe runs (each seed run1, completed missions only)

| run | status | score | found | FE | FAK | per | FP | cov% | dur (sim-s) |
|---|---|---|---|---|---|---|---|---|---|
| v2 @320w seed1 | SUCCESS | 76.2 B | 6/6 | 3/3 | 2/2 | 1/1 | 1 | 95.8 | 476 |
| v2 @320w seed4 | TIME_LIMIT | 71.6 B | 5/6 | 2/3 | 2/2 | 1/1 | 0 | 95.0 | 901 |
| v2 @1080p seed1 | TIME_LIMIT | 45.5 D | 3/6 | 3/3 | 0/2 | 0/1 | 3 | 62.7 | 900 |
| v2 @1080p seed4 | TIME_LIMIT | 62.2 C | 4/6 | 2/3 | 2/2 | 0/1 | 2 | 91.0 | 901 |

n=2 per resolution, cross-version vs the v1 5×3 cohort (mean 4.47/6) — probe numbers are hypothesis-plausibility only, NOT gate-grade evidence (that is what the Phase B 5×1 gate is for). FAK live sighting mass (per k-sim-s, per seed/run1): v1 [26–120, med 58] seed1 and [51–342, med 91] seed4; v2@1080p 63 and 354 — within the v1 seed4 band, above the v1 seed1 median. FAK confirmed 2/2 on seed4 in all three v2 runs (the v1 seed4 collapse does not appear at 1080p in these probes); seed1 FAK swings 2/2 → 0/2 between resolutions (n=1). Person 0/1 on both 1080p runs (timing lottery persists). Dup-rate on v2 runs 0.33–0.82 (v1 0.13) — the ~9× pixel-area effect the migration report flagged is real; FP count stayed low (0–3).

### Failed probe attempts (kept for the record)

- v2probe attempt-1: both runs hit the pre-fix 750 s wall deadline with no result; the runner's cleanup killed them (seed1 healthy at RTF 0.89, 669/900 sim-s; seed4 sim-clock froze at 65 s — separate stall). Not scored, streams kept.
- v2probe attempt-2 (seed4@862 s): healthy, 728/900 sim-s, RTF 0.84 — still short; timeout formula fixed (above). Not scored, stream kept.
- v2probe attempt-3: stale FDS port → sim-ready failure; clean relaunch procedure (kill_stack + tmux server rebuild) used as fix.
- mid-flight recorder downgrade catch: one fullres-marked pass still wrote 320-wide frames (flag not forwarded through start_stack.sh); start_stack.sh forwarding added (80d06e8), that pass discarded before scoring.

Files: `results/bakeoff_v2{,_fr{,_tol2}}/` (agent repo), probe raws `office_easy_001_20261017{T155606,T164447,T171228,T173015}.json` (testbed results root) and `results/submissions/derpbot-explorer-v2probe{,-fr}/`, streams+frames in the agent repo `results/streams/`.

---
## 2026-10-16 — t_649e32e7: keyframe capture pytest-mode tally FAILs fixed + __main__ runner repaired + TALLY_GATE opted in (test-harness only, no product change)

Follow-up to t_1779f975, fixes the pre-existing `tests/test_keyframe_capture.py` defects at `caa2b8f`. Root cause of the 6 hidden pytest-mode tally FAILs: `KeyframeRecorder.__init__` auto-starts the loop thread, and every test constructed recorders with `clock_node=None` — `_sim_time()` then falls back to `time.time()`, so the loop wrote `f_<wall-clock-epoch>.jpg` files into the tmp fixture dir, racing the direct `_write_one()` calls (301/300 cap inflation; the `[f_100.000.jpg, f_1792181487.xxx.jpg]` pollution in the collision and rate tests). Fix: a `_recorder_thread_disabled()` context manager patches `KeyframeRecorder._loop` to a no-op for the direct-write tests (loop due-gate/budget/cap logic stays verified via inline replication and source inspection — captured at import time so the checks inspect the real source, not the patch) — all file-side assertions are now exact and wall-clock-independent. Also: `test_pending_active_rate_1hz` asserted 1 file while its own two ungated direct writes at +0.5 s correctly produce two (the 100 ms collision guard only applies to ±0.1 s) — split into separate exact assertions for the first write and the past-guard write-through. The `hot path: stash is an assignment` check matched `self._latest_frame = (frame,` but `Detector._image_cb` actually stashes `(frame.copy(), ...)` — pattern relaxed to the real single-tuple-assignment contract. The `__main__` runner no longer crashes on `monkeypatch`-parameterised tests: a minimal `_MonkeyPatchShim` (setattr/undo only — the one feature this module uses; unsupported monkeypatch features raise → printed SKIP, never a fake PASS), per-test tmp dirs with cleanup, and per-test crash capture so one failing test no longer aborts the whole run.

After: tallied `python3.12 tests/test_keyframe_capture.py` 42 passed, 0 failed, exit 0 (was crash-on-first-monkeypatch-test; previously 34 checks with 2 cap FAILs); pytest 17 passed, 0 tally FAILs, exit 0. Module sets `TALLY_GATE = True` — the t_1779f975 conftest guard now enforces the tally here (negative control re-verified on this module: injected artificial FAIL → pytest session exit 1). Determinism: 5 consecutive pytest runs + 3 tallied runs all green. Rest of sweep unchanged vs parent record at `caa2b8f`: tracker gates 8/8, sentinel 37/0, wiring 27/0, assoc split 7/7, gpclip 7/7, replay fixture 10/10, watchdog 3/3 OK (tallied mode with PYTHONPATH+=repo root). No sim runs; no benchmark re-gate.

---

## 2026-10-16 — t_1779f975: slam sentinel wiring healthy-path tally test fix + pytest check() FAIL-list guard (test-harness only, no product change)

Pre-existing test defect found in t_cd52b6b5 round-2 review (`ac8e5ad`, 2026-10-13): `tests/test_slam_sentinel_wiring.py:179` appended to `flags_written` itself and then asserted it empty — tautologically failing, so the tallied `__main__` run was permanently `22 passed, 1 failed` (exit 1) while pytest read `6 passed` green (pytest never inspected the module's check() FAIL list; no conftest existed). Fixed the healthy-path check to what its name says: a spy on `frontier_explorer.write_restart_flag` (the exact callable `_explore_loop`'s restart branch calls) must never be invoked on the healthy path, plus the flag file must stay byte-identical, plus a loop-replica segment asserting zero restart/abort decisions and zero writes after map arrival (+4 checks). After: tallied `27 passed, 0 failed` exit 0, pytest 6 passed.

Secondary hardening (optional item 3): new `tests/conftest.py` pytest_sessionfinish guard — after the pytest run, each test module that opts in via a module-level `TALLY_GATE = True` marker has its FAIL tally inspected and the pytest session is failed if it recorded check() failures, so an opted-in module's pytest count can never silently drift from its tallied reality again (negative control verified: an artificial tally FAIL turns the pytest session exit 1). `test_slam_sentinel_wiring.py` opted in at this commit; modules with known-red pytest-mode tallies (e.g. keyframe cap/file-pollution FAILs that `17 passed` used to hide) stay out until fixed. The repo keeps both run modes per module: the tallied `python3.12 tests/<file>.py` run is the authoritative gate; pytest additionally runs test_* functions + the tally guard. Sweep at this commit, all unchanged vs parent record: tracker gates 8/8, assoc split 7/7, gpclip 7/7, slam sentinel 37/0, wiring 27/0, keyframe pytest 17, watchdog 3/3, replay fixture 10/10, closein + detector_roi module-level import-gated skips at HEAD (close 25/25 when armed). No sim runs; no benchmark re-gate.

---

## 2026-10-16 — t_3b4b2bd6: v10-assoc easy 5×3 protocol-parity verification — REPORT-ONLY, no submission (rule FAIL: 4.47 < 4.87 and 1 < 4 completions)

Agent: `derpbot-explorer-v10-assoc` @ HEAD `b4cd9f8` (same tree as the 5×1 gate, mechanism code untouched). Full 5 seeds × 3 runs, easy (900 s), speed 2, pkg1-pinned tmux server (10-19,30-39), one sim at a time, 10 new runs (run1 seeds 1–5 reused from the t_ca2ea380 gate — resumable skip per protocol), 15/15 written, 0 failed, 0 DOA (all statuses SUCCESS/TIME_LIMIT as designed; elapsed ≤904 s). Cleanup verified clean post-batch. Submission yaml validation note: batch-runner validator flags "Missing required keys: ['submitted']" — expected pre-decision state, decision below is report-only.

Pre-registered rule (operator-approved): submit + leaderboard v10 easy iff mean found ≥ 4.87/6 OR completions ≥ 4. **Result: mean found 4.47/6 (<4.87 — FAIL), completions 1/15 (<4 — FAIL) → NO submission, no leaderboard entry; v5-master (4.87/6, 4 completions) remains the easy leaderboard line. Nothing worse was published. Goal 12/15 completions: 1/15 — explicitly NOT met.**

| seed/run | found | FE | FAK | per | FP | dup | coll | cov% | score |
|----------|-------|----|----|-----|----|----|------|------|-------|
| 1/1 | 6/6 | 3/3 | 2/2 | 1/1 | 2 | 1 | 0 | 61.3 | 77.8 B |
| 1/2 | 5/6 | 2/3 | 2/2 | 1/1 | 5 | 15 | 9 | 85.5 | 54.8 D |
| 1/3 | 5/6 | 3/3 | 2/2 | 0/1 | 2 | 8 | 0 | 60.1 | 65.1 C |
| 2/1 | 3/6 | 2/3 | 0/2 | 1/1 | 5 | 2 | 1 | 69.0 | 48.8 D |
| 2/2 | 3/6 | 1/3 | 1/2 | 1/1 | 2 | 0 | 0 | 98.7 | 55.6 C |
| 2/3 | 4/6 | 3/3 | 0/2 | 1/1 | 4 | 7 | 2 | 93.2 | 56.4 C |
| 3/1 | 4/6 | 2/3 | 1/2 | 1/1 | 3 | 10 | 2 | 99.9 | 58.8 C |
| 3/2 | 5/6 | 3/3 | 1/2 | 1/1 | 1 | 8 | 0 | 98.6 | 67.0 C |
| 3/3 | 4/6 | 3/3 | 1/2 | 0/1 | 2 | 16 | 1 | 99.0 | 58.5 C |
| 4/1 | 5/6 | 2/3 | 2/2 | 1/1 | 2 | 8 | 0 | 71.6 | 64.6 C |
| 4/2 | 5/6 | 2/3 | 2/2 | 1/1 | 5 | 10 | 0 | 98.7 | 65.4 C |
| 4/3 | 5/6 | 2/3 | 2/2 | 1/1 | 2 | 12 | 0 | 98.0 | 68.4 C |
| 5/1 | 5/6 | 2/3 | 2/2 | 1/1 | 2 | 2 | 0 | 98.7 | 66.2 C |
| 5/2 | 3/6 | 1/3 | 1/2 | 1/1 | 4 | 2 | 0 | 94.3 | 53.3 D |
| 5/3 | 5/6 | 3/3 | 1/2 | 1/1 | 3 | 6 | 0 | 98.1 | 65.1 C |
| **mean** | **4.47/6 (0.744)** | | | | **2.93** | **7.1** | **1.0** | **88.3** | **61.7** |

Per-class recall (15 runs × GT 3 FE / 2 FAK / 1 person): fire_extinguisher 34/45 (0.76), first_aid_kit 20/30 (0.67), person 13/15 (0.87). Precision mean 0.61 (v5 0.53). FP 2.93/run (v5 4.73), collisions 1.0 (v5 0.87), score 61.7 (v5 62.7).

**Per-run attribution (14 non-6/6 runs, GT 3 FE + 2 FAK + 1 person; streams + frames dirs exist for all runs incl. gate run1s, e.g. `results/streams/frames/stream_easy_seed2_run2_20261016T163614Z/` 45 keyframes):**
- first_aid_kit is the dominant miss: 10/30 FAK missed across 14 runs (never-found in s2r1×2, s2r3, s3; and ≥1 FAK in ten more runs). Stream evidence: FAK sightings are the lowest-confidence class (e.g. s2r2: 6 sightings, conf 0.15–0.24; s2r3: a SINGLE 0.17 sighting all run; s2r1 13-Oct stream: 1 sighting 0.16) — the confirm gate (conf ≥ real-object margin, ≥2 diverse poses) almost never gets a confirmable FAK chain. Same detector-recall bucket the forensics already flagged (max FAK conf 0.36): detector-side, not association.
- seed2 regression is the batch's largest hole (3/3/4 vs v5 5/4/5 per-seed mean 0.56 vs 0.78): FAK sighting mass collapses on this seed's map layout (1–6 sightings/run). Not a split artifact (no near-dup FAK chain on seed2 in Phase-1 replays).
- person misses (s1r3, s3r3): zero person sightings in-stream all run (s1r3) — detector never saw the person walking route; timing lottery, not tracker.
- fire_extinguisher misses (11 total, ≥1 in 10 runs): the split converts the seed4 desk-pair chimera every run (4/×5/5 → 5/6 ×3) but the third-FE + partial-view FE singles still die in the confirm-gate lottery (cov 98% + dup 8–16: split halves exist, sibling half unconfirmed).
- dup mean 7.1 (v5 0.13): the split's quarantined children double-publish sibling candidates that never both confirm — FP cost 2.93 vs 4.73 on v5 is still NET −1.8 FP/run (bootstrap CI on per-seed paired diff: se 0.61), precision +0.08. The FP story improved; the recall story did not (dup children absorb the confirm budget).
- One anomalous run: s1r2 had 9 collisions (t=229–464 clustered mid-run) at cov 85.5 — navigation, not perception; cohort-variance outlier.

**Comparison verdict vs v5-master 5×3 at protocol parity (paired per-seed, run-level):** v10-assoc is NOT better: paired mean diff −0.40/6 (−0.067 ratio, t=−0.86, df=4), 95% bootstrap CI [−1.07, +0.47]/6 — within noise, sign suggests no gain. Per-seed: s1 −0.06, s2 −0.22, s3 −0.11, s4 +0.22, s5 −0.17. The association split delivers its engineered effect (seed4 chimera 3/3/5 → 5/5/5, stable) and a real FP/precision win (2.93 vs 4.73, precision 0.61 vs 0.53), but loses the mass elsewhere: found floor stays 3–5/6 with only 1 completion, and the FAK/detector-side + confirm-gate ceiling (previously measured the SAME way in the t_fd80a2f9 forensics — "found-mass dies at the tracker confirm gate") is the binding constraint, unchanged by the split. **Plain answer to the task question: the split+projfix stack is better on FP/precision and seed-4 consistency, but NOT better on found at protocol parity — equal-within-noise, slightly negative; the earlier gate-era numbers (4.60/6 5×1, FP 2.8) replicate (4.47/6 5×3, FP 2.93) rather than improve.**

Result JSONs: `robot-sandbox/results/submissions/derpbot-explorer-v10-assoc/easy_seed{1..5}_run{1..3}.json`; streams: `results/streams/stream_easy_seed{S}_run{R}_20261016T*.jsonl` (new runs) + `20261013T*` gate run1s; keyframes: `results/streams/frames/stream_easy_seed*_2026101*Z/`.

---

## 2026-10-16 — t_cd52b6b5: close-in verification detour RE-GATED on corrected projection (v11-cv2) — GATE REJECT again (easy 5×1, mechanism re-reverted)

Second try of the same mechanism with the premise fixed: the original gate (t_5e2d3407, `derpbot-explorer-v8-closein` @ 6ec2c61) failed because tracker candidates sat 2.5–8.6 m off-object; the ground-anchored depth clip (8378792) moved gate-run projection error to 0.10–0.34 m median (t_fd80a2f9 forensics). Mechanism re-applied unchanged (cherry-pick of 6ec2c61, one conflict vs slam-sentinel/keyframe attrs; commit 7abffb2), constants re-checked and kept: with candidates sub-0.4 m off-object, arrival at the candidate goal places the true object inside FAST_CONFIRM_DIST=1.5 m (no retune; hold 2.0 s = pre-registered dose, budget/rate/attempt rails unchanged). Unit tests `tests/test_frontier_closein.py` un-gated and green (all 25 checks); full regression green (tracker gates 8/8, assoc split 7/7, gpclip 7/7, slam sentinel 10+6, watchdog 3/3, replay fixture 10/10).

Agent: `derpbot-explorer-v11-cv2` @ 7abffb2 — easy 5×1 (900 s, speed 2, pkg1-pinned tmux anchor 10-19,30-39, one sim at a time), 5/5 completed, 0 failed, preflight + cleanup verified clean.

| seed | score | found/6 | FE | FAK | per | FP | dup | coll | cov% |
|------|-------|---------|----|----|-----|----|----|------|------|
| 1 | 62.8 C | 5/6 | 3/3 | 1/2 | 1/1 | 4 | 6 | 1 | 98.7 |
| 2 | 57.7 C | 4/6 | 3/3 | 0/2 | 1/1 | 3 | 6 | 1 | 98.9 |
| 3 | 68.2 C | 5/6 | 3/3 | 1/2 | 1/1 | 2 | 11 | 0 | 98.3 |
| 4 | 64.4 C | 4/6 | 2/3 | 2/2 | 0/1 | 1 | 20 | 0 | 98.6 |
| 5 | 53.9 D | 4/6 | 3/3 | 0/2 | 1/1 | 2 | 2 | 2 | 99.2 |
| **mean** | **61.4** | **4.40/6** | 14/15 | 4/10 | 4/5 | **2.40** | **9.0** | **0.8** | **98.7** |

**Pre-registered escalate bar: found mean ≥ 4.9/6 AND FP/run ≤ 5.0 AND ≥ 2 completions. Result: found 4.40 (FAIL), FP 2.40 (pass, not attributable per rule), completions 0/5 (FAIL) → REJECT. Mechanism re-reverted (a81e6da), tests kept import-gated, no 5×3 escalation.**

**Hold-confirmation attribution (the question this gate existed to answer — did holds confirm with correct geometry?): NO.** 19 detours started, 16 stop-and-stare holds across seeds 1–4 (s1 3, s2 4, s3 5, s4 4; seed5 profile md lost — the benchmark harness SIGKILLed the tmux server immediately post-mission, before the 5th run's `_write_timeline`; its result JSON + sighting stream are intact, so seed5 hold/detour counts are unknown, not zero). Every hold exited `confirmed=False` — **0/16 conversions (old gate: 0/9)**, and causality-checking TPs against hold windows finds zero same-class TP within any hold ±5 s — in fact every TP predates every hold (detector finds objects at ordinary in-FOV passes; near-dup re-approach holds add nothing). Guard rails behaved: 0 rate-limit/attempt-cap hits, budget 6/6 reached on seeds 3–4 (budget events fired), `close_in_skip_cell` = 176 suppressions total (83+53+21+19 per seeds 1–4 profile summaries; candidates repeatedly projected onto unknown/lethal costmap cells — the rails did real work). Mission-clock cost bounded: total hold dwell = 63.3 s wall over seeds 1–4 (`held=` sums 22.8 / 13.0 / 15.2 / 12.3 s wall; sim-clock dose 2.0 s/hold — `held=` is wall time, RTF-dependent).

Why it STILL fails (`close_in_*` timeline × submission_log × sighting streams, sim-time-checked): the residual easy misses are {s1 FAK1, s2 FAK2, s3 FAK1, s4 FE1+person1, s5 FAK2} = 8 misses total (30 targets − 22 found): **6/10 FAK + 1/15 FE + 1/5 person**. The hold fix targets candidates that fire (→ pending) but never re-confirm; these misses instead never accumulate a confirmable sighting chain AT ALL (FAK = detector's lowest-confidence class, 1–6 sightings/run, conf 0.15–0.24; person = zero-sighting timing lottery) — t_3b4b2bd6 attribution unchanged. Two new mechanism-surface observations: (1) seed4 dup=20 (vs cohort 9.0) — the split's quarantined children repeatedly re-fire as candidates and eat the budget on already-found siblings; (2) `close_in_skip_cell` heavy suppression means split-children candidates often project onto obstacle cells, i.e. they are duplicates of already-confirmed objects seen past obstacles. Neither is actionable by dwell time near the candidate.

Paired run1 vs v10-assoc run1 (same seeds): mean 4.40 vs 4.60, diff −0.2/6, per-seed [−1,+1,+1,−1,−1] — within noise; FP 2.40 vs 2.80 (−0.4). No evidence the detour changes cohort outcomes either way.

Result JSONs: `robot-sandbox/results/submissions/derpbot-explorer-v11-cv2/easy_seed{1..5}_run1.json`; gate streams `results/streams/stream_easy_seed{1..5}_run1_20261016T19????Z.jsonl`; timelines `results/profile_20261016T211003..213514.md` (seeds 1–4; seed5 note above). Commits: 7abffb2 re-arm → a81e6da re-revert (v5-master+sentinel+split semantics restored; closein tests import-gated again 1 skipped as designed).

---

# Benchmark Results

Historical performance snapshots. Append new entries on top; keep older ones for regression comparison.

## 2026-10-13 — t_ca2ea380: tracker data-association fix (spread-triggered 2-means split of near-duplicate chimeras) — GATE REJECT (easy 5×1; association fix KEPT in HEAD, no 5×3 escalation)

Agent: `derpbot-explorer-v10-assoc` @ `437aeb2` — builds on the t_fd80a2f9 projection fix (`8378792`, KEPT per parent pre-registration). The gates stay; they stop receiving chimera inputs. Mechanism (`70e22bd`, agent/tracker.py association layer ONLY — STABILITY_RADIUS/conf/veto untouched): `TrackedObject.try_split()` — when a same-class track reaches ≥4 sightings and pairwise spread > STABILITY_RADIUS (1.2 m), cluster the chained sightings (2-means, extremes-seeded); split fires only when centroids are ≥0.5 m apart AND both halves carry a corroborated core (≥2 sightings within 0.5 m = two re-sighted real objects); split children are QUARANTINED at TIGHT_MATCH_RADIUS 0.75 m so the 0.96 m sibling cannot re-merge the halves (distinct objects observed 0.96–1.17 m apart; same-object sighting scatter ≤0.3 m post-projection-fix). Unit tests `tests/test_tracker_assoc_split.py` 7/7; full regression green (tracker gates 8/8, replay fixture 10/10, gpclip 7/7, slam sentinel 23+wiring, keyframe 17, watchdog 3). Forensics: `results/assoc_split_phase1_forensics.md` in-repo.

**Phase 1 — zero-sim merge forensics + variant A/B (replay on the t_fd80a2f9 gate streams, replay-valid by construction):** confirmed the attribution — seed4 desk fire-extinguishers (0.96 m apart) match-radius-chained (`MATCH_RADIUS=2.0`) into one track with spread 2.2–4.3 m that gate-rejected despite 150 sightings ≤0.07 m from GT; seed3 desk pair same shape. 2 real-object merges found across 5 streams; 259 gate-reject events carried real mass. Variant comparison (`results/assoc_variant_results.json`, fresh anchor = current-code replay): pure MATCH_RADIUS tightening (1.2/0.9/0.75) REGRESSES found (tighter single-track association starves legitimate re-sightings — the object drifts out of one track without forming a healthy second); spread-triggered split wins: found 4.2→5.2/run (+1.0 ≥ +1.0 ✓), FP 0.8→1.6/run (vs FP budget ≤5.0 ✓), accuracy component +6.6 pts/run mean (seed4 +30.0: 2/6→5/6-equivalent). Phase 2 gate condition met → split implemented (`70e22bd`).

**Phase 3 — sim gate (easy seeds 1–5 × 1 run, speed 2, 5/5 completed; fresh anchor = v9-projfix easy 5×1: found 3.80, FP 1.2, coll 2.8, mean score 58.5):**

| seed | found | FP/run | coll | cov% | dup | score |
|------|-------|--------|------|------|-----|-------|
| 1 | 6/6 | 2.0 | 0 | 61.3 | 1 | 77.8 (B) |
| 2 | 3/6 | 5.0 | 1 | 69.0 | 2 | 48.8 (D) |
| 3 | 4/6 | 3.0 | 2 | 99.9 | 10 | 58.8 (C) |
| 4 | 5/6 | 2.0 | 0 | 71.6 | 8 | 64.6 (C) |
| 5 | 5/6 | 2.0 | 0 | 98.7 | 2 | 66.2 (C) |
| **mean** | **4.60/6** | **2.8** | **0.6** | | | **63.2** |

**Pre-registered escalate bar: found mean ≥4.9 AND FP/run ≤5.0 AND ≥2 completions. Result: found 4.60 (<4.9 — BINDING), FP 2.8 (≤5.0 ✓), 5/5 completions (✓) → REJECT, no 5×3 escalation.** Per pre-registration the association fix is KEPT in HEAD (correctness fix with zero-sim causality + 7/7 unit contract, not score-tuning); projection fix `8378792` kept either way.

Anchor context: mean score 63.2 vs 58.5; seed1 6/6 (77.8) is the best single easy run in the ledger; the forensics' original chimera seed converted 2/6→5/6. Regressions: seed2 3/6 vs 5/6 (v9), seed3 flat 4/6. FP 2.8/run vs anchor 1.2 — above anchor, consistent with the zero-sim A/B prediction (1.6) plus single-run noise.

**Gate attribution:** seed2's regression is NOT a split artifact (no ≥4-sighting near-dup chain on seed2 in Phase-1 replays) — 5 FP with dup 2 and cov 69% is confirm-gate behavior on sprawling partial-view tracks, cohort-variance territory. seed3's dup_count 10 at found 4/6 is the surviving chimera-lottery case: both split-halves exist but only one reaches the ≥2-sightings-in-0.5 m corroborated core inside the mission window, so the sibling stays unconfirmed. Residual FP pressure concentrates in the split's corroboration threshold, not the association radius.

Result JSONs: `robot-sandbox/results/submissions/derpbot-explorer-v10-assoc/easy_seed{1..5}_run1.json`; per-run streams recorded (`DERPBOT_RECORD_STREAM=1`, `stream_easy_seed*_run1_20261013T*.jsonl`). Runs + cleanup executed solo per invariant (one sim at a time; tmux/fds/anchor sessions killed, no stack processes, no stale flags at completion).

## 2026-10-13 — t_fd80a2f9: projection root-cause + ground-anchored depth clip — GATE REJECT (easy 5×1; projection defect FIXED, found-mass ceiling moved to the tracker confirm gate)

Agent: `derpbot-explorer-v9-projfix` @ `8378792` — tracker.py/detector.py untouched; the fix is entirely in `agent/depth_projector.py`: the 5×5 bbox-centre depth sample is clipped by the range implied by the bbox BOTTOM-edge ray hitting the floor (`min(depth, 1.35 × 0.25 m/tan(pitch_bottom))`, generic camera-geometry-derived, no class names).

**Phase 1 — root cause (zero-sim forensics, `results/projection_forensics.md`):** all 6 prior mechanism rejects traced to ONE defect: the bbox-centre depth patch reads the WALL BEHIND near objects (1215/3633 attributable sightings have depth > true range + 0.5 m; near-object err>3 m rows: depth median 3.92 m at GT range median 1.21 m). Bearing is clean (clean-row implied Δu median −0.2 px; implied object height constant 0.36–0.41 m across range = camera model exactly right). Operator addendum SLAM/odom-drift hypothesis REJECTED with numbers: corr(proj_err, elapsed sim-t) = +0.018 overall, per-run correlations mixed-sign and small (max +0.65 / −0.41 on different runs), early-vs-late-half error 1.28→1.35 m — a static geometry defect, not drift; no map re-anchoring warranted. FP corpus reclassified: 181/286 FPs across v4→v8 sit 1.5–3.5 m from a real same-class GT (the mis-projection band; ZERO FPs inside the window); 105/286 are genuine wrong-class hallucinations (detector-side, out of projection scope).

**Phase 3 — zero-sim A/B:** attribution-free per-sighting mean error 0.819→0.514 m (−61% ≥ 40% bar ✓), in-window ≤1.5 m share 0.79→0.93; 28-stream tracker replay v5-master semantics: FP/run 2.54→1.54, found 3.07→3.00 (merge-lottery flips both ways; corpus subset identical 3/3/1).

**Phase 4 — sim gate (easy seeds 1–5 × 1 run, speed 2, 5/5 runs no failures):**

| seed | found | FP/run | score |
|------|-------|--------|-------|
| 1 | 4/6 | 1.0 | 60.6 |
| 2 | 5/6 | 2.0 | 62.6 |
| 3 | 4/6 | 1.0 | 50.8 |
| 4 | 2/6 | 0.0 | 55.3 |
| 5 | 4/6 | 2.0 | 63.1 |
| **mean** | **3.80/6 (0.63)** | **1.20** | |

**Pre-registered escalate bar: found mean ≥4.9 AND FP/run ≤5.0 AND ≥2 completions. Result: found 3.80 (<4.9), FP 1.20 (best of any agent, was 4.8 on v5-master), 0 completions (<2) → REJECT. No escalation; mechanism KEPT in HEAD (projection defect fix is correctness, not score-tuning; FP/run 1.20 vs anchor 1.0-4.8 across the ledger).**

**Gate attribution (fresh per-run streams + replay, causality-checked):** projection accuracy on the gate runs is no longer the binding constraint — during in-FOV passes, attributable sightings (nearest same-class GT, any class) have median projection error 0.10–0.34 m and in-window 0.93–1.00 on ALL five gate streams. The found-mass now dies at the TRACKER:
- seed4 (2/6): two desk fire-extinguishers 0.96 m apart → Match-radius chaining merges them into one track (89 shared sighting timestamps); hi-conf (≥0.5) position spread 2.17–4.28 m > STABILITY_RADIUS 1.2 → gate-rejected (150 sightings at 0.03–0.07 m from GT wasted per desk FE). Same shape for both first_aid_kits (40 sightings at 0.04–0.07 m, spread 3.51 m, max conf 0.36).
- seed3/seed5 misses: 18–50 sightings each at 0.02–0.13 m from GT, "sightings existed and were potentially confirmable" per replay diagnose — confirm-gate lottery on merged/sprawling tracks, not projection.
- seed1/seed2/seed5 remaining miss: first_aid_kit#1 — NO sighting within 2 m on any gate run (detector recall bucket (c), already rejected at ROI gate; orthogonal).

**Next-lever candidates (for the orchestrator):** (1) tracker confirm gate on merged near-duplicate objects (per-sighting min-range candidate publish instead of running-mean centroid over a spread track — note t_6b03f03c already built and gate-rejected min-error publish under DIFFERENT pre-fix projection errors; worth re-eval now that positions are accurate); (2) STABILITY_RADIUS semantics when a track legitimately covers a close pair; (3) FA-kit low confidence (max 0.36 vs margin 0.0 — detector-side, NOT touched here). Tracker/detector remain OFF LIMITS on this task per scope.

## 2026-10-13 — t_8673a87f: slam first-map sentinel + one-restart recovery (score-neutral reliability fix, sim regression DEFERRED)

Reliability fix for the s3r2-class startup abort (`results/s3r2_collapse_diagnosis.md`): slam_toolbox silently activates without processing any scan → no `/map` within the legacy 120-s `STARTUP_MAP_TIMEOUT` window → explorer aborts while the robot sits still for the rest of the mission (s3r2: full 901.5-s budget burned, 0/6, robot never moved).

Mechanism (committed `ce3e972`): a pure-stdlib sentinel (`agent/slam_sentinel.py`, no ROS imports) runs inside the explorer's first-`/map` wait loop. Healthy runs (first-/map spans 3.6–19.5 sim-s; 116-run 2026-10 profile sample, p50 13.4, p95 19.4) never see any decision: the 45-s `SLAM_SENTINEL_GRACE_S` window is >2.3× the observed worst healthy startup. At one full grace window with no `/map`, the explorer requests exactly ONE slam_toolbox restart by writing `/tmp/derpbot_slam_restart_request` (diagnostic JSON incl. `scan_seen`: was the sim publishing `/derpbot_0/scan` at all?); `scripts/slam_restart_watch.sh` — armed by `scripts/start_stack.sh` as tmux session `slamwatch` (env marker `DERPBOT_SLAM_WATCHER_ARMED=1`) — consumes the flag and performs one clean kill+relaunch of the slam session (launch command byte-identical to start_stack.sh's). Exactly-once guarantees on all three layers: the sentinel state machine cannot emit two restart decisions; the watcher deletes the flag before restarting and exits after one restart; start_stack.sh deletes stale flags before arming. If `/map` still hasn't appeared after one further grace window (~90 sim-s worst case, vs 120 s legacy), the explorer aborts with a `startup_timeout` timeline tag exactly as before. Without the watcher env (manual dev runs) the sentinel goes straight to the abort path at one grace window — strictly less wasted budget than the legacy 120-s abort.

Unit tests (no sim): `tests/test_slam_sentinel.py` 10/10 (healthy-inside-window, late-map-after-restart recovery, exactly-one-restart across 10 windows, abort-after-restart at ~2×grace, no-watcher abort at one window with zero restarts, restart-flag JSON content + unwritable-path fallback, terminal-state behavior). `tests/test_slam_sentinel_wiring.py` 6/6 (explorer-loop restart/abort/timeline-tag flow with mocked clock, `DERPBOT_SLAM_WATCHER_ARMED` gating, scan diagnostic callback, grace-window sanity vs the 116-run healthy sample). Regression at `ce3e972`: sentinel 16/16, watchdog 3/3, tracker gates 8/8, replay fixture 10/10, ROI module import-gated skip.

**Sim regression (easy 3×1, seeds 1/4/5, `--agent-name derpbot-explorer-slam-sentinel`) — RUN 2026-10-13 14:28–14:55 after t_fd80a2f9 released the sim (one sim at a time honored; speed 2; pkg1 tmux anchor pin; 3/3 completed, 0 failed, 0 DOA):**

| seed | found | cov% | coll | score | grade |
|------|-------|------|------|-------|-------|
| 1 | 4/6 | 95.8 | 4.0 | 57.1 | C |
| 4 | 1/6 | 65.3 | 0.0 | 42.6 | D |
| 5 | 2/6 | 98.7 | 0.0 | 48.7 | D |
| **mean** | **2.33/6** | | | **49.5** | |

Sentinel behavior across all 3 runs: **fully passive** — startup timelines (`| startup |` profile rows: 17.5 / 16.2 / 12.8 sim-s) are identical in shape to pre-sentinel healthy runs (116-run sample p50 13.4 / p95 19.4), ZERO `slam_restart_request` or `startup_timeout` events in any stream, no restart flag written, slamwatch watcher exited clean at its timeout on every stack start (flag consumed: never). No behavior change when slam is healthy, confirmed live.

Score context (honest reading): single-run per-seed spread on this tree is wide; the 2026-10-12 v5-master 5×3 single-run spread for the same seeds includes 3/6 (53.4) and 4/6 (58.4) for seed 4, 3/6 (52.0) for seed 5, 5/6 for seed 1. This batch co-measures t_fd80a2f9's ground-anchored depth clip (kept in HEAD at `8378792`) plus its own variance, so it is a no-regression + health check, not a scored comparison: mean found 2.33 vs anchor per-seed-run values 3–6 shows no systematic startup damage (coverage 65–99%, all runs ran the full 900-s budget exploring). The seed-4 run (1/6, 4 FP-sparse) sits within observed single-run variance for that seed (v5-master 3.67 mean over 3 runs; v9 gate 2/6).

Forced-restart path validated LIVE beyond unit tests: with the post-regression stack (slam session still up), armed the watcher, wrote the restart flag manually, and observed the full contract: flag detected → consumed → slam session killed → relaunched with the fixed self-contained ROS env → slam_toolbox booted and stayed alive. This uncovered a real bug — the original relaunch line died with `ros2: command not found` when the watcher was armed from a non-ROS shell (new tmux sessions merge the CALLING client env over the server env) — fixed at `39baadc` (relaunch sources `/opt/ros/jazzy/setup.bash` itself) and re-verified end-to-end. So the restart-once path WAS forced-tested live, not only in unit tests. Healthy-sim happy path (sentinel passive, agent explores normally) validated by the 3 regression runs above.

---

## 2026-10-13 — t_5e2d3407: close-in verification detour + stop-and-stare hold — GATE REJECT (easy 5×1, mechanism reverted)

Agent: `derpbot-explorer-v8-closein` @ `6ec2c61` — v5-master semantics otherwise unchanged, tracker.py/detector.py untouched. Mechanism (operator-steered scope, forensics buckets b=4 glimpse + c=8 dwelled): detection detours become approach-to-close-range — on successful detour arrival with the track still pending, hold `CLOSE_IN_HOLD_S=2.0` sim-s (zero Twist, sim-clock deadline, early-exit on tracker confirm); guard rails `CLOSE_IN_MAX_DETOURS=6`/run, per-track rate limit 60 sim-s, per-track attempt cap 2, unknown/lethal costmap-cell skip; all events tagged `close_in_*` timeline phases + `source=close_in_verify`. Spec: task workspace `close_in_verify_spec.md` (spec-first). Zero-sim stream measurement in the spec: all 12 (b)/(c) target events have ZERO class sightings ≤2 m of GT (nearest sighting-to-GT 2.48–8.55 m; forensics-module GT transform) — candidate positions carry 2.5–8.6 m depth-projection error, pre-registered as the reachability risk.

Unit tests `tests/test_frontier_closein.py`: 25/25 (budget cap + once-only tag, attempt cap, rate limit + window expiry, costmap guard incl. -1/unknown/extent/no-snapshot, hold duration/early-exit/tagging). Regression at `6ec2c61`: watchdog 3/3, tracker gates 8/8, replay fixture 10/10. Incidental fix kept after revert: test_detector_roi.py module-level pytest skip (parent metadata said import-gated; gate was missing, broke full-suite collection).

Gate (easy seeds 1–5 × 1 run, speed 2, pinned pkg1 tmux anchor, 5/5 completed, 0 failed):

| seed | found | FP/run | col | cov% | score |
|------|-------|--------|-----|------|-------|
| 1 | 5/6 | 3.0 | 9.0 | 96.0 | 56.8 |
| 2 | 4/6 | 3.0 | 0.0 | 98.7 | 62.3 |
| 3 | 3/6 | 1.0 | 2.0 | 98.7 | 56.0 |
| 4 | 3/6 | 2.0 | 2.0 | 98.8 | 53.2 |
| 5 | 5/6 | 2.0 | 0.0 | 98.7 | 68.5 |
| **mean** | **4.00/6** | **2.2** | | | |

**Pre-registered escalate bar: found mean ≥4.9 AND FP/run ≤5.0 AND ≥2 completions. Result: found 4.00 (<4.9), FP 2.2 (≤5.0 ✓), 0 completions (<2) → REJECT.** Mechanism reverted at `5514194` (tests import-gated + spec + attribution kept).

Attribution (timeline `close_in_*` events × submission_log, causality-checked by sim-time):
- Mechanism fired sparingly: 11 close-in detours across 5 runs (budget never exhausted; max 3/6), 9 stop-and-stare holds (2.0–7.2 s each; total ≤20 s across all runs — mission-clock cost negligible). All holds exited `confirmed=False` — no candidate was EVER confirmed by the hold itself.
- Causality-checked conversions: seed2 track_1 (FE) TP at t=57 during its detour window 51.9→82.3 (1 attributable conversion, approach phase); seed5 track_5 (FE) TP t=142 after detour@38+hold@70.6 (probable, post-hold). All other detoured candidates never confirmed; seed4's apparent track-id matches were time-disqualified (incl. one tracker-worker-restart id collision — track_8 person TP@t=118 vs FE detour@t=654.7).
- The FP bar improvement (2.2 vs v5-master 4.5) is NOT attributable to this mechanism (found unchanged); treat as cohort variance.
- Root cause the mechanism cannot touch: candidate positions are 2.5–8.6 m off-object (depth-projection), so the robot approaches a false position — the object is rarely in the camera's close-range cone when the hold fires. Consistent with the pre-registered reachability risk. Combined with the (c)-bucket ROI finding, the evidence now says: the recall surface that matters is BETTER PROJECTED CANDIDATE POSITIONS (depth/bbox geometry), not more dwell time near wrong positions.
- (b)-bucket absences across this gate: only 1 mission-target absence (no class sighting ≤2 m of GT) in 30 GT-targets — s4 GT3 FE. Mechanism-attributable absences did not measurably drop vs the forensics corpus on this cohort (single runs, high variance — no conclusion drawn).

Result JSONs: `robot-sandbox/results/submissions/derpbot-explorer-v8-closein/easy_seed{1..5}_run1.json`; streams: `results/streams/stream_easy_seed*_run1_20261013T1*.jsonl`; timelines: `results/profile_20261013T12*.md`, `profile_20261013T130508.md`. Tree = v5-master semantics 4/15 (no change).

---

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