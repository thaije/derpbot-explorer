# Detector swap benchmark — comprehensive candidate sweep (t_bbe085d2)

Date: 2026-10-18 → 2026-10-19 · Author: coder (autonomous) · Method: zero-sim offline eval
Dataset: `results/recog_benchmark/` (4633 frames / 8473 instances; corpus 4081 / holdout 514 / stress 38)
Harness: `scripts/eval_recog.py` · FP-proxy mode: `gt_within_radius`, `--fp-conf 0.15` (bake-off operating point) in every row unless stated

Parent: t_fe1459a9 (dataset + harness, approved round 5). This card is the
operator-directed Phase 3 sweep. **The holdout split (seeds 6–10, v2 1080p,
live-captured) is the decision column** — it was never touched by any
mechanism iteration. Corpus rows are continuity; stress rows are robustness.

## Pre-registered decision bar (standing, t_b4361707)

A candidate replaces the OWLv2 baseline only if, at equal-or-better
FP-proxy/frame: **FAK rate ≥ 1.5× baseline AND min(FE, person) ≥ 1.0× baseline.**
Evaluated at the candidate's own suggested conf AND at matched-FP operating
points; the bar is applied per conf column from the full-rate curve, exactly
as `--bar bakeoff` does in-harness (same greedy matching, same FP definition —
verified below).

## Reproducibility

- Evaluated with the committed harness at `--fp-mode gt_within_radius --fp-conf 0.15`
  (holdout = `--split holdout`). Baseline row re-measured on the current
  manifest: FE 0.9489 / FAK 0.5706 / person 0.9478 @0.15, FP-proxy 1.0331/frame,
  latency ≈ 379 ms — consistent with the parent task's corpus reference
  (`r4_mission_bakedef` FE 0.9091 / FAK 0.5303 / person 0.8614, FP 0.3475, ~258 ms).
  (Holdout and corpus are different frames; magnitudes agree.)
- Bar mirror: `bar_eval.py` (task workspace copy archived below) reproduces the
  harness FP block from `frame_records.jsonl`; re-derivation matches the stored
  per-frame `fp` field on **514/514 frames for all six GPU runs (0 mismatches)**,
  and reproduces the baseline's `fp_proxy_per_frame` 1.0331 exactly.
- GPU: RTX 8 GB is the model-side constraint. Latency = wall per frame
  (single stream, GPU 0). pkg1 CPU pinning used for the CPU-only passes.
- Known label noise (parent docs §Known limitation): person jacket containment
  (7/251 frames), FE near-range boxes without red blob (144/716, 35 holdout),
  occlusion-unaware furniture labels. **Differences < 0.05 rate between
  candidates are treated as noise, not signal.**

## Holdout decision table (514 frames, gt_within_radius @0.15 unless noted)

| # | Candidate | FE @0.15 | FAK @0.15 | person @0.15 | FP-proxy/frame | lat ms | Verdict |
|---|-----------|----------|-----------|--------------|----------------|--------|---------|
| b1 | **owlv2-base-p16 (baseline, live)** | 0.9489 | 0.5706 | 0.9478 | 1.0331 | 379 | reference |
| b2 | owlv2-base-p16 (raw, non-ens) | 0.9649 | 0.2883 | 0.4776 | 1.4320 | 404 | ✗ FAK 0.51×, FP ↑ |
| b3 | owlv2-large-p14-ens | 0.9553 | 0.6012 | 0.1791 | 0.9339 | 1362 | ✗ person collapse |
| b4 | grounding-dino-tiny | 0.9265 | 0.2393 | 0.9925 | 2.1770 | 332 | ✗ FAK 0.42×, FP ×2.1 |
| b5 | grounding-dino-base | 0.9489 | 0.3067 | 1.0000 | 0.4436 | 433 | ✗ FAK 0.54× |
| b6 | yolo-world-l | 0.6134 | 0.0000 | 1.0000 | 0.4222 | 18 | ✗ no FAK, FE 0.65× |
| b7 | florence-2-large (per-prompt CPG) | 0.3355 | 0.4969 | 0.3433 | 13.7101 | 5580 (CPU) | ✗ over-detects |
| b8 | mm-grounding-dino-tiny (o365v1+goldg+v3det) | 0.9233 | 0.2270 | 0.9254 | **0.2218** | 329 | ✗ FAK 0.40× |
| vlm1 | glm-5.3-flash (94-frame stratified subset) | 0.9796 | 0.6364 | 1.0000 | 1.0106 | 5874 | bar legs pass on subset (n=94) |
| vlm2 | qwen3-vl:235b | incomplete (registry) | — | — | — | — | see VLM ladder |
| vlm3 | kimi-k3:cloud (94-frame subset) | 0.9796 | 0.5000 | 1.0000 | 0.8617 | 11727 | bar legs pass on subset (n=94) |

Notes per candidate:
- **b2** is the raw (non-ensemble) checkpoint of the SAME size the agent runs:
  strictly worse everywhere — the deployed ensemble choice is validated.
- **b3** (largest OWLv2 that fits 8 GB): FAK 1.05× baseline and better FP, but
  person rate collapses 0.9478 → 0.1791 @0.15 (person confs sit just below the
  bar) — fails the min(FE, person) ≥ 1.0× leg by a wide margin.
- **b4/b5** (G-DINO at raised thresholds): the FP ×3.5 was indeed a conf-0.15
  artifact on Corpus; on HOLDOUT b4 stays FP-inflated (2.18/frame) and b5's
  improvement (FP 0.4436, FE +0.000, person 1.0, **trash_can 0.0** — a
  class-blind spot) does not fix FAK: **0.3067 = 0.54× baseline**. The G-DINO
  conf distribution is sharply peaked (FAK conf_med 0.4296, p90 0.4628): it
  detects few FAKs at any threshold.
- **b8** (task item 7 scan — see "HF scan" below): the cleanest detector of the
  sweep — FP-proxy **0.2218/frame at 0.15 (4.7× better than baseline)** with
  FE 0.9233 / person 0.9254. **Matched-FP operating point (conf 0.30):** FAK
  0.1840 = 4.3× the baseline's 0.0429 ✓, person 0.9254 ≥ 0.3881 ✓, FP 0.1848 ≤
  0.4475 ✓ — the only failing leg is FE 0.9201 vs baseline 0.9457, a 0.026 gap
  that sits inside the documented sub-0.05 label-noise band. The bar verdict is
  FAILED as registered (the FE leg binds), but this is the one local operating
  point where every other leg clears with margin — the strongest near-miss of
  the sweep, and the reason b8 ranks first among local detectors despite the
  conf-0.15 FAK reading.
- **b7** Florence-2: per-prompt CAPTION_TO_PHRASE_GROUNDING returns uncalibrated
  constant conf=1.0 and over-detects (14.3 dets/frame; FP-proxy 13.7/frame);
  not salvageable without a confidence model.

### The matched-FP operating-point trap (b3/b5/b8 at conf 0.5)

At conf 0.5 the BASELINE's own FAK rate is 0.0 (0.0429 @0.30 → 0 baseline @0.5);
the bar's 1.5× leg reduces to ≥0 and the other legs to ≥0 — so **the bar letter
"clears" trivially at the 0.5 knee for b3, b5, b8** (all keep FE/person above the
baseline while FP stays below). This is a degenerate comparison at the baseline's
weakest column, not a win: the matched-FP reading of the same rows (above, at
0.15 where the baseline actually detects FAK) is what fails. Ranked at matched
FP, **no candidate clears the bar**.

## VLM ladder (operator cost protocol: glm-5.3-flash → qwen3-vl:235b → kimi)

≤300 stratified frames total; strict single-parse structured output; ≤2 retries;
429 = terminal for the candidate; >2 s/frame ⇒ also score as low-cadence
second-pass hybrid. Stratified sample = **94 frames** (class-visible × range
bins + zero-object negatives; `vlm_sample_ids.json`).

### vlm1 — glm-5.3-flash (94/94, 0 parse fails, 0 errors, no 429s)

| class | n_gt | rate@0.15 | rate@0.50 | conf_med |
|---|---|---|---|---|
| fire_extinguisher | 49 | 0.9796 | 0.9796 | 0.95 |
| first_aid_kit | 22 | **0.6364** | 0.5455 | 0.95 |
| person | 41 | 1.0000 | 1.0000 | 0.95 |
| office_chair | 27 | 1.0000 | 1.0000 | 0.95 |
| trash_can | 23 | 0.8261 | 0.8261 | 0.92 |
| drill | 16 | 0.5625 | 0.5625 | 0.93 |
| drink_can | 21 | 0.0952 | 0.0476 | 0.60 |
| suitcase | 10 | 0.0 | 0.0 | — |

FP-proxy **1.0106/frame** @0.15 (round-1 publication said 1.4362 — inflated
by the scorer bug fixed in round 2, below), latency 5.9 s median (wall). On
the SAME 94 frames the OWLv2 baseline scores FE 0.9796 / **FAK 0.3182** /
person 1.0000, **FP 1.2128/frame** (round-1 publication said 2.1064).
glm-5.3-flash: FAK **0.6364 = 2.00× baseline** ✓ (≥1.5×), FE 1.00× ✓,
person 1.00× ✓ — FP-proxy 1.0106 vs baseline 1.2128 ✓ — **all three bar legs
pass on the 94-frame subset** (single-parse protocol, 0 parse failures).

### vlm3 — kimi-k3:cloud (94/94, 0 parse fails, 0 errors, no 429s)

Gate check: glm-5.3-flash already shows a real grounding signal (all three bar
legs pass on the subset) → per the binding, kimi-k3 was run to test whether
Extra-High tokens buy the remaining FP problem.

| class | n_gt | rate@0.15 | conf_med |
|---|---|---|---|
| fire_extinguisher | 49 | 0.9796 | 0.98 |
| first_aid_kit | 22 | 0.5000 | 0.98 |
| person | 41 | 1.0000 | 0.99 |
| office_chair | 27 | 1.0000 | 0.99 |
| trash_can | 23 | 0.8261 | 0.95 |
| drill | 16 | 0.6875 | 0.96 |
| drink_can | 21 | 0.1905 | 0.55 |
| suitcase | 10 | 0.2000 | 0.82 |

FP-proxy **0.8617/frame** @0.15 (corrected from the round-1 1.2660, see the
round-2 note below), latency 11.7 s/frame (wall). vs subset
baseline (FAK 0.3182, FP 1.2128): FAK 1.57× ✓, FE 1.00× ✓, person 1.00× ✓,
FP 0.71× ✓ — **all legs pass**, like glm. Standalone kimi actually improves on
glm's FP too (0.8617 vs 1.0106) at the same FAK trade (0.500 vs 0.636) — but
at 2× the latency and Extra-High token cost. Hybrids (corrected, task-workspace
`kimi_hybrid.json` + `hybrid_scores_v2.json`): +kimi@0.5 → FAK 0.5 (1.57×),
FP 1.5319; +kimi@0.9 → FAK 0.4091 (1.29×), FP 1.3085 — same
FAK-gain-vs-FP-cost coupling as the glm hybrid; neither hybrid beats the
baseline FP (1.2128) at the conf that delivers the FAK gain.

### vlm2 — qwen3-vl:235b — REGISTRY-RETIRED, mark incomplete

`ollama show qwen3-vl:235b` → **"qwen3-vl:235b was retired on June 16, 2026"**
(ref 7d975908-42ba-4da3-9149-d6133339b98b). The ladder's rung 2 as written does
not exist in the registry any more; local `qwen3-vl:8b` was pulled as the
nearest substitute. 10/94 frames done before the two prior runs died
(8 scored, 2 timeouts at 240 s cap; ~2.5 min/frame — GPU was shared with the
detector passes; solo-local is faster but still ≫ 2 s/frame). Scored partial:
rate columns are unusable at n=10 (candidate marked **incomplete** per the
≤2-retry protocol).

### vlm3 — kimi-k3:cloud — (run launched after glm cleared the bar; results below)

### Hybrid second-pass configurations (94-frame subset, scored)

Protocol: OWLv2 primary at its live operating point (conf ≥ 0.15) at ~1.5 Hz,
VLM second pass merged on top (VLM det dropped if IoU ≥ 0.5 with a same-class
OWLv2 det).

| config | FE | FAK | person | FP-proxy/frame |
|---|---|---|---|---|
| OWLv2 only (subset) | 0.9796 (48/49) | 0.3182 (7/22) | 1.0000 (41/41) | **1.2128** |
| + glm@conf 0.9 | 0.9796 | 0.4091 (1.29×) | 1.0000 | 1.2872 |
| **+ glm@conf 0.5** | 0.9796 | **0.5455 (1.71×)** | 1.0000 | 1.6277 |
| + glm@conf 0.15 (all) | 0.9796 | 0.6364 (2.00×) | 1.0000 | 1.7234 |

(The round-1 table published FP 2.1064 / 2.1809 / 2.6915 for the first three
rows and left the @0.15 row unscored — all inflated by the scorer bug fixed in
round 2, see below. All six hybrid configs, glm and kimi, are now scored in
task-workspace `hybrid_scores_v2.json`.)

The hybrid second pass buys real FAK recall (1.71× at conf 0.5, 2.00× with all
glm dets) at the cost of the FP-proxy leg (1.6277 > 1.2128, 1.42× at the
all-dets floor) — the merged VLM detections that lift FAK are the same ones
the FP gate penalizes. This is the scored low-cadence configuration; it does
NOT hit equal-or-better FP at any conf that delivers the FAK gain (conf 0.9
does approach FP parity at 1.06× but its FAK gain shrinks to 1.29×, below the
1.5× leg).

## HF scan (task item 7)

- **MM-Grounding-DINO** (arXiv:2401.02361, "An Open and Comprehensive Pipeline
  for Unified Object Grounding and Detection") — open-source retraining of
  Grounding-DINO with GRIT+V3Det substituting the non-open Cap4M; published
  zero-shot LVIS/ODinW gains over G-DINO-T. Clean dependency (one
  `transformers` class, weights on HF: `openmmlab-community/mm_grounding_dino_tiny_o365v1_goldg_v3det`),
  fits 8 GB (2.6 GB VRAM measured). **Evaluated above as b8** — strongest FP in
  the sweep, fails the FAK leg.
- Florence-2 (arXiv:2311.06242) — evaluated above as b7.
- No further open-vocab zero-shot detector with a clean small dependency and a
  published strong zero-shot row was found that is not already in the tested
  set (OWLv2/G-DINO/YOLO-World cover the deployable families; larger models
  either exceed 8 GB or are API VLMs covered by the ladder).
- Original Grounding-DINO (arXiv:2303.05499) was already in the set (b4/b5).

## Verdict (ranked)

1. **No local detector clears the pre-registered bar at matched FP.** The FAK
   leg kills every one at conf 0.15 (best 1.05×, b3 — which fails person ≥ 1.0×).
   Closest near-miss: **b8 at conf 0.30** (FAK 4.3×, person ✓, FP ✓ by 2.4×;
   only FE fails, by 0.026 — inside the noise band documented above).
2. **b8 mm-GDINO-tiny-v3det is the strongest local detector** (task-item-7 HF
   scan): FP-proxy 0.2218 holdout / 0.1800 full-corpus / 0.1579 stress vs
   baseline 1.0331 (corpus 0.8042) — 4.6× better, with FE 0.7555 / person
   0.9031 on corpus (baseline corpus: FE 0.9113 / FAK 0.4901 / person 0.8699,
   FP 0.8042). FAK 0.3227 = 0.66× baseline on corpus — **fails the FAK leg;
   a candidate for a PRECISION-oriented config if the bar is ever re-registered
   for a low-FP objective (do NOT swap under the current bar).** Stress row:
   FAK 0.0 (14 gt) — G-DINO-family FAK blind spot persists under stress.
3. **glm-5.3-flash passes all three bar legs standalone on the 94-frame
   stratified subset** (FAK 2.00×, FE/person parity, FP 1.0106 = 0.83× the
   corrected subset baseline 1.2128) — but at 5.9 s/frame it is
   a <0.5 Hz candidate, i.e. a **second-pass hybrid** by the operator's own
   protocol, and the scored hybrid (FP 1.6277 = 1.34× baseline @FAK 1.71×;
   round-1 "FP 2.69 @FAK 1.71×" was inflated by the scorer bug) does NOT beat
   the baseline's FP at the operating point that clears FAK. The two effects
   (FAK gain vs FP cost inside the same merge) are the same detections.
4. **kimi-k3 was gated on "the first two show a real grounding signal worth
   Extra-High tokens":** glm-5.3-flash shows that signal (all bar legs pass on
   the subset) → the gate fired → kimi-k3 run (94/94, above). It passes all
   legs too but does NOT buy the FP problem out of the hybrid coupling, at 2×
   glm's latency and Extra-High token cost — no upgrade case over glm.
   qwen3-vl:235b: registry-retired → candidate marked incomplete (protocol).

## HANDOFF (per deliverable spec: name + config + interface plan only — implementation is a separate task)

- **Candidate:** glm-5.3-flash (ollama cloud) as a low-cadence second-pass VLM
  verifier, PRIMARY REMAINING AT OWLv2 (owlv2-base-patch16-ensemble, unchanged).
- **Config:** 94-frame stratified sampling cadence (class-visible × range bins,
  ~1 frame / 5 s of stream), strict single-parse structured output prompt
  (`vlm_ladder.py build_prompt`), conf floor 0.5 on merged dets, merge rule
  "drop VLM det with IoU ≥ 0.5 vs same-class OWLv2 det", ≤2 retries, 429 =
  skip pass (primary continues unprotected), parse-fail counted against pass rate.
- **Backend-interface plan:** extend the detector backend contract
  (`init(prompts)` / `infer(bgr) -> [{cls, conf, bbox}]`) with a SECOND-PASS
  mixin: `init_secondary(prompts, cadence)`, `infer_secondary(bgr, frame_meta)
  -> [{cls, conf, bbox}] | None` (None = this frame not sampled); the tracker
  fuses secondary dets with provenance tagging so the sim gate (v13) can credit
  FAKs and charge FPs separately per source. Gate candidate: v13 with the
  pre-registered bar re-checked on the FUSED stream (this bench's subset is
  n=94 — the sim gate is the only place the FP leg can be judged at full rate).
- **Do not implement in this task** (binding): recorded here as handoff only.

## Artifacts

- `results/eval_recog/b1..b8*` — harness runs (summary.json + frame_records.jsonl + README)
- `results/eval_recog/b8_mmgdino_full4633` (4633 frames) + `b8_mmgdino_stress`
  (38 frames) — corpus/stress continuity + robustness rows for the HF-scan candidate
- Task workspace `bar_eval.json` — per-conf bar legs + FP curves (mirror-validated)
- Task workspace `glm_score.json`, `kimi_score.json`, `kimi_hybrid.json`,
  `hybrid_scores_v2.json`, `florence2_score.json`, `qwen8b_partial_score.json`,
  `subset_scores.json` (subset baselines + hybrid configs); raw ladder outputs
  archived in the task workspace as `vlm_glm_probe.jsonl` (94),
  `vlm_kimi94.jsonl` (94), plus `/tmp/vlm_qwen3vl8b.jsonl` (10, volatile).

## Round-2 correction (review round 1, artifact lens)

The round-1 VLM/hybrid FP-proxy numbers were inflated by a scorer bug:
`vlm_score.py` (and the aggregate hybrid script) resolved the per-run GT JSONs
under `derpbot-explorer/results/submissions/`, which does not exist — the GTs
live in the robot-sandbox submission dirs used by the harness's `GT_DIRS`. The
missing-dir `FileNotFoundError` was swallowed, so the `gt_within_radius`
suppression never fired and every unmatched det @0.15 counted as FP. Fix:
GT source repointed to the harness's dirs (+ loud failure if a run's GT is
missing); all affected configs re-scored. Corrected numbers (rate columns were
never affected — no verdict flips):

| config | FP-proxy/frame round-1 (buggy) | corrected |
|---|---|---|
| subset OWLv2 baseline (94 frames) | 2.1064 | **1.2128** |
| glm-5.3-flash standalone | 1.4362 | **1.0106** |
| kimi-k3 standalone | 1.2660 | **0.8617** |
| hybrid +glm@0.9 | 2.1809 | **1.2872** |
| hybrid +glm@0.5 | 2.6915 | **1.6277** |
| hybrid +glm@0.15 (all) | unscored | **1.7234** |
| hybrid +kimi@0.9 | 2.2447 | **1.3085** |
| hybrid +kimi@0.5 | 2.5851 | **1.5319** |
| hybrid +kimi@0.15 (all) | unscored | **1.5638** |

Corrected-vs-buggy FP was re-derived independently (fresh matcher
implementation over `frame_records.jsonl` + manifest + robot-sandbox GT dirs)
with exact agreement on every row above. Verdict unchanged in substance:
standalone glm/kimi pass all bar legs on the subset (more strongly than first
published); the hybrids still couple FAK gain to FP cost (no hybrid reaches
FP ≤ 1.2128 at a conf with FAK ≥ 1.5×); the "+ glm@conf 0.15" row is now scored
(FAK 2.00×, FP 1.7234) replacing the round-1 unscored "0.7727 (2.43×)?" entry,
which cited a subset file that did not contain it. The kimi-hybrid multiplier
"2.27×" was stale; corrected to 1.57× in §vlm3.
- Harness change (scripts/eval_recog.py): mm_gdino + florence2 backend wiring
  (task 7 + b7), committed (8be6b9c, 8e4f8fa).