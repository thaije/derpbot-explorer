# Generic detector bake-off — offline keyframe evaluation (t_b4361707, Phase 1)

Date: 2026-10-17 · Author: coder (autonomous) · Method: zero-sim, offline
Repro: `.venv/bin/python scripts/bakeoff_run.py --models owlv2 yolo_world_s gdino_tiny [--pose-tol 2.0]`
Raw per-frame scores: `results/bakeoff/frame_scores_<model>.jsonl`, summaries `results/bakeoff/bakeoff_summary.json` (+ `results/bakeoff_tol2/` for the ±2.0 s sensitivity run).

## Protocol

- Corpus: every keyframe in `results/streams/frames/` — 28 runs (all easy-tier 5×5
  runs of v10/v11, i.e. 15 v10-assoc + 13 from 10-13/10-16 sessions), 5396 keyframes.
- Pose pairing: robot pose (odom `rx, ry, ryaw`) from the run's sighting stream at
  matched sim-time ±0.5 s (strict) — 1435 keyframes qualify. Sensitivity re-run at
  ±2.0 s (2228 keyframes) to check that the verdict is not a pairing artifact.
- GT: recorded per-run `ground_truth_objects` from the v10-assoc submission JSONs
  (mission targets: fire_extinguisher ×3, first_aid_kit ×2, person ×1 per run —
  seed-dependent placements; verified identical across the submissions that carry
  raw GT; independently re-established via robot-spawn-aware `ObjectPlacer` replay).
- Visibility (per frame, per GT instance): horizontal bearing within the URDF 90°
  hFOV (+2 mrad slack), wall-clear LOS on the GT PGM (bresenham, 0.5 m/cell),
  distance ≤ 8 m. Pose→world mapping and pinhole projection (hFOV 90°, camera
  z 0.25 m, per-class center height) were validated by replaying every recorded
  sighting of 6 runs: 84 % bearing agreement ≤ 0.10 rad, with the non-agreeing
  remainder being the known low-conf FP sightings (0.15–0.24 conf projections).
- Prompts (mission-derived, identical for all models): `fire extinguisher`,
  `first aid kit`, `person` (underscore→space conversion mirrors the live pipeline).
- Detection rule: conf ≥ 0.15 (live pipeline's threshold); a visible GT instance
  counts detected when some detection of that class has bearing error ≤ 0.20 rad.
- FP proxy: detection with no same-class GT instance within 2 m of the camera.
- HW: RTX 2070 SUPER (GPU run; the deployment target is the same GPU — pkg1 CPU
  figures not measured live, the relative comparison is unaffected).

## Results — 1435 keyframes, strict ±0.5 s pairing

| model | FE rate | FE conf med | FAK rate | FAK conf med | person rate | person conf med | FP-proxy/frame | lat med (p90) ms |
|---|---|---|---|---|---|---|---|---|
| OWLv2 (baseline) | 0.826 | 0.57 | 0.319 | 0.24 | 0.741 | 0.24 | 0.271 | 289 (313) |
| YOLO-World-S | 0.529 (×0.64) | 0.44 | **0.000** | — | 0.779 (×1.05) | 0.85 | 0.213 | **42 (55)** |
| Grounding-DINO-tiny | 0.812 (×0.98) | 0.76 | **0.585 (×1.83)** | 0.57 | 0.888 (×1.20) | 0.81 | **0.946 (×3.5)** | 328 (439) |

## Sensitivity — ±2.0 s pairing (2228 keyframes)

Broader pose pairing admits the far-from-sighting keyframes with slightly less
accurate geometry; verdict unchanged:

| model | FE rate | FAK rate | person rate | FP-proxy/frame |
|---|---|---|---|---|
| OWLv2 | 0.747 | 0.273 | 0.651 | 0.266 |
| YOLO-World-S | 0.457 (×0.61) | **0.000** | 0.717 | 0.226 |
| Grounding-DINO-tiny | 0.729 (×0.98) | 0.539 (×1.97) | 0.823 (×1.26) | **0.959 (×3.6)** |

## Pre-registered swap bar

The pre-registered bar (task body, Phase 1) is: winner must show
**FAK rate ≥ 1.5× OWLv2 AND FE+person ≥ 1.0× AND FP-proxy ≤ OWLv2** (all three).

Under-criterion results, both readings (the first leg is stated both ways here
because a strictly-FE form of that leg is mathematically vacuous — OWLv2 FE is
0.826, so ≥ 1.5× would demand FE ≥ 1.24 > 1.0, impossible; the pre-registered
FAK reading is the operative one):

| leg | requirement | YOLO-World-S | G-DINO-tiny |
|---|---|---|---|
| FAK ≥ 1.5× OWLv2 (pre-registered) | ≥ 0.478 | 0.000 (×0.00) — FAIL | 0.585 (×1.83) — pass |
| (FE ≥ 1.5× OWLv2 — vacuous form) | ≥ 1.240 | 0.529 — FAIL (leg unsatisfiable for all) | 0.812 — FAIL (leg unsatisfiable for all) |
| FE+person ≥ 1.0× OWLv2 (rate product) | ≥ 0.613 | 0.412 (×0.85) — FAIL | 0.721 (×1.18) — pass |
| FP-proxy ≤ OWLv2 (per frame) | ≤ 0.271 | 0.213 — pass | 0.946 (×3.5) — **FAIL** |

- YOLO-World-S: **FAIL** (all three legs); FAK visibility is a hard zero
  (512+ visible instances, no detection at any conf — probe at conf 0.02 with
  6 alternate phrasings still produced nothing: YOLO-World's closed-caption-aligned
  vocabulary does not cover the flat red box in this renderer at all).
- Grounding-DINO-tiny: clears the pre-registered FAK leg (×1.83) and the
  FE+person leg (×1.18) — but **FAIL**s the FP-proxy leg (×3.5); FAK gain is
  inseparable from its FP cost at the 0.15 threshold. Two-leg clearing is not
  enough: the bar is AND-of-three, so the verdict is FAIL.

**Verdict: no candidate clears the swap bar. Detector-swap is measured-negative.**
Per the pre-registration: STOP here — no Phase 2 backend work, no Phase 3 gate;
default detector stays OWLv2.

## Interpretation (attribution)

- The FAK sighting-mass collapse is a BASELINE property of OWLv2 on this corpus
  (0.319 instance-rate, conf 0.15–0.46), and both alternatives fail on it too:
  YOLO-World cannot see FAK at any prompt, and G-DINO trades a FAK gain for a
  3.5× FP explosion at the 0.15 threshold (its low-conf tail is long).
- The person rate is high for G-DINO/YOLO-World (0.89/0.78 vs 0.74) with much
  better confidence, so if the person timing lottery is the primary blocker a
  *person-only hybrid* would help — but that means running two detectors per frame
  (latency 289+328 ms serial, >0.5 Hz HALT bar on shared GPU) and does not touch
  the FAK collapse without G-DINO's FP cost. Not cleared by the pre-registration.
- Zero-sim caveat: keyframes are trigger-gated (candidate-dense phases dominate),
  so rates are per visible GT *instance*; frames where a GT is geometrically
  in-FOV but occluded by furniture (PGM only models walls) count as positives for
  every model equally — comparative verdicts unaffected; absolute rates are
  lower bounds.

## Decision

Measured-negative on the pre-registered bar → detector-swap closed. Remaining
agent-side lever for the FAK collapse: none identified by this bake-off (G-DINO's
FAK gain is real but inseparable from its FP cost under a single-model setup);
testbed resources (resolution/texture richness) become the only remaining lever.