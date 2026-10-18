# Labeled recognition benchmark from sim keyframes (t_fe1459a9)

Date: 2026-10-17 → 2026-10-18 · Author: coder (autonomous) · Method: zero-sim offline eval

A reusable labeled object-recognition benchmark built from the sim's auto-GT:
every keyframe gets its visible-GT instances (world→camera projection, 90° hFOV,
LOS on the GT PGM, range ≤ 6 m), projected pixel bbox, range/class/bin metadata.
Any prompt-driven open-vocab backend can be scored against it in minutes with
`scripts/eval_recog.py` — no sim required.

```
results/recog_benchmark/
  manifest.jsonl        1 line per keyframe: image, camera model, robot pose,
                        objects=[{key, cls, mission, range, bearing, bbox,
                        range_bin, area_bin, area_px}]
  dataset_stats.json    frames/instances/runs/class-bins + config + prompts
  generated_gt/         per-run GT for runs with no result JSON (stress splits)
  split_corpus.txt      57 runs (seeds 1–5 easy, v1 + v2 FULLRES; tuned-on data)
  split_holdout.txt     5 runs (seeds 6–10, sandbox v2, FULLRES — Phase 2,
                        never touched by any mechanism iteration)
  split_stress.txt      2 runs (perception_stress seeds 6–7 — Phase 2)
```

## Dataset

| | |
|---|---|
| frames | **4817 of 15304 stored keyframes** — the retained frames have an in-tolerance pose pairing; **372 of them are zero-object frames** (327 corpus, 45 holdout) with only negative labels. `dataset_stats.json` reports both total frames (4817) and per-run object-bearing frames (`run_list[*].frames`) |
| visible-GT instances | **8764** |
| splits | corpus 4256 frames / 57 runs · holdout 523 frames / 5 runs · stress 38 frames / 2 runs |
| GT source | v10-assoc / stream-corpus / v2holdout result JSONs (`ground_truth_objects`, all 8 renderable scenario classes; `mission_target` flag preserved); stress runs use `generated_gt/` (deterministic WorldGenerator replay, ≤0.0004 m vs in-sim SDF) |
| pose pairing | sighting-stream pose at matched sim-time ±0.5 s, odom→world via spawn header (bake-off pipeline, 84 % bearing agreement) |
| camera provenance | per-run stream-header `camera` block (v2: fx≈960 @ 1920×1080 from camera_info, 3112 frames incl. holdout+stress, `source=camera_info`) or the v1 640×480 pinhole constants (1705 frames, `source=v1_no_header`); bboxes/areas in the STORED keyframe resolution |
| labels per class | fire_extinguisher 3494 · first_aid_kit 1880 · office_chair 908 · person 763 · drill 571 · drink_can 478 · trash_can 403 · suitcase 266 (exit_sign: 0 visible — SDF-posed at z = 1.25 m on the office door headers `wall_door_header_*`; the PGM wall-clearance LOS test blocks it from every reachable viewpoint, treated like the other unreachable env classes) |
| bins | range ≤1.5 / 1.5–3 / 3–6 m · area tiny(<100)/small(<1000)/large px² · clutter = per-frame instance count |
| config | pose_tol 0.5 s · max_range 6.0 m · hfov 1.5708 rad · range_bins [1.5, 3.0] · area_bins [100, 1000] |

Label-verification overlay: `scripts/verify_labels.py --frame <run>/<frame>`
draws the projected GT bboxes onto the stored JPEG (spot-checked on v1 and v2
FULLRES frames; residual ≈0.03 rad from the ±0.5 s pairing tolerance on a
spinning robot).

### Instrument fix inside this dataset (frame-t parser)

The initial build carried a frame-t parser bug: the fractional part of the
keyframe filename timestamp was truncated (`f_41.824` → t = 41.0 s), so pose
pairing matched against poses up to 1 s STALE. Every v1 + v2/1080p corpus label
was rebuilt after the fix (`f_41.824` pairs against t = 41.824 s). All
downstream pairing work inherits the same correction.

### Review-round-1 geometry rederivation (2026-10-18)

Per-class box geometry was re-derived from the rendered model chain
(mesh vertex bbox × the DAE unit scale, + the SDF link/model pose z):

- class `CLASS_GEO` values in `scripts/build_recog_dataset.py` are now
  rendered-truth. **person**: world z span [0, 0.724] (floor contact → head
  top; the link z −0.624 cancels the sitting mesh's head-origin offset) and
  x/y half-extents as the max over the mesh axes (placed model yaws are
  ∈ {0°, 90°, 180°, 270°} per `object_placer.py` while the GT JSONs carry no
  yaw, so per-axis hulls are yaw-invariant and slightly over-cover instead of
  clipping legs/arms); **first_aid_kit**: world z [0.11, 0.33] (model.sdf link
  z +0.11 was previously omitted — the box used to sit 0.11 m too low);
  **suitcase**: mesh is centre-origin (world z [−0.07, +0.07]).
- **exit_sign**: `SPAWN_Z['exit_sign'] = 1.25` now encodes the door-header
  model pose (previously the constant read as floor-placed; docs now use the
  SDF phrasing).
- verified: on the rebuilt labels, the box-bottom→floor-contact gap for
  unclipped person instances is median −7.6 px, p90 −1.3 px, and **0 % of
  instances sit >50 px above the floor-contact row** (was 47 % before the
  fix). Overlay re-checks: `results/recog_benchmark/review_overlays_r2/`
  (3 holdout person frames incl. seed10 f157 — the reviewer's original
  exhibit, plus seed9 f221/f430).

### Known limitation — labels are occlusion-unaware

The LOS ray-cast clears **walls** on the 0.5 m/cell GT PGM only; furniture
(chairs, cabinets, desks, the extinguisher's own stand) is not in the PGM. A
GT instance fully hidden behind furniture is still labeled visible — e.g.
`stream_easy_seed10_run1_20261018T083757Z/f_98.800.jpg` where the person box
overlaps the tall cabinet but only the boot sticks out at its edge. The
per-instance `occluded` field the v1 manifest carried was a hardcoded `False`
stub (never computed) and was **removed** in this revision; treat
furniture-occluded instances as label noise (they depress measurable precision
for pixel-perfect detectors but are part of the sim's real operating
condition). No partial-LOS flag exists in the current manifest schema.

## Eval harness — `scripts/eval_recog.py`

```
.venv/bin/python scripts/eval_recog.py --model owlv2 [--split corpus|holdout|stress] \
    [--runs <run names>] [--classes fire_extinguisher ...] \
    [--conf-points 0.05 0.10 ...] [--fp-mode unmatched|gt_within_radius] \
    [--bar bakeoff --bar-baseline <baseline summary.json>] \
    [--out results/eval_recog/<name>]
```

Backend interface (prompt-driven open-vocab): `init(prompts)` /
`infer(bgr) -> [{class, conf, bbox...}]` — built-ins `owlv2`,
`yolo_world_s/m`, `gdino_tiny` (same weights + prompts as the live pipeline;
underscore→space). Add a backend by subclassing.

Per class × conf-threshold: instance rate (per-frame class-wise **greedy first-fit
matching** — GT order, first qualifying detection; IoU ≥ 0.10, conf-desc;
bake-off bearing fallback |Δbearing| ≤ 0.20 rad) → PR curve over the conf grid,
rate@0.15/0.30/0.50, **precision@0.15** (TP@0.15 / (TP@0.15 + unmatched dets at
conf ≥ 0.15), per class), conf median.

FP-proxy — TWO configurable modes (`--fp-mode`):

- `unmatched` (default, strict): detection matches no in-frame GT.
- `gt_within_radius` (bake-off-compatible): detection whose same-class GT is
  not within 2 m of the camera, matched dets excluded — the definition behind
  the bake-off's FP-proxy numbers. GT lookup covers all splits: v10-assoc /
  stream-corpus / **v2holdout** result-JSON dirs + the benchmark's
  `generated_gt/` for the stress runs (reviewer round 1, item 5) — the
  lenient mode no longer silently degrades to strict on holdout/stress.

Bar logic — `--bar bakeoff` re-evaluates the t_b4361707 swap bar in-harness:
per conf ∈ {0.15, 0.30, 0.50}, FAK rate ≥ 1.5× the baseline's, min(FE,
person) rate ≥ 1.0×, and FP-proxy/frame ≤ the baseline's (same `--fp-mode`
both sides; baseline summary.json via `--bar-baseline`). Written to
`summary.json["bar"]` + the README table. `--bar none` (default) skips it.

Outputs: `summary.json` + `frame_records.jsonl` (per-frame dets, matches, FP,
latency) + `README.md` table.

## Reproduction check (dataset validity)

**One run per protocol, both from the rebuilt (frame-t-fixed) manifest — no
mixed-protocol table this time (reviewer round 1, item 1).** Population
statement: the rebuilt corpus pairing gate retains **1317 of the bake-off's
1435 v1 frames** (118 frames lost — with correct fractional-second pairing
they have no ±0.5 s pose match; they were only pairable via the staled pose).
All numbers below are from single executed runs in `results/eval_recog/`.

Mission-3 prompts (fire_extinguisher / first_aid_kit / person), full rebuilt
corpus (4256 frames):

| run | fp-mode | FE | FAK | person | FP-proxy/frame | precision@0.15 (FE/FAK/person) |
|---|---|---|---|---|---|---|
| `r2_mission_strict` | unmatched | 0.8987 | 0.5212 | 0.8495 | 1.512 | 0.53 / 0.94 / 0.76 |
| `r2_mission_bakedef` | gt_within_radius | 0.8987 | 0.5212 | 0.8495 | 0.966 | 0.53 / 0.94 / 0.76 |

Restricted to the 1317-frame intersection with the bake-off population (same
model, same prompts, same `gt_within_radius` FP definition):

| metric | harness (rebuilt labels) | bake-off (stale-pose labels) |
|---|---|---|
| FE rate | 0.8979 | 0.8263 |
| FAK rate | 0.6285 | 0.3186 |
| person rate | 0.9403 | 0.7414 |
| FP-proxy/frame | 0.8998 | 0.2711 |

These DO NOT reproduce the bake-off numbers, and that is expected: the
rebuild fixed the stale-pose instrument, and the bake-off's numbers were
measured ON the stale instrument. Decomposition on the shared 1317 frames:

- GT visibility sets differ on 611/1317 frames across all 9 classes (164 on
  the 3 mission classes): 167 old-label mission instances are REMOVED by the
  corrected pairing (visible-with-a-1-s-stale-pose but actually out of
  range/FOV — verified visually: the "first_aid_kit" at f_63.888 of
  seed1_run1_20261013T113817Z is a few-pixel smudge at ~6 m, not a 0.9 m-range
  kit the stale label claimed), 17 are ADDED (in-tolerance now).
- Person/FAK rate jumps are also amplified by the hull fixes above (the old
  person hull sat 0.37 m too high, missing every lower body, and FAK hulls
  sat 0.11 m too low).

So the claim is NOT "reproduces the bake-off table" any more — **the bake-off
table is a measurement of the stale-pairing instrument and is superseded by
this corrected benchmark**. What IS validated:

1. **GT consistency**: on the 1317 shared frames, corrected labels keep the
   mission-class set identical on 1153/1317 (87 %); every disagreement
   inspected visually sided with the corrected label (overlay + frame
   reading). Instance geometry is mesh/SDF-derived and renders true on
   overlay spot-checks (v1 + v2 FULLRES).
2. **Harness == bake-off protocol**: the harness reuses bakeoff_run.py's
   pose→camera pipeline, prompts, matching fallback, and FP definition; the
   identical-model sanity run `r2_mission_*` behaves like the bake-off's
   other runs (same detector, latency family ~0.26-0.29 s/frame GPU).
3. The old stale-numbers runs are kept for provenance
   (`results/eval_recog/corpus_owlv2_repro*` — pre-rebuild, NOT comparable to
   the current manifest; see their mtimes) but are superseded by `r2_mission_*`.

Latency between runs is NOT comparable (shared GPU).

Protocol differences vs the bake-off (both documented, both selectable): the
harness matches IoU-first with the bearing fallback, sweeps an arbitrary conf
grid from ONE low-threshold (0.02) inference per backend per frame, and its
default FP mode is the stricter `unmatched`.

## Zero-gate check

Same rules as the swap bar: dataset validity = label correction provenance
(above) + GT consistency + overlay verification. No new bar is introduced by
this task. NOTE (reviewer round 1, item 6): the earlier "harness cleanly
separates the three existing backends" statement was only ever backed by an
OWLv2 row — the YOLO-World / G-DINO rows were never executed under the
harness, so no separation claim is made here. Full multi-backend sweeps land
in the operator-directed comprehensive sweep (Phase 3, comments).

## Phase 2 — holdout capture (sim)

5 fresh easy runs on sandbox v2 (seeds 6–10, speed 2, pkg1,
`--agent-name derpbot-explorer-v2holdout`, runs_per_seed 1) + 2
perception_stress runs (seeds 6–7); keyframes FULLRES=1 + streams + GT
recorded → `split_holdout.txt` (never seen by any mechanism iteration) +
`split_stress.txt`. `scripts/build_recog_dataset.py` picks the new runs up by
name (seed ≥ 6 → holdout; `perception_stress` in the run name → stress).

Stress-seed-7 GT provenance: the sim was up but result-JSON writing was
interrupted; both stress run GTs were recovered via the deterministic seeded
world generator (seed-7 GT == the /tmp/arst_worlds SDF values to ≤0.0004 m);
seed-7's keyframes (2) pair fine, seed-6's 36 frames use the same recovered
GT. The stress-split wall-clock overran the runner's 1200 s soft stop in both
runs (sim-time 900.5 s elapsed) — both runs are REPORT-ONLY (no
`submitted:true` in the v2holdout cohort).

## Cadence caveat for tracker-feeding backends

The 0.5 Hz FFE cadence bar (t_ea3db560) still applies to anything feeding the
tracker live: a backend slower than ~2 s/frame CANNOT be the primary detector
(tiled 1080p inference measured-negative at that cadence). Backends slower
than 2 s/frame should be evaluated as a LOW-CADENCE SECOND-PASS hybrid —
fast primary (e.g. OWLv2 @ 1.5 Hz) + sparse second pass — which is a
legitimate scored configuration in the harness (run both splits separately
and combine offline).

## Phase 3 (operator-directed, next task)

A comprehensive detector sweep runs OUTSIDE this task on the full candidate
set: larger OWLv2 checkpoints, G-DINO base at raised thresholds, YOLO-World-L,
Florence-2, and cloud VLMs (qwen3-vl:235b / kimi-k3 / glm-5.3-flash) — VLMs on
a ≤300-frame stratified sample, run cheapest-first, quota-guarded. Backends
slower than 2 s/frame go through the hybrid second-pass configuration above.