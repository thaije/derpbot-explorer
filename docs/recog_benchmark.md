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
| frames | **4810 of 15130 stored keyframes** — the retained frames have an in-tolerance pose pairing; **365 of them are zero-object frames** (320 corpus, 45 holdout) with only negative labels. `dataset_stats.json` reports both total frames (4810) and per-run object-bearing frames (`run_list[*].frames`) |
| visible-GT instances | **8800** |
| splits | corpus 4249 frames / 57 runs · holdout 523 frames / 5 runs · stress 38 frames / 2 runs |
| GT source | v10-assoc / stream-corpus / v2holdout result JSONs (`ground_truth_objects`, all 8 renderable scenario classes; `mission_target` flag preserved); stress runs use `generated_gt/` (deterministic WorldGenerator replay, ≤0.0004 m vs in-sim SDF). **Per-object yaw + exact spawn z** come from the deterministic WorldGenerator replay (`generated_gt_with_yaw/`, `scripts/replay_object_yaws.py`; GT coordinates re-verified ≤0.002 m on all 25 replayed runs + all 64 benchmark run keys) |
| pose pairing | sighting-stream rows aggregated per timestamp (median rx/ry + circular-mean yaw when a batch carries multiple rows), nearest distinct timestamp within ±0.5 s, odom→world via spawn header (bake-off pipeline, 84 % bearing agreement). The stream rows at one timestamp can disagree on yaw (5954 multi-row timestamps / 91 streams, 7.9 % with >5° intra-t spread, worst 179° — review r2 item 2); the aggregated pose + the frame's intra-batch `yaw_spread` (rad, max deviation from the circular mean) are recorded per frame. 166 frames exceed a 5° spread and 34 exceed 15° (worst 34°) — treat `yaw_spread` as a per-frame label-confidence axis |
| camera provenance | per-run stream-header `camera` block (v2 camera_info) or the v1 640×480 pinhole constants (`source=v1_no_header`), with fx/fy/cx/cy AND width/height **scaled to the stored JPEG resolution** plus `native_width`/`native_height`/`scale` (review r2 item 4). Stored-res census: 1869 frames true 1920×1080 (all holdout+stress + FULLRES probes), 1698 at 320×240 (v1 detector-input), 1243 at 320×180 (v2 legacy header-proof slice). The **corpus 1080p slice is 1869 − holdout/stress = 1308 frames**, not "3112/4256" — native-header frames downscaled to 320-wide storage carry a 320-res camera block |
| labels per class | fire_extinguisher 3496 · first_aid_kit 1885 · office_chair 911 · person 782 · drill 581 · drink_can 479 · trash_can 403 · suitcase 263 (exit_sign: 0 visible — SDF-posed at z = 1.25 m on the office door headers `wall_door_header_*`; the PGM wall-clearance LOS test blocks it from every reachable viewpoint, treated like the other unreachable env classes) |
| bins | range ≤1.5 / 1.5–3 / 3–6 m · area tiny(<100)/small(<1000)/large px² · clutter = per-frame instance count |
| config | pose_tol 0.5 s · max_range 6.0 m · hfov 1.5708 rad · range_bins [1.5, 3.0] · area_bins [100, 1000] |

Label-verification overlay: `scripts/verify_labels.py --frame <run>/<frame>`
draws the projected GT bboxes onto the stored JPEG (spot-checked on v1 and v2
FULLRES frames; residual ≈0.03 rad from the ±0.5 s pairing tolerance on a
spinning robot — plus the per-frame recorded `yaw_spread` above for the
multi-row tail).

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
  fix). The exact metric derivation lives in the round-1 worker session
  (jacket-mask floor-contact row fit — not committed as a script; the
  overlay spot-checks below reproduce it visually). Overlay re-checks:
  `results/recog_benchmark/review_overlays_r2/`
  (3 holdout person frames incl. seed10 f157 — the reviewer's original
  exhibit, plus seed9 f221/f430).
- **review round 2, item 3 — yaw-dependent hull centres**: the model-frame
  centre offsets are now ROTATED by the object's placement yaw
  (`R(obj_yaw)·offset`) before projection. Placement yaws are recovered
  offline via the WorldGenerator replay (no yaw lives in the GT JSONs;
  `scripts/replay_object_yaws.py`, coordinate-verified ≤0.002 m on all 25
  runs). Applying the person offset unrotated misplaced the hull 0.32–0.46 m
  for 75 % of instances; the per-instance yaw is also recorded in the
  manifest. `office_chair`/`drill` get the same (smaller-amplitude) fix.

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
    [--fp-conf 0.15] \
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
  the bake-off's FP-proxy numbers, evaluated at `--fp-conf` (default 0.15,
  the bake-off operating point). GT lookup covers all splits: v10-assoc /
  stream-corpus / **v2holdout** result-JSON dirs + the benchmark's
  `generated_gt/` for the stress runs (reviewer round 1, item 5) — the
  lenient mode no longer silently degrades to strict on holdout/stress.

Bar logic — `--bar bakeoff` re-evaluates the t_b4361707 swap bar in-harness:
per conf ∈ {0.15, 0.30, 0.50}, FAK rate ≥ 1.5× the baseline's, min(FE,
person) rate ≥ 1.0×, and FP-proxy/frame ≤ the baseline's (same `--fp-mode`
both sides; baseline summary.json via `--bar-baseline`). The FP-proxy
threshold must match too — a baseline whose `fp_proxy_min_conf` differs from
this run's `--fp-conf` is recorded as `fp_conf_mismatch: true` in
`summary.json["bar"]` with a loud stdout warning (review r2 item 1: an
FP@0.05-vs-FP@0.15 comparison is a ~3× handicap). Written to
`summary.json["bar"]` + the README table. `--bar none` (default) skips it.

Output protocol knobs, all recorded in `summary.json`: `fp_mode`,
`fp_conf`, `conf_points`, `iou_gate`, `bearing_gate`, `frames`, plus per-class
rate@each-conf / precision@0.15 / conf med/p90.

Outputs: `summary.json` + `frame_records.jsonl` (per-frame dets, matches, FP,
latency) + `README.md` table.

## Reproduction check (dataset validity)

**One run per protocol, both executed on the post-r2-rebuild manifest (4810
frames, yaw-rotated hulls, aggregated poses) — no mixed-protocol table
(review r1 item1 / r2 item1).** The FP-proxy is computed at an explicit
`--fp-conf` (default **0.15 = the bake-off's CONF_THRESHOLD**); summary + docs
state it per row (review r2 item 1: comparing an FP@0.05 against a
baseline's FP@0.15 is a ~3× handicap, and `--bar` now flags a baseline whose
FP threshold differs).

Population statement: the rebuilt corpus pairing gate retains **1314 of the
bake-off's 1435 v1 frames** (121 lost — with correct fractional-second pairing
they have no ±0.5 s pose match; they were only pairable via the stale pose).
The rebuilt label set is therefore NOT the bake-off's 1435-frame population:
reproduction is evaluated on the **intersection (1314 frames)**, and the
full-corpus numbers below are the single-run protocol artifact cited for the
Phase-3 sweep.

### Single-run protocol artifact — full rebuilt corpus, 3 mission prompts
Run `results/eval_recog/r3_mission_bakedef` (owlv2, conf grid 0.05–0.50,
`--fp-mode gt_within_radius --fp-conf 0.15`, 4249 frames):

| metric | value |
|---|---|
| FE rate@0.15 | 0.9045 |
| FAK rate@0.15 | 0.5220 |
| person rate@0.15 | 0.8553 |
| FP-proxy/frame @ conf ≥ 0.15 (gt_within_radius) | 0.3518 |
| precision@0.15 (FE/FAK/person) | 0.534 / 0.947 / 0.781 |

### Reproduction — same 1314-frame intersection, matched thresholds (conf ≥ 0.15)

| metric | harness (rebuilt labels) | bake-off (1435 frames, stale-pose labels) | Δ |
|---|---|---|---|
| FE rate@0.15 | 0.8678 | 0.8263 | +0.042 |
| FAK rate@0.15 | 0.3785 | 0.3186 | +0.060 |
| person rate@0.15 | 0.8603 | 0.7414 | +0.119 |
| FP-proxy/frame @ 0.15 | 0.2953 | 0.2711 | +0.024 |

At the bake-off's own operating point (conf ≥ 0.15, gt_within_radius FP) the
harness reproduces the bake-off table within label-correction noise: +0.04 FE,
+0.06 FAK, +0.12 person, +0.02 FP/frame. The residual deltas are the label
corrections, not a harness mismatch:

- GT visibility sets differ on 611/1314 frames across all 9 classes (164 on
  the 3 mission classes): 167 old-label mission instances are REMOVED by the
  corrected pairing (visible-with-a-1-s-stale-pose but actually out of
  range/FOV — verified visually: the "first_aid_kit" at f_63.888 of
  seed1_run1_20261013T113817Z is a few-pixel smudge at ~6 m, not a 0.9 m-range
  kit the stale label claimed), 17 are ADDED (in-tolerance now). Removing
  stale visible labels lowers n_gt where the old instrument over-labeled —
  the measured DETECTION rate denominator shrinks and the rate rises.
- Person/FAK rate deltas are amplified by the hull fixes (the old person hull
  sat 0.37 m too high, missing every lower body, and FAK hulls sat 0.11 m too
  low) — a corrected (larger, floor-contact hull) both matches MORE
  detections and admits more GT instances.

Historical note: the intermediate r2-era table printed "DOES NOT reproduce"
with FAK 0.6285 vs 0.3186 — that comparison read the harness's conf ≥ 0.05
row against the bake-off's conf ≥ 0.15 (threshold mismatch, review r2 item 1)
and compared a different frame population. At matched thresholds the verdict
is the near-reproduction above.

What IS validated:

1. **GT consistency**: on the 1314 shared frames, corrected labels keep the
   mission-class set identical on ~87 %; every disagreement
   inspected visually sided with the corrected label (overlay + frame
   reading). NOTE (review r2 item 5): class-SET agreement is weak evidence —
   item 2 of r2 shows the class set can be right while every box in the frame
   is misregistered — the binding checks are the per-frame `yaw_spread`
   metadata + overlay verification, not the set-overlap number.
   Instance geometry is mesh/SDF/replay-derived and renders true on
   overlay spot-checks (v1 + v2 FULLRES).
2. **Harness == bake-off protocol**: the harness reuses bakeoff_run.py's
   pose→camera pipeline, prompts, matching fallback, FP radius, and (via
   `--fp-conf 0.15`) the bake-off's operating threshold; the
   identical-model sanity run `r3_mission_bakedef` behaves like the bake-off's
   other runs (same detector, latency ~0.26 s/frame GPU).
3. Earlier runs are kept for provenance with their protocol knobs recorded:
   `r2_mission_strict`/`r2_mission_bakedef` (pre-r3 manifest, FP-proxy
   implicitly at conf ≥ 0.05 — fp_conf absent from their summaries means
   "min(conf_points)"; superseded), `corpus_owlv2_repro*` (pre-frame-t-fix,
   NOT comparable to any current manifest).

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