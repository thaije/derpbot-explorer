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
                        pose provenance (pose_interpolated / pose_src_t /
                        pose_samples_fresh), objects=[{key, cls, mission,
                        range, bearing, bbox, range_bin, area_bin, area_px}]
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
| frames | **4633 of 15130 stored keyframes** — the retained frames have a bounded-stale pose pairing (see "Pose staleness" below); **354 of them are zero-object frames** (310 corpus, 44 holdout) with only negative labels. `dataset_stats.json` reports both total frames (4633) and per-run object-bearing frames (`run_list[*].frames`) |
| visible-GT instances | **8473** |
| splits | corpus 4081 frames / 57 runs · holdout 514 frames / 5 runs · stress 38 frames / 2 runs |
| GT source | v10-assoc / stream-corpus / v2holdout result JSONs (`ground_truth_objects`, all 8 renderable scenario classes; `mission_target` flag preserved); stress runs use `generated_gt/` (deterministic WorldGenerator replay, ≤0.0004 m vs in-sim SDF). **Per-object yaw + exact spawn z** come from the deterministic WorldGenerator replay (`generated_gt_with_yaw/`, `scripts/replay_object_yaws.py`; GT coordinates re-verified ≤0.002 m on all 25 replayed runs + all 64 benchmark run keys) |
| pose pairing | sighting-stream rows aggregated per timestamp (median rx/ry + circular-mean yaw when a batch carries multiple rows), then reduced to FRESH distinct pose samples (carried-over duplicates from sighting batches without odometry ticks are DROPPED — 2459/20518 stream timestamps across 93 streams re-emit the previous pose byte-identical); the frame pose is INTERPOLATED between the bracketing fresh samples (span ≤ 1 s, yaw lerped after unwrapping; 1539/4633 frames interpolated), nearest fresh sample otherwise, always within ±0.5 s; odom→world via spawn header (bake-off pipeline) |
| camera provenance | per-run stream-header `camera` block (v2 camera_info) or the v1 640×480 pinhole constants (`source=v1_no_header`), with fx/fy/cx/cy AND width/height **scaled to the stored JPEG resolution** plus `native_width`/`native_height`/`scale`. Stored-res census: 1822 frames true 1920×1080 (all holdout+stress + corpus FULLRES probes), 1588 at 320×240 (v1 detector-input), 1223 at 320×180 (v2 legacy header-proof slice) — the **corpus 1080p slice is 1270 frames** |
| labels per class | fire_extinguisher 3350 · first_aid_kit 1810 · office_chair 884 · person 753 · drill 562 · drink_can 461 · trash_can 394 · suitcase 259 (exit_sign: 0 visible — SDF-posed at z = 1.25 m beside the office door headers; the PGM wall-clearance LOS test blocks it from every reachable viewpoint, treated like the other unreachable env classes) |
| bins | range ≤1.5 / 1.5–3 / 3–6 m · area tiny(<100)/small(<1000)/large px² · clutter = per-frame instance count |
| config | pose_tol 0.5 s · interp_max_span 1.0 s · max_range 6.0 m · hfov 1.5708 rad · range_bins [1.5, 3.0] · area_bins [100, 1000] |

Label-verification overlay: `scripts/verify_labels.py --frame <run>/<frame>`
draws the projected GT bboxes onto the stored JPEG.

### Pose staleness fix (review round 3, item 1) — rebuild 2026-10-18 21:18

**Root cause (measured).** When no odometry tick arrived between sightings the
stream re-emits the PREVIOUS pose at the later timestamp: 2459 of 20518 stream
timestamps across 93 streams carry a pose byte-identical to the prior distinct
one (carry-over gaps up to >1.9 s; median 0.03 s). Nearest-timestamp pairing
trusted those rows at face value, and intra-batch aggregation (r2 item 2)
cannot see the problem — the rows AGREE with each other, they are just old.

**Fix.** `pose_at` now (a) aggregates rows per timestamp, (b) drops
carried-over samples (`distinct_pose_samples`), and (c) when the frame time
falls strictly between two bracketing fresh samples, LINEARLY INTERPOLATES the
pose (xy lerped; yaw lerped on the closest unwrapped branch; r2-requested
remedy). Interpolation is span-capped at `--interp-max-span 1.0` — wider
in-tol brackets fall back to the nearer flank; the pose is never derived from
an observation older than ±0.5 s. Effect on the corpus: 1500/4626 common
frames changed pose, 1539/4633 frames are interpolated, 184 frames (3.8 %,
previously paired ONLY via a stale carried-over pose) are DROPPED — including
the review's `f_713.312` exhibit — and 7 frames gained a pose that previously
had none. Every frame records `pose_interpolated`, `pose_src_t` (the source
sample times), and `pose_samples_fresh` for auditability.

**Measured residual tail (image-grounded, `scripts/measure_person_tail.py`,
committed; data in `person_tail_measure.json`/`person_tail_summary.json`).**
On hi-vis jacket HSV-blob containment, 251 unclipped person frames
(box AND mask ≥3 px from every border, ≥150 jacket px):

- jacket-pixel containment: median 1.000, mean 0.965; **7 frames below 0.5**,
  4 of those at ≤0.05 (one holdout: seed10 f_711.504);
- box-centre → blob-centre lateral offset at the person's range
  (|Δbearing|·range): median 0.22 m, p90 0.52 m, p95 0.67 m, max 1.65 m;
  95/251 frames exceed 0.3 m, 32 exceed 0.5 m;
- rendered exhibits from the DELIVERED manifest:
  `results/recog_benchmark/review_overlays_r4_seed10_f711.png` (person
  reclining; box covers torso+legs, boot pokes out right — correct
  registration, hull limitation) and
  `review_overlays_r4_seed2_f244.png` (settled/reclining person, box extends
  past the resting body).

**The two remaining tail causes, stated as limitations:**

1. **Reclining/settled person exceeds the standing-hull geometry** (review r3
   item 1d): the person mesh is a sitting/stationary character; when it
   settles or reclines the body mass shifts up to ~1 body width outside the
   mesh-derived hull, even with perfectly registered camera pose. The worst
   frames (f_203.344, f_131.120, f_244.600) are all reclining/settled poses;
   several have an EXACT pose (fresh sample at the frame's own timestamp,
   spread 0) — pose staleness is NOT the cause there.
2. **Keyframe-vs-sighting micro-latency on a spinning robot**: the stream may
   carry one fresh sample exactly at the frame's t while the NEXT sample
   (0.1 s later) shows a 10°+ jump — the pixels were captured sometime inside
   that window (e.g. seed5 f_203.344: rows 157.8°@t, 169.0°@t+0.11 s;
   measured residual up to ~0.2 rad ≈ the inter-sample jump). Bounded by the
   bracketing span; not visible to `yaw_spread` (intra-t only).

`yaw_spread` REMAINS usable only as the intra-batch disagreement axis (173
frames >5°, 41 >15°, worst 170° on the current manifest); it CANNOT detect
stale nearest-row poses any more — those frames no longer exist in the
dataset — and it cannot see either residual cause above. Use
`pose_interpolated`/`pose_src_t` + the measured tail above when weighting
per-frame label confidence.

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

**One executed run per protocol — `r4_mission_bakedef` — executed on the
post-r3 (pose-staleness) rebuild of the manifest (4633 frames, 2026-10-18
21:18).** The prior executed run `r3_mission_bakedef` is PRE-rebuild: it paired
against the 2026-10-18 19:15 manifest (backup:
`results/recog_benchmark/manifest_prev_r3.jsonl`), 175 of its frames no longer
exist in the delivered manifest, and it is retained for provenance with its
stale-artifact note; it backs NO claim here. The FP-proxy is computed at an
explicit `--fp-conf` (default **0.15 = the bake-off's CONF_THRESHOLD**);
summary + docs state it per row (review r2 item 1).

Population statement: the pose-staleness rebuild retains **1228 of the
bake-off's 1435 v1 frames** (207 lost — 121 already lost to the frame-t fix,
86 more now dropped because their only in-tol pose source was a carried-over
stale sample: they were only pairable via the stale pose, or their bracket is
out of tolerance). The delivered label set is therefore NOT the bake-off's
1435-frame population: reproduction is evaluated on the **intersection (1228
frames)**, and the full-corpus numbers below are the single-run protocol
artifact cited for the Phase-3 sweep.

### Single-run protocol artifact — full rebuilt corpus, 3 mission prompts
Run `results/eval_recog/r4_mission_bakedef` (owlv2, conf grid 0.05–0.50,
`--fp-mode gt_within_radius --fp-conf 0.15`, 4081 frames):

| metric | value |
|---|---|
| FE rate@0.15 | 0.9091 |
| FAK rate@0.15 | 0.5303 |
| person rate@0.15 | 0.8614 |
| FP-proxy/frame @ conf ≥ 0.15 (gt_within_radius) | 0.3475 |
| precision@0.15 (FE/FAK/person) | 0.537 / 0.953 / 0.784 |

### Reproduction — matched thresholds (conf ≥ 0.15) on the shared 1228-frame population

Two rescores of stored detections on the frames the pose-corrected manifest
shares with the bake-off's 1435 (both from `frame_records.jsonl`, single
inference generation each; NOT new inference):

- **harness column** = `r4_mission_bakedef` records scored against the
  CURRENT (delivered) labels — `summary_intersect1228_fpconf15.json`;
- **bake-off column** = the bake-off-era `corpus_owlv2_repro_bakedef`
  records scored against their own era's labels
  (`summary_intersect1228_bakeoff_fpconf15.json`; 3 of the 1228 lack an
  old-manifest pose, scored only in the harness column). Each column is one
  protocol's detections on one population — no mixed-protocol cell.

| metric | harness (current labels) | bake-off (stale-pose labels) | Δ |
|---|---|---|---|
| FE rate@0.15 | 0.8731 | 0.8732 | −0.000 |
| FAK rate@0.15 | 0.3830 | 0.3753 | +0.008 |
| person rate@0.15 | 0.8682 | 0.8295 | +0.039 |
| FP-proxy/frame @ 0.15 | 0.2923 | 0.2963 | −0.004 |

At the bake-off's own operating point the harness reproduces the bake-off
table within label-correction noise: |Δ| ≤ 0.04 on all rates, ≤ 0.005 on the
FP proxy — on a population the stale-pose pipeline cannot even label
consistently (86 of these frames were only pairable via a carried-over stale
pose). The r3-era table's larger deltas (+0.12 person) came from the same
comparison with a less-corrected label set; the pose-staleness fix and
reproduction are two views of the same correction.

Historical note: the intermediate r2-era table printed "DOES NOT reproduce"
with FAK 0.6285 vs 0.3186 — that comparison read the harness's conf ≥ 0.05
row against the bake-off's conf ≥ 0.15 (threshold mismatch, review r2 item 1)
and compared a different frame population. At matched thresholds the verdict
is the near-reproduction above.

What IS validated:

1. **GT consistency**: on the shared population, the rate columns above agree
   within noise between the delivered labels and the bake-off-era labels
   where the old instrument was correct, and the measured label-correction
   deltas (stale-pose frames REMOVED from the dataset instead of
   relabeled-wrong) side with the delivered labels. The binding per-frame
   check is now the pose provenance (`pose_interpolated` / `pose_src_t`) +
   the measured image-grounded tail (above) + overlay verification.
   Instance geometry is mesh/SDF/replay-derived and renders true on
   overlay spot-checks (v1 + v2 FULLRES).
2. **Harness == bake-off protocol**: the harness reuses bakeoff_run.py's
   pose→camera pipeline, prompts, matching fallback, FP radius, and (via
   `--fp-conf 0.15`) the bake-off's operating threshold; the
   identical-model sanity run `r4_mission_bakedef` behaves like the bake-off's
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