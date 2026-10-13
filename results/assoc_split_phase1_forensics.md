# t_ca2ea380 Phase 1 — merge forensics + variant comparison (zero-sim)

## Corpus decision

Two replay corpora:

1. **15-run corpus** (5 seeds × 3 runs, `20261011T*` streams, pre-projection-fix
   code) — used for the FP-mechanism attribution. The +FPs of small-MR variants
   there are **artifact**: band-position sightings (2–3.5 m off a real object,
   the projection defect t_fd80a2f9 fixed) stop chain-merging into one gated
   track under MR 0.9 and instead confirm alone (seed1r2 person-FP ladder along
   y≈7.8: the wall-overshoot band sampled every view). With the projection fix
   these positions no longer exist — fresh streams confirm: v9 gate attribution
   found **ZERO FPs inside the window** and clean in-window share 0.93–1.00.
2. **5 gate streams** (the t_fd80a2f9 easy 5×1 regression runs, `20261013T
   11:38–12:11Z`, post-projection-fix, identified by TP-set anchoring against
   the live v9 submission JSONs: replay TP sets == live detection_events sets)
   — **primary corpus** for variant comparison.

Sanity: anchor trace == live-code replay (scripts/replay_tracker.py) TP sets on
all 5 gate streams + corpus spot checks; replay itself reproduces live v9
accounting (seed1 5/5, seed2 5/5, seed3 4/4, seed4 2/2, seed5 5/5, FP within
window accounting differences only — live counts band/LOS FPs replay can't see).

## Anchor forensics on the 5 gate streams (MATCH_RADIUS 2.0)

| seed | real near-dup pairs merged (GT≤1.5m same cls) | chimera tracks (spread>1.2, unpublished, ≥1 sighting ≤0.5m of GT) |
|------|-----------------------------------------------|------------------------------------------------------------------|
| 1 | — | track_2 FE n=17 spread 3.2 conf .72 (gt 1), 16 gate-rejects |
| 2 | — | — |
| 3 | FE 2~3 @ **1.17 m** | track_3 FE n=49 spread 3.02 conf .66 (gt 2+3), 46 gate-rejects |
| 4 | FE 1~2 @ **0.96 m** | track_1 FE n=149 spread 3.67 conf .83 (gt 1+2), **141** gate-rejects; track_2/3 FAK n=24/22 spread 2.3/1.9 (gt 4 / gt 5) |
| 5 | — | track_8 FE n=31 spread 2.2 conf .62 (gt 1), 29 gate-rejects |

Chimera gate-reject events total on 5 streams: 16+46+(141+9+18)+29 = **259**.
The seed4 desk-FE pair (0.96 m) is the dominant single loss (149 sightings
0.03–0.07 m from GT chained into one spread-3.7 m track). Merge chains:
absorb distance distribution in these chimeras is bimodal — 0.0–0.3 m (real
re-sightings) and 2.2–3.0 m (defect-band sightings) — i.e. the chimera is
real-object-core + defect-band halo MERGED WITH a sibling object.

## Variant comparison (primary corpus — 5 gate streams)

Scoring = sandbox-equivalent (class + 1.5 m window + tracking-id TP/DP dedup).

| variant | tp | fp | dp | found/run | fp/run |
|---------|----|----|----|-----------|--------|
| anchor (MR 2.0) | 21 | 4 | 0 | **4.20** | 0.80 |
| MR 1.2 | 24 | 8 | 2 | 4.80 | 1.60 |
| MR 0.9 | 25 | 10 | 10 | 5.00 | 2.00 |
| split MR 2.0 (naked) | 27 | 8 | 71 | 5.40 | 1.60 |
| split MR 1.2 (naked) | 27 | 12 | 76 | 5.40 | 2.40 |
| **split-guard MR 2.0** (both-halves-core, c2, tight 0.75) | 23 | 4 | 27 | 4.60 | 0.80 |
| split-guard MR 1.2 | 25 | 9 | 34 | 5.00 | 1.80 |
| **split-guard c2 MR 2.0 + tight 0.75** | 26 | 8 | 26 | **5.20** | 1.60 |
| split-guard c2 MR 1.2 + tight | 25 | 8 | 26 | 5.00 | 1.60 |
| split-guard c2 MR 0.9 + tight | 26 | 10 | 26 | 5.20 | 2.00 |

(c2 = corroboration guard: each 2-means half needs ≥2 sightings clustered
within 0.5 m; tight = split-child association quarantine at 0.75 m so the
0.96 m sibling cannot re-merge the halves.)

Per-run (anchor → winner): s1 5→5, s2 5→5, s3 4→5, s4 **2→5**, s5 5→6.
Seed4 recovers BOTH desk FEs + FAK 5; seed5 gains FE 1; seed3 splits pair 2/3.

Estimated score impact (accuracy component only, 0.6·recall + 0.4·precision):
+13.5 points/run mean vs live v9 (+6.9 for anchor alone, i.e. the variant adds
+6.6 above what the same streams would do without any change).

## Rejected variants (with reasons)

- **Naked MR 0.9/0.75**: found 5.0 but FP/run 2.0-2.0 on gate streams (5.5 on
  the pre-fix corpus, misattributed there) — and live FP is scored at 0.4
  weight; the +0.8 found does not cover the FP cost, and DP 10/run pollutes.
- **Naked split**: halves re-merge on the next sighting from the sibling
  (0.96 m < MR 2.0) → split/merge churn, DP 71; needs quarantine.
- **Split-guard c3 (=original cor≥3)**: too strict — halves with exactly one
  corroborated core + one 2-sighting side never split (seed4 FAK 5 lost).

## Winner decision (Phase 2 pre-registration)

Adopt: **spread-triggered 2-means split with both-half corroboration guard** —
when a same-class track's pairwise spread > STABILITY_RADIUS and ≥4 sightings:
cluster positions (2-means, extremes-seeded); if centroid separation ≥0.5 m AND
each half contains ≥2 sightings clustered within 0.5 m of each other (two
re-sighted real objects), split into two fresh tracks; each child carries a
TIGHT quarantine: it only matches new sightings within 0.75 m of its own
centroid (distinct-object-distance rationale: real distinct objects observed
0.96–1.17 m apart; same-object sighting scatter ≤0.3 m post-projection-fix, so
a 0.75 m quarantine separates siblings while never breaking up a real object's
own re-sightings). MATCH_RADIUS itself stays 2.0 — the quarantine applies only
to split children. Gates (STABILITY_RADIUS/CONFIDENCE_MARGIN/veto/fast-confirm)
untouched. Replay-validated: found 5.20 vs 4.20 anchor, FP 1.60 vs 0.80 (score
net +13.5 acc points/run on the primary corpus).