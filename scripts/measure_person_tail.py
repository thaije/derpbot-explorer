#!/usr/bin/env python3
"""Measured label-residual tail on the recog_benchmark manifest (REGENERATOR).

Review round 4, item 1-2: the shipped evidence artifacts for the residual
registration tail had to be reproducible from one committed script with ONE
offset definition, and the tail had to be reported on a RIGID class too (FE),
not only the person. This script is that single generator — it re-measures
everything from the delivered manifest + the stored JPEGs and writes BOTH
artifact files:

  person_tail_measure.json   per frame (all person frames): hi-vis jacket
                             HSV blob vs person box — containment (null when
                             the box/mask is clipped), signed bearing error
                             err = (jcx − bcx)/fx [rad] (jacket-centre minus
                             box-centre, positive = blob right of box),
                             off_m = |err| · range [metres], pose provenance.
  person_tail_summary.json   containment/offset stats on the UNCLIPPED
                             subset + worst_frames (worst FIRST, by
                             containment ascending, tie-break |err| desc).
  fe_tail_measure.json       per instance (near-range unclipped FE boxes,
                             >=30 px wide): does a red-blob centroid
                             (area >= 40 px) sit inside the box, else the
                             bearing error to the nearest blob; the
                             zero-containment subset is the rigid-class
                             label-off-object tail.
  fe_tail_summary.json       per-split counts + errors.

Unclipped rule (the one the shipped numbers used): the box AND the mask are
both >= --border px from every image border. Jacket floor: --min-jacket-px.

Metric definitions (ONE each):
  containment = jacket px inside the box / total jacket px
  off_m       = |err| · range  (box-centre -> jacket-centre LATERAL offset
                at the object's range, in metres)

Usage (repo root):
    .venv/bin/python scripts/measure_person_tail.py \
        [--manifest results/recog_benchmark/manifest.jsonl] \
        [--out-dir results/recog_benchmark]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

EXPLORER_ROOT = Path("/home/plip/Projects/derpbot-explorer")
sys.path.insert(0, str(EXPLORER_ROOT / "scripts"))
import cv2  # noqa: E402
import numpy as np  # noqa: E402

JACK_LO = (30, 110, 110)   # HSV lo for hi-vis yellow-green
JACK_HI = (85, 255, 255)
RED_LO_1 = (0, 130, 70)    # red (hue 0-8)
RED_HI_1 = (8, 255, 255)
RED_LO_2 = (172, 130, 70)  # red (hue wrap 172-180)
RED_HI_2 = (180, 255, 255)
MIN_BLOB_PX = 40           # red blob area floor (reviewer r3/r4 rule)
BORDER_PX = 3              # unclipped definition: box AND mask >= 3 px inside
MIN_JACKET_PX = 150        # jacket pixel floor for the person metric
FE_MAX_RANGE_M = 2.5       # rigid-class probe: near-range FE labels
FE_MIN_WIDTH_PX = 30       # rigid-class probe: label must be resolvable
SUMMARY_WORST_N = 6


def percentiles(sorted_vals, ps):
    n = len(sorted_vals)
    return {p: sorted_vals[min(n - 1, int(p * n))] for p in ps}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default="results/recog_benchmark/manifest.jsonl")
    ap.add_argument("--out-dir", default="results/recog_benchmark")
    ap.add_argument("--border", type=int, default=BORDER_PX)
    ap.add_argument("--min-jacket-px", type=int, default=MIN_JACKET_PX)
    args = ap.parse_args()

    out_dir = EXPLORER_ROOT / args.out_dir
    man_path = EXPLORER_ROOT / args.manifest
    man = [json.loads(l) for l in open(man_path)]

    person_rows = []   # per FRAME (person-bearing)
    fe_rows = []       # per INSTANCE (near-range unclipped FE)
    for r in man:
        persons = [o for o in r["objects"] if o["cls"] == "person"]
        fes = [o for o in r["objects"] if o["cls"] == "fire_extinguisher"]
        if not persons and not fes:
            continue
        img = cv2.imread(str(EXPLORER_ROOT / r["image"]))
        if img is None:
            continue
        H, W = img.shape[:2]
        fx = r["camera"]["fx"]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # ── person (hi-vis jacket) ───────────────────────────────────────────
        if persons:
            m = cv2.inRange(hsv, JACK_LO, JACK_HI)
            ys, xs = np.nonzero(m)
            total = int(len(xs))
            mask_in = (total > 0 and xs.min() >= args.border
                       and xs.max() <= W - 1 - args.border
                       and ys.min() >= args.border
                       and ys.max() <= H - 1 - args.border)
            for o in persons:
                x1, y1, x2, y2 = o["bbox"]
                box_in = (x1 >= args.border and y1 >= args.border
                          and x2 <= W - 1 - args.border
                          and y2 <= H - 1 - args.border)
                unclipped = box_in and mask_in and total >= args.min_jacket_px
                cont = None
                err = None
                off_m = None
                if total >= args.min_jacket_px:
                    jcx, jcy = float(xs.mean()), float(ys.mean())
                    bcx, bcy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                    err = (jcx - bcx) / fx
                    off_m = abs(err) * o["range"]
                    if unclipped:
                        sub = m[int(y1):int(y2) + 1, int(x1):int(x2) + 1]
                        cont = round(float((sub > 0).sum()) / total, 4)
                person_rows.append({
                    "fid": r["frame_id"], "split": r["split"],
                    "cont": cont, "err": None if err is None else round(err, 6),
                    "unclipped": bool(unclipped),
                    "off_m": None if off_m is None else round(off_m, 3),
                    "spread": r.get("yaw_spread", 0.0),
                    "interp": r.get("pose_interpolated", False),
                    "range": o["range"],
                })

        # ── fire_extinguisher (rigid class, red blob) ────────────────────────
        if fes:
            m1 = cv2.inRange(hsv, RED_LO_1, RED_HI_1)
            m2 = cv2.inRange(hsv, RED_LO_2, RED_HI_2)
            m = m1 | m2
            n, _lab, st, cent = cv2.connectedComponentsWithStats(m)
            blobs = [(float(cent[i][0]), float(cent[i][1]), int(st[i, 4]))
                     for i in range(1, n) if st[i, 4] >= MIN_BLOB_PX]
            for o in fes:
                x1, y1, x2, y2 = o["bbox"]
                box_in = (x1 >= args.border and y1 >= args.border
                          and x2 <= W - 1 - args.border
                          and y2 <= H - 1 - args.border)
                if not (box_in and o["range"] <= FE_MAX_RANGE_M
                        and (x2 - x1) >= FE_MIN_WIDTH_PX):
                    continue
                bcx, bcy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                inside = [b for b in blobs if x1 <= b[0] <= x2 and y1 <= b[1] <= y2]
                if inside:
                    b = min(inside, key=lambda b: math.hypot(b[0] - bcx, b[1] - bcy))
                    fe_rows.append({
                        "fid": r["frame_id"], "split": r["split"],
                        "range": o["range"], "containment": 1.0,
                        "n_red_blobs": len(blobs),
                        "bearing_err_rad": round(abs(b[0] - bcx) / fx, 4),
                    })
                else:
                    if blobs:
                        b = min(blobs, key=lambda b: math.hypot(b[0] - bcx, b[1] - bcy))
                        fe_rows.append({
                            "fid": r["frame_id"], "split": r["split"],
                            "range": o["range"], "containment": 0.0,
                            "n_red_blobs": len(blobs),
                            "bearing_err_rad": round(abs(b[0] - bcx) / fx, 4),
                            "nearest_blob_px": round(math.hypot(b[0] - bcx, b[1] - bcy), 1),
                        })
                    else:
                        fe_rows.append({
                            "fid": r["frame_id"], "split": r["split"],
                            "range": o["range"], "containment": 0.0,
                            "n_red_blobs": 0, "bearing_err_rad": None,
                            "note": "no red blob anywhere in frame",
                        })

    # ── person summary ───────────────────────────────────────────────────────
    uncl = [r_ for r_ in person_rows if r_["unclipped"] and r_["cont"] is not None]
    ps = sorted(r_["cont"] for r_ in uncl)
    offs = sorted(r_["off_m"] for r_ in uncl)
    n = len(uncl)
    psummary = {
        "n_person_frames_unclipped_ge_minpx": n,
        "containment_median": ps[n // 2],
        "containment_mean": round(sum(ps) / n, 4),
        "containment_lt_0.5": sum(1 for c in ps if c < 0.5),
        "containment_lt_0.05": sum(1 for c in ps if c < 0.05),
        "lateral_offset_m": {
            "definition": "|err| * range; err = (jacket_centre - box_centre)/fx [rad]",
            "median": round(sorted(offs)[n // 2], 3),
            "p90": round(percentiles(offs, (0.9,)) [0.9], 3),
            "p95": round(percentiles(offs, (0.95,))[0.95], 3),
            "max": round(offs[-1], 3),
            "gt_0.3m": sum(1 for o in offs if o > 0.3),
            "gt_0.5m": sum(1 for o in offs if o > 0.5),
        },
        "worst_frames": sorted(uncl, key=lambda r_: (r_["cont"], -abs(r_["err"] or 0)))[:SUMMARY_WORST_N],
        "unclipped_rule": "box AND jacket mask >= %d px from every border; >= %d jacket px" % (args.border, args.min_jacket_px),
        "metric": "hi-vis jacket HSV-blob containment + lateral offset at the person range",
    }
    # ── FE summary ───────────────────────────────────────────────────────────
    def _by_split(rows, pred):
        return [r_ for r_ in rows if r_["split"] == s] if (s := pred) else rows
    fe_by_split = {}
    for sp in ("corpus", "holdout", "stress"):
        rows_s = [r_ for r_ in fe_rows if r_["split"] == sp]
        zero = [r_ for r_ in rows_s if r_["containment"] == 0.0]
        errs = sorted(r_["bearing_err_rad"] for r_ in zero if r_["bearing_err_rad"] is not None)
        fe_by_split[sp] = {
            "near_unclipped_boxes": len(rows_s),
            "zero_containment": len(zero),
            "zero_bearing_err_rad": {
                "median": round(errs[len(errs) // 2], 3) if errs else None,
                "p90": round(errs[int(0.9 * len(errs))], 3) if errs else None,
                "max": round(errs[-1], 3) if errs else None,
            },
            "no_red_blob_in_frame": sum(1 for r_ in zero if r_["n_red_blobs"] == 0),
        }
    in_errs = sorted(r_["bearing_err_rad"] for r_ in fe_rows if r_["containment"] == 1.0)
    fsummary = {
        "probe": "near-range (<= %.1f m) unclipped FE boxes >= %d px wide" % (FE_MAX_RANGE_M, FE_MIN_WIDTH_PX),
        "n_boxes": len(fe_rows),
        "in_box": sum(1 for r_ in fe_rows if r_["containment"] == 1.0),
        "zero_containment": sum(1 for r_ in fe_rows if r_["containment"] == 0.0),
        "in_box_bearing_err_rad_median": round(in_errs[len(in_errs) // 2], 3) if in_errs else None,
        "by_split": fe_by_split,
        "metric": "red-blob (HSV 0-8/172-180, s>=130, v>=70, area >= %d px) centroid inside the FE box" % MIN_BLOB_PX,
    }

    # frame-level dispersion diagnostics (review r4 item 1: registration
    # error modes, not per-object noise)
    from collections import defaultdict
    by_frame = defaultdict(list)
    for r_ in fe_rows:
        by_frame[r_["fid"]].append(r_)
    frames_2plus_zero = sum(1 for rows in by_frame.values()
                            if sum(1 for x in rows if x["containment"] == 0.0) >= 2)
    mixed = [rows for rows in by_frame.values()
             if any(x["containment"] == 0.0 for x in rows)
             and any(x["containment"] == 1.0 for x in rows)]
    mixed_same_sign = 0
    for rows in mixed:
        outs = [x["bearing_err_rad"] for x in rows
                if x["containment"] == 0.0 and x["bearing_err_rad"] is not None]
        if outs and all(abs(o) > 0.05 for o in outs) \
                and (min(outs) > 0 or max(outs) < 0):
            mixed_same_sign += 1
    person_big = sum(1 for r_ in person_rows
                     if r_["unclipped"] and r_["err"] is not None
                     and abs(r_["err"]) > 0.20)
    fsummary["frame_level_dispersion"] = {
        "fe_frames_with_2plus_zero": frames_2plus_zero,
        "fe_mixed_frames": len(mixed),
        "fe_mixed_same_sign_outs": mixed_same_sign,
        "person_unclipped_err_gt_0.20rad": person_big,
    }
    (out_dir / "person_tail_measure.json").write_text(json.dumps(person_rows, indent=1))
    (out_dir / "person_tail_summary.json").write_text(json.dumps(psummary, indent=1))
    (out_dir / "fe_tail_measure.json").write_text(json.dumps(fe_rows, indent=1))
    (out_dir / "fe_tail_summary.json").write_text(json.dumps(fsummary, indent=1))
    print("person:", json.dumps({k: psummary[k] for k in
          ["n_person_frames_unclipped_ge_minpx", "containment_median", "containment_mean",
           "containment_lt_0.5", "containment_lt_0.05", "lateral_offset_m"]}, indent=1))
    print("worst (worst first):")
    for w in psummary["worst_frames"]:
        print("  ", w["fid"], "cont", w["cont"], "err", w["err"], "off_m", w["off_m"], w["split"])
    print("FE:", json.dumps({k: fsummary[k] for k in
          ["n_boxes", "in_box", "zero_containment", "in_box_bearing_err_rad_median",
           "by_split"]}, indent=1))
    print(f"-> {out_dir}/person_tail_measure.json person_tail_summary.json "
          f"fe_tail_measure.json fe_tail_summary.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
