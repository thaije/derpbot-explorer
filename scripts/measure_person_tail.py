#!/usr/bin/env python3
"""Measured person-label residual tail on the recog_benchmark manifest.

Review round 3, item 1(c): the docs claimed a "≈0.03 rad" residual from the
±0.5 s pairing tolerance while the real (stale-pose) tail reached 0.23 rad —
and the `yaw_spread` quality axis they recommended cannot see a stale pose
(the carried-over rows agree with each other). This script measures the
label-quality tail on an IMAGE-GROUNDED metric instead, from the delivered
manifest alone:

  - person: hi-vis jacket (HSV yellow-green) containment inside the person
    bbox, on UNCLIPPED boxes (box and mask ≥3 px from every border), plus
    box-centre → jacket-centre offset in metres (range is in the manifest);
  - all classes: |Δbearing| between the manifest's recorded object bearing
    and the jacket/blob-centre bearing is not available class-generic, so
    the bearing tail is reported on the person metric + the raw per-frame
    jacket-x vs person-box-x comparison (px → rad via f=x-scale).

Writes JSON to stdout. Usage:
    .venv/bin/python scripts/measure_person_tail.py \
        [--manifest results/recog_benchmark/manifest.jsonl] [--split corpus]
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

JACK_LO = (30, 110, 110)   # HSV lo for hi-vis yellow-green (tuned on seed10 f_711.504)
JACK_HI = (85, 255, 255)
BORDER_PX = 3              # unclipped definition
MIN_JACKET_PX = 300        # reviewer's floor for containment stats


def jacket_mask(hsv):
    return cv2.inRange(hsv, JACK_LO, JACK_HI)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default="results/recog_benchmark/manifest.jsonl")
    ap.add_argument("--split", default=None, choices=[None, "corpus", "holdout", "stress"])
    args = ap.parse_args()
    rows = []
    for line in open(EXPLORER_ROOT / args.manifest):
        r = json.loads(line)
        if args.split and r["split"] != args.split:
            continue
        persons = [o for o in r["objects"] if o["cls"] == "person"]
        if not persons:
            continue
        rows.append(r)
    print(f"person frames ({args.split or 'all'}): {len(rows)}")
    stats = []
    for r in rows:
        img = cv2.imread(str(EXPLORER_ROOT / r["image"]))
        if img is None:
            continue
        H, W = img.shape[:2]
        if any(abs(v) < 0.02 for o in r["objects"] if o["cls"] == "person"
               for v in (o["bearing"],)):
            pass
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m = jacket_mask(hsv)
        for o in r["objects"]:
            if o["cls"] != "person":
                continue
            x1, y1, x2, y2 = o["bbox"]
            unclipped = (x1 >= BORDER_PX and y1 >= BORDER_PX
                         and x2 <= W - BORDER_PX and y2 <= H - BORDER_PX)
            sub = m[max(0, int(y1)):int(y2) + 1, max(0, int(x1)):int(x2) + 1]
            box_px = int((sub > 0).sum())
            total_px = int((m > 0).sum())
            if total_px < MIN_JACKET_PX:
                continue
            if not unclipped:
                continue
            ys, xs = np.nonzero(m)
            if len(xs) == 0:
                continue
            jcx, jcy = xs.mean(), ys.mean()
            bcx, bcy = (x1 + x2) / 2, (y1 + y2) / 2
            stats.append({
                "frame_id": r["frame_id"], "split": r["split"],
                "range": o["range"],
                "containment": round(box_px / total_px, 4),
                "box_minus_jacket_x_px": round(bcx - jcx, 1),
                "offset_m": round(math.hypot(bcx - jcx, bcy - jcy)
                                  / r["camera"]["fx"], 3),
                "spread": r.get("yaw_spread", 0.0),
                "interpolated": r.get("pose_interpolated", False),
            })
    n = len(stats)
    if not n:
        print(json.dumps({"n": 0}))
        return 1
    cont = sorted(s["containment"] for s in stats)
    offs = sorted(s["offset_m"] for s in stats)
    below = [s for s in stats if s["containment"] < 0.5]
    zero = [s for s in stats if s["containment"] < 0.05]
    bear = sorted(abs(s["box_minus_jacket_x_px"]) / stats[0].get("f", 1)
                  for s in stats)  # not used; keep for reference
    out = {
        "person_frames_unclipped_ge_minpx": n,
        "containment_median": cont[n // 2],
        "containment_mean": round(sum(cont) / n, 4),
        "frac_cont_lt_0.5": round(len(below) / n, 4),
        "frac_cont_lt_0.05": round(len(zero) / n, 4),
        "offset_m_median": offs[n // 2],
        "offset_m_p90": offs[int(0.9 * n)],
        "offset_m_max": offs[-1],
        "worst_frames": sorted(stats, key=lambda s: s["containment"])[:6],
        "worst_by_offset": sorted(stats, key=lambda s: -s["offset_m"])[:6],
    }
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())