#!/usr/bin/env python3
"""Label-verification overlay for recog_benchmark (t_fe1459a9).

Renders the projected GT pixel bboxes (from the benchmark manifest) onto the
actual keyframe JPEG so labels can be eyeballed against the image.

    .venv/bin/python scripts/verify_labels.py --manifest results/recog_benchmark/manifest.jsonl \
        --frame stream_easy_seed1_run1_20261016T190215Z/f_17.880 --out /tmp/label_check.png
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

EXPLORER_ROOT = Path("/home/plip/Projects/derpbot-explorer")
sys.path.insert(0, str(EXPLORER_ROOT / "scripts"))

import cv2  # noqa: E402

BGR = {
    "fire_extinguisher": (0, 0, 255),
    "first_aid_kit": (255, 0, 255),
    "person": (0, 255, 255),
    "trash_can": (0, 255, 0),
    "office_chair": (255, 255, 0),
    "drink_can": (0, 128, 255),
    "drill": (255, 128, 0),
    "suitcase": (128, 0, 255),
    "exit_sign": (255, 255, 255),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default="results/recog_benchmark/manifest.jsonl")
    ap.add_argument("--frame", required=True, help="<run_stem>/<frame_name>")
    ap.add_argument("--out", default="/tmp/label_check.png")
    args = ap.parse_args()

    rows = {}
    with open(EXPLORER_ROOT / args.manifest) as fh:
        for line in fh:
            r = json.loads(line)
            rows[r["frame_id"]] = r

    needle = args.frame.strip("/")
    if needle not in rows:
        cands = [k for k in rows if needle in k]
        if not cands:
            print(f"frame {needle!r} not in manifest ({len(rows)} frames)")
            return 1
        needle = cands[0]
    r = rows[needle]

    img = cv2.imread(str(EXPLORER_ROOT / r["image"]))
    if img is None:
        print(f"cannot read {r['image']}")
        return 1
    for o in r["objects"]:
        x1, y1, x2, y2 = o["bbox"]
        c = BGR.get(o["cls"], (200, 200, 200))
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), c, 2)
        cv2.putText(img, f'{o["cls"]} {o["range"]:.1f}m', (int(x1), max(12, int(y1) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1, cv2.LINE_AA)
        print(f'{o["cls"]:18s} key={o["key"]} range={o["range"]:.2f} '
              f'bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})')
    cv2.imwrite(args.out, img)
    print(f"-> {args.out}  (frame_id={needle}, H×W={img.shape[0]}×{img.shape[1]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())