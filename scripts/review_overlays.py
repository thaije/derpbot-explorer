#!/usr/bin/env python3
"""Render review overlays: burned-in GT bboxes + labels on keyframes (t_<id> review tool).

Usage:
  review_overlays.py [--n-per-bin N] [--out results/recog_benchmark/review_overlays]

Picks a stratified subset from results/recog_benchmark/manifest.jsonl
(corpus/holdout/stress x range-bin x mission-class-visible) and renders
each frame with: bbox (green=mission, gray=env), class+key+range+mission
labels, provenance strip (run/seed/split/t).
"""
import argparse
import json
import collections
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "results/recog_benchmark/manifest.jsonl"
OUT = ROOT / "results/recog_benchmark/review_overlays"

MISSION_CLS = ("fire_extinguisher", "first_aid_kit", "person")


def pick(manifest, n_per_bin):
    by_bin = collections.defaultdict(list)
    for row in manifest:
        mission_objs = [o for o in row["objects"] if o.get("mission")]
        if not mission_objs:
            continue
        # use the closest mission object's range bin as the frame's bin
        closest = min(mission_objs, key=lambda o: o["range"])
        key = (row["split"], closest["range_bin"], closest["cls"])
        by_bin[key].append(row)
    picks = []
    seen_frames = set()
    for key in sorted(by_bin):
        for row in by_bin[key]:
            if row["frame_id"] in seen_frames:
                continue
            picks.append(row)
            seen_frames.add(row["frame_id"])
            if sum(1 for p in picks if (p["split"], min((o for p_ in [p] for o in p["objects"] if o.get("mission")), key=lambda o: o["range"])["range_bin"], min((o for p_ in [p] for o in p["objects"] if o.get("mission")), key=lambda o: o["range"])["cls"]) == key) >= n_per_bin:
                break
    return picks


def render(row, out_dir):
    img_path = ROOT / row["image"]
    if not img_path.exists():
        return None
    img = Image.open(img_path).convert("RGB")
    d = ImageDraw.Draw(img)
    w, h = img.size
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", max(12, w // 26)
        )
        small = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", max(10, w // 42)
        )
    except Exception:
        font = ImageFont.load_default()
        small = font
    font_size = max(12, w // 26)
    small_size = max(10, w // 42)
    for o in row["objects"]:
        x1, y1, x2, y2 = [float(v) for v in o["bbox"]]
        if (x2 - x1) < 3 or (y2 - y1) < 3:
            continue  # sub-pixel labels unreadable; skip drawing (counted in manifest)
        color = (0, 220, 0) if o.get("mission") else (150, 150, 150)
        d.rectangle([x1, y1, x2, y2], outline=color, width=max(2, w // 320))
        label = f"{o['cls']} #{o['key']} {o['range']:.2f}m"
        if o.get("occluded"):
            label += " OCCL"
        ty = y1 - (font_size + 4) if y1 > font_size + 4 else y2 + 2
        d.text((max(2, x1), ty), label, fill=color, font=font)
    strip = f"{row['split']} seed{row['seed']} r{row['run_n']} t={row['t']} cam={row['camera']['width']}x{row['camera']['height']} ({row['camera']['source']})"
    d.text((4, h - small_size - 8), strip, fill=(255, 255, 255), font=small, stroke_width=2, stroke_fill=(0, 0, 0))
    name = row["frame_id"].replace("/", "__")
    dest = out_dir / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    img.save(dest, quality=80)
    return dest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-per-bin", type=int, default=2)
    ap.add_argument("--out", default=str(OUT))
    args = ap.parse_args()
    manifest = [json.loads(l) for l in MANIFEST.open() if l.strip()]
    picks = pick(manifest, args.n_per_bin)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    done = []
    for row in picks:
        dest = render(row, out_dir)
        if dest:
            done.append(str(dest))
    print(f"rendered {len(done)} overlays -> {out_dir}")


if __name__ == "__main__":
    main()