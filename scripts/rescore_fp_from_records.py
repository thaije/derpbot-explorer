#!/usr/bin/env python3
"""Regenerate an eval_recog run's summary.json/README from its own records.

Review round 3, item 2: a run's summary.json can carry contradictory FP-conf
labels (top-level `fp_conf` vs `_global.fp_proxy_min_conf` vs the README
header) when the artifact is edited after the run instead of regenerated.
This tool closes that hole: it re-derives the FP-proxy PER FRAME from
`frame_records.jsonl` (whose stored per-frame `fp` field is ground truth for
what the executed run computed) and validates the re-derivation against it;
then it rewrites `summary.json` + `README.md` so all three labels name the
conf the data actually supports.

Semantics reproduced here (mirrors eval_recog.py's FP block exactly):
  - per class: d_keep = dets of that class with conf >= FP_CONF, sorted
    conf-desc; the records' `matched[c]` entries are [di, gi, overlap] with
    di = index into d_keep;
  - gt_within_radius mode: count unmatched d_keep only when NO same-class
    GT instance lies within FP_RADIUS_M (2 m) of the frame's robot_xy
    (robot_xy from the manifest, resolved against --manifest);
  - per-frame validation: re-derived fp must equal the record's stored
    `fp` on EVERY frame, else the tool aborts (records and labels then
    disagree about more than a threshold — a human should look).

Usage (repo root):
    .venv/bin/python scripts/rescore_fp_from_records.py \
        --run-dir results/eval_recog/r3_mission_bakedef \
        --manifest results/recog_benchmark/manifest_prev_r3.jsonl \
        --fp-conf 0.15 [--out-dir <dir> (default: in place)]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

EXPLORER_ROOT = Path("/home/plip/Projects/derpbot-explorer")
FP_RADIUS_M = 2.0   # eval_recog's gt_within_radius constant
GT_DIRS = [
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v10-assoc"),
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-stream-corpus"),
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v2holdout"),
]


def gt_all_for(run: str, cache: dict):
    if run in cache:
        return cache[run]
    ms = re.search(r"seed(\d+)", run)
    mr = re.search(r"run(\d+)", run)
    out = defaultdict(list)
    if "perception_stress" in run:
        p = (EXPLORER_ROOT / "results/recog_benchmark/generated_gt" / f"{run}.json")
        if p.exists():
            for v in json.load(open(p)).values():
                out[v["type"]].append((v["x"], v["y"]))
    elif ms and mr:
        for d in GT_DIRS:
            p = d / f"easy_seed{ms.group(1)}_run{mr.group(1)}.json"
            if p.exists():
                raw = json.load(open(p))["raw_metrics"]["ground_truth_objects"]
                for v in raw.values():
                    out[v["type"]].append((v["x"], v["y"]))
                break
    cache[run] = out
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--manifest", default="results/recog_benchmark/manifest.jsonl",
                    help="manifest that pairs the run's frame_ids to robot_xy")
    ap.add_argument("--fp-conf", type=float, required=True)
    ap.add_argument("--out-dir", default=None, help="(default: in place)")
    args = ap.parse_args()
    fp_conf = args.fp_conf
    run_dir = EXPLORER_ROOT / args.run_dir
    out_dir = EXPLORER_ROOT / args.out_dir if args.out_dir else run_dir

    summary = json.load(open(run_dir / "summary.json"))
    manifest = {}
    for line in open(EXPLORER_ROOT / args.manifest):
        r = json.loads(line)
        manifest[r["frame_id"]] = r
    gt_cache: dict = {}

    fp_per_frame_rederived = 0
    fp_per_frame_recorded = 0
    mismatched = 0
    unmatched_at_conf = defaultdict(int)   # per class, unmatched det count (== fp15)
    matched_at_conf = defaultdict(int)     # per class, matched det count
    n_frames = 0
    with open(run_dir / "frame_records.jsonl") as fh:
        for line in fh:
            r = json.loads(line)
            dets = r["dets"]
            matched = {c: [di for di, _, _ in pairs]
                       for c, pairs in r["matched"].items()}
            if r["frame_id"] not in manifest:
                print(f"ABORT: record frame_id {r['frame_id']} not in {args.manifest} "
                      f"— the manifest pairing this run's records is a different one; "
                      f"pass --manifest with the run's manifest")
                return 1
            frame = manifest[r["frame_id"]]
            robot = frame["robot_xy"]
            n_frames += 1
            fp_frame = 0
            for c, m_idx in matched.items():
                mset = set(m_idx)
                dkeep = sorted((d for d in dets if d["class"] == c
                                and d["conf"] >= fp_conf),
                               key=lambda d: -d["conf"])
                matched_at_conf[c] += len(m_idx)
                gtl = gt_all_for(r["frame_id"].split("/")[0], gt_cache)
                if summary["fp_mode"] == "gt_within_radius":
                    near = any(math.hypot(g[0] - robot[0], g[1] - robot[1]) <= FP_RADIUS_M
                               for g in gtl.get(c, []))
                    if near:
                        continue
                fp = len(dkeep) - len(mset & set(range(len(dkeep))))
                fp_frame += fp
                unmatched_at_conf[c] += fp
            if fp_frame != r["fp"]:
                mismatched += 1
                if mismatched <= 5:
                    print(f"  per-frame FP mismatch {r['frame_id']}: "
                          f"re-derived {fp_frame} vs recorded {r['fp']}")
            fp_per_frame_rederived += fp_frame
            fp_per_frame_recorded += r["fp"]

    if mismatched:
        print(f"ABORT: {mismatched}/{n_frames} frames disagree between the "
              f"re-derivation and the records — the records are not a valid "
              f"basis at fp_conf={fp_conf}; investigate before regenerating")
        return 1
    fp_rate = fp_per_frame_rederived / max(1, n_frames)
    print(f"validated on ALL {n_frames} frames: re-derived FP/frame @{fp_conf} "
          f"= {fp_rate:.4f} == recorded {fp_per_frame_recorded / n_frames:.4f}")

    # precision@0.15 rewrite per class
    for c in summary["classes"]:
        tp15 = matched_at_conf.get(c, 0)
        fp15 = unmatched_at_conf.get(c, 0)
        denom = tp15 + fp15
        if denom:
            summary["classes"][c]["precision@0.15"] = round(tp15 / denom, 4)
    summary["fp_conf"] = fp_conf
    summary["_global"]["fp_proxy_per_frame"] = round(fp_rate, 4)
    summary["_global"]["fp_proxy_min_conf"] = fp_conf
    summary["regenerated"] = {
        "tool": "scripts/rescore_fp_from_records.py",
        "manifest": args.manifest,
        "note": "summary/README labels regenerated from the run's own "
                "frame_records.jsonl after a review finding that the labels "
                "were edited post-run (fp_conf / fp_proxy_min_conf / README "
                "header disagreed); the FP numbers themselves were validated "
                "per-frame against the records and unchanged",
        "rederived_fp_per_frame": round(fp_rate, 4),
        "per_frame_mismatches": 0,
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    # regenerated README (matches eval_recog.py's writer)
    classes = summary["classes"]
    conf_points = summary["conf_points"]
    md = [f"# eval_recog — {summary['model']} on `{summary['split']}`",
          "",
          f"frames {summary['frames']}, matching IoU ≥ {summary['iou_gate']} "
          f"(bearing ≤ {summary['bearing_gate']} rad fallback), "
          f"FP-proxy @ conf ≥ {fp_conf} ({summary['fp_mode']})",
          "",
          "| class | n_gt | " + " | ".join(f"rate@{c:.2f}" for c in conf_points) +
          " | precision@0.15 | conf med |",
          "|---|" + "---|" * (2 + len(conf_points) + 2)]
    for c in classes:
        s = classes[c]
        cells = [f"{s[f'rate@{cf:.2f}']}" if s.get(f"rate@{cf:.2f}") is not None
                 else "—" for cf in conf_points]
        prec = s.get("precision@0.15")
        md.append(f"| {c} | {s['n_gt']} | " + " | ".join(cells) +
                  f" | {prec if prec is not None else '—'} | {s.get('conf_med')} |")
    g = summary["_global"]
    md += ["", f"FP-proxy/frame: **{g['fp_proxy_per_frame']}** "
               f"({summary['fp_mode']}, conf ≥ {fp_conf}); latency med "
               f"{g['lat_ms_median']} ms (p90 {g['lat_ms_p90']})", ""]
    (out_dir / "README.md").write_text("\n".join(md))
    print(f"-> {out_dir / 'summary.json'} + README.md (all conf labels = {fp_conf})")
    return 0


if __name__ == "__main__":
    sys.exit(main())