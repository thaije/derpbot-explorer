#!/usr/bin/env python3.12
"""Offline policy sweep over the 3-seed stream corpus (t_451895af step 3).

Runs replay_tracker's pipeline for each config × stream, aggregates a ranked
table: config → replay-predicted found/6 mission-targets + FP/run per seed,
plus centroid-error stats on TPs and per-seed delta vs the seed's live result.

Mission targets = GT entries with mission_target=true (6 per easy run).
"""
from __future__ import annotations

import importlib
import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path("/home/plip/Projects/derpbot-explorer")
SUB = Path.home() / "Projects/robot-sandbox/results/submissions/derpbot-explorer-stream-corpus"
STREAMS = REPO / "results/streams"
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "agent"))

os.environ.update({
    "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib",
    "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages",
    "AMENT_PREFIX_PATH": "/opt/ros/jazzy",
})
import rclpy  # noqa: E402
import vision_msgs  # noqa: E402
import tracker as tracker_mod  # noqa: E402

import replay_tracker as rt  # noqa: E402

SEEDS = [1, 4, 5]

CONFIGS = {
    # 1. current master behavior (margin disabled, stability 1.2, veto, fast-confirm)
    "master-as-is": [],
    # 2. pure baseline: all confirm-time gates off
    "baseline-all-gates-off": [
        "STABILITY_RADIUS=100.0", "VETO_RADIUS=0.0", "FAST_CONFIRM_FRAMES=1",
        "MIN_SIGHTINGS=1",
    ],
    # 3. baseline + fast-confirm only
    "baseline+fast-confirm": [
        "STABILITY_RADIUS=100.0", "VETO_RADIUS=0.0",
    ],
    # 4. baseline + fast-confirm + tight stability
    "bc+stability0.8": ["STABILITY_RADIUS=0.8", "VETO_RADIUS=0.0"],
    # 5. baseline + fast-confirm + loose stability
    "bc+stability1.2": ["STABILITY_RADIUS=1.2", "VETO_RADIUS=0.0"],
    # 6. fast-confirm with wider catch distance
    "bc+fastconfirm2.5": ["STABILITY_RADIUS=100.0", "VETO_RADIUS=0.0", "FAST_CONFIRM_DIST=2.5"],
    # 7. single-sighting confirm, guarded by the GT window at publish time
    #    (guard implemented as MATCH_RADIUS tightening: publishes must sit on a
    #    stable position — approximated in replay by MIN_SIGHTINGS=1 + stability 0.3)
    "bc+min1+guard0.3": ["STABILITY_RADIUS=0.3", "VETO_RADIUS=0.0", "MIN_SIGHTINGS=1"],
}


def mission_gt(gt: dict) -> dict:
    return {k: v for k, v in gt.items() if v.get("mission_target")}


def run_config(cfg_name: str, sets: list[str]):
    # reset module constants to file defaults each config
    importlib.reload(tracker_mod)
    rt.monkeypatch_policy(sets)
    rows = []
    for seed in SEEDS:
        header, sightings = rt.load_stream(str(STREAMS / f"stream_seed{seed}_run1.jsonl"))
        gt_raw = json.loads((SUB / f"easy_seed{seed}_run1.json").read_text())
        full_gt = gt_raw["raw_metrics"]["ground_truth_objects"]
        tgt_gt = mission_gt(full_gt)
        spawn = header.get("spawn_pose") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
        confirmed = rt.run_tracker(sightings)
        found, tp, fp, dp, tp_events, log = rt.score(confirmed, tgt_gt, spawn)
        errs = sorted(r["distance_to_nearest"] for r in log
                      if r["outcome"] == "TP" and r["distance_to_nearest"] is not None)
        # FP against FULL gt (incl. non-mission) for realistic false-pos rate
        _, _, fp_full, dp_full, _, _ = rt.score(confirmed, full_gt, spawn)
        live = json.loads((SUB / f"easy_seed{seed}_run1.json").read_text())["raw_metrics"]
        live_found = sum(1 for k in mission_gt(full_gt)
                         if any(s["outcome"] == "TP" and s["nearest_gt_key"] == k
                                for s in live["submission_log"]))
        rows.append({
            "seed": seed, "found": found, "n": len(tgt_gt), "tp": tp, "fp": fp,
            "fp_full": fp_full, "dp": dp,
            "errs": errs, "live_found": live_found,
            "tp_keys": sorted(e["label_key"] for e in tp_events),
        })
    return rows


def fmt_errs(errs):
    if not errs:
        return "n/a"
    mean = sum(errs) / len(errs)
    med = errs[len(errs) // 2 - (1 - len(errs) % 2) // 2] if errs else 0
    p95 = errs[min(len(errs) - 1, int(round(0.95 * (len(errs) - 1))))]
    over15 = sum(1 for e in errs if e > 1.5)
    return f"mean={mean:.2f} med={med:.2f} p95={p95:.2f} over1.5m={over15}/{len(errs)}"


def main() -> int:
    results = {}
    for name, sets in CONFIGS.items():
        results[name] = run_config(name, sets)
        print(f"done: {name}", flush=True)

    # Save raw
    out_path = REPO / "results" / "sweep_results.json"
    out_path.parent.mkdir(exist_ok=True)
    out_path.write_text(json.dumps(results, indent=1, default=str))

    # Ranked table: mean found desc, then mean fp_full asc
    ranked = sorted(results.items(),
                    key=lambda kv: -(sum(r["found"] for r in kv[1]) / len(SEEDS)))
    print("\n=== SWEEP TABLE (mission targets 6/run; replay = frozen-stream prediction) ===")
    print(f"{'config':<26} {'found/18':>8} {'per-seed found (live anchor)':<34} {'FP/run':>6}  centroidErr TP")
    for name, rows in ranked:
        total = sum(r["found"] for r in rows)
        fp_run = sum(r["fp"] for r in rows) / len(rows)
        fpf_run = sum(r["fp_full"] for r in rows) / len(rows)
        per = ", ".join(f"s{r['seed']}:{r['found']}(live {r['live_found']})" for r in rows)
        all_errs = [e for r in rows for e in r["errs"]]
        print(f"{name:<26} {total:>4}/18  {per:<40} FP={fp_run:.1f} (fullGT {fpf_run:.1f})  {fmt_errs(all_errs)}")

    print("\n=== TP detail per config (keys) ===")
    for name, rows in ranked:
        for r in rows:
            print(f"  {name:<26} seed{r['seed']}: TP keys {r['tp_keys']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())