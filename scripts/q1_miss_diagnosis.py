#!/usr/bin/env python3.12
"""Q1 miss-diagnostics under master-as-is policy on the 3-seed corpus.

Note: the __main__ guard is REQUIRED — run_tracker spawns the tracker_worker
in a multiprocessing 'spawn' context, and every child re-executes this module's
top-level code. Without the guard that recurses into run_tracker → RuntimeError.
"""
import importlib, json, os, sys
os.environ.update({"LD_LIBRARY_PATH": "/opt/ros/jazzy/lib",
                   "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages",
                   "AMENT_PREFIX_PATH": "/opt/ros/jazzy"})
sys.path.insert(0, "/home/plip/Projects/derpbot-explorer/scripts")
sys.path.insert(0, "/home/plip/Projects/derpbot-explorer/agent")
import replay_tracker as rt
import tracker as tm

SUB = os.path.expanduser("~/Projects/robot-sandbox/results/submissions/derpbot-explorer-stream-corpus")


def print_main():
    out = []
    for seed in [1, 4, 5]:
        header, sights = rt.load_stream(f"/home/plip/Projects/derpbot-explorer/results/streams/stream_seed{seed}_run1.jsonl")
        gt_raw = json.load(open(f"{SUB}/easy_seed{seed}_run1.json"))
        gt = gt_raw["raw_metrics"]["ground_truth_objects"]
        spawn = header.get("spawn_pose")
        importlib.reload(tm)
        rt.monkeypatch_policy([])
        confirmed = rt.run_tracker(sights)
        found, tp, fp, dp, tp_events, log = rt.score(
            confirmed, {k: v for k, v in gt.items() if v["mission_target"]}, spawn)
        tp_keys = {e["label_key"] for e in tp_events}
        lines = rt.diagnose_misses(sights, {k: v for k, v in gt.items() if v["mission_target"]},
                                   spawn, tp_keys)
        out.append(f"=== seed{seed} master-as-is mission misses === (found {found}/{len(gt)})\n" + "\n".join(lines))
    print("\n\n".join(out))


if __name__ == "__main__":
    print_main()