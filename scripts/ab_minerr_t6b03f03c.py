#!/usr/bin/env python3.12
"""t_6b03f03c Step 3 — zero-sim A/B, FINAL: baseline (dc3515d running-mean)
vs min-error (78b6fb5) — each replayed from its own checkout so the spawned
worker imports its own tracker.py. Also parses submission_log from the live
v5-master run JSONs for the TP-key sanity anchor."""
import importlib, json, os, subprocess, sys
os.environ.update({"LD_LIBRARY_PATH": "/opt/ros/jazzy/lib",
                   "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages",
                   "AMENT_PREFIX_PATH": "/opt/ros/jazzy"})
SUB = os.path.expanduser("~/Projects/robot-sandbox/results/submissions/derpbot-explorer-stream-corpus")
REPO = "/home/plip/Projects/derpbot-explorer"
BASE = "/tmp/ab_baseline"


def anchor_tp_keys(seed):
    """TP keys from the live v5-master submission log (sanity anchor)."""
    raw = json.load(open(f"{SUB}/easy_seed{seed}_run1.json"))
    rm = raw["raw_metrics"]
    keys = set()
    # submission_log rows: submission accepted → detection_events / submission_log
    for ev in rm.get("submission_log", []):
        if ev.get("result") == "accepted" or ev.get("outcome") == "TP":
            keys.add(str(ev.get("label_key") or ev.get("target_key") or ""))
    # fallback: detection_events
    if not keys:
        for ev in rm.get("detection_events", []):
            if ev.get("outcome", "TP") == "TP" or ev.get("result") == "accepted":
                keys.add(str(ev.get("label_key") or ev.get("target_key") or ""))
    keys.discard("")
    return keys


def run_from(root, seed):
    """Run replay in <root> checkout; the child imports that root's tracker.py."""
    cmd = [
        sys.executable, f"{root}/scripts/replay_tracker.py",
        "--stream", f"{REPO}/results/streams/stream_seed{seed}_run1.jsonl",
        "--gt", f"{SUB}/easy_seed{seed}_run1.json",
        "--verbose",
    ]
    env = dict(os.environ)
    env["PYTHONPATH"] = f"{root}/agent:{root}/scripts:" + env.get("PYTHONPATH", "")
    proc = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=600)
    return proc


def parse(verdict_line, by_type_line, err_line):
    import re
    m = re.match(r"found=(\d+)/(\d+) TP=(\d+) FP=(\d+) DP=(\d+)", verdict_line)
    if not m:
        raise SystemExit(f"cannot parse verdict: {verdict_line!r}")
    found, n, tp, fp, dp = map(int, m.groups())
    e = re.search(r"mean=([\d.]+) med=([\d.]+) p95=([\d.]+)", err_line or "")
    stats = (float(e.group(1)), float(e.group(2)), float(e.group(3))) if e else None
    return found, n, tp, fp, dp, stats


def main():
    for seed in [1, 4, 5]:
        a_keys = anchor_tp_keys(seed)
        rows = []
        for label, root in [("running-mean", BASE), ("min-error", REPO)]:
            p = run_from(root, seed)
            log_rows = []
            verdict = err_line = None
            for l in p.stdout.splitlines():
                if l.startswith("found="):
                    verdict = l
                elif "centroid-err TP" in l:
                    err_line = l
                elif l.lstrip().startswith("{") and "class_type" in l:
                    log_rows.append(json.loads(l.strip().replace("'", '"')))
            found, n, tp, fp, dp, stats = parse(verdict, None, err_line)
            tp_keys = sorted({str(r["nearest_gt_key"]) for r in log_rows
                              if r["outcome"] == "TP" and r["nearest_gt_key"] is not None},
                             key=lambda k: int(k) if k.isdigit() else 0)
            rows.append((label, found, n, tp, fp, dp, stats))
            print(f"  {label:>12}: found={found}/{n} TP={tp} FP={fp} DP={dp} "
                  f"centroid-err m/med/p95={stats[0]:.3f}/{stats[1]:.3f}/{stats[2]:.3f} "
                  f"TPkeys={tp_keys}")
        print(f"=== seed{seed} === anchor(live v5-master) TP keys: {sorted(a_keys)}")
        print()


if __name__ == "__main__":
    main()