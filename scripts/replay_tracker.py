#!/usr/bin/env python3
"""
replay_tracker.py — offline tracker-policy replayer (t_451895af).

Replays a recorded sighting stream (JSONL from the tracker's StreamRecorder)
through the LIVE tracker_worker with arbitrary policy constants monkeypatched,
then scores the published detections against the run's ground truth with the
SAME match semantics as the sandbox scorer (class match + nearest-GT distance,
spawn-offset map→world transform, tracking-id dedup).

CLI:
  python3.12 scripts/replay_tracker.py --stream <jsonl> --gt <run.json> \
      [--set CONFIDENCE_MARGIN=0.05,STABILITY_RADIUS=0.8] [--spawn '{"x":1,"y":1,"yaw":0}']

Prints: "found=X/N TP=N FP=N DP=N" — exits 0.

Interpreter rules (same as tests/test_tracker_gates.py): needs rclpy +
vision_msgs → run with python3.12 (ROS jazzy site-packages on sys.path).

Scorer reference: robot-sandbox/src/metrics/object_detection_tracker.py
  - map→world: wx = spawn_x + cos(yaw)*mx - sin(yaw)*my
                wy = spawn_y + sin(yaw)*mx + cos(yaw)*my
  - nearest GT of same type within match_threshold=1.5 m → in range
  - tracking_id TP/DP dedup: a track_id that already scored TP/DP is skipped
  - DP: in-range but that GT object already confirmed; FP: nothing in range
  - LOS (FP_LOS) needs the PGM + camera pose — NOT modelled: replay scores
    position matches only, and sightings that reached the tracker required
    LOS at detection time anyway.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
AGENT_DIR = REPO_ROOT / "agent"

# Interpreter guard identical to tests/test_tracker_gates.py
try:
    import rclpy  # noqa: F401
    import vision_msgs  # noqa: F401
    sys.path.insert(0, str(AGENT_DIR))
    import tracker as tracker_mod  # repo agent/tracker.py
except ImportError as exc:
    sys.exit(
        "skipped: rclpy/vision_msgs/tracker unavailable on this interpreter — "
        f"run with python3.12 (ROS jazzy site-packages on sys.path) ({exc})"
    )

# match_threshold mirrors the sandbox scorer default
MATCH_THRESHOLD = 1.5


def load_stream(path: str) -> tuple[dict, list[dict]]:
    """Parse a stream JSONL → (header, [sighting dicts])."""
    header: dict = {}
    sightings: list[dict] = []
    with open(path) as fh:
        for raw in fh:
            raw = raw.strip()
            if not raw:
                continue
            rec = json.loads(raw)
            if rec.get("type") == "header":
                header = rec
            elif rec.get("type") == "sighting":
                sightings.append(rec)
    if not header:
        raise SystemExit(f"ERROR: {path} has no header line")
    return header, sightings


def _policy_env(sets: list[str]) -> None:
    """Export policy overrides as env for the spawned worker.

    tracker_worker runs in a `spawn`-context subprocess which re-imports
    tracker.py with DEFAULT constants — parent-side monkeypatching does not
    reach it. The worker applies DERPBOT_POLICY_OVERRIDES at import time
    (bootstrap in tracker.py), so this is still the live tracker code running
    the patched constants.
    """
    import os
    overrides = {}
    for chunk in sets:
        for pair in chunk.split(","):
            pair = pair.strip()
            if pair:
                name, _, _ = pair.partition("=")
                overrides[name] = pair
    os.environ["DERPBOT_POLICY_OVERRIDES"] = ",".join(sorted(overrides.values()))


def monkeypatch_policy(sets: list[str]) -> dict:
    """Apply --set CONF=VALUE[,CONF=VALUE...] to the live tracker module AND
    stage the same overrides for the spawned worker process (see _policy_env)."""
    changed: dict = {}
    for chunk in sets:
        for pair in chunk.split(","):
            pair = pair.strip()
            if not pair:
                continue
            if "=" not in pair:
                raise SystemExit(f"ERROR: --set expects NAME=VALUE, got {pair!r}")
            name, _, value = pair.partition("=")
            if not hasattr(tracker_mod, name):
                raise SystemExit(f"ERROR: tracker has no constant {name!r}")
            old = getattr(tracker_mod, name)
            val = float(value)
            # int constants stay int (MIN_SIGHTINGS, FAST_CONFIRM_FRAMES)
            new = int(val) if isinstance(old, int) and not isinstance(old, bool) and val == int(val) else val
            changed[name] = old
            setattr(tracker_mod, name, new)
    _policy_env(sets)
    return changed


def run_tracker(sightings: list[dict]) -> list[dict]:
    """Feed sightings through tracker_worker (spawned proc); return confirmed msgs.

    Feed is paced (0.01 s per sighting): the poison pill must not overtake
    sightings sitting in the un-drained queue — the worker would exit on the
    pill and silently drop everything behind it.
    """
    import multiprocessing as mp
    import threading

    ctx = mp.get_context("spawn")
    iq, oq, ev = ctx.Queue(), ctx.Queue(), ctx.Event()
    class_names = sorted({s["class_name"] for s in sightings})
    # The stream only ever contained mission-target classes (target filter ran
    # upstream at record time), so pass them all as the tracked target list.
    p = ctx.Process(target=tracker_mod.tracker_worker, args=(class_names, iq, oq, ev))
    p.start()
    ev.wait(timeout=10.0)

    def feed():
        for s in sightings:
            try:
                iq.put((s["class_name"], (s["x"], s["y"]), (s["rx"], s["ry"]), s["confidence"]))
            except Exception:
                return
            time.sleep(0.01)
        try:
            iq.put(None)  # poison pill — after ALL sightings are enqueued
        except Exception:
            pass

    t = threading.Thread(target=feed, daemon=True)
    t.start()

    out = []
    p.join(timeout=max(30.0, len(sightings) * 0.02 + 15.0))
    while True:
        try:
            out.append(oq.get_nowait())
        except Exception:
            break
    return [m for m in out if m.get("type") == "confirmed"]


# ── Scorer-equivalent evaluation ─────────────────────────────────────────────

def map_to_world(mx: float, my: float, spawn: dict) -> tuple[float, float]:
    sx = float(spawn.get("x", 0.0))
    sy = float(spawn.get("y", 0.0))
    yaw = float(spawn.get("yaw", 0.0))
    s_, c_ = math.sin(yaw), math.cos(yaw)
    return sx + c_ * mx - s_ * my, sy + s_ * mx + c_ * my


def find_nearest_of_type(class_type: str, wx: float, wy: float, gt: dict):
    best_key, best_dist = None, math.inf
    for key, entry in gt.items():
        if entry["type"] != class_type:
            continue
        d = math.hypot(wx - float(entry["x"]), wy - float(entry["y"]))
        if d < best_dist:
            best_dist, best_key = d, key
    return best_key, best_dist


def score(confirmed: list[dict], gt: dict, spawn: dict,
          match_threshold: float = MATCH_THRESHOLD):
    """Scorer-equivalent TP/DP/FP accounting over the replay's publications.

    One published message == one scorer subscription callback (each confirmed
    publication became its own Detection2DArray live).
    Returns (found, tp_count, fp_count, dp_count, tp_events, log).
    """
    confirmed_objects: dict[str, dict] = {}
    seen_tracking_ids: set[str] = set()
    tp_events: list[dict] = []
    fp_count = 0
    dp_count = 0
    log: list[dict] = []

    for msg in confirmed:
        class_type = msg.get("class_name", "")
        wx, wy = map_to_world(float(msg["world_x"]), float(msg["world_y"]), spawn)
        tracking_id = msg.get("track_id", "") or ""

        # tracking-id dedup (scorer step 2): a track_id that already produced a
        # TP/DP is skipped entirely on later messages.
        if tracking_id and tracking_id in seen_tracking_ids:
            continue

        label_key, dist = find_nearest_of_type(class_type, wx, wy, gt)
        if label_key is not None and dist <= match_threshold:
            if label_key in confirmed_objects:
                dp_count += 1
                if tracking_id:
                    seen_tracking_ids.add(tracking_id)
                outcome = "DP"
            else:
                confirmed_objects[label_key] = {"tracking_id": tracking_id}
                if tracking_id:
                    seen_tracking_ids.add(tracking_id)
                tp_events.append({
                    "label_key": label_key,
                    "class_type": class_type,
                    "tracked_as": tracking_id,
                    "location_error": round(dist, 3),
                })
                outcome = "TP"
        else:
            outcome = "FP"
            fp_count += 1
        log.append({
            "class_type": class_type,
            "submitted_x": round(wx, 3),
            "submitted_y": round(wy, 3),
            "nearest_gt_key": label_key,
            "distance_to_nearest": None if dist == math.inf else round(dist, 3),
            "outcome": outcome,
            "tracking_id": tracking_id,
        })

    return (len(confirmed_objects), len(tp_events),
            fp_count, dp_count, tp_events, log)


# ── Miss diagnosis (operator steer #2) ───────────────────────────────────────

def diagnose_misses(sightings: list[dict], gt: dict, spawn: dict, tp_keys: set,
                    match_threshold: float = MATCH_THRESHOLD) -> list[str]:
    """One line per MISSING (non-TP) mission-target GT object.

    Q1: does ≥1 sighting of the class exist near the object (map frame)? If
    yes: why did confirmation fail — 2nd sighting never came / gate /
    centroid-drift (report nearest raw-sighting distance AND the submitted
    centroid distance of any in-window-ish publication).
    Q2 (absence case): the stream only contains frames the detector actually
    fired on — a class missing from the stream entirely means it was never
    detected ≥ threshold while in FOV; state that plainly (no synthetic
    FOV reconstruction from bbox stats: bboxes exist only on fired frames).
    """
    lines = []
    for key, entry in sorted(gt.items(), key=lambda kv: int(kv[0]) if kv[0].isdigit() else 0):
        if key in tp_keys:
            continue
        ctype = entry["type"]
        cls_sight = [s for s in sightings if s["class_name"] == ctype]
        if not cls_sight:
            lines.append(
                f" MISS {key} {ctype}#{entry['instance']}: NO sighting of this class "
                "in stream (never detected ≥threshold in FOV → detector-side gap, Q2)"
            )
            continue
        # nearest raw sighting to this object (sightings are map frame)
        best_d, best_s = math.inf, None
        for s in cls_sight:
            wx, wy = map_to_world(s["x"], s["y"], spawn)
            d = math.hypot(wx - float(entry["x"]), wy - float(entry["y"]))
            if d < best_d:
                best_d, best_s = d, s
        # cluster count within 2 m of the object (would-be track size)
        near = []
        for s in cls_sight:
            wx, wy = map_to_world(s["x"], s["y"], spawn)
            if math.hypot(wx - float(entry["x"]), wy - float(entry["y"])) <= 2.0:
                near.append(s)
        confs = [s["confidence"] for s in near]
        pose_span = 0.0
        if len(near) >= 2:
            for i, a in enumerate(near):
                for b in near[i + 1:]:
                    pose_span = max(pose_span, math.hypot(
                        a["rx"] - b["rx"], a["ry"] - b["ry"]))
        best_d_str = f"{best_d:.2f}m" if best_s is not None else "n/a"
        verdict = "no sighting within 2.0m" if not near else (
            f"nearest {best_d_str}, {len(near)} sighting(s) within 2m, "
            f"conf {min(confs):.2f}–{max(confs):.2f}, robot-pose span {pose_span:.2f}m"
        )
        cause = ""
        if near:
            if len(near) < 2:
                cause = "→ 2nd sighting never arrived (only 1 within window)"
            elif pose_span < 0.2:
                cause = "→ poses not diverse enough (<0.2m) and fast-confirm didn't fire (distance/conf gate)"
            else:
                cause = "→ sightings existed and were potentially confirmable; if unfound, gate or centroid drift (check config)"
        lines.append(
            f" MISS {key} {ctype}#{entry['instance']}: {verdict} {cause}"
        )
    return lines


def centroid_error_stats(log: list[dict]) -> str:
    """Published-centroid→matched-GT distance stats on TP submissions."""
    errs = sorted(row["distance_to_nearest"] for row in log
                  if row["outcome"] == "TP" and row["distance_to_nearest"] is not None)
    if not errs:
        return "centroid-err: n/a (no TPs)"
    def pct(p):
        i = min(len(errs) - 1, max(0, int(round(p * (len(errs) - 1)))))
        return errs[i]
    mean = sum(errs) / len(errs)
    med = pct(0.5)
    p95 = pct(0.95)
    n_over = sum(1 for e in errs if e > 1.0)
    return f"centroid-err TP: mean={mean:.3f} med={med:.3f} p95={p95:.3f} n={len(errs)} (>1.0m: {n_over}, >1.5m window: {sum(1 for e in errs if e > 1.5)})"


def main() -> int:
    ap = argparse.ArgumentParser(description="Offline tracker replay")
    ap.add_argument("--stream", required=True, help="Recorded JSONL sighting stream")
    ap.add_argument("--gt", required=True,
                    help="Run result JSON (raw_metrics.ground_truth_objects) or a bare GT JSON")
    ap.add_argument("--set", dest="sets", action="append", default=[],
                    help="CONF=VALUE[,CONF=VALUE...] policy overrides (repeatable)")
    ap.add_argument("--spawn", default="",
                    help="Spawn pose JSON override; default = stream header spawn_pose")
    ap.add_argument("--verbose", action="store_true", help="Print submission log")
    ap.add_argument("--diagnose-misses", action="store_true",
                    help="Per-missing-GT-object sighting/failure diagnosis (steer #2)")
    args = ap.parse_args()

    header, sightings = load_stream(args.stream)
    gt_raw = json.loads(Path(args.gt).read_text())
    gt = (gt_raw.get("raw_metrics", gt_raw).get("ground_truth_objects")
          or gt_raw.get("ground_truth_objects"))
    if not gt:
        raise SystemExit("ERROR: no ground_truth_objects in GT file")

    spawn = json.loads(args.spawn) if args.spawn else header.get("spawn_pose") or {"x": 0.0, "y": 0.0, "yaw": 0.0}

    changed = monkeypatch_policy(args.sets)

    t0 = time.time()
    confirmed = run_tracker(sightings)
    dt = time.time() - t0

    found, tp, fp, dp, tp_events, log = score(confirmed, gt, spawn)
    n_targets = len(gt)
    verdict = (
        f"found={found}/{n_targets} TP={tp} FP={fp} DP={dp} "
        f"sightings={len(sightings)} published={len(confirmed)} replay={dt:.2f}s"
    )
    if changed:
        verdict += " set=" + ",".join(
            f"{k}:{v_old}->{getattr(tracker_mod, k, '?')}" for k, v_old in changed.items()
        )
    print(verdict)

    by_type: dict[str, list[int]] = {}
    for entry in gt.values():
        t = entry["type"]
        by_type.setdefault(t, [0, 0])[1] += 1
    for ev in tp_events:
        by_type.setdefault(ev["class_type"], [0, 0])[0] += 1
    print("   " + "  ".join(f"{t}:{d[0]}/{d[1]}" for t, d in sorted(by_type.items())))
    print("   " + centroid_error_stats(log))
    if args.diagnose_misses:
        tp_keys = {ev["label_key"] for ev in tp_events}
        for line in diagnose_misses(sightings, gt, spawn, tp_keys):
            print(line)
    if args.verbose:
        for row in log:
            print("   ", row)
    return 0


if __name__ == "__main__":
    sys.exit(main())