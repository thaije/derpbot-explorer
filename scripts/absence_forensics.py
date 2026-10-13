#!/usr/bin/env python3.12
"""Detector-absence forensics (zero-sim): FOV/dwell classification of every
mission-target absence over the recorded sighting streams.

Frames (verified t_ea3a57f5):
- Sightings x,y and robot pose rx,ry,ryaw are all in the tracker's map frame
  (map == odom == spawn frame in this testbed; slam_toolbox keeps them glued).
- GT in result JSONs is WORLD frame. Convert: map = R(-spawn.yaw)·(world - spawn).
- Camera: 90° H-FOV, forward-facing, mounted at base_link +0.05 m forward
  (0.05 m offset < half-cell 0.25 m on the 0.5 m map — ignored).
- Projection gate: depth valid 0.30–5.0 m (agent/depth_projector.py), so a
  sighting exists only when range ≤ 5 m. In-FOV here = |bearing − yaw| ≤ 45°
  AND range ≤ 5 m (≤ 6 m per spec reported as variant).

Robot-pose track: sightings record the pose at DETECTOR-FIRE events only
(sparse, 0.6–0.9 s median on corpus, seconds on v6). For every mission-target
absence we report the FOV evidence at the two coarsest reliable resolutions:
  - fire-sampled: consecutive sighting events (N=1 / N=3 consecutive)
  - time-dwelled: consecutive samples spanning ≥ 2 s (category (c) dwell bar)

Categories:
  a  never-in-FOV   (0 in-FOV samples)
  b  in-FOV-brief   (<3 in-FOV samples or dwell < 2 s)
  c  in-FOV-dwelled (≥3 samples AND ≥2 s) but zero class sightings  → detector recall
  d  sighted-but-rejected (≥1 class sighting within 2 m) — tracker exhausted

Usage: python3.12 scripts/absence_forensics.py [--out results/detector_absence_forensics.md]
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import statistics
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STREAMS = os.path.join(REPO, "results", "streams")
SUBMISSIONS = os.path.expanduser(
    "~/Projects/robot-sandbox/results/submissions")
GT_MAP_PGM = os.path.expanduser(
    "~/Projects/robot-sandbox/worlds/templates/indoor_office/ground_truth_map"
    ".pgm")
MAP_RES = 0.5  # m/cell, ground_truth_map.yaml

FOV_HALF_DEG = 45.0
RANGE_SIGHT = 5.0   # depth-projector validity ceiling
RANGE_SPEC = 6.0    # spec variant
NEAR_M = 2.0        # "class sighted near object" bar (doc vocabulary)
DWELL_SAMPLES = 3
DWELL_SECONDS = 2.0


# ── frames ───────────────────────────────────────────────────────────────────
def world_to_map(wx: float, wy: float, spawn: dict) -> tuple[float, float]:
    sx, sy = float(spawn.get("x", 0.0)), float(spawn.get("y", 0.0))
    yaw = float(spawn.get("yaw", 0.0))
    dx, dy = wx - sx, wy - sy
    c, s = math.cos(-yaw), math.sin(-yaw)
    return c * dx - s * dy, s * dx + c * dy


def ang_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _linspace_t(t0: float, t1: float, step: float):
    if t1 <= t0:
        return [t0]
    n = int(math.ceil((t1 - t0) / step))
    return [t0 + i * step for i in range(n + 1)]


def _pose_at(poses, t: float):
    """Linear interpolation between bracketing pose track samples."""
    if t <= poses[0][0]:
        return poses[0][1:]
    if t >= poses[-1][0]:
        return poses[-1][1:]
    lo, hi = 0, len(poses) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if poses[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = poses[lo], poses[hi]
    f = (t - a[0]) / max(b[0] - a[0], 1e-9)
    return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3])


# ── ground-truth map ─────────────────────────────────────────────────────────
def load_pgm(path: str):
    with open(path, "rb") as fh:
        data = fh.read()
    assert data.startswith(b"P5")
    # header: magic + 3 whitespace-separated fields (W, H, maxval)
    idx, fields = 2, []
    while len(fields) < 3 and idx < len(data):
        if data[idx:idx + 1].isspace():
            idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        fields.append(data[start:idx])
    idx += 1  # single whitespace after maxval before raster
    w, h, _max = int(fields[0]), int(fields[1]), int(fields[2])
    pixels = data[idx:idx + w * h]
    return w, h, pixels


W, H, PIX = load_pgm(GT_MAP_PGM)


def ray_clear(ox: float, oy: float, tx: float, ty: float,
              clearance: float = 0.35) -> bool:
    """Bresenham ray on the GT occupancy grid; True if no occupied cell is
    hit closer than `clearance` m from the target (wall-face occlusion).
    Coordinates are WORLD frame: pgm row 0 = top = world y = H·RES."""
    dist = math.hypot(tx - ox, ty - oy)
    if dist < 1e-6:
        return True
    steps = max(2, int(dist / (MAP_RES / 2)))
    for i in range(1, steps + 1):
        f = i / steps
        x, y = ox + f * (tx - ox), oy + f * (ty - oy)
        cx, cy = int(x / MAP_RES), int((H * MAP_RES - y) / MAP_RES)
        if 0 <= cx < W and 0 <= cy < H:
            v = PIX[cy * W + cx]
            # negate:0, occupied_thresh 0.65 → 0 = occupied
            if v < 65:
                if dist * (1 - f) <= clearance:
                    return False
    return True


# ── streams / GT ─────────────────────────────────────────────────────────────
def load_stream(path: str):
    header, sightings = {}, []
    with open(path) as fh:
        for raw in fh:
            raw = raw.strip()
            if not raw:
                continue
            rec = json.loads(raw)
            if rec.get("type") == "header":
                header = rec
            else:
                sightings.append(rec)
    return header, sightings


def inventory() -> list[dict]:
    """Every stream whose result JSON exists (scenario+seed+run in filename).

    Disambiguation: `stream_easy_<seed>_<run>_<stamp>.jsonl` = per-run batch
    naming (derpbot-explorer-v6-fastconfirm); `stream_<seed>_<run>.jsonl`
    (no stamp, no scenario) = corpus naming (derpbot-explorer-stream-corpus).
    """
    agent_by_naming = [
        ("derpbot-explorer-stream-corpus", "corpus"),
        ("derpbot-explorer-v6-fastconfirm", "easy"),
    ]
    runs = []
    for f in sorted(glob.glob(os.path.join(STREAMS, "*.jsonl"))):
        base = os.path.basename(f)[len("stream_"):-len(".jsonl")]
        base = base.split("_2026")[0].strip("_").replace(".jsonl", "")
        is_v6 = base.startswith("easy_")
        for agname, naming in agent_by_naming:
            if (naming == "easy") != is_v6:
                continue
            rest = base[len("easy_"):] if is_v6 else base
            cand = os.path.join(SUBMISSIONS, agname, f"easy_{rest}.json")
            if os.path.exists(cand):
                runs.append({"stream": f, "gt": cand, "agent": agname,
                             "key": rest})
                break
        else:
            print(f"  !! no result JSON for stream {os.path.basename(f)} "
                  "(skipped)", file=sys.stderr)
    return runs


def gt_map_positions(gt: dict, spawn: dict) -> dict:
    out = {}
    for k, e in gt.items():
        mx, my = world_to_map(float(e["x"]), float(e["y"]), spawn)
        out[k] = {"type": e["type"], "mission": bool(e.get("mission_target")),
                  "mx": mx, "my": my}
    return out


# ── FOV evidence ─────────────────────────────────────────────────────────────
def fov_evidence(poses, t0, t1, mx, my, spawn, step=5.0,
                 range_max=RANGE_SIGHT):
    """In-FOV evidence on a uniform pose grid over the whole stream.

    `poses` is the sorted [(t, rx, ry, ryaw)] track from the sighting stream
    (one pose per sighted-frame — object-presence-dependent by nature). On the
    grid, pose is linearly interpolated between the bracketing stream samples;
    a run-edge sample is repeated outside the span. Grid step 5 s < dwell bar
    (2 s). LOS ray-cast is evaluated in WORLD frame (map→world via spawn).
    """
    world_spawn = (float(spawn.get("x", 0.0)), float(spawn.get("y", 0.0)),
                   float(spawn.get("yaw", 0.0)))

    def to_world(x, y):
        sx, sy, yaw = world_spawn
        c, s = math.cos(yaw), math.sin(yaw)
        return sx + c * x - s * y, sy + s * x + c * y

    wx, wy = to_world(mx, my)
    grid_t = _linspace_t(t0, t1, step)
    samples, lost = [], 0
    for t in grid_t:
        rx, ry, ryaw = _pose_at(poses, t)
        r = math.hypot(mx - rx, my - ry)
        if r > range_max:
            continue
        b = abs(math.degrees(ang_diff(math.atan2(my - ry, mx - rx), ryaw)))
        if b <= FOV_HALF_DEG:
            rox, roy = to_world(rx, ry)
            clear = ray_clear(rox, roy, wx, wy, clearance=0.35)
            if not clear:
                lost += 1
            samples.append((t, r, b, clear))
    best_n, best_dwell = 0, 0.0
    if samples:
        run_start, prev_t, n = samples[0][0], samples[0][0], 1
        best_n = 1
        for t, _, _, _ in samples[1:]:
            if t - prev_t <= step + 1.0:   # same "pass" on the grid
                n += 1
            else:
                n = 1
                run_start = t
            if t - run_start > best_dwell:
                best_dwell = t - run_start
            if n > best_n:
                best_n = n
            prev_t = t
    vis = [s for s in samples if s[3]]
    return {
        "n": len(samples),
        "n_visible": len(vis),
        "n_los_lost": lost,
        "best_consecutive": best_n,
        "dwell_s": best_dwell,
        "min_range": min((r for _, r, _, _ in samples), default=None),
        "min_bearing_deg": min((b for _, _, b, _ in samples), default=None),
    }


def classify(ev, ev6, cls_sightings_near):
    if cls_sightings_near:
        return "d", "class sighted within 2 m (tracker-side, out of scope)"
    for ev_ in (ev, ev6):
        if ev_["n"] > 0 and ev_["n"] == ev_["n_los_lost"]:
            continue  # "in-FOV" only behind walls → not genuine FOV exposure
    if ev["n_visible"] == 0 and ev6["n_visible"] == 0:
        return "a", ("never in FOV (0 visible samples @ 5 and 6 m; "
                     "LOS-lost excluded)")
    if ev6["n_visible"] > 0 and ev["n_visible"] == 0:
        return "b", (f"in-FOV only @ 5–6 m ({ev6['n_visible']} visible "
                     "samples)")
    ev_eff = ev if ev["n_visible"] > 0 else ev6
    rng = ev_eff["min_range"]
    note = f", min range {rng:.1f} m" if rng is not None else ""
    if ev_eff["best_consecutive"] >= DWELL_SAMPLES and \
            ev_eff["dwell_s"] >= DWELL_SECONDS:
        return "c", (f"in-FOV-dwelled ({ev_eff['n_visible']} visible "
                     f"samples, {ev_eff['best_consecutive']} consec, "
                     f"{ev_eff['dwell_s']:.0f}s{note}) → detector recall")
    return "b", (f"in-FOV-brief ({ev_eff['n_visible']} visible samples, "
                 f"{ev_eff['best_consecutive']} consec, "
                 f"{ev_eff['dwell_s']:.0f}s{note})")


# ── main ─────────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        REPO, "results", "detector_absence_forensics.md"))
    args = ap.parse_args()

    runs = inventory()
    events, anchors = [], []
    class_counts, per_agent = {}, {}

    for run in runs:
        header, sightings = load_stream(run["stream"])
        spawn = header["spawn_pose"]
        raw = json.loads(open(run["gt"]).read())
        gt = (raw.get("raw_metrics", raw).get("ground_truth_objects")
              or raw.get("ground_truth_objects"))
        gtm = gt_map_positions(gt, spawn)
        by_cls: dict[str, list] = {}
        for s in sightings:
            by_cls.setdefault(s["class_name"], []).append(s)
        tp_keys = set()
        for row in raw.get("raw_metrics", raw).get("submission_log", []):
            if row.get("outcome") == "TP":
                tp_keys.add(row.get("nearest_gt_key"))
        poses = sorted((s["t"], s["rx"], s["ry"], s["ryaw"])
                       for s in sightings)
        span0 = poses[0][0] if poses else 0.0
        span1 = poses[-1][0] if poses else 0.0

        for k, e in gt.items():
            if not e["type"] or not gtm[k]["mission"]:
                continue
            if k in tp_keys:
                continue  # found — not an absence
            near = 0
            for s in by_cls.get(e["type"], []):
                if math.hypot(gtm[k]["mx"] - s["x"],
                              gtm[k]["my"] - s["y"]) <= NEAR_M:
                    near += 1
            ev = fov_evidence(poses, span0, span1, gtm[k]["mx"], gtm[k]["my"],
                              spawn, range_max=RANGE_SIGHT)
            ev6 = fov_evidence(poses, span0, span1, gtm[k]["mx"], gtm[k]["my"],
                               spawn, range_max=RANGE_SPEC)
            cat, why = classify(ev, ev6, near)
            rec = {
                "agent": run["agent"], "key": run["key"], "gt": k,
                "type": e["type"], "cat": cat, "why": why,
                "n5": ev["n"], "n_vis": ev["n_visible"],
                "n6v": ev6["n_visible"],
                "consec": ev["best_consecutive"], "dwell": ev["dwell_s"],
                "rng": ev["min_range"], "bear": ev["min_bearing_deg"],
                "near": near, "n_pose": len(sightings),
                "map": (round(gtm[k]["mx"], 2), round(gtm[k]["my"], 2)),
            }
            events.append(rec)
            class_counts.setdefault(e["type"], {"a": 0, "b": 0, "c": 0,
                                                "d": 0})
            class_counts[e["type"]][cat] += 1
            ag = per_agent.setdefault(run["agent"], {})
            ag.setdefault(e["type"], {"a": 0, "b": 0, "c": 0, "d": 0})
            ag[e["type"]][cat] += 1

        # pipeline-validation anchors: KNOWN zero-sighting absences
        if run["agent"] == "derpbot-explorer-stream-corpus":
            for k, e in gt.items():
                if e["type"] == "first_aid_kit" and not by_cls.get(
                        "first_aid_kit"):
                    ev = fov_evidence(poses, span0, span1, gtm[k]["mx"],
                                      gtm[k]["my"], spawn,
                                      range_max=RANGE_SIGHT)
                    ev6 = fov_evidence(poses, span0, span1, gtm[k]["mx"],
                                       gtm[k]["my"], spawn,
                                       range_max=RANGE_SPEC)
                    anchors.append({
                        "gt": k, "n5": ev["n"], "n_vis": ev["n_visible"],
                        "n6v": ev6["n_visible"],
                        "consec": ev["best_consecutive"],
                        "dwell": ev["dwell_s"], "rng": ev["min_range"],
                        "bear": ev["min_bearing_deg"],
                        "map": (round(gtm[k]["mx"], 2), round(gtm[k]["my"], 2)),
                    })

    # ── report ────────────────────────────────────────────────────────────
    lines = []
    a = lines.append
    a("# Detector-absence forensics — FOV/dwell classification (zero-sim)")
    a("")
    a("Inputs: all recorded streams whose result JSON exists "
      "(%d streams): corpus v5-master-recorded + v6-fastconfirm per-run batch "
      "streams. Ground truth from the corresponding submission result JSONs. "
      "No sim time spent."
      % len(runs))
    a("")
    a("Camera model (code-verified): 90° H-FOV, forward-facing, "
      "depth validity 0.30–5.0 m (`agent/depth_projector.py:56-57`), "
      "GT map-frame transform via spawn pose (scorer-equivalent: "
      "`robot-sandbox/src/metrics/object_detection_tracker.py:286-295`). "
      "In-FOV = |bearing−yaw| ≤ 45° AND range ≤ 5 m (6 m variant reported).")
    a("")
    a("Categories: (a) never-in-FOV → exploration/camera-aim; "
      "(b) in-FOV-brief (<3 consecutive samples or dwell <2s) → dwell/patrol; "
      "(c) in-FOV-dwelled (≥3 consecutive samples AND ≥2s) but zero class "
      "sightings → detector recall; (d) sightings existed but rejected "
      "(tracker — exhausted, counted for completeness only).")
    a("")
    a("Method: pose track = one sample per sighted frame (detector-fire "
      "dependent). FOV exposure is evaluated on a 5 s uniform grid with "
      "linear interpolation between bracketing stream samples, over the full "
      "stream span. A wall-occlusion (LOS) ray-cast on the GT occupancy map "
      "filters 'in-FOV but behind walls' samples — a run whose in-FOV samples "
      "are ALL LOS-lost counts as never-in-FOV.")
    a("")
    a("## Pipeline validation anchors (known zero-FA-sighting absences, "
      "seed5 corpus, must classify b or c)")
    a("")
    a("| GT | class | pos (map) | in-FOV @≤5m | min range | min bearing | dwell |")
    a("|----|-------|-----------|-------------|-----------|-------------|-------|")
    for an in anchors:
        a("| GT%s | first_aid_kit | (%.2f, %.2f) | %d visible (grid %d) "
          "(consec %d) "
          "| %s m | %s° | %.0f s |"
          % (an["gt"], an["map"][0], an["map"][1], an["n_vis"], an["n5"],
             an["consec"],
             f"{an['rng']:.2f}" if an["rng"] is not None else "—",
             f"{an['bear']:.1f}" if an["bear"] is not None else "—",
             an["dwell"]))
    a("")
    ok = all((x["n_vis"] + x["n6v"]) > 0 for x in anchors)
    a("**Anchor verdict: %s** — %s"
      % ("PASS" if ok else "FAIL",
         "both known FA zero-sighting absences show genuine FOV exposure "
         "(pipeline validated)" if ok else
         "no FOV exposure found on a known-absence — pipeline suspect"))
    a("")
    a("## Per-event table (every mission-target absence, per stream × run)")
    a("")
    a("| agent | run | GT | class | cat | why | in-FOV@5m | min rng/bear | sightings of class within 2m |")
    a("|---|---|---|---|---|---|---|---|---|")
    for e in sorted(events, key=lambda x: (x["agent"], x["key"], x["gt"])):
        a("| %s | %s | GT%s | %s | **%s** | %s | vis=%d/%d (6m: %d), %d "
          "consec, %.0fs | %s m / %s° | %d |"
          % (e["agent"], e["key"], e["gt"], e["type"], e["cat"], e["why"],
             e["n_vis"], e["n5"], e["n6v"], e["consec"], e["dwell"],
             f"{e['rng']:.2f}" if e["rng"] is not None else "—",
             f"{e['bear']:.1f}" if e["bear"] is not None else "—",
             e["near"]))
    a("")
    agg = {"a": 0, "b": 0, "c": 0, "d": 0}
    for e in events:
        agg[e["cat"]] += 1
    a("## Aggregate")
    a("")
    a("| class | (a) never-in-FOV | (b) in-FOV-brief | (c) in-FOV-dwelled | (d) sighted-rejected | total |")
    a("|---|---|---|---|---|---|")
    for ct, cc in sorted(class_counts.items()):
        a("| %s | %d | %d | %d | %d | %d |"
          % (ct, cc["a"], cc["b"], cc["c"], cc["d"],
             sum(cc.values())))
    a("| **total** | %d | %d | %d | %d | %d |"
      % (agg["a"], agg["b"], agg["c"], agg["d"], len(events)))
    a("")
    a("## Per-agent breakdown")
    a("")
    for agent, ag in per_agent.items():
        row = {c: 0 for c in "abcd"}
        tot = 0
        for ct, cc in ag.items():
            for c in "abcd":
                row[c] += cc[c]
                tot += cc[c]
        a("- **%s**: total %d — a=%d b=%d c=%d d=%d"
          % (agent, tot, row["a"], row["b"], row["c"], row["d"]))
        for ct, cc in sorted(ag.items()):
            a("  - %s: a=%d b=%d c=%d d=%d (n=%d)"
              % (ct, cc["a"], cc["b"], cc["c"], cc["d"], sum(cc.values())))
    a("")
    if agg["a"] + agg["b"] + agg["c"] > 0:
        dominant = max((("a", agg["a"]), ("b", agg["b"]), ("c", agg["c"])),
                       key=lambda kv: kv[1])
        verdict = {
            "a": ("exploration / camera-aim fix (never-in-FOV dominates)"),
            "b": ("dwell / patrol tweak (brief-FOV dominates)"),
            "c": ("detector-recall fix — generic multi-scale / ROI second-pass "
                 "inference on upscaled crops (no class names; prompt set stays "
                 "mission-derived)"),
        }[dominant[0]]
        a("## Verdict")
        a("")
        a("Majority mechanism: category **(%s)** with %d/%d absence events "
          "(%4.1f%%). Recommended mechanism: %s."
          % (dominant[0], dominant[1], agg["a"] + agg["b"] + agg["c"],
             100.0 * dominant[1] / (agg["a"] + agg["b"] + agg["c"]), verdict))
        a("")
        a("Ranking by miss-count coverage:")
        for cat, why in (("c", "detector recall (multi-scale/ROI second-pass)"),
                         ("b", "dwell / patrol tweak"),
                         ("a", "exploration / camera-aim fix")):
            a("- (%s) %s — %d events" % (cat, why, agg[cat]))
        a("")
    a("Sanity anchors: absence-event totals vs the doc's attributions — "
      "the parity batch's 9 far-misses are NOT reproducible from streams "
      "(recorder postdates that batch); the v6 batch's detector-absence "
      "bucket is computed above from its 15 streams (the doc header's '12' "
      "counts only its 7 named + s3r2-enumerated objects under a different "
      "rule — see the per-event table for the ground-up classification).")
    a("")
    # dwell-bar sensitivity: strict variant requires dwell >= 2*step (>=2
    # grid transitions, i.e. exposure spans >= 10 s) vs the task bar (>=2 s).
    strict_c = sum(1 for e in events if e["cat"] == "c" and e["dwell"] < 9.0)
    a("Dwell-bar sensitivity: category (c) re-classification under a strict "
      "dwell (dwell ≥ 2 grid steps ≈ 10 s, i.e. two consecutive 5 s grid "
      "transitions instead of the 2 s bar): %d of %d (c) events fall below "
      "10 s and would degrade to (b)."
      % (strict_c, agg["c"]))
    a("")

    with open(args.out, "w") as fh:
        fh.write("\n".join(lines))
    print("wrote", args.out)
    print("events:", len(events), "anchors:", len(anchors))
    print("aggregate:", agg)
    return 0


if __name__ == "__main__":
    sys.exit(main())