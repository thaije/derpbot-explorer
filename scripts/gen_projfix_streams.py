#!/usr/bin/env python3.12
"""Generate corrected sighting streams (projection A/B, zero-sim) — t_fd80a2f9 Phase 3.

For each recorded stream, rewrite every sighting's x,y with the corrected
projection mechanism (min(depth, 1.35 × ground-plane range)) — bbox bearing and
robot pose unchanged. The tracker replay (scripts/replay_tracker.py) then runs
unchanged on old vs new streams under v5-master semantics.
"""
from __future__ import annotations

import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))
from projection_forensics import load_stream, inventory, CX, CY, FX, FY  # noqa

GP_CORRECTION = 1.35


def corrected_xy(s):
    cx, cy, w, h = s["bbox"]
    u, dp = cx, s.get("depth")
    if dp is None:
        return None
    vb = cy + h / 2.0
    pitch = math.atan((vb - CY) / FY)
    if pitch <= 1e-3:
        return None   # bbox bottom at/above horizon: no ground contact → keep old
    gp = 0.25 / math.tan(pitch)
    rng = min(dp, gp * GP_CORRECTION)
    x_cam = (u - CX) / FX * rng
    y_cam = (cy - CY) / FY * rng
    rx, ry, ryaw = s["rx"], s["ry"], s["ryaw"]
    c_, s_ = math.cos(ryaw), math.sin(ryaw)
    return (rx + c_ * rng - s_ * (-x_cam), ry + s_ * rng + c_ * (-x_cam))


def main():
    outdir = os.path.join(REPO, "results", "streams_projfix")
    os.makedirs(outdir, exist_ok=True)
    runs = inventory()
    n_changed = 0
    for run in runs:
        header, sightings = load_stream(run["stream"])
        keep = (run["agent"] == "derpbot-explorer-v6-fastconfirm" and "easy_"
                or "")
        name = os.path.basename(run["stream"])
        lines = [json.dumps(header)]
        for s in sightings:
            s2 = dict(s)
            if s.get("depth") is not None:
                xy = corrected_xy(s)
                if xy is not None:
                    s2["x"], s2["y"] = round(xy[0], 3), round(xy[1], 3)
                    n_changed += 1
            lines.append(json.dumps(s2))
        out = os.path.join(outdir, name)
        with open(out, "w") as fh:
            fh.write("\n".join(lines) + "\n")
    print(f"wrote {len(runs)} corrected streams → {outdir} "
          f"({n_changed} sightings re-projected)")


if __name__ == "__main__":
    main()