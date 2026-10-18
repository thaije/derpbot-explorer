#!/usr/bin/env python3
"""Build the labeled recognition benchmark from sim keyframes (t_fe1459a9).

Per keyframe (results/streams/frames/<run>/f_<t>.jpg):
  - pair the robot pose from the run's sighting stream at matched sim-time
    (±POSE_TOL_S), map odom → world via the spawn-pose header (bake-off
    pipeline, 84 % bearing agreement — scripts/bakeoff_run.py);
  - camera model per run from the stream header (v2 1080p camera_info) or the
    v1 640×480 constants (projection_forensics.stream_camera());
  - for EVERY GT class (all 9 scenario classes from the v10-assoc result
    JSONs, mission_target flag preserved), compute geometric camera
    visibility: horizontal bearing within the URDF 90° hFOV (+ slack), LOS
    wall-clear on the GT PGM (bresenham, 0.5 m/cell), range ≤ MAX_RANGE_M;
  - project each visible GT instance to a PIXEL BBOX via the per-run pinhole
    camera (class-size aware: per-class world-space (cx,cy,cz)+half-extents).
  - metadata per instance: range bin, px-area bin, clutter = #visible GT.
    NOTE: labels are OCCLUSION-UNAWARE (see docs/recog_benchmark.md): the LOS
    ray-cast clears Walls on the 0.5 m/cell GT PGM only — an instance hidden
    behind furniture (chairs, cabinets, desks are not in the PGM) still counts
    as visible. The old per-instance "occluded" field was a hardcoded False
    stub and was REMOVED in this revision (review round 1, item 3).
  - manifest JSONL + split files (corpus / holdout / stress) + stats.

Splits:
  corpus  — seeds 1–5 v1 + v2 full-res probe runs (tuned-on data);
  holdout — seeds 6–10 (Phase 2, added when captured);
  stress  — perception_stress scenarios (Phase 2).

Run from the explorer repo root:
    .venv/bin/python scripts/build_recog_dataset.py [--out results/recog_benchmark]
Verification overlay:
    .venv/bin/python scripts/verify_labels.py --frame <run>/<frame>
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

EXPLORER_ROOT = Path("/home/plip/Projects/derpbot-explorer")
FRAMES_DIR = EXPLORER_ROOT / "results" / "streams" / "frames"
STREAMS_DIR = EXPLORER_ROOT / "results" / "streams"
GT_DIRS = [
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v10-assoc"),
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-stream-corpus"),
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v2holdout"),
]
PGM_PATH = Path("/home/plip/Projects/robot-sandbox/worlds/templates/indoor_office/ground_truth_map.pgm")
PGM_RES = 0.5

# ── Camera (URDF fixed geometry, both v1 and v2) ─────────────────────────────
CAMERA_FORWARD_OFFSET = 0.05   # camera_joint x offset (m)
CAMERA_HEIGHT_M = 0.25         # camera z above ground (0.05 base + 0.20 joint)
V1_CAMERA = {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
             "width": 640, "height": 480, "source": "v1_no_header"}
HFOV = math.pi / 2
FOV_EDGE_SLACK = 0.02          # rad

POSE_TOL_S = 0.5
MAX_RANGE_M = 6.0
MIN_RANGE_M = 0.4

# ── Per-class appearance geometry (model-frame) ──────────────────────────────
# (cx, cy, cz, rx, ry, rz) in metres — center + half-extents, derived from the
# model meshes with the DAE unit-scale applied (person mesh is millimetre,
# extinguisher/drill centimetre; verified against bakeoff_run.py's CENTER_Z for
# the three mission classes). cz = center height above the object's spawn z.
# GT JSONs carry no z; easy + perception_stress place every class on the floor
# (only drink_can uses surface=desk, spawn z 0.74 — flagged per instance).
#
# The (cx, cy) centre offsets are MODEL-FRAME (mesh centroid minus model
# origin) and MUST be rotated by the object's placement yaw into world axes
# before projection (review round 2, item 3): applying them unrotated put the
# person hull ~0.32–0.46 m off for 75 % of instances. The object yaw is not in
# the GT JSONs but IS recoverable offline: the deterministic WorldGenerator
# replay (scripts/replay_object_yaws.py) regenerates each run's world from the
# result JSON's stored seed and matches GT coordinates to ≤0.002 m on all 25
# runs — the generated SDF carries every object's full 6-DOF pose, so yaw (and
# exact z) come along for free. The builder loads generated_gt_with_yaw/
# <run>.json when present and falls back to yaw 0 with the raw GT otherwise.
# GT-side extents rx/ry are yaw-invariant (max over the mesh's x/y axes).
CLASS_GEO = {
    "fire_extinguisher": (0.0, 0.0, 0.191, 0.075, 0.042, 0.189),
    "first_aid_kit":     (0.0, 0.0, 0.220, 0.150, 0.060, 0.110),
    "person":            (-0.0993, -0.2052, 0.3618, 0.5092, 0.5092, 0.3618),
    "trash_can":         (0.0, 0.0, 0.368, 0.283, 0.319, 0.368),
    "office_chair":      (-0.037, 0.001, 0.548, 0.448, 0.448, 0.548),
    "drink_can":         (0.0, 0.0, 0.072, 0.030, 0.030, 0.072),
    "drill":             (0.0, 0.030, 0.127, 0.111, 0.111, 0.127),
    "suitcase":          (0.0, 0.0, 0.0005, 0.0715, 0.029, 0.0695),
    "exit_sign":         (0.0, 0.0, 0.116, 0.150, 0.029, 0.116),
}
# Yaw note (review round 1): the placer assigns model yaws ∈ {0, π/2, π, 3π/2}
# (object_placer.py) and the GT JSONs carry no yaw, so the per-axis half-extents
# above are the MAX over the mesh's x/y axes for the asymmetric classes (person,
# office_chair, drill) — a yaw-invariant hull that slightly over-covers for
# yaw = 0/π/2; preferred over under-covering labels.
# Person z: rendered truth = world z span [0, 0.7235] (floor contact → head
# top; the SDF link z −0.624 exactly cancels the sitting mesh's internal
# 0.62 m head-origin offset, so the *model-frame* z span is the rendered span).
# spawn-z offsets: classes placed on furniture / fixtures in the scenario yamls
# + the model.sdf link-level z offsets folded into cz above (FAK link 0.11,
# person link −0.624 + mm-mesh unit scale). exit_sign is model-posed at 1.25 m
# on the office door headers (office_easy_001.sdf).
SPAWN_Z = {"drink_can": 0.74, "exit_sign": 1.25}
GT_CLASSES = tuple(CLASS_GEO)

# ── Prompts per class (live-pipeline underscore→space convention) ────────────
PROMPTS = {
    "fire_extinguisher": "fire extinguisher",
    "first_aid_kit": "first aid kit",
    "person": "person",
    "trash_can": "trash can",
    "office_chair": "office chair",
    "drink_can": "soda can",
    "drill": "power drill",
    "suitcase": "suitcase",
    "exit_sign": "exit sign",
}

RANGE_BIN_EDGES = (1.5, 3.0)          # ≤1.5 / 1.5–3 / 3–6
AREA_BIN_EDGES = (100, 1000)          # tiny / small / large (px² on stored res)


# ─── Geometry helpers (shared with bakeoff_run.py / projection_forensics.py) ─


def read_pgm(path: Path):
    m = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s", path.read_bytes()[:64])
    data = path.read_bytes()
    w, h = int(m.group(1)), int(m.group(2))
    off = m.end()
    pix = np.frombuffer(data[off:off + w * h], dtype=np.uint8).reshape(h, w)
    return pix, w, h


def bresenham(x0, y0, x1, y1):
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 > -dy:
            err -= dy; x0 += sx
        if e2 < dx:
            err += dx; y0 += sy


class LOS:
    def __init__(self, pgm_path=PGM_PATH):
        self.pix, self.w, self.h = read_pgm(pgm_path)

    def clear(self, cx_w, cy_w, ox_w, oy_w) -> bool:
        c0, r0 = int(cx_w / PGM_RES), self.h - 1 - int(cy_w / PGM_RES)
        c1, r1 = int(ox_w / PGM_RES), self.h - 1 - int(oy_w / PGM_RES)
        for c, r in bresenham(c0, r0, c1, r1):
            if 0 <= r < self.h and 0 <= c < self.w:
                if self.pix[r][c] < 128:
                    return False
        return True


def load_stream(path: Path):
    header, poses = {}, []
    for line in path.read_text().splitlines():
        r = json.loads(line)
        if r.get("type") == "header":
            header = r
        else:
            poses.append((r["t"], r["rx"], r["ry"], r["ryaw"]))
    return header, poses


def aggregate_pose_rows(rows):
    """Deterministic pose for one timestamp with multiple stream rows.

    Sighting records at the same sim-time carry the pose AS OF EACH
    SIGHTING; rows within one batch can disagree on yaw (measured: 5954
    multi-row timestamps across 91 streams, 7.9 % with >5° intra-t spread,
    worst 179° — review round 2, item 2; argmin over a sorted array picked
    an arbitrary row). Aggregate instead: median rx/ry over the rows, and
    the circular mean of ryaw (rows are near-uniform angles; unwrap via
    complex mean, which is also the robust central value for a small spread).
    Returns (x, y, yaw, spread_rad).
    """
    n = len(rows)
    xs = [float(r[1]) for r in rows]
    ys = [float(r[2]) for r in rows]
    yaws = [float(r[3]) for r in rows]
    cy_ = sum(math.sin(a) for a in yaws) / n
    cx_ = sum(math.cos(a) for a in yaws) / n
    yaw = math.atan2(cy_, cx_)
    # circular spread: max angular deviation from the mean
    spread = max(abs((a - yaw + math.pi) % (2 * math.pi) - math.pi) for a in yaws)
    return (sorted(xs)[n // 2], sorted(ys)[n // 2], yaw, spread)


def load_frames_dir(d: Path):
    # f.stem = "f_<t>" → strip the "f_" prefix (old [2:-4] truncated the
    # fractional seconds off "f_41.824" → pose paired up to 1 s stale).
    return sorted((float(f.stem[2:]), f) for f in d.glob("f_*.jpg"))


def pose_at(poses: dict, t: float, tol=POSE_TOL_S, max_spread: float = None):
    """Best timestamps: distinct stream timestamps within ±tol of the frame.

    Yields ONE aggregated pose via aggregate_pose_rows from the rows whose
    |Δt| equals the minimum (the stream is not uniformly sampled — multiple
    rows may sit at slightly different, equally-near times; all minimal-Δt
    rows are aggregated together). Returns (x, y, yaw, spread) or None when
    nothing is within tol, or when the intra-batch yaw spread exceeds
    max_spread (unregisterable frame — review round 2, item 2).
    """
    if not isinstance(poses, dict):
        raise TypeError("poses must be a {t: [rows]} dict from load_stream")
    cand = [pt for pt in poses if abs(pt - t) <= tol]
    if not cand:
        return None
    dt_min = min(abs(pt - t) for pt in cand)
    best = [pt for pt in cand if abs(pt - t) == dt_min]
    rows = []
    for pt in sorted(best):
        rows.extend(poses[pt])
    x, y, yaw, spread = aggregate_pose_rows(rows)
    if max_spread is not None and spread > max_spread:
        return None
    return x, y, yaw, spread


def stream_camera(header: dict) -> dict:
    """Per-run camera model (projection_forensics.stream_camera)."""
    cam = header.get("camera")
    if isinstance(cam, dict) and cam.get("fx") and cam.get("width"):
        fx = float(cam["fx"])
        w = int(cam["width"]); h = int(cam["height"])
        return {"fx": fx, "fy": float(cam.get("fy", fx)),
                "cx": float(cam.get("cx", w / 2.0)), "cy": float(cam.get("cy", h / 2.0)),
                "width": w, "height": h, "source": cam.get("source", "header")}
    return dict(V1_CAMERA)


def odom_to_world(header, rx, ry, ryaw):
    sp = header["spawn_pose"]
    sy = sp["yaw"]
    wx = sp["x"] + math.cos(sy) * rx - math.sin(sy) * ry
    wy = sp["y"] + math.sin(sy) * rx + math.cos(sy) * ry
    return wx, wy, sy + ryaw


def camera_pos(wx, wy, yaw_w):
    return wx + CAMERA_FORWARD_OFFSET * math.cos(yaw_w), \
        wy + CAMERA_FORWARD_OFFSET * math.sin(yaw_w)


def load_gt(seed: int, run: int, difficulty: str = "easy", run_key: str = ""):
    """Per-run GT → {key: {cls,x,y,mission,yaw,z}}.

    easy GT comes from the result JSONs (v10-assoc / stream-corpus); other
    difficulties (perception_stress) from the replay-verified generated_gt.
    Placement yaw + exact z per object come from the deterministic
    WorldGenerator replay (generated_gt_with_yaw/, scripts/replay_object_yaws.py,
    ≤0.002 m coordinate match on all 25 runs) — used to rotate the
    model-frame hull offsets into world axes (review round 2, item 3).
    Missing replay entry (a run added before the replay was generated) falls
    back to yaw 0.0 + the per-class spawn-z constants.
    """
    yawed_path = (EXPLORER_ROOT / "results/recog_benchmark/generated_gt_with_yaw"
                  / f"{run_key}.json")
    yawed = json.load(open(yawed_path)) if yawed_path.exists() else None
    if difficulty != "easy":
        p = EXPLORER_ROOT / "results/recog_benchmark/generated_gt" / f"{run_key}.json"
        if not p.exists():
            return None
        gt = json.load(open(p))
        return {k: {**_gt_entry(v), "yaw": _yaw_of(yawed, k),
                    "z": (yawed or {}).get(k, {}).get("z", v.get("z", 0.0))}
                for k, v in sorted(gt.items(), key=lambda kv: int(kv[0]))}
    for d in GT_DIRS:
        p = d / f"{difficulty}_seed{seed}_run{run}.json"
        if p.exists():
            gt = json.load(open(p))["raw_metrics"]["ground_truth_objects"]
            out = {}
            for k, v in sorted(gt.items(), key=lambda kv: int(kv[0])):
                out[k] = {**_gt_entry(v), "yaw": _yaw_of(yawed, k),
                          "z": (yawed or {}).get(k, {}).get("z", 0.0)}
            return out
    return None


def _gt_entry(v: dict) -> dict:
    return {"cls": v["type"], "x": v["x"], "y": v["y"],
            "mission": bool(v.get("mission_target", False))}


def _yaw_of(yawed: dict | None, key: str) -> float:
    return float((yawed or {}).get(key, {}).get("yaw", 0.0))


def rel_bearing(cx, cy, yaw_w, gx, gy):
    return (math.atan2(gy - cy, gx - cx) - yaw_w + math.pi) % (2 * math.pi) - math.pi


def range_bin(r):
    if r <= RANGE_BIN_EDGES[0]:
        return "near"
    if r <= RANGE_BIN_EDGES[1]:
        return "mid"
    return "far"


def area_bin(a):
    if a < AREA_BIN_EDGES[0]:
        return "tiny"
    if a < AREA_BIN_EDGES[1]:
        return "small"
    return "large"


def project_bbox(cx, cy, cz_cam, yaw_w, x, y, z_spawn, geo, W, H,
                 fx, fy, cx0, cy0, obj_yaw=0.0):
    """World-space object AABB → pixel bbox.

    Projects the 8 corners of the object's world box — centre
    (x + R(obj_yaw)·geo_cx, y + R(obj_yaw)·geo_cy, z_spawn + geo_cz),
    half-extents geo — and takes the pixel hull. geo[0]/geo[1] are the
    MODEL-FRAME centre offsets for asymmetric meshes (person arms/legs); they
    are rotated by the object's placement yaw into world axes here (review
    round 2, item 3 — an unrotated model-frame offset is wrong for 3 of the
    4 possible placement yaws). Returns (x1, y1, x2, y2) or None when fully
    outside.
    """
    gcx, gcy, cz, hx, hy, hz = geo
    s, c = math.sin(obj_yaw), math.cos(obj_yaw)
    x += c * gcx - s * gcy
    y += s * gcx + c * gcy
    zc = z_spawn + cz
    us, vs = [], []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (-hz, hz):
                px, py, pz = x + dx, y + dy, zc + dz
                d = math.hypot(px - cx, py - cy)
                if d < 0.05:
                    continue
                rel = (math.atan2(py - cy, px - cx) - yaw_w + math.pi) \
                    % (2 * math.pi) - math.pi
                if abs(rel) > HFOV / 2 + FOV_EDGE_SLACK:
                    continue
                # pinhole: x right = -bearing (live map convention, bakeoff_run)
                u = cx0 + fx * (-math.tan(rel))
                v_ang = math.atan((pz - cz_cam) / max(d, 0.1))
                v = cy0 - fy * math.tan(v_ang)
                us.append(u); vs.append(v)
    if not us:
        return None
    x1, x2 = max(0.0, min(us)), min(float(W), max(us))
    y1, y2 = max(0.0, min(vs)), min(float(H), max(vs))
    if x2 - x1 < 1 or y2 - y1 < 1:
        return None
    return x1, y1, x2, y2


# ─── Driver ──────────────────────────────────────────────────────────────────


def run_split(run_key: str) -> str:
    if "perception_stress" in run_key:
        return "stress"
    m = re.search(r"seed(\d+)", run_key)
    if m and int(m.group(1)) >= 6:
        return "holdout"
    return "corpus"


def instance_visible_bbox(cx, cy, cz_cam, yaw_w, x, y, z_spawn, geo, W, H,
                          fx, fy, cx0, cy0, obj_yaw=0.0):
    """FOV-gated single-instance projection (center + 4 extremes per axis).

    Keeps the object when its CENTER bearing is inside the FOV, so objects
    straddling the frame edge still get a (clipped) box. geo[0]/geo[1] are the
    MODEL-FRAME centre offsets (asymmetric meshes), rotated by obj_yaw into
    world axes (see project_bbox).
    """
    gcx, gcy, cz, hx, hy, hz = geo
    s, c = math.sin(obj_yaw), math.cos(obj_yaw)
    x += c * gcx - s * gcy
    y += s * gcx + c * gcy
    d = math.hypot(x - cx, y - cy)
    rel = (math.atan2(y - cy, x - cx) - yaw_w + math.pi) % (2 * math.pi) - math.pi
    if abs(rel) > HFOV / 2 + FOV_EDGE_SLACK:
        return None
    corners = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (z_spawn + cz - hz, z_spawn + cz + hz):
                corners.append((x + dx, y + dy, dz))
    us, vs = [], []
    for (px, py, pz) in corners:
        dd = math.hypot(px - cx, py - cy)
        if dd < 0.05:
            continue
        r = (math.atan2(py - cy, px - cx) - yaw_w + math.pi) % (2 * math.pi) - math.pi
        u = cx0 + fx * (-math.tan(r))
        v_ang = math.atan((pz - cz_cam) / max(dd, 0.1))
        v = cy0 - fy * math.tan(v_ang)
        us.append(u); vs.append(v)
    if not us:
        return None
    x1, x2 = max(0.0, min(us)), min(float(W), max(us))
    y1, y2 = max(0.0, min(vs)), min(float(H), max(vs))
    if x2 - x1 < 1 or y2 - y1 < 1:
        return None
    return x1, y1, x2, y2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results/recog_benchmark")
    ap.add_argument("--pose-tol", type=float, default=POSE_TOL_S)
    ap.add_argument("--yaw-spread-gate", type=float, default=None,
                    help="drop frames whose aggregated stream rows disagree on "
                         "yaw by more than this many RADIANS (unregisterable "
                         "pose; default: no gate, spread recorded per frame)")
    ap.add_argument("--runs", nargs="*", default=None)
    args = ap.parse_args()

    los = LOS()
    out_dir = EXPLORER_ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    run_dirs = sorted(d for d in FRAMES_DIR.iterdir()
                      if d.is_dir() and d.name != "latest")
    if args.runs:
        run_dirs = [d for d in run_dirs if d.name in set(args.runs)]

    man_path = out_dir / "manifest.jsonl"
    n_frames = 0
    n_inst = 0
    per_run = {}
    bin_histo = defaultdict(Counter)
    with open(man_path, "w") as man:
        for run_dir in run_dirs:
            run_key_full = run_dir.name
            stream_path = STREAMS_DIR / f"{run_key_full}.jsonl"
            if not stream_path.exists():
                print(f"== skip {run_key_full}: no stream JSONL")
                continue
            m_seed = re.search(r"seed(\d+)", run_key_full)
            m_run = re.search(r"run(\d+)", run_key_full)
            if not (m_seed and m_run):
                print(f"== skip {run_key_full} (no seed/run)")
                continue
            seed, run = int(m_seed.group(1)), int(m_run.group(1))
            difficulty = "perception_stress" \
                if "perception_stress" in run_key_full else "easy"
            gt = load_gt(seed, run, difficulty, run_key_full)
            if not gt:
                print(f"== skip {run_key_full}: no GT JSON")
                continue
            header, pose_rows = load_stream(stream_path)
            cam = stream_camera(header)
            # group stream rows by timestamp for deterministic aggregation
            from collections import defaultdict as _dd
            poses = _dd(list)
            for (t, rx, ry, ryaw) in pose_rows:
                poses[round(float(t), 6)].append((t, rx, ry, ryaw))
            split = run_split(run_key_full)
            n_vis_frames = 0
            n_dropped_spread = 0
            spread_worst = 0.0
            for (t, fpath) in load_frames_dir(run_dir):
                pose = pose_at(poses, t, args.pose_tol, args.yaw_spread_gate)
                if pose is None:
                    continue
                px, py, pyaw, spread = pose
                spread_worst = max(spread_worst, spread)
                img = cv2.imread(str(fpath))
                if img is None:
                    continue
                H, W = img.shape[:2]
                # Sightings/geometry live in the CAMERA frame; the camera_info
                # header carries NATIVE cam dims (1920×1080). The stored JPEG
                # is the detector-input frame (native for FULLRES dirs, 320w
                # downscale otherwise) — scale intrinsics AND the reported
                # width/height to the STORED res (review round 2, item 4:
                # native dims next to stored-res intrinsics misprojected
                # for any consumer that trusted camera.width).
                scale = W / cam["width"]
                fx, fy = cam["fx"] * scale, cam["fy"] * scale
                cx0, cy0 = cam["cx"] * scale, cam["cy"] * scale
                cam_out = {**cam, "fx": fx, "fy": fy, "cx": cx0, "cy": cy0,
                           "width": int(W), "height": int(H),
                           "native_width": cam["width"], "native_height": cam["height"],
                           "scale": round(scale, 6)}
                wx, wy, yaw_w = odom_to_world(header, px, py, pyaw)
                cp = camera_pos(wx, wy, yaw_w)
                objects = []
                for k, v in gt.items():
                    dx, dy = v["x"] - cp[0], v["y"] - cp[1]
                    d = math.hypot(dx, dy)
                    if d < MIN_RANGE_M or d > MAX_RANGE_M:
                        continue
                    rel = rel_bearing(cp[0], cp[1], yaw_w, v["x"], v["y"])
                    if abs(rel) > HFOV / 2 + FOV_EDGE_SLACK:
                        continue
                    if not los.clear(cp[0], cp[1], v["x"], v["y"]):
                        continue
                    geo = CLASS_GEO[v["cls"]]
                    z_spawn = SPAWN_Z.get(v["cls"], 0.0)
                    # exact per-instance spawn z from the WorldGenerator replay
                    # (desk placements, exit signs); yaw-rotated hull offsets
                    z_spawn = v.get("z", z_spawn)
                    bb = instance_visible_bbox(cp[0], cp[1], CAMERA_HEIGHT_M,
                                               yaw_w, v["x"], v["y"], z_spawn,
                                               geo, W, H, fx, fy, cx0, cy0,
                                               obj_yaw=v.get("yaw", 0.0))
                    if bb is None:
                        continue
                    x1, y1, x2, y2 = bb
                    area = (x2 - x1) * (y2 - y1)
                    objects.append({
                        "key": k, "cls": v["cls"], "mission": v["mission"],
                        "range": round(d, 3), "bearing": round(rel, 4),
                        "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                        "range_bin": range_bin(d), "area_bin": area_bin(area),
                        "area_px": round(area, 1), "yaw": round(v.get("yaw", 0.0), 4),
                    })
                    n_inst += 1
                    bin_histo[v["cls"]][range_bin(d)] += 1
                    bin_histo[v["cls"]]["area_" + area_bin(area)] += 1
                if objects:
                    n_vis_frames += 1
                man.write(json.dumps({
                    "frame_id": f"{run_key_full}/{fpath.name}",
                    "image": str(fpath.relative_to(EXPLORER_ROOT)),
                    "run": run_key_full, "split": split, "seed": seed, "run_n": run,
                    "t": t, "camera": cam_out,
                    "robot_xy": [round(cp[0], 3), round(cp[1], 3)], "yaw": round(yaw_w, 4),
                    "pose_rows": len(poses.get(round(t, 6), [])),
                    "pose_aggregated": True,
                    "yaw_spread": round(spread, 4),
                    "objects": objects, "n_objects": len(objects),
                }) + "\n")
                n_frames += 1
            per_run[run_key_full] = {"split": split, "frames": n_vis_frames,
                                     "seed": seed, "camera": cam["source"]}
            print(f"== {run_key_full} [{split}]: {n_vis_frames} frames "
                  f"(of {len(load_frames_dir(run_dir))}, worst yaw spread "
                  f"{math.degrees(spread_worst):.1f}°)")
    # split files
    split_runs = defaultdict(list)
    for rk, info in per_run.items():
        split_runs[info["split"]].append(rk)
    for sname, rks in split_runs.items():
        (out_dir / f"split_{sname}.txt").write_text("\n".join(sorted(rks)) + "\n")

    stats = {
        "frames": n_frames,
        "instances": n_inst,
        "runs": {s: len(v) for s, v in split_runs.items()},
        "run_list": per_run,
        "class_bins": {c: dict(v) for c, v in bin_histo.items()},
        "config": {"pose_tol": POSE_TOL_S, "max_range": MAX_RANGE_M,
                   "hfov": HFOV, "range_bins": RANGE_BIN_EDGES,
                   "area_bins": AREA_BIN_EDGES},
        "prompts": PROMPTS,
    }
    (out_dir / "dataset_stats.json").write_text(json.dumps(stats, indent=2))
    print(json.dumps({k: stats[k] for k in ["frames", "instances", "runs"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())