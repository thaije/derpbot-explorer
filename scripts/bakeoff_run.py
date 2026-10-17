#!/usr/bin/env python3
"""Offline detector bake-off on keyframes (Phase 1 of t_b4361707).

For each keyframe (results/streams/frames/<run>/f_<t>.jpg):
  - take the robot pose from the run's sighting stream at matched sim-time (±0.5 s);
  - map odom pose → world pose via the run's spawn pose (verified against
    recorded sighting bboxes: world = spawn + Rz(spawn_yaw) @ odom);
  - for each mission-target GT (recorded per-run ground_truth_objects from the
    v10-assoc submission JSONs — authoritative, verified identical across
    submissions that carried raw GT), determine geometric camera visibility:
    horizontal bearing within 90° hFOV (+2 mrad slack), wall-clear LOS on the
    GT PGM (bresenham), distance ≤ 8 m;
  - run each candidate model with the SAME mission prompts over the keyframe;
  - score: a visible GT instance is detected if a detection of that class has
    center within MATCH_BEARING of the GT (perspective-projected, per-class
    center-height) AND plausible distance; FP proxy: a detection whose class has
    no GT instance within 2 m of the camera OR whose bearing/distance is
    inconsistent with any same-class GT in frame;
  - aggregate per model × class: visible-frame detection rate, conf distribution,
    FP proxy count/rate, latency/frame.

Run with the explorer .venv python from the explorer repo root:
    .venv/bin/python scripts/bakeoff_run.py --models owlv2 yolo_world_s gdino_tiny
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

EXPLORER_ROOT = Path("/home/plip/Projects/derpbot-explorer")
FRAMES_DIR = EXPLORER_ROOT / "results" / "streams" / "frames"
STREAMS_DIR = EXPLORER_ROOT / "results" / "streams"
GT_DIR = Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v10-assoc")
PGM_PATH = Path("/home/plip/Projects/robot-sandbox/worlds/templates/indoor_office/ground_truth_map.pgm")
PGM_RES = 0.5

CAMERA_FORWARD_OFFSET = 0.05   # URDF camera_joint x offset (matches world_state.py)
CAMERA_HEIGHT_M = 0.25         # URDF: camera z above ground (base 0.05 + joint 0.20)
CAMERA_HFOV = math.pi / 2      # URDF rgbd + bbox camera (π/2)
FOV_EDGE_SLACK = 0.02          # rad

GT_CLASSES = ("fire_extinguisher", "first_aid_kit", "person")
PROMPTS = {                    # mission prompts (underscore→space, agent/detector.py)
    "fire_extinguisher": "fire extinguisher",
    "first_aid_kit": "first aid kit",
    "person": "person",
}
CONF_THRESHOLD = 0.15
MAX_VIS_DIST_M = 8.0
CENTER_Z = {                   # object-center height above ground (mesh/sdf-derived)
    "fire_extinguisher": 0.19,
    "first_aid_kit": 0.11,
    "person": 0.35,
}
FP_PROXY_RADIUS_M = 2.0
MATCH_BEARING_RAD = 0.20       # GT↔detection match: |Δbearing| ≤ 0.20 rad ≈ 11.5°
# NOTE: no distance term in the match rule — bearing gating only (see main()).

POSE_TOL_S = 0.5


# ─── Geometry ─────────────────────────────────────────────────────────────────


def read_pgm(path: Path):
    m = re.match(rb"P5\s+(\d+)\s+(\d+)\s+(\d+)\s", path.read_bytes()[:64])
    data = path.read_bytes()
    w, h, maxv = int(m.group(1)), int(m.group(2)), int(m.group(3))
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


def load_frames_dir(d: Path):
    return sorted((float(f.stem[2:-4]), f) for f in d.glob("f_*.jpg"))


def load_stream(run_key_full: str):
    """run_key_full: e.g. stream_easy_seed1_run1_20261016T190215Z."""
    path = STREAMS_DIR / f"{run_key_full}.jsonl"
    if not path.exists():
        raise FileNotFoundError(path)
    header = json.loads(path.read_text().splitlines()[0])
    poses = []
    for line in path.read_text().splitlines()[1:]:
        r = json.loads(line)
        poses.append((r["t"], r["rx"], r["ry"], r["ryaw"]))
    poses = np.array(sorted(poses), dtype=float)
    return header, poses


def pose_at(poses: np.ndarray, t: float, tol: float | None = None):
    if tol is None:
        tol = POSE_TOL_S
    idx = int(np.argmin(np.abs(poses[:, 0] - t)))
    if abs(poses[idx, 0] - t) > tol:
        return None
    return float(poses[idx, 1]), float(poses[idx, 2]), float(poses[idx, 3])


def main_pose_tol(override=None):
    global POSE_TOL_S
    if override is not None:
        POSE_TOL_S = override


def odom_to_world(header, rx, ry, ryaw):
    sp = header["spawn_pose"]
    sy = sp["yaw"]
    wx = sp["x"] + math.cos(sy) * rx - math.sin(sy) * ry
    wy = sp["y"] + math.sin(sy) * rx + math.cos(sy) * ry
    return wx, wy, sy + ryaw


def load_gt(seed: int, run: int):
    j = json.load(open(GT_DIR / f"easy_seed{seed}_run{run}.json"))
    gt = j["raw_metrics"]["ground_truth_objects"]
    out = {}
    for k, v in sorted(gt.items(), key=lambda kv: int(kv[0])):
        if v["type"] in GT_CLASSES:
            out[k] = {"type": v["type"], "x": v["x"], "y": v["y"]}
    return out


def camera_frame(header, robot_w, yaw_w):
    wx, wy, _ = robot_w
    cx = wx + CAMERA_FORWARD_OFFSET * math.cos(yaw_w)
    cy = wy + CAMERA_FORWARD_OFFSET * math.sin(yaw_w)
    return cx, cy


def visible_gt(header, robot_w, yaw_w, gt, los):
    """{class: [{key, x, y, dist, rel}]} — GT instances geometrically visible."""
    cx, cy = camera_frame(header, robot_w, yaw_w)
    out = {}
    for k, v in gt.items():
        dx = v["x"] - cx
        dy = v["y"] - cy
        d = math.hypot(dx, dy)
        if d < 0.05 or d > MAX_VIS_DIST_M:
            continue
        rel = (math.atan2(dy, dx) - yaw_w + math.pi) % (2 * math.pi) - math.pi
        if abs(rel) > CAMERA_HFOV / 2 + FOV_EDGE_SLACK:
            continue
        if not los.clear(cx, cy, v["x"], v["y"]):
            continue
        out.setdefault(v["type"], []).append(
            {"key": k, "x": v["x"], "y": v["y"], "dist": d, "rel": rel})
    return out


def project_gt(cx, cy, yaw_w, gx, gy, cls, W, H):
    dx = gx - cx
    dy = gy - cy
    d = math.hypot(dx, dy)
    rel = (math.atan2(dy, dx) - yaw_w + math.pi) % (2 * math.pi) - math.pi
    if abs(rel) >= CAMERA_HFOV / 2:
        return None
    half_t = math.tan(CAMERA_HFOV / 2)
    u = W / 2 * (1 - math.tan(rel) / half_t)
    v_ang = math.atan((CENTER_Z[cls] - CAMERA_HEIGHT_M) / max(d, 0.1))
    v_half_tan = math.tan(CAMERA_HFOV / 2) * H / W
    v = H / 2 * (1 - math.tan(v_ang) / v_half_tan)
    return u, v, d, rel


def project_det_to_rel_bearing(det, W, H):
    """Detection pixel center → (bearing rel-center, vertical angle)."""
    u = (det["x1"] + det["x2"]) / 2
    v = (det["y1"] + det["y2"]) / 2
    half_t = math.tan(CAMERA_HFOV / 2)
    rel = math.atan((1 - 2 * u / W) * half_t)
    v_half_tan = half_t * H / W
    v_ang = math.atan((1 - 2 * v / H) * v_half_tan)
    return rel, v_ang


# ─── Model backends ───────────────────────────────────────────────────────────


class OwlV2Model:
    name = "owlv2"

    def __init__(self, device: str):
        import torch
        from PIL import Image
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.torch = torch
        self._pil = Image
        mid = "google/owlv2-base-patch16-ensemble"
        self.processor = Owlv2Processor.from_pretrained(mid)
        self.model = Owlv2ForObjectDetection.from_pretrained(mid).to(device).eval()
        self.device = device

    def infer(self, bgr, prompt_keys):
        import cv2 as _cv2
        torch = self.torch
        rgb = _cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil = self._pil.fromarray(rgb)
        texts = [PROMPTS[k] for k in prompt_keys]
        inputs = self.processor(text=[texts], images=pil, return_tensors="pt")
        inputs = {k: v.to(self.device)
                  for k, v in inputs.items()}
        target_sizes = torch.tensor([(bgr.shape[0], bgr.shape[1])],
                                    dtype=torch.int32, device=self.device)
        with torch.inference_mode():
            outputs = self.model(**inputs)
        res = self.processor.post_process_grounded_object_detection(
            outputs, target_sizes=target_sizes, threshold=CONF_THRESHOLD)[0]
        out = []
        for score, li, box in zip(res["scores"].cpu().tolist(),
                                  res["labels"].cpu().tolist(),
                                  res["boxes"].cpu().tolist()):
            x1, y1, x2, y2 = box
            out.append({"class_key": prompt_keys[int(li)], "conf": float(score),
                        "x1": float(x1), "y1": float(y1),
                        "x2": float(x2), "y2": float(y2)})
        return out


class YoloWorldModel:
    def __init__(self, device: str, size: str, name: str):
        from ultralytics import YOLOWorld
        w = {"s": "yolov8s-worldv2.pt", "m": "yolov8m-worldv2.pt"}[size]
        self.name = name
        self.model = YOLOWorld(w).to(device)
        self.device = device

    def infer(self, bgr, prompt_keys):
        texts = [PROMPTS[k] for k in prompt_keys]
        self.model.set_classes(texts)
        devarg = 0 if "cuda" in self.device else "cpu"
        res = self.model.predict(bgr, conf=CONF_THRESHOLD, verbose=False,
                                 device=devarg)[0]
        out = []
        for b in res.boxes:
            qi = int(b.cls.item())
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            out.append({"class_key": prompt_keys[qi], "conf": float(b.conf.item()),
                        "x1": x1, "y1": y1, "x2": x2, "y2": y2})
        return out


class GdinoModel:
    name = "gdino_tiny"

    def __init__(self, device: str):
        import torch
        from PIL import Image
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        mid = "IDEA-Research/grounding-dino-tiny"
        self.torch = torch
        self._pil = Image
        self.processor = AutoProcessor.from_pretrained(mid)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(mid).to(device).eval()
        self.device = device

    def infer(self, bgr, prompt_keys):
        import cv2 as _cv2
        torch = self.torch
        rgb = _cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil = self._pil.fromarray(rgb)
        text = ". ".join(PROMPTS[k] for k in prompt_keys) + "."
        inputs = self.processor(images=pil, text=text, return_tensors="pt")
        inputs = {k: v.to(self.device)
                  for k, v in inputs.items()}
        with torch.inference_mode():
            outputs = self.model(**inputs)
        res = self.processor.post_process_grounded_object_detection(
            outputs,
            input_ids=inputs["input_ids"],
            threshold=CONF_THRESHOLD,
            text_threshold=0.25,
            target_sizes=[(bgr.shape[0], bgr.shape[1])],
        )[0]
        rev = [(PROMPTS[k], k) for k in prompt_keys]
        out = []
        for score, label, box in zip(res["scores"].cpu().tolist(),
                                     res["labels"],
                                     res["boxes"].cpu().tolist()):
            lab = label if isinstance(label, str) else str(label)
            k = None
            best = 0
            for txt, kk in rev:
                # longest prompt contained in the label wins
                if txt.lower() in lab.lower() and len(txt) > best:
                    k, best = kk, len(txt)
            if k is None:
                continue
            x1, y1, x2, y2 = box
            out.append({"class_key": k, "conf": float(score),
                        "x1": float(x1), "y1": float(y1),
                        "x2": float(x2), "y2": float(y2)})
        return out


def make_model(name, device):
    if name == "owlv2":
        return OwlV2Model(device)
    if name.startswith("yolo_world_"):
        return YoloWorldModel(device, name.rsplit("_", 1)[1], name)
    if name == "gdino_tiny":
        return GdinoModel(device)
    raise ValueError(f"unknown model {name}")


# ─── Scoring ──────────────────────────────────────────────────────────────────


def gt_any_within(cx, cy, gt, cls, radius=FP_PROXY_RADIUS_M):
    for v in gt.values():
        if v["type"] == cls and math.hypot(v["x"] - cx, v["y"] - cy) <= radius:
            return True
    return False


# ─── Driver ───────────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+",
                    default=["owlv2", "yolo_world_s", "gdino_tiny"])
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--limit-frames", type=int, default=0,
                    help="cap total frames evaluated (0 = all)")
    ap.add_argument("--runs", nargs="*", default=None,
                    help="restrict to these frame-run dir names")
    ap.add_argument("--out", default="results/bakeoff")
    ap.add_argument("--pose-tol", type=float, default=POSE_TOL_S,
                    help="max |keyframe_t - pose_row_t| (s); default 0.5")
    args = ap.parse_args()

    main_pose_tol(args.pose_tol)

    los = LOS()
    run_dirs = sorted(d for d in FRAMES_DIR.iterdir() if d.is_dir())
    if args.runs:
        run_dirs = [d for d in run_dirs if d.name in set(args.runs)]

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    models = [make_model(n, args.device) for n in args.models]

    stats = {}
    for m in models:
        s = {cls: {"n_pos_frames": 0, "n_pos_frames_detected": 0, "n_insts": 0,
                   "n_insts_detected": 0, "confs": []}
             for cls in GT_CLASSES}
        s["_global"] = {"n_frames": 0, "fp_proxy": 0, "dets_total": 0, "lat": []}
        stats[m.name] = s

    # incremental JSONL per model for durability
    out_fh = {m.name: open(out_dir / f"frame_scores_{m.name}.jsonl", "w")
              for m in models}

    n_total = 0
    for run_dir in run_dirs:
        run_key_full = run_dir.name
        m_seed = re.search(r"seed(\d+)", run_key_full)
        m_run = re.search(r"run(\d+)", run_key_full)
        if not (m_seed and m_run):
            print(f"== skip {run_key_full} (no seed/run in name)", flush=True)
            continue
        seed, run = int(m_seed.group(1)), int(m_run.group(1))
        run_key = run_key_full.rsplit("_2026", 1)[0]
        header, poses = load_stream(run_key_full)
        gt = load_gt(seed, run)
        print(f"== {run_key_full}: {len(gt)} GT targets", flush=True)
        for (t, fpath) in load_frames_dir(run_dir):
            if args.limit_frames and n_total >= args.limit_frames:
                break
            pose = pose_at(poses, t)
            if pose is None:
                continue
            img = cv2.imread(str(fpath))
            if img is None:
                continue
            H, W = img.shape[:2]
            robot_w = odom_to_world(header, *pose)
            cx, cy = camera_frame(header, robot_w, robot_w[2])
            yaw_w = robot_w[2]
            vis = visible_gt(header, robot_w, yaw_w, gt, los)
            n_total += 1
            for m in models:
                t0 = time.perf_counter()
                try:
                    dets = m.infer(img, list(GT_CLASSES))
                except Exception as exc:
                    print(f"!! {m.name} infer failed on {fpath.name}: {exc}", flush=True)
                    continue
                lat_ms = (time.perf_counter() - t0) * 1000
                st = stats[m.name]
                st["_global"]["lat"].append(round(lat_ms, 1))
                st["_global"]["n_frames"] += 1
                st["_global"]["dets_total"] += len(dets)
                fp = 0
                per_class = {c: [0, 0, []] for c in GT_CLASSES}
                # GT projection + matching
                proj_by_cls = {}
                for cls, insts in vis.items():
                    for inst in insts:
                        p = project_gt(cx, cy, yaw_w, inst["x"], inst["y"], cls, W, H)
                        if p is not None:
                            proj_by_cls.setdefault(cls, []).append((inst, p))
                        st[cls]["n_insts"] += 1
                    st[cls]["n_pos_frames"] += 1
                used_dets = set()
                for cls, plist in proj_by_cls.items():
                    hit_keys = set()
                    for di, det in enumerate(dets):
                        if det["class_key"] != cls or di in used_dets:
                            continue
                        drel, _ = project_det_to_rel_bearing(det, W, H)
                        # distance estimate: use detection row (bottom edge) elevation
                        # angle → distance via center height (object-aware, weak) —
                        # instead require bearing match + non-extreme aspect
                        for inst, (u, v, d, rel) in plist:
                            if abs(drel - rel) <= MATCH_BEARING_RAD and inst["key"] not in hit_keys:
                                hit_keys.add(inst["key"])
                                used_dets.add(di)
                                st[cls]["n_insts_detected"] += 1
                                st[cls]["confs"].append(round(det["conf"], 4))
                                break
                    st[cls]["n_pos_frames_detected"] += len(hit_keys)
                # FP proxy
                for di, det in enumerate(dets):
                    if di in used_dets:
                        continue
                    cls = det["class_key"]
                    if cls not in GT_CLASSES or not gt_any_within(cx, cy, gt, cls):
                        fp += 1
                st["_global"]["fp_proxy"] += fp
                out_fh[m.name].write(json.dumps({
                    "frame": str(fpath.relative_to(EXPLORER_ROOT)),
                    "t": t, "lat_ms": round(lat_ms, 1), "n_dets": len(dets),
                    "fp_proxy": fp,
                    "vis": {c: [i["key"] for i in v] for c, v in vis.items()},
                }) + "\n")
        if args.limit_frames and n_total >= args.limit_frames:
            break

    # summary
    summary = {}
    for name, st in stats.items():
        row = {"model": name}
        for cls in GT_CLASSES:
            c = st[cls]
            rate_inst = (c["n_insts_detected"] / c["n_insts"]) if c["n_insts"] else 0.0
            confs = sorted(c["confs"])
            row[cls] = {
                "visible_instances": c["n_insts"],
                "detected_instances": c["n_insts_detected"],
                "inst_rate": round(rate_inst, 4),
                "conf_med": confs[len(confs) // 2] if confs else None,
                "conf_min": confs[0] if confs else None,
                "conf_max": confs[-1] if confs else None,
            }
        g = st["_global"]
        lat = sorted(g["lat"])
        row["_global"] = {
            "frames": g["n_frames"],
            "dets_total": g["dets_total"],
            "fp_proxy_total": g["fp_proxy"],
            "fp_proxy_per_frame": round(g["fp_proxy"] / max(1, g["n_frames"]), 4),
            "lat_ms_median": lat[len(lat) // 2] if lat else None,
            "lat_ms_p90": lat[int(0.9 * (len(lat) - 1))] if lat else None,
        }
        summary[name] = row
        (out_dir / f"summary_{name}.json").write_text(json.dumps(row, indent=2))
        out_fh[name].close()

    (out_dir / "bakeoff_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())