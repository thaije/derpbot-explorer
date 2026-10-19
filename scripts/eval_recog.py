#!/usr/bin/env python3
"""Recognition eval harness over the labels in a recog_benchmark manifest (t_fe1459a9).

Generalizes scripts/bakeoff_run.py: any prompt-driven open-vocab detector that
implements init(prompts: list[str]) / infer(bgr) -> list[dict] can be dropped
in and scored against the benchmark split. The sim GT (visible-instance labels,
pixel bboxes) is precomputed in the manifest; the harness never opens the sim.

Per model × class it reports:
  - inst_rate and PR curve (conf-threshold sweep at the manifest conf grid),
  - rate@conf 0.15/0.30/0.50, precision@0.15,
  - FP-proxy/frame (detection with no matchable same-class GT in frame),
  - latency/frame.
Matching = per-frame class-wise greedy IoU matching (greedy first-fit on
detection↔GT instance, IoU ≥ MATCH_IOU), plus a bearing fallback identical to
bakeoff_run.py (|Δbearing| ≤ MATCH_BEARING_RAD) for sub-pixel-ish objects.
Bar logic (detection@conf, FP cap, class mix) is configurable via --bar.

Backends live in scripts/detector_backends_owlv2-style modules or are selected
by --model {owlv2,yolo_world_s,gdino_tiny} (same weights as the live pipeline).

Run from the explorer repo root:
    .venv/bin/python scripts/eval_recog.py --model owlv2 \
        --manifest results/recog_benchmark/manifest.jsonl --split corpus \
        --out results/eval_recog/corpus_owlv2
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time
from collections import defaultdict
from pathlib import Path

EXPLORER_ROOT = Path("/home/plip/Projects/derpbot-explorer")
sys.path.insert(0, str(EXPLORER_ROOT / "scripts"))

import numpy as np  # noqa: E402

MATCH_IOU = 0.10                 # detection↔GT-instance IoU gate (per frame)
MATCH_BEARING_RAD = 0.20         # fallback (bakeoff rule): |Δbearing| ≤ 0.20
FP_RADIUS_M = 2.0                # gt_within_radius FP mode (bakeoff rule)
CONF_POINTS = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50)
RATE_CONFS = (0.15, 0.30, 0.50)
# gt_within_radius FP mode: per-run full GT result JSONs. v10-assoc covers the
# v1 corpus runs (seeds 1–5); v2holdout covers the Phase-2 holdout runs (seeds
# 6–10); stream-corpus covers the probe runs. Non-easy runs (perception_*
# stress) have no result JSONs — their GT lives in the benchmark's generated_gt
# dir (deterministic WorldGenerator replay, ≤0.0004 m vs in-sim SDF).
GT_DIRS = [
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v10-assoc"),
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-stream-corpus"),
    Path("/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v2holdout"),
]
# bar presets (--bar): name → (conf, rate floor, FP-proxy cap). The swap bar of
# t_b4361707 is 'bakeoff'; 'none' disables bar evaluation entirely.
BAR_PRESETS = {
    "bakeoff": {"confs": RATE_CONFS, "min_rate_fak": 1.5,   # × baseline FAK
                "min_rate_other": 1.0, "max_fp_ratio": 1.0,
                "baseline": None},   # baseline rates read from comparison JSON
    "none": None,
}


def run_key_seed(run_key: str):
    m = re.search(r"seed(\d+)", run_key)
    return int(m.group(1)) if m else None


def run_key_run(run_key: str):
    m = re.search(r"seed\d+_run(\d+)", run_key)
    return int(m.group(1)) if m else None


# ─── Backends (thin wrappers; identical weights/config to bakeoff_run.py) ────

class OwlV2Backend:
    name = "owlv2"

    def __init__(self, device: str, model_id: str | None = None):
        import torch
        from PIL import Image
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.torch = torch
        self._pil = Image
        mid = "google/owlv2-base-patch16-ensemble"
        if model_id:
            mid = model_id
            self.name = "owlv2:" + model_id.split("/")[-1].replace("-ensemble", "_ens")
        self.processor = Owlv2Processor.from_pretrained(mid)
        self.model = Owlv2ForObjectDetection.from_pretrained(mid).to(device).eval()
        self.device = device
        self.prompts: list[str] = []

    def init(self, prompts: list[str]):
        self.prompts = list(prompts)

    def infer(self, bgr) -> list[dict]:
        import cv2
        torch = self.torch
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil = self._pil.fromarray(rgb)
        inputs = self.processor(text=[self.prompts], images=pil,
                                return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        target_sizes = torch.tensor([(bgr.shape[0], bgr.shape[1])],
                                    dtype=torch.int32, device=self.device)
        with torch.inference_mode():
            outputs = self.model(**inputs)
        res = self.processor.post_process_grounded_object_detection(
            outputs, target_sizes=target_sizes, threshold=0.02)[0]
        out = []
        for score, li, box in zip(res["scores"].cpu().tolist(),
                                  res["labels"].cpu().tolist(),
                                  res["boxes"].cpu().tolist()):
            x1, y1, x2, y2 = box
            out.append({"class": self.prompts[int(li)], "conf": float(score),
                        "bbox": (float(x1), float(y1), float(x2), float(y2))})
        return out


class YoloWorldBackend:
    def __init__(self, device: str, size: str, name: str):
        from ultralytics import YOLOWorld
        w = {"s": "yolov8s-worldv2.pt", "m": "yolov8m-worldv2.pt",
             "l": "yolov8l-worldv2.pt"}[size]
        self.name = name
        self.model = YOLOWorld(w).to(device)
        self.device = device
        self.prompts = []

    def init(self, prompts: list[str]):
        self.prompts = list(prompts)
        self.model.set_classes(self.prompts)

    def infer(self, bgr) -> list[dict]:
        devarg = 0 if "cuda" in self.device else "cpu"
        res = self.model.predict(bgr, conf=0.02, verbose=False,
                                 device=devarg)[0]
        out = []
        for b in res.boxes:
            qi = int(b.cls.item())
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            out.append({"class": self.prompts[qi], "conf": float(b.conf.item()),
                        "bbox": (x1, y1, x2, y2)})
        return out


class GdinoBackend:
    def __init__(self, device: str, model_id: str | None = None):
        import torch
        from PIL import Image
        from transformers import (AutoModelForZeroShotObjectDetection,
                                  AutoProcessor)
        mid = model_id or "IDEA-Research/grounding-dino-tiny"
        self.name = "gdino_base" if "base" in mid else "gdino_tiny"
        self.torch = torch
        self._pil = Image
        self.processor = AutoProcessor.from_pretrained(mid)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
            mid).to(device).eval()
        self.device = device
        self.prompts = []

    def init(self, prompts: list[str]):
        self.prompts = list(prompts)

    def infer(self, bgr) -> list[dict]:
        import cv2
        torch = self.torch
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil = self._pil.fromarray(rgb)
        text = ". ".join(self.prompts) + "."
        inputs = self.processor(images=pil, text=text, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.inference_mode():
            outputs = self.model(**inputs)
        res = self.processor.post_process_grounded_object_detection(
            outputs, input_ids=inputs["input_ids"], threshold=0.02,
            text_threshold=0.25,
            target_sizes=[(bgr.shape[0], bgr.shape[1])])[0]
        out = []
        for score, label, box in zip(res["scores"].cpu().tolist(),
                                     res["labels"],
                                     res["boxes"].cpu().tolist()):
            lab = label if isinstance(label, str) else str(label)
            best, k = 0, None
            for p in self.prompts:
                if p.lower() in lab.lower() and len(p) > best:
                    k, best = p, len(p)
            if k is None:
                continue
            x1, y1, x2, y2 = box
            out.append({"class": k, "conf": float(score),
                        "bbox": (float(x1), float(y1), float(x2), float(y2))})
        return out


def make_backend(name: str, device: str):
    if name.startswith("mm_gdino:"):
        # MM-Grounding-DINO (HF task 7): openmmlab-community/mm_grounding_dino_*
        # checkpoints via the built-in MMGroundingDinoForObjectDetection class
        # (transformers >= 4.44) — same interface as GdinoBackend.
        mid = name.split(":", 1)[1]
        b = GdinoBackend(device, model_id=mid)
        b.name = "mmgdino:" + mid.split("/")[-1]
        return b
    if name == "owlv2":
        return OwlV2Backend(device)
    if name.startswith("owlv2:"):
        # explicit HF checkpoint override, e.g. owlv2:google/owlv2-large-patch14-ensemble
        return OwlV2Backend(device, model_id=name.split(":", 1)[1])
    if name.startswith("yolo_world_"):
        return YoloWorldBackend(device, name.rsplit("_", 1)[1], name)
    if name == "gdino_tiny":
        return GdinoBackend(device)
    if name == "gdino_base":
        return GdinoBackend(device, model_id="IDEA-Research/grounding-dino-base")
    if name.startswith("florence2"):
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "florence2_backend",
            str(EXPLORER_ROOT / "scripts" / "florence2_backend.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        size = name.rsplit("_", 1)[1] if name.startswith("florence2_") \
            else "large"
        return mod.Florence2Backend(device, size=size)
    raise ValueError(f"unknown model {name}")


# ─── Matching ────────────────────────────────────────────────────────────────


def iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / union if union > 0 else 0.0


def gt_bearing(man_obj, yaw_w) -> float:
    return float(man_obj["bearing"])


def det_bearing(det, W, H, hfov=math.pi / 2) -> float:
    u = (det["bbox"][0] + det["bbox"][2]) / 2
    half = math.tan(hfov / 2)
    return math.atan((1 - 2 * u / W) * half)


def match_frame(det_rows, gt_rows, W, H, iou_gate, bear_gate):
    """Greedy IoU→bearing matching per class. Returns matched (di, gi) pairs."""
    pairs = []
    used_d = set()
    for gi, g in enumerate(gt_rows):
        for di, d in enumerate(det_rows):
            if di in used_d:
                continue
            if iou(d["bbox"], tuple(g["bbox"])) >= iou_gate:
                pairs.append((di, gi, iou(d["bbox"], tuple(g["bbox"]))))
                used_d.add(di)
                break
    # bearing fallback for unmatched GT (small/edge objects)
    for gi, g in enumerate(gt_rows):
        if any(gi == pg for _, pg, _ in pairs):
            continue
        for di, d in enumerate(det_rows):
            if di in used_d:
                continue
            db = det_bearing(d, W, H)
            gb = float(g["bearing"])
            if abs(db - gb) <= bear_gate:
                pairs.append((di, gi, -1.0))
                used_d.add(di)
                break
    return pairs


# ─── Driver ──────────────────────────────────────────────────────────────────


def load_manifest_rows(manifest: Path, split: str | None, runs: list[str] | None):
    rows = []
    for line in manifest.read_text().splitlines():
        r = json.loads(line)
        if split and r["split"] != split:
            continue
        if runs and r["run"] not in set(runs):
            continue
        rows.append(r)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="owlv2")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--manifest", default="results/recog_benchmark/manifest.jsonl")
    ap.add_argument("--split", default=None, choices=[None, "corpus", "holdout", "stress"])
    ap.add_argument("--runs", nargs="*", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--iou", type=float, default=MATCH_IOU)
    ap.add_argument("--bearing", type=float, default=MATCH_BEARING_RAD)
    ap.add_argument("--limit-frames", type=int, default=0)
    ap.add_argument("--conf-points", nargs="*", type=float, default=list(CONF_POINTS))
    ap.add_argument("--classes", nargs="*", default=None,
                    help="restrict scoring to these classes (fewer prompts = "
                         "less open-vocab attention dilution; default all)")
    ap.add_argument("--fp-mode", default="unmatched",
                    choices=["unmatched", "gt_within_radius"],
                    help="FP-proxy definition (default: unmatched; "
                         "gt_within_radius = bake-off-compatible)")
    ap.add_argument("--fp-conf", type=float, default=0.15,
                    help="confidence threshold for the FP-proxy (default "
                         "0.15 = the bake-off operating point; review r2 "
                         "item 1 — the FP-proxy conf must match the "
                         "baseline's when compared via --bar)")
    ap.add_argument("--gt-json-dir", default=None,
                    help="dir of per-run result JSONs (for gt_within_radius "
                         "FP mode; default auto-locate)")
    ap.add_argument("--bar", default="none",
                    choices=sorted(BAR_PRESETS),
                    help="bar preset for pass/fail evaluation (default "
                         "'none'); 'bakeoff' = t_b4361707 swap bar, needs "
                         "--bar-baseline summary.json")
    ap.add_argument("--bar-baseline", default=None,
                    help="summary.json of the baseline run the bar compares "
                         "against (required with --bar bakeoff)")
    args = ap.parse_args()

    import cv2
    rows = load_manifest_rows(EXPLORER_ROOT / args.manifest, args.split, args.runs)
    if not rows:
        print("no frames matched the split/run filter")
        return 1
    if args.limit_frames:
        rows = rows[:args.limit_frames]

    backend = make_backend(args.model, args.device)
    # prompt set: from the manifest's own prompts map if present
    prompt_map = {}
    stats_path = (EXPLORER_ROOT / args.manifest).parent / "dataset_stats.json"
    if stats_path.exists():
        prompt_map = json.load(open(stats_path)).get("prompts", {})
    classes = sorted({o["cls"] for r in rows for o in r["objects"]} | {c for c in prompt_map})
    if args.classes:
        classes = [c for c in classes if c in set(args.classes)]
    prompts = [prompt_map.get(c, c.replace("_", " ")) for c in classes]
    print(f"classes: {list(zip(classes, prompts))}", flush=True)
    try:
        backend.init(prompts, cls_names=classes)
    except TypeError:
        backend.init(prompts)
    cls_of_prompt = dict(zip(prompts, classes))

    conf_points = sorted(args.conf_points)
    per_cls = {c: {"tp": {conf: 0 for conf in conf_points}, "n_gt": 0,
                   "confs": [], "fp15": 0}
               for c in classes}
    fp_at_min = 0
    fp_conf = args.fp_conf
    lat, n_frames = [], 0
    # Batch backends (florence2: pipe-stdin interacts badly with its
    # transformers-4.49 worker) prefetch in blocks via infer_batch().
    batch_backend = hasattr(backend, "infer_batch")
    batch_size = int(os.environ.get("EVAL_RECOG_BATCH", "64")) \
        if batch_backend else 0
    out_path = Path(args.out) if args.out else (
        EXPLORER_ROOT / "results" / "eval_recog" /
        f"{(args.split or 'all')}_{args.model}")
    out_path.mkdir(parents=True, exist_ok=True)
    fh = open(out_path / "frame_records.jsonl", "w")

    # per-run full GT (all classes) for the gt_within_radius FP mode — from
    # the same result JSONs the builder used (seed/run indexed). Non-easy
    # runs fall back to the benchmark's generated_gt dir.
    generated_gt_dir = (EXPLORER_ROOT / args.manifest).parent / "generated_gt"
    gt_all_cache = {}
    gt_dirs = ([Path(args.gt_json_dir)] if args.gt_json_dir else GT_DIRS)

    def gt_all_for(run: str):
        if run in gt_all_cache:
            return gt_all_cache[run]
        m_s = run_key_seed(run)
        m_r = run_key_run(run)
        out = {}
        if "perception_stress" in run:
            p = generated_gt_dir / f"{run}.json"
            if p.exists():
                out = defaultdict(list)
                for v in json.load(open(p)).values():
                    out[v["type"]].append({"x": v["x"], "y": v["y"]})
        elif m_s is not None and m_r is not None:
            for d in gt_dirs:
                p = d / f"easy_seed{m_s}_run{m_r}.json"
                if p.exists():
                    raw = json.load(open(p))["raw_metrics"]["ground_truth_objects"]
                    out = defaultdict(list)
                    for v in raw.values():
                        out[v["type"]].append({"x": v["x"], "y": v["y"]})
                    break
        gt_all_cache[run] = out
        return out

    def score_frame(r, img, dets, gt_by_cls):
        """Shared scoring body — one frame's detections against GT.
        Mutates per_cls / fp_at_min (via nonlocal) and appends a record."""
        nonlocal fp_at_min
        H, W = img.shape[:2]
        det_by_cls = defaultdict(list)
        for d in dets:
            det_by_cls[d["class"]].append(d)
        # score per class at each conf point. Per threshold: filter dets to
        # conf ≥ θ, sort conf-desc, then greedy — mirrors a detector DEPLOYED
        # at θ (a low-conf det must never squat a GT and block the high-conf
        # one, which a single greedy on the unfiltered list would allow).
        for c in classes:
            gts = gt_by_cls.get(c, [])
            s = per_cls[c]
            s["n_gt"] += len(gts)
            for conf in conf_points:
                dets_keep = sorted(
                    (d for d in det_by_cls.get(c, []) if d["conf"] >= conf),
                    key=lambda d: -d["conf"])
                pairs = match_frame(dets_keep, gts, W, H, args.iou, args.bearing)
                s["tp"][conf] += len(pairs)
                if conf == 0.15:
                    s["fp15"] += max(0, len(dets_keep) - len(pairs))
            # matched-conf distribution at the loosest threshold
            dets_all = sorted(det_by_cls.get(c, []), key=lambda d: -d["conf"])
            pairs_lo = match_frame(dets_all, gts, W, H, args.iou, args.bearing)
            for di, _, _ in pairs_lo:
                s["confs"].append(round(dets_all[di]["conf"], 4))
        # FP proxy at --fp-conf (default 0.15 = the bake-off operating point).
        # Two modes (--fp-mode):
        #   unmatched        — det matches no in-frame GT instance (strict);
        #   gt_within_radius — bake-off-compatible: det whose class has NO GT
        #                      instance within FP_RADIUS_M of the camera
        #                      (lenient: in-frame-but-unmatched dets near a
        #                      same-class GT don't count).
        # NOTE the FP conf is NOT tied to conf_points: an FP proxy read at
        # conf ≥ 0.05 vs a baseline computed at 0.15 is a ~3× handicap
        # (review r2 item 1) — --bar rejects mismatched thresholds.
        fp_frame = 0
        matched_pairs = {}
        for c, dets_c in det_by_cls.items():
            gts = gt_by_cls.get(c, [])
            d_keep = sorted((d for d in dets_c if d["conf"] >= fp_conf),
                            key=lambda d: -d["conf"])
            pairs = match_frame(d_keep, gts, W, H, args.iou, args.bearing)
            matched_idx = {di for di, _, _ in pairs}
            matched_pairs[c] = [[di, gi, round(iv, 3)] for di, gi, iv in pairs]
            if args.fp_mode == "gt_within_radius":
                gt_all = gt_all_for(r["run"])
                near = [g for g in gt_all.get(c, [])
                        if math.hypot(g["x"] - r["robot_xy"][0],
                                      g["y"] - r["robot_xy"][1]) <= FP_RADIUS_M]
                if not near:
                    fp_frame += sum(1 for di in range(len(d_keep))
                                    if di not in matched_idx)
            else:
                fp_frame += sum(1 for di in range(len(d_keep))
                                if di not in matched_idx)
        fp_at_min += fp_frame
        fh.write(json.dumps({
            "frame_id": r["frame_id"], "lat_ms": round(lat[-1], 1),
            "n_gt": {c: len(v) for c, v in gt_by_cls.items()},
            "dets": [{**{k: d[k] for k in ("class", "conf")},
                      "bbox": [round(x, 1) for x in d["bbox"]]} for d in dets],
            "matched": matched_pairs,
            "fp": fp_frame,
        }) + "\n")

    if batch_backend:
        # Batch path: preload frames + GT in blocks, one worker pass per
        # block (pipe-stdin workers hang — see florence2_batch.py).
        def frames_with_gt(block_rows):
            out = []
            for r in block_rows:
                img = cv2.imread(str(EXPLORER_ROOT / r["image"]))
                if img is None:
                    continue
                gt_by_cls = defaultdict(list)
                for o in r["objects"]:
                    gt_by_cls[o["cls"]].append(o)
                out.append((r, img, gt_by_cls))
            return out

        for i in range(0, len(rows), batch_size):
            block = frames_with_gt(rows[i:i + batch_size])
            results = backend.infer_batch([
                (r["frame_id"], img) for r, img, _ in block])
            for r, img, gt_by_cls in block:
                H, W = img.shape[:2]
                reply = results.get(r["frame_id"], [])
                if reply and isinstance(reply[0], dict) \
                        and "error" in reply[0]:
                    print(f"WARN frame {r['frame_id']}: {reply[0]['error']}",
                          flush=True)
                    reply = []
                lat.append(float(backend.wall_ms or 0))
                n_frames += 1
                dets = [{**d, "class": cls_of_prompt.get(
                    d["class"], d["class"])} for d in reply]
                score_frame(r, img, dets, gt_by_cls)
            print(f"  {n_frames}/{len(rows)} frames (batch)", flush=True)
        fh.close()
    else:
        for r in rows:
            img = cv2.imread(str(EXPLORER_ROOT / r["image"]))
            if img is None:
                continue
            H, W = img.shape[:2]
            gt_by_cls = defaultdict(list)
            for o in r["objects"]:
                gt_by_cls[o["cls"]].append(o)
            t0 = time.perf_counter()
            dets = backend.infer(img)
            lat.append((time.perf_counter() - t0) * 1000)
            n_frames += 1
            dets = [{**d, "class": cls_of_prompt.get(d["class"], d["class"])}
                    for d in dets]
            score_frame(r, img, dets, gt_by_cls)
            if n_frames % 50 == 0:
                print(f"  {n_frames}/{len(rows)} frames", flush=True)
        fh.close()

    # aggregate
    summary = {"model": backend.name, "split": args.split or "all",
               "frames": n_frames,
               "iou_gate": args.iou, "bearing_gate": args.bearing,
               "fp_mode": args.fp_mode, "fp_conf": fp_conf,
               "conf_points": conf_points, "classes": {}}
    for c in classes:
        s = per_cls[c]
        cls_sum = {"n_gt": s["n_gt"]}
        for conf in conf_points:
            cls_sum[f"rate@{conf:.2f}"] = round(s["tp"][conf] / s["n_gt"], 4) \
                if s["n_gt"] else None
        tp15 = s["tp"].get(0.15, 0)
        cls_sum["precision@0.15"] = round(tp15 / (tp15 + s["fp15"]), 4) \
            if (tp15 + s["fp15"]) else None
        confs = sorted(s["confs"])
        if confs:
            cls_sum["conf_med"] = confs[len(confs) // 2]
            cls_sum["conf_p90"] = confs[int(0.9 * (len(confs) - 1))]
        else:
            cls_sum["conf_med"] = cls_sum["conf_p90"] = None
        summary["classes"][c] = cls_sum
    summary["_global"] = {
        "fp_proxy_per_frame": round(fp_at_min / max(1, n_frames), 4),
        "fp_proxy_min_conf": fp_conf,
        "lat_ms_median": sorted(lat)[len(lat) // 2] if lat else None,
        "lat_ms_p90": sorted(lat)[int(0.9 * (len(lat) - 1))] if lat else None,
    }

    # ── Bar evaluation (--bar; default none) ─────────────────────────────────
    # 'bakeoff' = the t_b4361707 swap bar pre-registered there: candidate vs
    # baseline (its summary.json via --bar-baseline) at confs 0.15/0.30/0.50:
    #   FAK rate ≥ 1.5× baseline AND (FE+person) rate ≥ 1.0× baseline AND
    #   FP-proxy/frame ≤ baseline × 1.0.
    if args.bar == "bakeoff":
        if not args.bar_baseline:
            print("--bar bakeoff requires --bar-baseline (baseline summary.json)")
            return 1
        base = json.loads(Path(args.bar_baseline).read_text())
        bc = (base.get("classes") or {})
        g = summary["_global"]

        def rate(m, c, conf):
            return ((m.get("classes") or {}).get(c) or {}).get(f"rate@{conf:.2f}")

        b_fp = ((base.get("_global") or {}).get("fp_proxy_per_frame"))
        b_fp_conf = (base.get("_global") or {}).get(
            "fp_proxy_min_conf", (base.get("_global") or {}).get("fp_conf"))
        # review r2 item 1: an FP-proxy compared at different confidence
        # thresholds is unsound (FP@0.05 vs FP@0.15 is a ~3× handicap) —
        # record and FLAG the mismatch rather than silently fail.
        fp_conf_mismatch = (b_fp_conf is not None
                            and abs(float(b_fp_conf) - fp_conf) > 1e-9)
        bar_eval = {
            "preset": "bakeoff",
            "baseline_run": str(args.bar_baseline),
            "same_fp_mode": base.get("fp_mode") == args.fp_mode,
            "fp_conf": fp_conf,
            "baseline_fp_conf": b_fp_conf,
            "fp_conf_mismatch": bool(fp_conf_mismatch),
            "per_conf": {},
        }
        if fp_conf_mismatch:
            print(f"WARNING --bar bakeoff: baseline FP-proxy computed at conf "
                  f">= {b_fp_conf} but this run used --fp-conf {fp_conf}; the "
                  f"FP comparison is UNSOUND (re-run with matching --fp-conf).")
        checks = []
        for conf in RATE_CONFS:
            fak = rate(summary, "first_aid_kit", conf)
            fak_b = rate(base, "first_aid_kit", conf)
            fe = min(rate(summary, "fire_extinguisher", conf) or 0,
                     rate(summary, "person", conf) or 0)
            fe_b = min(rate(base, "fire_extinguisher", conf) or 0,
                       rate(base, "person", conf) or 0)
            fp = summary["_global"]["fp_proxy_per_frame"]
            row = {
                "fak": fak, "fak_baseline": fak_b,
                "fak_ratio": round(fak / fak_b, 3) if fak and fak_b else None,
                "min_fe_person": fe, "min_fe_person_baseline": fe_b,
                "fp_proxy": fp, "fp_proxy_baseline": b_fp,
            }
            ok = (
                fak is not None and fak_b and fak >= 1.5 * fak_b
                and fe >= 1.0 * fe_b
                and b_fp is not None and fp <= b_fp
            )
            row["pass"] = bool(ok)
            checks.append(row)
            bar_eval["per_conf"][f"{conf:.2f}"] = row
        bar_eval["pass_any_conf"] = any(r["pass"] for r in checks)
        summary["bar"] = bar_eval
    elif args.bar != "none":
        summary["bar"] = {"preset": args.bar, "note": "preset stub — no rules"}

    (out_path / "summary.json").write_text(json.dumps(summary, indent=2))

    # markdown table
    md = [f"# eval_recog — {backend.name} on `{args.split or 'all'}`",
          "",
          f"frames {n_frames}, matching IoU ≥ {args.iou} (bearing ≤ {args.bearing} rad fallback), "
          f"FP-proxy @ conf ≥ {fp_conf} ({args.fp_mode})",
          "",
          "| class | n_gt | " + " | ".join(f"rate@{c:.2f}" for c in conf_points) +
          " | precision@0.15 | conf med |",
          "|---|" + "---|" * (2 + len(conf_points) + 2)]
    for c in classes:
        s = summary["classes"][c]
        cells = [f"{s[f'rate@{cf:.2f}']}" if s[f"rate@{cf:.2f}"] is not None
                 else "—" for cf in conf_points]
        prec = s["precision@0.15"]
        md.append(f"| {c} | {s['n_gt']} | " + " | ".join(cells) +
                  f" | {prec if prec is not None else '—'} | {s['conf_med']} |")
    g = summary["_global"]
    md += ["", f"FP-proxy/frame: **{g['fp_proxy_per_frame']}** ({args.fp_mode}, "
               f"conf ≥ {fp_conf}); latency med "
               f"{g['lat_ms_median']} ms (p90 {g['lat_ms_p90']})", ""]
    if "bar" in summary:
        b = summary["bar"]
        md += [f"## bar: {b['preset']} — {'PASS' if b.get('pass_any_conf') else 'FAIL'}", ""]
        for conf, row in b.get("per_conf", {}).items():
            md.append(f"- conf {conf}: FAK {row['fak']} vs base {row['fak_baseline']} "
                      f"(×{row['fak_ratio']}); min(FE,person) {row['min_fe_person']} "
                      f"vs {row['min_fe_person_baseline']}; FP {row['fp_proxy']} "
                      f"vs {row['fp_proxy_baseline']} → {'PASS' if row['pass'] else 'FAIL'}")
        md.append("")
    (out_path / "README.md").write_text("\n".join(md))
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())