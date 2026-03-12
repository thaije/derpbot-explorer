#!/usr/bin/env python3
"""
Standalone detector debug script.

Grabs one frame from /derpbot_0/rgbd/image, saves it, then runs YOLOE/YOLO-World
inference inline (no subprocess) with various class prompts and thresholds.

Usage (with scenario running):
    cd /home/plip/Projects/derpbot-explorer
    source /opt/ros/jazzy/setup.bash
    .venv/bin/python agent/test_detector.py
"""
from __future__ import annotations

import sys
import os
import time

import cv2
import numpy as np

# ── 1. Grab a frame from ROS ──────────────────────────────────────────────────

def grab_frame_from_ros(topic="/derpbot_0/rgbd/image", timeout=15.0):
    """Subscribe once, grab one BGR frame, return numpy array."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rclpy.init()
    node = rclpy.create_node("detector_test")
    bridge = CvBridge()
    frame_holder = [None]

    qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    def cb(msg):
        if frame_holder[0] is None:
            frame_holder[0] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    node.create_subscription(Image, topic, cb, qos)

    print(f"Waiting for frame on {topic} (up to {timeout}s)...")
    deadline = time.monotonic() + timeout
    while frame_holder[0] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()

    if frame_holder[0] is None:
        raise RuntimeError(f"No frame received on {topic} within {timeout}s")
    return frame_holder[0]


# ── 2. Run inference with various prompts ─────────────────────────────────────

def run_inference(frame: np.ndarray, classes: list[str], conf_thresh: float, model):
    """Run model.predict and return list of (class_name, conf, cx, cy, w, h)."""
    model.set_classes(classes)
    results = model.predict(source=frame, conf=conf_thresh, verbose=False)
    dets = []
    for r in results:
        if r.boxes is None:
            continue
        for i in range(len(r.boxes)):
            conf = float(r.boxes.conf[i])
            cls_id = int(r.boxes.cls[i])
            name = classes[cls_id] if cls_id < len(classes) else str(cls_id)
            xywh = r.boxes.xywh[i].tolist()
            dets.append((name, conf, xywh[0], xywh[1], xywh[2], xywh[3]))
    return dets


def draw_detections(frame, dets, label):
    vis = frame.copy()
    for name, conf, cx, cy, w, h in dets:
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(vis, f"{name} {conf:.2f}", (x1, max(y1 - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return vis


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # ── grab frame ────────────────────────────────────────────────────────────
    saved_frame = os.path.join(script_dir, "debug_frame.jpg")
    if os.path.exists(saved_frame) and "--no-grab" not in sys.argv:
        print(f"Re-using existing frame: {saved_frame}  (pass --regrab to overwrite)")
    if not os.path.exists(saved_frame) or "--regrab" in sys.argv:
        frame = grab_frame_from_ros()
        cv2.imwrite(saved_frame, frame)
        print(f"Saved frame → {saved_frame}")
    frame = cv2.imread(saved_frame)
    print(f"Frame shape: {frame.shape}")

    # ── load model ───────────────────────────────────────────────────────────
    sys.path.insert(0, script_dir)
    models_dir = os.path.join(os.path.dirname(script_dir), "models")
    # ultralytics looks for mobileclip_blt.ts in CWD
    os.chdir(models_dir)
    weights_path = os.path.join(models_dir, "yoloe-11s-seg.pt")
    fallback_weights = os.path.join(models_dir, "yolov8s-worldv2.pt")

    from ultralytics import YOLO

    model = None
    model_name = None
    for weights, name in [(weights_path, "yoloe-11s-seg"), (fallback_weights, "yolo-world-s")]:
        try:
            print(f"\nLoading {name} from {weights}...")
            model = YOLO(weights)
            # quick set_classes test
            model.set_classes(["fire extinguisher"])
            model.to("cuda:0")
            # warmup
            model.predict(source=np.zeros((480, 640, 3), dtype=np.uint8), conf=0.99, verbose=False)
            model_name = name
            print(f"  ✓ loaded {name}")
            break
        except Exception as e:
            print(f"  ✗ {name} failed: {e}")
            model = None

    if model is None:
        print("ERROR: could not load any model")
        sys.exit(1)

    # ── test matrix ──────────────────────────────────────────────────────────
    # Various prompt variations × thresholds
    class_sets = [
        ["fire extinguisher", "first aid kit"],
        ["fire_extinguisher", "first_aid_kit"],
        ["red fire extinguisher", "red first aid kit"],
        ["extinguisher", "first aid"],
        ["fire extinguisher", "first aid kit", "hazard sign", "person", "box"],
        # Sim-appearance prompts (the objects are low-poly)
        ["red cylinder", "white box with red cross"],
        ["red bottle", "medical kit"],
        ["fire extinguisher cylinder", "first aid box"],
        # Very broad
        ["object", "item", "thing"],
    ]
    thresholds = [0.05, 0.10, 0.15, 0.20, 0.30]

    print(f"\n{'='*70}")
    print(f"Model: {model_name}   Frame: {frame.shape}")
    print(f"{'='*70}")

    best_dets = []
    best_label = ""

    for classes in class_sets:
        print(f"\nClasses: {classes}")
        for thresh in thresholds:
            dets = run_inference(frame, classes, thresh, model)
            if dets:
                print(f"  conf≥{thresh:.2f}  → {len(dets)} detections:")
                for d in dets:
                    print(f"    {d[0]:30s}  conf={d[1]:.3f}  bbox=({d[2]:.0f},{d[3]:.0f} {d[4]:.0f}×{d[5]:.0f})")
                if len(dets) > len(best_dets):
                    best_dets = dets
                    best_label = f"{classes} @ conf≥{thresh}"
            else:
                print(f"  conf≥{thresh:.2f}  → (no detections)")

    # ── also try raw predict with no set_classes (if supported) ──────────────
    print(f"\n{'='*70}")
    print("Raw predict (no set_classes, very low conf=0.01):")
    try:
        raw = model.predict(source=frame, conf=0.01, verbose=True)
        for r in raw:
            if r.boxes is not None and len(r.boxes):
                names = r.names  # class id → name mapping
                for i in range(len(r.boxes)):
                    cid = int(r.boxes.cls[i])
                    cname = names.get(cid, str(cid)) if names else str(cid)
                    conf = float(r.boxes.conf[i])
                    print(f"  {cname:30s}  conf={conf:.3f}")
    except Exception as e:
        print(f"  Raw predict failed: {e}")

    # ── CLIP re-rank approach ─────────────────────────────────────────────────
    print(f"\n{'='*70}")
    print("Two-stage: YOLOE proposals + CLIP re-classification")
    try:
        import clip as openai_clip
        import torch
        from PIL import Image as PILImage

        # Load CLIP ViT-B/32
        clip_model, clip_preprocess = openai_clip.load("ViT-B/32", device="cuda:0")
        clip_model.eval()

        target_prompts = ["fire extinguisher", "first aid kit", "hazard sign"]

        # Precompute text features for target classes
        with torch.no_grad():
            text_tokens = openai_clip.tokenize(target_prompts).to("cuda:0")
            text_feats = clip_model.encode_text(text_tokens)
            text_feats = text_feats / text_feats.norm(dim=-1, keepdim=True)

        # Broad proposal set: target names + visual appearance anchors
        # ensures YOLOE finds objects even when semantic match is weak
        proposal_classes = target_prompts + ["red cylinder", "white box", "red object", "container", "box"]
        model.set_classes(proposal_classes)
        proposal_results = model.predict(source=frame, conf=0.01, verbose=False)

        print(f"  Proposals at conf≥0.01 with classes {proposal_classes}:")
        all_proposals = []
        for r in proposal_results:
            if r.boxes is None:
                continue
            for i in range(len(r.boxes)):
                yoloe_conf = float(r.boxes.conf[i])
                cls_id = int(r.boxes.cls[i])
                yoloe_class = proposal_classes[cls_id] if cls_id < len(proposal_classes) else str(cls_id)
                xywh = r.boxes.xywh[i].tolist()
                cx, cy, w, h = xywh
                all_proposals.append((yoloe_conf, yoloe_class, cx, cy, w, h))
                print(f"    proposal: {yoloe_class:25s} yoloe_conf={yoloe_conf:.3f}  bbox=({cx:.0f},{cy:.0f} {w:.0f}×{h:.0f})")

        # CLIP re-classify each proposal crop
        CLIP_SIM_THRESHOLD = 0.20  # raw cosine similarity
        print(f"\n  CLIP re-classification (cosine sim threshold={CLIP_SIM_THRESHOLD}):")
        clip_dets = []
        for yoloe_conf, yoloe_class, cx, cy, w, h in all_proposals:
            pad = 0.15
            x1 = max(0, int(cx - w/2 * (1 + pad)))
            y1 = max(0, int(cy - h/2 * (1 + pad)))
            x2 = min(frame.shape[1], int(cx + w/2 * (1 + pad)))
            y2 = min(frame.shape[0], int(cy + h/2 * (1 + pad)))
            if x2 - x1 < 8 or y2 - y1 < 8:
                continue

            crop_bgr = frame[y1:y2, x1:x2]
            crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
            pil_crop = PILImage.fromarray(crop_rgb)

            with torch.no_grad():
                img_input = clip_preprocess(pil_crop).unsqueeze(0).to("cuda:0")
                img_feats = clip_model.encode_image(img_input)
                img_feats = img_feats / img_feats.norm(dim=-1, keepdim=True)
                sims = (img_feats @ text_feats.T)[0].cpu().numpy()  # shape: (n_targets,)

            best_idx = int(sims.argmax())
            best_sim = float(sims[best_idx])
            best_class = target_prompts[best_idx]
            sim_str = "  ".join(f"{target_prompts[j]}={sims[j]:.3f}" for j in range(len(target_prompts)))
            flag = " ✓" if best_sim >= CLIP_SIM_THRESHOLD else " ✗"
            print(f"    [{yoloe_class:20s}] → best={best_class}({best_sim:.3f}){flag}  all=[{sim_str}]")

            if best_sim >= CLIP_SIM_THRESHOLD:
                clip_dets.append((best_class, best_sim, cx, cy, w, h))

        print(f"\n  Final CLIP detections ({len(clip_dets)}):")
        for d in clip_dets:
            print(f"    {d[0]:30s}  sim={d[1]:.3f}  bbox=({d[2]:.0f},{d[3]:.0f} {d[4]:.0f}×{d[5]:.0f})")

        if clip_dets:
            vis = draw_detections(frame, clip_dets, "CLIP re-rank")
            cv2.imwrite(os.path.join(script_dir, "debug_clip_rerank.jpg"), vis)
            print(f"  → saved debug_clip_rerank.jpg")

    except Exception as e:
        import traceback
        print(f"  CLIP re-rank failed: {e}")
        traceback.print_exc()

    # ── try YOLO-World-v2 ─────────────────────────────────────────────────────
    print(f"\n{'='*70}")
    print("Trying YOLO-World-v2 (yolov8s-worldv2.pt)...")
    try:
        wm = YOLO(os.path.join(models_dir, "yolov8s-worldv2.pt"))
        wm.to("cuda:0")
        target_classes = ["fire extinguisher", "first aid kit"]
        wm.set_classes(target_classes)
        for thresh in [0.05, 0.10, 0.15, 0.20]:
            wdets = run_inference(frame, target_classes, thresh, wm)
            print(f"  WORLD conf≥{thresh:.2f} → {len(wdets)} det(s): {[(d[0], round(d[1],3)) for d in wdets]}")
        # Also try broad class set
        broad = ["fire extinguisher", "first aid kit", "red cylinder", "white box", "hazard sign", "bottle", "box"]
        wm.set_classes(broad)
        wdets = wm.predict(source=frame, conf=0.05, verbose=False)
        print(f"\n  WORLD broad classes {broad}:")
        for r in wdets:
            if r.boxes is not None:
                for i in range(len(r.boxes)):
                    cid = int(r.boxes.cls[i])
                    cname = broad[cid] if cid < len(broad) else str(cid)
                    conf = float(r.boxes.conf[i])
                    xywh = r.boxes.xywh[i].tolist()
                    print(f"    {cname:30s}  conf={conf:.3f}  bbox=({xywh[0]:.0f},{xywh[1]:.0f} {xywh[2]:.0f}×{xywh[3]:.0f})")
    except Exception as e:
        print(f"  YOLO-World failed: {e}")

    # ── try base COCO model (yolov8s.pt) ──────────────────────────────────────
    print(f"\n{'='*70}")
    print("Trying yolov8s.pt (COCO, no set_classes — see what it natively detects):")
    try:
        cm = YOLO(os.path.join(models_dir, "yolov8s.pt"))
        cm.to("cuda:0")
        coco_raw = cm.predict(source=frame, conf=0.05, verbose=False)
        for r in coco_raw:
            if r.boxes is not None and len(r.boxes):
                names = r.names
                for i in range(len(r.boxes)):
                    cid = int(r.boxes.cls[i])
                    cname = names.get(cid, str(cid))
                    conf = float(r.boxes.conf[i])
                    xywh = r.boxes.xywh[i].tolist()
                    print(f"  {cname:30s}  conf={conf:.3f}  bbox=({xywh[0]:.0f},{xywh[1]:.0f} {xywh[2]:.0f}×{xywh[3]:.0f})")
        if not any(r.boxes is not None and len(r.boxes) for r in coco_raw):
            print("  (no detections at conf≥0.05)")
    except Exception as e:
        print(f"  COCO model failed: {e}")

    # ── save annotated image ─────────────────────────────────────────────────
    if best_dets:
        vis = draw_detections(frame, best_dets, best_label)
        out_path = os.path.join(script_dir, "debug_detections.jpg")
        cv2.imwrite(out_path, vis)
        print(f"\nBest result ({best_label}):")
        for d in best_dets:
            print(f"  {d[0]}  conf={d[1]:.3f}")
        print(f"Annotated image → {out_path}")
    else:
        print("\nNo detections in any configuration!")

    print("\nDone.")


if __name__ == "__main__":
    main()
