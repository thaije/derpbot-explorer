#!/usr/bin/env python3
"""
Live detector comparison — run while driving the robot in the sim.

Continuously grabs frames from /derpbot_0/rgbd/image and runs one or more
detector backends. Prints per-detection confidence scores so you can see
what each model sees in real time.

Supported backends (selected via --backend):
  yolo-world-s   yolov8s-worldv2.pt (default, already downloaded)
  yolo-world-m   yolov8m-worldv2.pt  (download first, see below)
  yolo-world-l   yolov8l-worldv2.pt
  gdino          Grounding DINO tiny  (requires: pip install groundingdino-py)
  owlv2          OWL-v2 base          (requires: pip install transformers)

Usage:
    # default: YOLO-World-S, targets from easy scenario
    python3 agent/test_detector_live.py

    # specific targets + backend
    python3 agent/test_detector_live.py --backend yolo-world-s \\
        --targets fire_extinguisher first_aid_kit hazard_sign

    # save annotated frames to debug_frames/
    python3 agent/test_detector_live.py --save

    # download larger YOLO-World weights before using:
    #   yolo export model=yolov8m-worldv2.pt  (auto-downloads on first run)

Output format per detection:
    [HH:MM:SS] BACKEND  class_name          conf=0.42  bbox=(cx,cy wxh)
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import threading

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────

MODELS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
FRAME_TOPIC = "/derpbot_0/rgbd/image"
CONF_THRESHOLD = 0.05       # lower than production — we want to see marginal detections
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "debug_frames")

YOLO_WORLD_WEIGHTS = {
    "yolo-world-s": os.path.join(MODELS_DIR, "yolov8s-worldv2.pt"),
    "yolo-world-m": os.path.join(MODELS_DIR, "yolov8m-worldv2.pt"),
    "yolo-world-l": os.path.join(MODELS_DIR, "yolov8l-worldv2.pt"),
}

# ── ROS frame subscriber ──────────────────────────────────────────────────────

class FrameGrabber:
    """Subscribes to the RGB topic; latest_frame always holds the newest BGR array."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge

        rclpy.init()
        self._node = rclpy.create_node("test_detector_live")
        self._bridge = CvBridge()
        self.latest_frame = None
        self.latest_stamp = None
        self._lock = threading.Lock()

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._node.create_subscription(Image, FRAME_TOPIC, self._cb, qos)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        print(f"Waiting for frames on {FRAME_TOPIC}...", flush=True)

    def _cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self._lock:
            self.latest_frame = frame
            self.latest_stamp = msg.header.stamp

    def _spin(self):
        import rclpy
        while rclpy.ok():
            import rclpy
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def get(self):
        with self._lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def shutdown(self):
        import rclpy
        self._node.destroy_node()
        rclpy.shutdown()


# ── Detector backends ─────────────────────────────────────────────────────────

def load_yolo_world(backend: str, targets: list[str]):
    weights = YOLO_WORLD_WEIGHTS[backend]
    if not os.path.exists(weights):
        print(f"Weights not found at {weights}. Ultralytics will auto-download on first use.")
    os.chdir(MODELS_DIR)
    from ultralytics import YOLO
    model = YOLO(weights)
    model.set_classes(targets)
    model.to("cuda:0")
    # warmup
    model.predict(source=np.zeros((480, 640, 3), dtype=np.uint8), conf=0.99, verbose=False)
    print(f"  ✓ {backend} loaded", flush=True)

    def predict(frame):
        results = model.predict(source=frame, conf=CONF_THRESHOLD, verbose=False)
        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for i in range(len(r.boxes)):
                conf = float(r.boxes.conf[i])
                cls_id = int(r.boxes.cls[i])
                name = targets[cls_id] if cls_id < len(targets) else str(cls_id)
                cx, cy, w, h = r.boxes.xywh[i].tolist()
                dets.append((name, conf, cx, cy, w, h))
        return dets

    return predict


def load_gdino(targets: list[str]):
    try:
        from groundingdino.util.inference import load_model, predict as gdino_predict
        import torch
    except ImportError:
        raise ImportError("Grounding DINO not installed. Run: pip install groundingdino-py")

    config = os.path.join(MODELS_DIR, "GroundingDINO_SwinT_OGC.py")
    weights = os.path.join(MODELS_DIR, "groundingdino_swint_ogc.pth")
    if not os.path.exists(config) or not os.path.exists(weights):
        raise FileNotFoundError(
            f"Grounding DINO weights/config not found in {MODELS_DIR}.\n"
            "Download from: https://github.com/IDEA-Research/GroundingDINO"
        )
    model = load_model(config, weights)
    model = model.cuda().eval()
    text_prompt = " . ".join(targets)  # DINO uses ". "-separated prompts
    print(f"  ✓ Grounding DINO loaded, prompt: '{text_prompt}'", flush=True)

    from groundingdino.util.inference import predict as gdino_predict
    from torchvision import transforms as T
    transform = T.Compose([
        T.ToPILImage(),
        T.Resize((800, 800)),
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    def predict(frame):
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_t = transform(rgb).unsqueeze(0).cuda()
        boxes, logits, phrases = gdino_predict(
            model=model, image=img_t,
            caption=text_prompt,
            box_threshold=0.30, text_threshold=0.20,
        )
        dets = []
        for box, logit, phrase in zip(boxes, logits, phrases):
            # box is cx,cy,w,h normalised
            cx_px = float(box[0]) * w
            cy_px = float(box[1]) * h
            w_px = float(box[2]) * w
            h_px = float(box[3]) * h
            dets.append((phrase, float(logit), cx_px, cy_px, w_px, h_px))
        return dets

    return predict


def load_owlv2(targets: list[str]):
    try:
        from transformers import Owlv2Processor, Owlv2ForObjectDetection
        import torch
    except ImportError:
        raise ImportError("OWL-v2 not installed. Run: pip install transformers")

    model_id = "google/owlv2-base-patch16-ensemble"
    print(f"  Loading OWL-v2 ({model_id}) — may download on first run...", flush=True)
    processor = Owlv2Processor.from_pretrained(model_id)
    model = Owlv2ForObjectDetection.from_pretrained(model_id).cuda().eval()
    print(f"  ✓ OWL-v2 loaded", flush=True)
    queries = [[t.replace("_", " ") for t in targets]]

    import torch
    def predict(frame):
        from PIL import Image as PILImage
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        inputs = processor(text=queries, images=pil, return_tensors="pt")
        inputs = {k: v.cuda() for k, v in inputs.items()}
        with torch.no_grad():
            outputs = model(**inputs)
        h, w = frame.shape[:2]
        target_sizes = torch.tensor([(h, w)]).cuda()
        results = processor.post_process_grounded_object_detection(
            outputs, target_sizes=target_sizes, threshold=CONF_THRESHOLD
        )[0]
        dets = []
        for score, label_idx, box in zip(results["scores"], results["labels"], results["boxes"]):
            x1, y1, x2, y2 = box.tolist()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            bw = x2 - x1
            bh = y2 - y1
            name = targets[label_idx] if label_idx < len(targets) else str(int(label_idx))
            dets.append((name, float(score), cx, cy, bw, bh))
        return dets

    return predict


# ── Drawing ───────────────────────────────────────────────────────────────────

def draw(frame, dets, backend):
    vis = frame.copy()
    for name, conf, cx, cy, w, h in dets:
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 50), 2)
        cv2.putText(vis, f"{name} {conf:.2f}", (x1, max(y1 - 5, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 50), 1)
    cv2.putText(vis, backend, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 255), 2)
    return vis


# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Live detector test")
    parser.add_argument("--backend", default="yolo-world-s",
                        choices=list(YOLO_WORLD_WEIGHTS.keys()) + ["gdino", "owlv2"])
    parser.add_argument("--targets", nargs="+",
                        default=["fire_extinguisher", "first_aid_kit", "hazard_sign"])
    parser.add_argument("--save", action="store_true", help="Save annotated frames to debug_frames/")
    parser.add_argument("--hz", type=float, default=2.0, help="Inference rate (default 2 Hz)")
    args = parser.parse_args()

    targets = [t.replace("_", " ") for t in args.targets]
    print(f"Backend : {args.backend}")
    print(f"Targets : {targets}")
    print(f"Rate    : {args.hz} Hz\n", flush=True)

    print(f"Loading {args.backend}...", flush=True)
    if args.backend in YOLO_WORLD_WEIGHTS:
        predict = load_yolo_world(args.backend, targets)
    elif args.backend == "gdino":
        predict = load_gdino(targets)
    elif args.backend == "owlv2":
        predict = load_owlv2(targets)

    if args.save:
        os.makedirs(SAVE_DIR, exist_ok=True)
        print(f"Saving frames to {SAVE_DIR}/", flush=True)

    grabber = FrameGrabber()
    interval = 1.0 / args.hz
    frame_idx = 0

    print("\nRunning — drive the robot. Ctrl+C to stop.\n", flush=True)
    try:
        while True:
            t0 = time.monotonic()
            frame = grabber.get()
            if frame is None:
                time.sleep(0.1)
                continue

            dets = predict(frame)
            ts = time.strftime("%H:%M:%S")

            if dets:
                for name, conf, cx, cy, w, h in dets:
                    print(f"[{ts}] {args.backend:14s}  {name:25s}  conf={conf:.3f}  "
                          f"bbox=({cx:.0f},{cy:.0f} {w:.0f}×{h:.0f})", flush=True)
            else:
                print(f"[{ts}] {args.backend:14s}  (no detections)", flush=True)

            if args.save:
                vis = draw(frame, dets, args.backend)
                cv2.imwrite(os.path.join(SAVE_DIR, f"frame_{frame_idx:05d}.jpg"), vis)
                frame_idx += 1

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        grabber.shutdown()


if __name__ == "__main__":
    main()
