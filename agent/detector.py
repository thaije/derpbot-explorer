"""
Detector — open-vocabulary object detection using YOLOE26-S (or YOLO-World-S fallback).

Runs inference on the RGB camera stream (/derpbot_0/rgbd/image) at ~5 Hz on GPU 0.
Detected bounding boxes + class names are posted to an internal thread-safe queue
consumed by tracker.py.

Architecture: inference runs in a *separate OS process* (via multiprocessing) to
completely avoid Python GIL contention with ROS2's MultiThreadedExecutor threads.
The ROS2 callback converts images and pushes them to a multiprocessing Queue; the
inference subprocess pulls frames, runs model.predict(), and pushes results back.

Pipeline (two-stage):
  Stage 1 — YOLOE region proposals (conf=0.01, very low — just for bbox generation):
    Classes: mission targets (underscores→spaces, e.g. "fire extinguisher") +
             visual anchors: "red cylinder", "white box", "red object",
             "container", "box", "sign".
    Visual anchors exist because YOLOE won't propose "fire extinguisher" on a plain
    red cylinder mesh, but will propose "red cylinder" — giving CLIP something to work with.

  Stage 2 — CLIP ViT-B/32 re-classification (semantic label assignment):
    Each YOLOE crop (+ top-half crop for tall/narrow boxes) is compared against
    text embeddings of the mission target prompts only.
    Accepted if: cosine_sim >= 0.20 (CLIP_SIM_THRESHOLD)
             AND sim_best - sim_2nd >= 0.010 (CLIP_MARGIN_THRESHOLD).
    Output class_name is always one of the mission targets.
    The "X boxes" count in agent logs is post-CLIP (passed both thresholds).

Tuning notes:
  - CLIP_SIM_THRESHOLD=0.20 is at the floor for fire extinguisher (red cylinder mesh).
    Raising it eliminates fire extinguisher detections entirely — do not increase.
  - CLIP_MARGIN_THRESHOLD=0.010 is the main guard against ambiguous matches.
  - Prompts are targets[i].replace("_", " ") — already optimised for low-poly sim meshes.

Usage:
  detector = Detector(node, targets=["fire extinguisher", "hazard sign"])
  detector.start()
  # detector.detections is a queue.Queue of DetectionResult items
"""

from __future__ import annotations

import multiprocessing as mp
import os
import queue
import threading
from dataclasses import dataclass
from typing import Optional

# Model weights live in <repo_root>/models/
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(_REPO_ROOT, "models")

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

# cv_bridge for ROS Image → numpy
try:
    from cv_bridge import CvBridge
except ImportError:
    raise ImportError("cv_bridge not found — install ros-jazzy-cv-bridge")

PROCESS_EVERY_N_FRAMES = 2      # process every 2nd frame → ~5 Hz from 10 Hz stream
GPU_DEVICE = 0                  # GPU index for inference

# Two-stage detection: YOLOE region proposals → CLIP re-classification
# YOLOE fires these extra "visual anchor" classes alongside mission targets
# to reliably produce candidate bboxes for low-poly sim objects.
PROPOSAL_EXTRA_CLASSES = [
    "red cylinder", "white box", "red object", "container", "box", "sign",
]
PROPOSAL_CONF = 0.01            # very low — we just want region proposals
CLIP_SIM_THRESHOLD = 0.20       # cosine similarity to accept a CLIP match
CLIP_MARGIN_THRESHOLD = 0.010   # best-vs-second-best gap; rejects ambiguous / furniture hits


@dataclass
class DetectionResult:
    class_name: str
    confidence: float
    # Bounding box in pixel coords (centre x, centre y, width, height)
    cx_px: float
    cy_px: float
    w_px: float
    h_px: float
    # ROS timestamp as (sec, nanosec) tuple
    stamp: object


# ---------------------------------------------------------------------------
# Subprocess worker — runs in a completely separate Python process (no ROS2)
# ---------------------------------------------------------------------------

def _inference_worker(
    targets: list[str],
    frame_queue: mp.Queue,
    result_queue: mp.Queue,
    ready_event: mp.Event,
) -> None:
    """
    Runs in a child process. Two-stage pipeline:
      1. YOLOE: region proposals at very low conf (broad class set for recall).
      2. CLIP (ViT-B/32): re-classify each crop against target class names.
         This decouples localization (YOLOE) from semantics (CLIP), so any
         target class name works without manual prompt tuning.
    """
    try:
        from ultralytics import YOLO
        import clip as openai_clip
        import torch
        from PIL import Image as PILImage
        import cv2 as _cv2
    except ImportError as exc:
        result_queue.put({"error": f"missing dependency: {exc}"})
        return

    import os as _os
    _models_dir = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), "models")
    # ultralytics looks for mobileclip_blt.ts in CWD — point it at models/
    _os.chdir(_models_dir)

    prompts = [t.replace("_", " ") for t in targets]
    proposal_classes = prompts + PROPOSAL_EXTRA_CLASSES

    # ── Load YOLOE ────────────────────────────────────────────────────────────
    model = None
    model_name = None
    for weights, name in [
        (_os.path.join(_models_dir, "yoloe-11s-seg.pt"), "yoloe-11s"),
        (_os.path.join(_models_dir, "yolov8s-worldv2.pt"), "yolo-world-s"),
    ]:
        try:
            model = YOLO(weights)
            model.set_classes(proposal_classes)
            model.to(f"cuda:{GPU_DEVICE}")
            model.predict(
                source=np.zeros((480, 640, 3), dtype=np.uint8),
                conf=0.99, verbose=False,
            )
            model_name = name
            break
        except Exception:
            model = None

    if model is None:
        result_queue.put({"error": "could not load YOLOE model"})
        return

    # ── Load CLIP + precompute target text features ───────────────────────────
    try:
        clip_model, clip_preprocess = openai_clip.load("ViT-B/32", device=f"cuda:{GPU_DEVICE}")
        clip_model.eval()
        with torch.no_grad():
            text_tokens = openai_clip.tokenize(prompts).to(f"cuda:{GPU_DEVICE}")
            text_feats = clip_model.encode_text(text_tokens)
            text_feats = text_feats / text_feats.norm(dim=-1, keepdim=True)
        clip_ok = True
    except Exception as exc:
        result_queue.put({"warning": f"CLIP unavailable ({exc}), falling back to YOLOE-only"})
        clip_ok = False

    result_queue.put({"ready": f"{model_name}+clip" if clip_ok else model_name})
    ready_event.set()

    while True:
        try:
            item = frame_queue.get(timeout=1.0)
        except Exception:
            continue

        if item is None:  # poison pill — shutdown
            break

        frame_bytes, shape, stamp_sec, stamp_nanosec = item
        frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape(shape)

        # ── Stage 1: YOLOE region proposals ──────────────────────────────────
        try:
            yoloe_results = model.predict(
                source=frame,
                conf=PROPOSAL_CONF,
                verbose=False,
            )
        except Exception as exc:
            result_queue.put({"warning": f"inference error: {exc}"})
            continue

        proposals = []
        for result in yoloe_results:
            if result.boxes is None:
                continue
            for i in range(len(result.boxes)):
                yoloe_conf = float(result.boxes.conf[i])
                cls_id = int(result.boxes.cls[i])
                xywh = result.boxes.xywh[i].tolist()
                proposals.append((yoloe_conf, cls_id, xywh[0], xywh[1], xywh[2], xywh[3]))

        # ── Stage 2: CLIP re-classification ──────────────────────────────────
        detections = []
        if clip_ok and proposals:
            try:
                for yoloe_conf, cls_id, cx, cy, w, h in proposals:
                    pad = 0.15
                    x1 = max(0, int(cx - w / 2 * (1 + pad)))
                    y1 = max(0, int(cy - h / 2 * (1 + pad)))
                    x2 = min(shape[1], int(cx + w / 2 * (1 + pad)))
                    y2 = min(shape[0], int(cy + h / 2 * (1 + pad)))
                    if x2 - x1 < 8 or y2 - y1 < 8:
                        continue

                    # Build candidate crops: always full bbox, plus top-half for
                    # tall narrow bboxes (pole-mounted signs where YOLOE captures the
                    # whole pole but the interesting region is at the top).
                    crops = [(x1, y1, x2, y2)]
                    if h > 2.5 * w:
                        mid_y = y1 + (y2 - y1) // 2
                        crops.append((x1, y1, x2, mid_y))  # top half

                    best_sims = None
                    for cx1, cy1, cx2, cy2 in crops:
                        if cx2 - cx1 < 8 or cy2 - cy1 < 8:
                            continue
                        crop_bgr = frame[cy1:cy2, cx1:cx2]
                        crop_rgb = _cv2.cvtColor(crop_bgr, _cv2.COLOR_BGR2RGB)
                        pil_crop = PILImage.fromarray(crop_rgb)
                        with torch.no_grad():
                            img_input = clip_preprocess(pil_crop).unsqueeze(0).to(f"cuda:{GPU_DEVICE}")
                            img_feats = clip_model.encode_image(img_input)
                            img_feats = img_feats / img_feats.norm(dim=-1, keepdim=True)
                            sims = (img_feats @ text_feats.T)[0].cpu().numpy()
                        if best_sims is None or float(sims.max()) > float(best_sims.max()):
                            best_sims = sims

                    sims = best_sims
                    sorted_idx = sims.argsort()[::-1]
                    best_idx = int(sorted_idx[0])
                    best_sim = float(sims[best_idx])
                    margin = float(sims[best_idx] - sims[sorted_idx[1]])
                    if best_sim < CLIP_SIM_THRESHOLD or margin < CLIP_MARGIN_THRESHOLD:
                        continue

                    detections.append({
                        "class_name": targets[best_idx],
                        "confidence": best_sim,
                        "cx_px": cx,
                        "cy_px": cy,
                        "w_px": w,
                        "h_px": h,
                        "stamp_sec": stamp_sec,
                        "stamp_nanosec": stamp_nanosec,
                    })
            except Exception as exc:
                result_queue.put({"warning": f"CLIP re-rank error: {exc}"})

        elif proposals:
            # CLIP unavailable — fall back to YOLOE classification for target classes only
            for yoloe_conf, cls_id, cx, cy, w, h in proposals:
                if cls_id >= len(targets):
                    continue  # visual-anchor proposal class, skip
                if yoloe_conf < 0.10:
                    continue
                detections.append({
                    "class_name": targets[cls_id],
                    "confidence": yoloe_conf,
                    "cx_px": cx, "cy_px": cy, "w_px": w, "h_px": h,
                    "stamp_sec": stamp_sec, "stamp_nanosec": stamp_nanosec,
                })

        result_queue.put({"detections": detections})


class Detector:
    """
    Subscribes to the RGB image topic and runs open-vocabulary detection.

    Inference runs in a separate subprocess (no GIL contention with ROS2 executor).
    All detection results are posted to self.detections (a thread-safe queue.Queue).
    """

    def __init__(self, node: Node, targets: list[str]):
        self._node = node
        self._targets = targets
        self._logger = node.get_logger()
        self._bridge = CvBridge()

        # Output queue consumed by Tracker
        self.detections: queue.Queue[DetectionResult] = queue.Queue(maxsize=100)

        self._frame_count = 0
        self._lock = threading.Lock()
        self._model_name: Optional[str] = None

        # Multiprocessing queues — picklable, cross-process safe
        ctx = mp.get_context("spawn")   # spawn avoids fork+CUDA issues
        self._mp_frames: mp.Queue = ctx.Queue(maxsize=2)
        self._mp_results: mp.Queue = ctx.Queue(maxsize=50)
        self._ready_event = ctx.Event()

        self._logger.info(f"Detector: starting inference subprocess for targets: {targets}")
        self._worker = ctx.Process(
            target=_inference_worker,
            args=(targets, self._mp_frames, self._mp_results, self._ready_event),
            daemon=True,
            name="detector_worker",
        )
        self._worker.start()

        # Background thread: relay results from subprocess → self.detections queue
        self._running = True
        self._relay_thread = threading.Thread(
            target=self._relay_loop, daemon=True, name="detector_relay"
        )
        self._relay_thread.start()

        # Subscribe to RGB image topic (RELIABLE publisher from Gazebo bridge)
        _sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        node.create_subscription(
            Image,
            "/derpbot_0/rgbd/image",
            self._image_cb,
            _sensor_qos,
        )

    def start(self) -> None:
        """Wait for subprocess warmup, then log ready."""
        if self._ready_event.wait(timeout=30.0):
            self._logger.info(
                f"Detector: subprocess ready ({self._model_name or 'unknown'})."
            )
        else:
            self._logger.error("Detector: subprocess did not become ready in 30s.")

    # ------------------------------------------------------------------
    # Image callback — fast path: convert and enqueue frame
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        with self._lock:
            self._frame_count += 1
            fc = self._frame_count
        if fc % PROCESS_EVERY_N_FRAMES != 0:
            return

        try:
            frame: np.ndarray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._logger.warning(f"Detector: cv_bridge error — {exc}")
            return

        # Send to subprocess via multiprocessing Queue (must be picklable)
        stamp = msg.header.stamp
        item = (frame.tobytes(), frame.shape, stamp.sec, stamp.nanosec)
        try:
            self._mp_frames.put_nowait(item)
        except Exception:
            # Queue full — drop oldest, add newest
            try:
                self._mp_frames.get_nowait()
                self._mp_frames.put_nowait(item)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Relay loop — moves results from subprocess queue → self.detections
    # ------------------------------------------------------------------

    def _relay_loop(self) -> None:
        import queue as _queue
        import time as _time
        inference_count = 0
        last_heartbeat = _time.monotonic()
        while self._running:
            try:
                result = self._mp_results.get(timeout=0.5)
            except _queue.Empty:
                # Heartbeat log every 30s to confirm loop is alive
                now = _time.monotonic()
                if now - last_heartbeat > 30.0:
                    self._logger.info(
                        f"Detector: relay alive, {inference_count} inferences so far, "
                        f"subprocess alive={self._worker.is_alive()}"
                    )
                    last_heartbeat = now
                continue
            except Exception as exc:
                self._logger.warning(f"Detector: relay_loop mp_results.get error — {type(exc).__name__}: {exc}")
                continue

            if "ready" in result:
                self._model_name = result["ready"]
                continue
            if "error" in result:
                self._logger.error(f"Detector: subprocess error — {result['error']}")
                continue
            if "warning" in result:
                self._logger.warning(f"Detector: {result['warning']}")
                continue

            dets = result.get("detections", [])
            inference_count += 1
            last_heartbeat = _time.monotonic()  # reset heartbeat on activity
            if inference_count % 5 == 0:
                self._logger.info(
                    f"Detector: inference #{inference_count}, {len(dets)} boxes"
                )
            for d in dets:
                det = DetectionResult(
                    class_name=d["class_name"],
                    confidence=d["confidence"],
                    cx_px=d["cx_px"],
                    cy_px=d["cy_px"],
                    w_px=d["w_px"],
                    h_px=d["h_px"],
                    stamp=(d["stamp_sec"], d["stamp_nanosec"]),
                )
                try:
                    self.detections.put_nowait(det)
                except queue.Full:
                    try:
                        self.detections.get_nowait()
                        self.detections.put_nowait(det)
                    except queue.Empty:
                        pass

    def stop(self) -> None:
        self._running = False
        try:
            self._mp_frames.put_nowait(None)  # poison pill
        except Exception:
            pass
        if self._worker.is_alive():
            self._worker.terminate()
            self._worker.join(timeout=3.0)
