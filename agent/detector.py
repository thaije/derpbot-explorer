"""
Detector — open-vocabulary object detection using OWL-v2 (google/owlv2-base-patch16-ensemble).

Runs inference on the RGB camera stream (/derpbot_0/rgbd/image) at ~5 Hz on GPU 0.
Detected bounding boxes + class names are posted to an internal thread-safe queue
consumed by tracker.py.

Architecture: inference runs in a *separate OS process* (via multiprocessing) to
completely avoid Python GIL contention with ROS2's MultiThreadedExecutor threads.
The ROS2 callback converts images and pushes them to a multiprocessing Queue; the
inference subprocess pulls frames, runs inference, and pushes results back.

Pipeline (single-stage):
  OWL-v2 (ViT-B/16) — text-to-bbox in one pass:
    Takes mission target names as text queries (underscores→spaces).
    Returns bboxes + confidence scores directly; no separate re-classification needed.
    Accepted if: confidence >= 0.20 (OWL_CONF_THRESHOLD).
    Output class_name is always one of the mission targets.

Tuning notes:
  - OWL_CONF_THRESHOLD=0.20 validated on low-poly Gazebo sim: no false positives,
    all three target types detected reliably. Do not lower (FP increase).
  - Model weights (~850 MB) are downloaded from HuggingFace on first run.
  - On RTX 2070 SUPER: ~10-15 FPS — comfortable at the 5 Hz processing rate.
  - Compared alternatives on sim: see agent/test_detector_live.py for results.

Usage:
  detector = Detector(node, targets=["fire_extinguisher", "hazard_sign"])
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

OWL_MODEL_ID = "google/owlv2-base-patch16-ensemble"
OWL_CONF_THRESHOLD = 0.20       # validated on low-poly sim — do not lower (FP increase)


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
    Runs in a child process. Single-stage OWL-v2 pipeline:
      OWL-v2 takes text prompts (one per target) and returns bboxes + scores directly.
      No separate re-classification stage needed.
    """
    try:
        import torch
        from PIL import Image as PILImage
        import cv2 as _cv2
        from transformers import Owlv2Processor, Owlv2ForObjectDetection
    except ImportError as exc:
        result_queue.put({"error": f"missing dependency: {exc}"})
        return

    prompts = [t.replace("_", " ") for t in targets]
    queries = [prompts]  # batch of one image

    # ── Load OWL-v2 ───────────────────────────────────────────────────────────
    try:
        processor = Owlv2Processor.from_pretrained(OWL_MODEL_ID)
        model = Owlv2ForObjectDetection.from_pretrained(OWL_MODEL_ID)
        model = model.to(f"cuda:{GPU_DEVICE}").eval()
    except Exception as exc:
        result_queue.put({"error": f"could not load OWL-v2: {exc}"})
        return

    # Warmup pass
    try:
        _dummy = PILImage.fromarray(np.zeros((480, 640, 3), dtype=np.uint8))
        _inputs = processor(text=queries, images=_dummy, return_tensors="pt")
        _inputs = {k: v.to(f"cuda:{GPU_DEVICE}") for k, v in _inputs.items()}
        with torch.no_grad():
            model(**_inputs)
    except Exception as exc:
        result_queue.put({"warning": f"OWL-v2 warmup failed: {exc}"})

    result_queue.put({"ready": "owlv2"})
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

        try:
            rgb = _cv2.cvtColor(frame, _cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb)
            inputs = processor(text=queries, images=pil_image, return_tensors="pt")
            inputs = {k: v.to(f"cuda:{GPU_DEVICE}") for k, v in inputs.items()}

            with torch.no_grad():
                outputs = model(**inputs)

            target_sizes = torch.tensor([(shape[0], shape[1])], device=f"cuda:{GPU_DEVICE}")
            results = processor.post_process_grounded_object_detection(
                outputs, target_sizes=target_sizes, threshold=OWL_CONF_THRESHOLD,
            )[0]
        except Exception as exc:
            result_queue.put({"warning": f"inference error: {exc}"})
            continue

        detections = []
        for score, label_idx, box in zip(results["scores"], results["labels"], results["boxes"]):
            x1, y1, x2, y2 = box.tolist()
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            w = x2 - x1
            h = y2 - y1
            idx = int(label_idx)
            if idx >= len(targets):
                continue
            detections.append({
                "class_name": targets[idx],
                "confidence": float(score),
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
