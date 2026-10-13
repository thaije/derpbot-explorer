"""
Detector — open-vocabulary object detection using OWL-v2 (google/owlv2-base-patch16-ensemble).

Runs inference on the RGB camera stream (/derpbot_0/rgbd/image) at ~5 Hz on GPU 0 (shared with Gazebo renderer).
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

# cv_bridge for ROS Image → numpy
try:
    from cv_bridge import CvBridge
except ImportError:
    raise ImportError("cv_bridge not found — install ros-jazzy-cv-bridge")

PROCESS_EVERY_N_FRAMES = 2  # process every 2nd frame → ~5 Hz from 10 Hz stream
# GPU index for inference within the subprocess's CUDA_VISIBLE_DEVICES scope.
# CUDA_VISIBLE_DEVICES is set to "1" inside _inference_worker (before torch import)
# so "cuda:0" here maps to nvidia-smi GPU 1 (RTX 2070 SUPER),
# completely hiding GPU 0 (Gazebo's rendering GPU) from PyTorch.
GPU_DEVICE = 0

OWL_MODEL_ID = "google/owlv2-base-patch16-ensemble"
OWL_CONF_THRESHOLD = 0.15  # tuned: 0.10 gave 3 FPs; 0.12 gave 3 FPs with no net benefit; 0.15 gives 1 FP best precision (fire_ext#2 marginal but found at good nav coverage)


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
    # Only 1 GPU available — PyTorch and Gazebo share GPU 0.
    # (Previously CUDA_VISIBLE_DEVICES=1 kept PyTorch off Gazebo's rendering GPU;
    # that GPU no longer exists. RTF may be lower than the 3-GPU baseline.)
    import os as _os

    _os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    _os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
    _os.environ["HF_HUB_OFFLINE"] = "1"

    try:
        import torch

        torch.set_num_threads(4)
        torch.set_num_interop_threads(2)
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
        model = model.to(f"cuda:{GPU_DEVICE}").half().eval()
    except Exception as exc:
        result_queue.put({"error": f"could not load OWL-v2: {exc}"})
        return

    # Warmup pass
    _dummy = _inputs = None
    try:
        _dummy = PILImage.fromarray(np.zeros((480, 640, 3), dtype=np.uint8))
        _inputs = processor(text=queries, images=_dummy, return_tensors="pt")
        _inputs = {
            k: (v.half() if v.dtype == torch.float32 else v).to(f"cuda:{GPU_DEVICE}")
            for k, v in _inputs.items()
        }
        with torch.inference_mode():
            model(**_inputs)
    except Exception as exc:
        result_queue.put({"warning": f"OWL-v2 warmup failed: {exc}"})
    finally:
        del _dummy, _inputs
        torch.cuda.empty_cache()

    result_queue.put({"ready": "owlv2"})
    ready_event.set()

    _inf_n = 0
    while True:
        try:
            item = frame_queue.get(timeout=1.0)
        except Exception:
            continue

        if item is None:  # poison pill — shutdown
            break

        frame_bytes, shape, stamp_sec, stamp_nanosec = item
        frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape(shape)

        outputs = inputs = target_sizes = results = None
        try:
            rgb = _cv2.cvtColor(frame, _cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb)
            inputs = processor(text=queries, images=pil_image, return_tensors="pt")
            inputs = {
                k: (v.half() if v.dtype == torch.float32 else v).to(
                    f"cuda:{GPU_DEVICE}"
                )
                for k, v in inputs.items()
            }
            target_sizes = torch.tensor(
                [(shape[0], shape[1])], dtype=torch.int32, device=f"cuda:{GPU_DEVICE}"
            )
            with torch.inference_mode():
                outputs = model(**inputs)
            results = processor.post_process_grounded_object_detection(
                outputs,
                target_sizes=target_sizes,
                threshold=OWL_CONF_THRESHOLD,
            )[0]
            # Convert GPU tensors to CPU/numpy immediately to release CUDA memory.
            # post_process returns {"scores": tensor, "labels": tensor, "boxes": tensor}
            # on GPU — keeping them alive prevents the CUDA allocator from reclaiming.
            det_scores = results["scores"].cpu().tolist()
            det_labels = results["labels"].cpu().tolist()
            det_boxes = results["boxes"].cpu().tolist()
        except Exception as exc:
            result_queue.put({"warning": f"inference error: {exc}"})
            continue
        finally:
            del outputs, inputs, target_sizes, results
            torch.cuda.empty_cache()

        _inf_n += 1
        if _inf_n % 20 == 0:
            _alloc = torch.cuda.memory_allocated() / 1024 / 1024
            _reserved = torch.cuda.memory_reserved() / 1024 / 1024
            result_queue.put({
                "vram": f"inf#{_inf_n} alloc={_alloc:.0f}MB reserved={_reserved:.0f}MB"
            })

        detections = []
        for score, label_idx, box in zip(det_scores, det_labels, det_boxes):
            x1, y1, x2, y2 = box
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            w = x2 - x1
            h = y2 - y1
            idx = int(label_idx)
            if idx >= len(targets):
                continue
            detections.append(
                {
                    "class_name": targets[idx],
                    "confidence": float(score),
                    "cx_px": cx,
                    "cy_px": cy,
                    "w_px": w,
                    "h_px": h,
                    "stamp_sec": stamp_sec,
                    "stamp_nanosec": stamp_nanosec,
                }
            )

        try:
            result_queue.put({"detections": detections}, timeout=5.0)
        except Exception:
            # Queue full or blocked — drop result, don't hang subprocess
            pass


class Detector:
    """
    Wraps the OWLv2 object-detection subprocess and exposes a simple
    detections: list[DetectionResult] property that the agent can poll.

    Runs the heavy inference in a *separate process* (spawn) so the
    Python GIL does not block the ROS2 callbacks on the main thread.

    NOTE: The main-thread callbacks only do:
      - Convert ROS image → numpy (cheap)
      - Push to multiprocessing.Queue (non-blocking when not full)
    """

    def __init__(self, node: Node, targets: list[str], create_subscriber: bool = True):
        self._node = node
        self._logger = node.get_logger()
        self._create_subscriber = create_subscriber
        self._bridge = CvBridge()
        self._targets = targets  # saved for _restart_worker

        # Output queue consumed by Tracker
        self.detections: queue.Queue[DetectionResult] = queue.Queue(maxsize=100)

        self._frame_count = 0
        self._lock = threading.Lock()
        self._model_name: Optional[str] = None

        # Multiprocessing queues — picklable, cross-process safe
        ctx = mp.get_context("spawn")  # spawn avoids fork+CUDA issues
        self._mp_frames: mp.Queue = ctx.Queue(maxsize=2)
        self._mp_results: mp.Queue = ctx.Queue(maxsize=50)
        self._ready_event = ctx.Event()

        self._logger.info(
            f"Detector: starting inference subprocess for targets: {targets}"
        )
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

        # Subscribe to RGB image topic.
        # ReentrantCallbackGroup: image callback must not be serialized behind
        # odom/map callbacks in the default MutuallyExclusiveCallbackGroup —
        # starvation there caused #35 (image_cb stops firing after ~100 calls).
        # _image_cb is thread-safe (only touches self._lock and mp.Queue).
        if create_subscriber:
            _sensor_qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            _cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
            self._last_cb_time = 0.0  # Wall-clock rate limiter
            self._cb_interval = 0.2  # 5 Hz callback rate limit
            node.create_subscription(
                Image,
                "/derpbot_0/rgbd/image",
                self._image_cb,
                _sensor_qos,
                callback_group=_cb_group,
            )
            # Watchdog timer: warn if _image_cb stops being called (#35)
            self._image_watchdog_timer = node.create_timer(
                30.0,
                self._image_watchdog,
                callback_group=_cb_group,
            )
        else:
            self._logger.info("Detector: subscriber disabled for GIL probe.")

    def start(self) -> None:
        """Wait for subprocess warmup, then log ready."""
        if self._ready_event.wait(timeout=60.0):
            self._logger.info(
                f"Detector: subprocess ready ({self._model_name or 'unknown'})."
            )
        else:
            self._logger.error("Detector: subprocess did not become ready in 30s.")

    def _image_watchdog(self) -> None:
        """Timer callback: warns if _image_cb hasn't been called recently (#35)."""
        import time
        if self._last_cb_time > 0 and time.monotonic() - self._last_cb_time > 30.0:
            self._logger.warning(
                f"Detector: no image callback for {time.monotonic() - self._last_cb_time:.0f}s "
                f"(raw_count={getattr(self, '_raw_cb_count', 0)}) — "
                f"camera topic may have stopped publishing"
            )

    # ------------------------------------------------------------------
    # Image callback — fast path: convert and enqueue frame
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        import time
        now = time.monotonic()
        # Diagnostic: log every 500th raw callback to verify camera is publishing
        self._raw_cb_count = getattr(self, "_raw_cb_count", 0) + 1
        if self._raw_cb_count % 100 == 1:
            self._logger.info(
                f"Detector: _image_cb raw={self._raw_cb_count}, "
                f"passed_rate_limiter={self._frame_count}"
            )
        if now - self._last_cb_time < self._cb_interval:
            return
        self._last_cb_time = now

        with self._lock:
            self._frame_count += 1
            fc = self._frame_count
        if fc % PROCESS_EVERY_N_FRAMES !=0:
            return

        try:
            frame: np.ndarray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._logger.warning(f"Detector: cv_bridge error — {exc}")
            return

        stamp = msg.header.stamp
        item = (frame.tobytes(), frame.shape, stamp.sec, stamp.nanosec)
        try:
            self._mp_frames.put_nowait(item)
        except Exception:
            try:
                self._mp_frames.get_nowait()
                self._mp_frames.put_nowait(item)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Relay loop — moves results from subprocess queue → self.detections
    # ------------------------------------------------------------------

    def _restart_worker(self) -> None:
        """Kill the stalled inference subprocess and start a fresh one."""
        self._logger.warning("Detector: restarting stalled inference subprocess.")
        try:
            if self._worker.is_alive():
                self._worker.terminate()
                self._worker.join(timeout=5.0)
                if self._worker.is_alive():
                    self._worker.kill()
                    self._worker.join(timeout=2.0)
        except Exception as exc:
            self._logger.warning(f"Detector: error terminating worker — {exc}")

        # Drain old queues so they don't confuse the new worker
        for q in (self._mp_frames, self._mp_results):
            try:
                while True:
                    q.get_nowait()
            except Exception:
                pass

        ctx = mp.get_context("spawn")
        self._ready_event = ctx.Event()
        self._worker = ctx.Process(
            target=_inference_worker,
            args=(self._targets, self._mp_frames, self._mp_results, self._ready_event),
            daemon=True,
            name="detector_worker",
        )
        self._worker.start()
        self._logger.info(
            "Detector: new inference subprocess started — waiting for ready."
        )

    def _relay_loop(self) -> None:
        import queue as _queue
        import time as _time

        inference_count = 0
        last_result_time = _time.monotonic()
        last_heartbeat = _time.monotonic()
        STALL_TIMEOUT = 90.0  # seconds — restart subprocess if no result in this time
        while self._running:
            try:
                result = self._mp_results.get(timeout=0.5)
            except _queue.Empty:
                now = _time.monotonic()
                # Watchdog: restart subprocess if it stalls (CUDA hang, OOM, etc.)
                # But skip if no frames are queued — the robot is stationary/patrolling
                # and a restart won't help (no images to process).
                if now - last_result_time > STALL_TIMEOUT and self._worker.is_alive():
                    frame_q_empty = self._mp_frames.empty()
                    if frame_q_empty:
                        # No work available — not a stall, just idle.
                        # Reset timer so we don't keep hitting this branch.
                        last_result_time = now
                        self._logger.info(
                            f"Detector: no results for {STALL_TIMEOUT:.0f}s but "
                            f"frame_q_empty — subprocess idle, not hung."
                        )
                    else:
                        self._logger.error(
                            f"Detector: no inference results for {STALL_TIMEOUT:.0f}s — "
                            f"subprocess hung, restarting. "
                            f"(frame_q_empty={frame_q_empty})"
                        )
                        self._restart_worker()
                        last_result_time = _time.monotonic()
                        last_heartbeat = last_result_time
                # Heartbeat log every 30s to confirm loop is alive
                elif now - last_heartbeat > 30.0:
                    self._logger.info(
                        f"Detector: relay alive, {inference_count} inferences so far, "
                        f"subprocess alive={self._worker.is_alive()}"
                    )
                    last_heartbeat = now
                continue
            except Exception as exc:
                self._logger.warning(
                    f"Detector: relay_loop mp_results.get error — {type(exc).__name__}: {exc}"
                )
                continue

            # Reset watchdog on ANY result from subprocess (ready/error/warning/detections)
            last_result_time = _time.monotonic()
            last_heartbeat = last_result_time

            try:
                if "ready" in result:
                    self._model_name = result["ready"]
                    continue
                if "error" in result:
                    self._logger.error(f"Detector: subprocess error — {result['error']}")
                    continue
                if "warning" in result:
                    self._logger.warning(f"Detector: {result['warning']}")
                    continue
                if "vram" in result:
                    self._logger.info(f"Detector: {result['vram']}")
                    continue

                dets = result.get("detections", [])
                inference_count += 1
                if inference_count % 5 == 0:
                    box_summary = ""
                    if dets:
                        box_summary = " [" + ", ".join(
                            f"{d_['class_name']}({d_['confidence']:.2f})" for d_ in dets
                        ) + "]"
                    self._logger.info(
                        f"Detector: inference #{inference_count}, {len(dets)} boxes{box_summary}"
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
            except Exception as exc:
                self._logger.error(
                    f"Detector: relay_loop result processing error — {type(exc).__name__}: {exc}"
                )
                continue

    def stop(self) -> None:
        self._running = False
        try:
            self._mp_frames.put_nowait(None)  # poison pill
        except Exception:
            pass
        if self._worker.is_alive():
            self._worker.terminate()
            self._worker.join(timeout=3.0)
