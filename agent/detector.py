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

  Second-pass ROI recall (event-triggered, t_c6b546ac):
  Trigger (chosen from code reality): the relay consumes results at the
  inference cadence (~5 Hz). A class is ROI-eligible when ALL hold:
  1. the tracker reports that class as a PENDING candidate (unconfirmed,
     >=1 sighting) — read-only subscription, Tracker.pending_class_positions();
  2. the full-frame pass has produced NO sighting of that class for
     K=10 relay cycles (~2 s — matches the task's 2 s dwell budget);
  3. >= ROI_COOLDOWN_S wall-s since this class's last ROI pass.
  ONE second-pass inference then runs on a 320 px center crop around that
  class's last-known IMAGE position, upscaled 2x (>=2x per spec), at
  ROI_CONF_THRESHOLD (lower than full-frame: the 2x upscale restores pixels
  for far/small objects — the forensics (c) bucket sits at 0.7-4.9 m —
  while the tracker still applies its normal confirmation gates
  (MIN_SIGHTINGS/STABILITY/VETO), so added recall cannot publish on its own).
  Detections from the ROI pass carry source="roi_second_pass" for later
  attribution. Compute budget: event-triggered + cooldown => at most
  1 ROI pass per class per 4 s (<=6% extra inference at 5 Hz full-frame);
  the pass runs in the same subprocess worker, so the GIL invariant holds.

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

# ── ROI second-pass (event-triggered recall, t_c6b546ac) ─────────────────────
# Trigger: pending-candidate class silent for K relay cycles (~2 s at 5 Hz).
ROI_SILENT_CYCLES_K = 10        # full-frame passes without a sighting of that class
ROI_COOLDOWN_S = 4.0            # wall-s between ROI passes for the same class
ROI_CONF_THRESHOLD = 0.12       # ROI pass threshold (upscaled crop; tracker still gates)
ROI_CROP_PX = 320               # square crop centred on candidate's last image position
ROI_UPSCALE = 2.0               # crop is upscaled by this factor before inference
ROI_SOURCE_TAG = "roi_second_pass"


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
    # Origin of this detection: "full_frame" (primary pass) or
    # "roi_second_pass" (ROI recall pass) — attribution only, tracker ignores.
    source: str = "full_frame"


# ---------------------------------------------------------------------------
# Subprocess worker — runs in a completely separate Python process (no ROS2)
# ---------------------------------------------------------------------------


def _inference_worker(
    targets: list[str],
    frame_queue: mp.Queue,
    result_queue: mp.Queue,
    ready_event: mp.Event,
    roi_queue: Optional[mp.Queue] = None,
) -> None:
    """
    Runs in a child process. Single-stage OWL-v2 pipeline:
      OWL-v2 takes text prompts (one per target) and returns bboxes + scores directly.
      No separate re-classification stage needed.

    ROI second-pass (t_c6b546ac): if roi_queue is set and carries a
    {"class_name": str, "cx_px", "cy_px", "upscale"} request, ONE extra
    inference runs on a crop around the given image position at
    ROI_CONF_THRESHOLD; results come back on result_queue as a
    {"roi_detections": [...]} message with source=ROI_SOURCE_TAG.
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

    # --------------------------------------------------------------
    # ROI second-pass inference (t_c6b546ac): crop → upscale → infer
    # --------------------------------------------------------------
    def _run_roi_pass(roi_req: dict) -> None:
        """One inference on an upscaled crop around the pending candidate's
        last-known image position. Emits {"roi_detections": [...]} with
        source=ROI_SOURCE_TAG; boxes are mapped back to full-frame px."""
        cls = roi_req.get("roi_class")
        cx_req = roi_req.get("cx_px") or 320.0
        cy_req = roi_req.get("cy_px") or 240.0
        upscale = float(roi_req.get("upscale", ROI_UPSCALE))
        stamp_sec = roi_req.get("stamp_sec", 0)
        stamp_nanosec = roi_req.get("stamp_nanosec", 0)
        frame_bytes, shape = roi_req.get("frame_bytes"), roi_req.get("shape")
        if cls is None or frame_bytes is None or shape is None:
            return
        outputs = inputs = target_sizes = results = None
        try:
            frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape(shape)
            rgb = _cv2.cvtColor(frame, _cv2.COLOR_BGR2RGB)
            H, W = shape[0], shape[1]
            half = ROI_CROP_PX // 2
            x1c = int(max(0, min(W - 1, cx_req - half)))
            y1c = int(max(0, min(H - 1, cy_req - half)))
            x2c = int(min(W, x1c + half * 2))
            y2c = int(min(H, y1c + half * 2))
            crop = rgb[y1c:y2c, x1c:x2c]
            if crop.size == 0:
                return
            upscaled = _cv2.resize(
                crop, None, fx=upscale, fy=upscale,
                interpolation=_cv2.INTER_CUBIC,
            )
            pil_crop = PILImage.fromarray(upscaled)
            inputs = processor(text=queries, images=pil_crop, return_tensors="pt")
            inputs = {
                k: (v.half() if v.dtype == torch.float32 else v).to(
                    f"cuda:{GPU_DEVICE}"
                )
                for k, v in inputs.items()
            }
            target_sizes = torch.tensor(
                [(upscaled.shape[0], upscaled.shape[1])],
                dtype=torch.int32,
                device=f"cuda:{GPU_DEVICE}",
            )
            with torch.inference_mode():
                outputs = model(**inputs)
            results = processor.post_process_grounded_object_detection(
                outputs,
                target_sizes=target_sizes,
                threshold=ROI_CONF_THRESHOLD,
            )[0]
            det_scores = results["scores"].cpu().tolist()
            det_labels = results["labels"].cpu().tolist()
            det_boxes = results["boxes"].cpu().tolist()
        except Exception as exc:
            result_queue.put({"warning": f"roi inference error: {exc}"})
            return
        finally:
            del outputs, inputs, target_sizes, results
            torch.cuda.empty_cache()

        # Map crop coords back to full-frame pixel coords; ROI pass answers
        # only for the pending class.
        roi_dets = []
        for score, label_idx, box in zip(det_scores, det_labels, det_boxes):
            bx1, by1, bx2, by2 = box
            # box coords are in upscaled-crop px → downscale, then offset.
            fx1 = x1c + bx1 / upscale
            fy1 = y1c + by1 / upscale
            fx2 = x1c + bx2 / upscale
            fy2 = y1c + by2 / upscale
            idx = int(label_idx)
            if idx >= len(targets) or targets[idx] != cls:
                continue
            roi_dets.append(
                {
                    "class_name": cls,
                    "confidence": float(score),
                    "cx_px": (fx1 + fx2) / 2,
                    "cy_px": (fy1 + fy2) / 2,
                    "w_px": fx2 - fx1,
                    "h_px": fy2 - fy1,
                    "stamp_sec": stamp_sec,
                    "stamp_nanosec": stamp_nanosec,
                    "source": ROI_SOURCE_TAG,
                }
            )
        try:
            result_queue.put({"roi_detections": roi_dets}, timeout=5.0)
        except Exception:
            pass  # queue full — drop ROI result, never block the worker

    _inf_n = 0
    while True:
        try:
            item = frame_queue.get(timeout=1.0)
        except Exception:
            # Idle between frames: service any pending ROI request so a
            # triggered pass runs even when the camera stream lapses.
            if roi_queue is not None:
                try:
                    roi_req = roi_queue.get_nowait()
                except Exception:
                    roi_req = None
                if roi_req is not None:
                    _run_roi_pass(roi_req)
            continue

        if item is None:  # poison pill — shutdown
            break

        # ROI trigger request from the relay (dict payload, tuple = frame).
        if isinstance(item, dict) and item.get("_is_roi"):
            _run_roi_pass(item)
            continue

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
        self._mp_roi: mp.Queue = ctx.Queue(maxsize=4)
        self._ready_event = ctx.Event()

        self._logger.info(
            f"Detector: starting inference subprocess for targets: {targets}"
        )
        self._worker = ctx.Process(
            target=_inference_worker,
            args=(
                targets,
                self._mp_frames,
                self._mp_results,
                self._ready_event,
                self._mp_roi,
            ),
            daemon=True,
            name="detector_worker",
        )
        self._worker.start()

        # ── ROI second-pass trigger state (t_c6b546ac) ────────────────────────
        # Pending-candidate signal (read-only subscription on the tracker):
        # agent_node polls tracker.get_pending_candidates() and forwards the
        # class names via update_pending_classes(). No tracker behavior change.
        self._pending_classes: set[str] = set()
        # Last-known IMAGE position per class (from the detector's own
        # full-frame sightings) — the ROI crop anchor.
        self._last_class_img: dict[str, tuple[float, float]] = {}
        # Wall-clock of the last full-frame sighting per class.
        self._last_class_sighting: dict[str, float] = {}
        # Wall-clock of the last ROI pass per class (cooldown).
        self._last_roi_pass: dict[str, float] = {}
        # Most recent full frame (bytes, shape) — the ROI crop source.
        self._last_roi_frame: tuple[bytes, tuple] = (b"", (0, 0, 3))
        # Stats for gate attribution logging.
        self._roi_pass_count = 0
        self._roi_hit_count = 0

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
        # Keep the latest full frame as the ROI crop source (t_c6b546ac).
        with self._lock:
            self._last_roi_frame = (item[0], frame.shape)
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
        for q in (self._mp_frames, self._mp_results, self._mp_roi):
            try:
                while True:
                    q.get_nowait()
            except Exception:
                pass

        ctx = mp.get_context("spawn")
        self._ready_event = ctx.Event()
        self._worker = ctx.Process(
            target=_inference_worker,
            args=(
                self._targets,
                self._mp_frames,
                self._mp_results,
                self._ready_event,
                self._mp_roi,
            ),
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

                # ── ROI second-pass result (t_c6b546ac) ──────────────────────
                if "roi_detections" in result:
                    roi_dets = result["roi_detections"] or []
                    now_roi = _time.monotonic()
                    for d in roi_dets:
                        self._roi_hit_count += 1
                        det = DetectionResult(
                            class_name=d["class_name"],
                            confidence=d["confidence"],
                            cx_px=d["cx_px"],
                            cy_px=d["cy_px"],
                            w_px=d["w_px"],
                            h_px=d["h_px"],
                            stamp=(d["stamp_sec"], d["stamp_nanosec"]),
                            source=ROI_SOURCE_TAG,
                        )
                        # A ROI sighting IS a sighting: refresh the class's
                        # full-frame-clock so the trigger doesn't re-fire
                        # immediately (cooldown also bounds the cadence).
                        self._last_class_sighting[d["class_name"]] = now_roi
                        self._last_class_img[d["class_name"]] = (
                            d["cx_px"], d["cy_px"]
                        )
                        try:
                            self.detections.put_nowait(det)
                        except queue.Full:
                            try:
                                self.detections.get_nowait()
                                self.detections.put_nowait(det)
                            except queue.Empty:
                                pass
                    if roi_dets:
                        self._logger.info(
                            f"Detector: ROI pass hit — {len(roi_dets)} box(es) "
                            f"[{', '.join(d_['class_name'] + '(' + format(d_['confidence'], '.2f') + ')' for d_ in roi_dets)}] "
                            f"(passes={self._roi_pass_count}, hits={self._roi_hit_count})"
                        )
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
                now_ff = _time.monotonic()
                for d in dets:
                    self._last_class_sighting[d["class_name"]] = now_ff
                    self._last_class_img[d["class_name"]] = (
                        d["cx_px"], d["cy_px"]
                    )
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

                # ── ROI trigger evaluation (once per full-frame result) ──────
                self._maybe_roi_pass(now_ff)

            except Exception as exc:
                self._logger.error(
                    f"Detector: relay_loop result processing error — {type(exc).__name__}: {exc}"
                )
                continue

    def update_pending_classes(self, class_names) -> None:
        """Read-only subscription to the tracker's pending-candidate set
        (t_c6b546ac). The agent_node polls tracker.get_pending_candidates()
        each cycle and forwards the class names here; the tracker is never
        called back and its behavior is unchanged."""
        with self._lock:
            self._pending_classes = set(class_names)

    def _maybe_roi_pass(self, now: float) -> None:
        """If a pending class has been silent for K full-frame cycles (~2 s)
        and its cooldown expired, queue ONE upscaled-crop inference around
        its last-known image position. Rate-limited by ROI_COOLDOWN_S so ROI
        passes stay rare (≤1/class/4 s ⇒ ≤6% extra inference at 5 Hz)."""
        with self._lock:
            pend = set(self._pending_classes)
            last_frame = self._last_roi_frame
        if not pend:
            return
        fbytes, fshape = last_frame
        if not fbytes:
            return
        for cls in pend:
            img = self._last_class_img.get(cls)
            if img is None:
                continue  # no image anchor from this detector yet
            last_seen = self._last_class_sighting.get(cls)
            if last_seen is not None and (now - last_seen) * 5.0 < ROI_SILENT_CYCLES_K:
                # 5 Hz relay cadence: silence < K cycles → skip
                continue
            last_pass = self._last_roi_pass.get(cls)
            if last_pass is not None and (now - last_pass) < ROI_COOLDOWN_S:
                continue  # cooldown honored — ROI passes stay rare
            self._last_roi_pass[cls] = now
            self._roi_pass_count += 1
            req = {
                "_is_roi": True,
                "roi_class": cls,
                "cx_px": img[0],
                "cy_px": img[1],
                "upscale": ROI_UPSCALE,
                "frame_bytes": fbytes,
                "shape": fshape,
                "stamp_sec": 0,
                "stamp_nanosec": 0,
            }
            try:
                self._mp_roi.put_nowait(req)
            except Exception:
                # ROI queue full — skip this trigger; cooldown will retry later.
                self._last_roi_pass.pop(cls, None)
                self._roi_pass_count -= 1
                continue
            self._logger.info(
                f"Detector: ROI second-pass triggered for pending class "
                f"'{cls}' at ({img[0]:.0f},{img[1]:.0f}) "
                f"(passes={self._roi_pass_count})"
            )

    def stop(self) -> None:
        self._running = False
        try:
            self._mp_frames.put_nowait(None)  # poison pill
        except Exception:
            pass
        if self._worker.is_alive():
            self._worker.terminate()
            self._worker.join(timeout=3.0)
