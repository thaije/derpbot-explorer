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

Tiled native-resolution inference (t_ea3db560):
  DERPBOT_TILED_INFERENCE=1 splits each incoming frame into 4 quadrant tiles
  and runs OWL-v2 per tile at NATIVE crop scale (each 320x240 tile is padded,
  not downscaled — the object pixel density the model sees doubles vs the
  full-frame 640x480 resize). Bboxes are remapped to full-frame coords so the
  detection stream is transparent to consumers; each detection carries
  source="tile_<q>" for attribution. Frames may also carry a boolean
  `tiled` flag through the mp queue (absent → not tiled).

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
import time
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

# ── Tiled native-resolution inference (t_ea3db560) ────────────────────────────
# Enabled with DERPBOT_TILED_INFERENCE=1 (read in BOTH processes: the relay
# uses the flag to tag detections that arrive without a source marker, the
# worker to select the tiled inference path). TILING_GRID=(2,2): 4 quadrant
# tiles over the incoming frame; remap stays exact for ragged edges because
# tile_grid() absorbs the remainder into the LAST row/column tile (v2 1080p:
# rows 360+360+360 = 1080, still exact — only odd frame sizes get ragged).
# DERPBOT_TILED_GRID="<gy>,<gx>" overrides the grid per process without
# touching code (measured on RTX 2070 SUPER, noise frames, offline):
#   full 1080p ≈ 139 ms; (2,2) 4 tiles ≈ 554 ms → 1.81 Hz FFE;
#   (3,2) 6 tiles ≈ 837 ms → 1.20 Hz FFE; (3,3) 9 tiles ≈ 1261 ms → 0.79 Hz
#   FFE — all three grids clear the 0.5 Hz FFE HALT bar.
TILING_GRID = (2, 2)
TILING_GRID_ENV = os.environ.get("DERPBOT_TILED_GRID", "")
TILING_ENABLED = os.environ.get("DERPBOT_TILED_INFERENCE", "0") == "1"


def _tiling_grid() -> tuple[int, int]:
    """DERPBOT_TILED_GRID="<gy>,<gx>" override; falls back to (2,2)."""
    if TILING_GRID_ENV.strip():
        try:
            gy, gx = (int(p) for p in TILING_GRID_ENV.split(","))
            if gy >= 1 and gx >= 1:
                return (gy, gx)
        except ValueError:
            pass
    return TILING_GRID
# Full-frame-equivalent wall of one tiled frame = GRID tiles × per-tile wall.
# Measured on RTX 2070 SUPER (GPU shared w/ Gazebo at RTF 2.0): full-frame
# ~161 ms, 4-tile ~645 ms ⇒ ~1.55 Hz FFE — above the 0.5 Hz HALT bar.
TILING_SOURCE_MARKERS = ["tile_q0", "tile_q1", "tile_q2", "tile_q3"]
# Grid-size-independent marker (grids beyond 2x2 produce q >= 4): the relay
# only pattern-matches the "tile_" prefix, so never truncate/clip on a
# fixed-size list (t_ea3db560 v2 re-plan: 1080p may run 3x2/3x3 grids).
TILING_SOURCE_MARKER_FMT = "tile_q{}"


def tile_grid(h: int, w: int, grid: tuple[int, int] = TILING_GRID):
    """Yield (q, y0, y1, x0, x1) tiles covering the frame in row-major order.

    Exact for (h % gy, w % gx) == 0 (camera frames qualify: 480/640 × (2,2));
    ragged edges are absorbed into the LAST row/column tile so coverage stays
    total (a tile only shrinks, never drops or overlaps).
    """
    gy, gx = grid
    ys = [(h * i) // gy for i in range(gy + 1)]
    xs = [(w * i) // gx for i in range(gx + 1)]
    tiles = []
    q = 0
    for iy in range(gy):
        for ix in range(gx):
            tiles.append((q, ys[iy], ys[iy + 1], xs[ix], xs[ix + 1]))
            q += 1
    return tiles


def remap_tile_box_to_full(tile, box):
    """(q, y0, y1, x0, x1) + xyxy box in tile-local px → full-frame xyxy px."""
    _, y0, _y1, x0, _x1 = tile
    x1, y1, x2, y2 = box
    return (x1 + x0, y1 + y0, x2 + x0, y2 + y0)

# Keyframe capture (t_85fe2e5c): the Tracker's KeyframeRecorder samples the
# latest raw camera frame through latest_frame(). The stash is a single tuple
# reference assignment in _image_cb (~zero cost, off the inference path); the
# recorder does its JPEG encode + downscale on its own background thread.
KEYFRAME_WIDTH = 320       # downscaled JPEG width (px)
KEYFRAME_HEIGHT = 240      # downscaled JPEG height (px)
KEYFRAME_QUALITY = 70      # JPEG quality (0-95)
KEYFRAME_ACTIVE_INTERVAL = 1.0   # min seconds between frames while a pending candidate exists
KEYFRAME_IDLE_INTERVAL = 10.0    # min seconds between sparse idle context frames
KEYFRAME_MAX_PER_RUN = 300       # hard cap: total JPEGs written per run


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
    # Attribution marker: "full_frame", "tile_<q>", or a mechanism tag
    # ("roi_second_pass"). Default keeps every pre-tiling consumer unchanged.
    source: str = "full_frame"


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

        def _infer_crop(crop_bgr):
            """One OWL-v2 inference on a BGR crop → list[dict] in CROP coords.

            Same single-stage pipeline as the non-tiled path; the caller owns
            coordinate remapping and source markers so this helper stays
            shared between both paths.
            """
            outputs = inputs = target_sizes = results = None
            try:
                rgb = _cv2.cvtColor(crop_bgr, _cv2.COLOR_BGR2RGB)
                pil_image = PILImage.fromarray(rgb)
                inputs = processor(text=queries, images=pil_image, return_tensors="pt")
                inputs = {
                    k: (v.half() if v.dtype == torch.float32 else v).to(
                        f"cuda:{GPU_DEVICE}"
                    )
                    for k, v in inputs.items()
                }
                target_sizes = torch.tensor(
                    [(crop_bgr.shape[0], crop_bgr.shape[1])],
                    dtype=torch.int32,
                    device=f"cuda:{GPU_DEVICE}",
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
            finally:
                del outputs, inputs, target_sizes, results
                torch.cuda.empty_cache()
            out = []
            for score, label_idx, box in zip(det_scores, det_labels, det_boxes):
                x1, y1, x2, y2 = box
                idx = int(label_idx)
                if idx >= len(targets):
                    continue
                out.append(
                    {
                        "class_name": targets[idx],
                        "confidence": float(score),
                        "cx_px": (x1 + x2) / 2,
                        "cy_px": (y1 + y2) / 2,
                        "w_px": x2 - x1,
                        "h_px": y2 - y1,
                    }
                )
            return out

        detections = []
        try:
            t_inf_start = time.perf_counter()
            if TILING_ENABLED:
                # ── Tiled native-resolution path (t_ea3db560) ────────────────
                # DERPBOT_TILED_GRID tiles, each inferred at native crop
                # scale; boxes remapped to full-frame coords so downstream
                # consumers see a single unchanged detection stream. Tile-local
                # detections are marked source="tile_<q>" for attribution;
                # tile-boundary dupes (object straddling an edge) are left to
                # tracker-side dedup.
                for tile in tile_grid(shape[0], shape[1], _tiling_grid()):
                    q, y0, y1_, x0, x1_ = tile
                    tile_dets = _infer_crop(frame[y0:y1_, x0:x1_])
                    for d in tile_dets:
                        fx1, fy1, fx2, fy2 = remap_tile_box_to_full(
                            tile,
                            (
                                d["cx_px"] - d["w_px"] / 2,
                                d["cy_px"] - d["h_px"] / 2,
                                d["cx_px"] + d["w_px"] / 2,
                                d["cy_px"] + d["h_px"] / 2,
                            ),
                        )
                        d["cx_px"] = (fx1 + fx2) / 2
                        d["cy_px"] = (fy1 + fy2) / 2
                        d["w_px"] = fx2 - fx1
                        d["h_px"] = fy2 - fy1
                        d["source"] = TILING_SOURCE_MARKER_FMT.format(q)
                        detections.append(d)
            else:
                detections = _infer_crop(frame)
                for d in detections:
                    d["source"] = "full_frame"
            t_inf_end = time.perf_counter()
        except Exception as exc:
            result_queue.put({"warning": f"inference error: {exc}"})
            continue

        for d in detections:
            d["stamp_sec"] = stamp_sec
            d["stamp_nanosec"] = stamp_nanosec

        _inf_n += 1
        # Tiling probe instrumentation (t_ea3db560 step 2): per-frame inference
        # wall every 5th frame so the FFE detection rate is measurable from
        # the agent logs during the dev probe.
        if _inf_n % 5 == 0:
            if TILING_ENABLED:
                result_queue.put({
                    "vram": f"inf#{_inf_n} tiled_wall={(t_inf_end - t_inf_start) * 1000:.0f}ms"
                })
            elif _inf_n % 100 == 0:
                result_queue.put({
                    "vram": f"inf#{_inf_n} fullframe_wall={(t_inf_end - t_inf_start) * 1000:.0f}ms"
                })

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

    def latest_frame(self) -> Optional[tuple]:
        """Latest processed camera frame as (BGR ndarray, cam_timestamp_s).

        Keyframe-capture plumbing (t_85fe2e5c — write-only observer): the
        Tracker's KeyframeRecorder samples this from its own background
        thread. Returns None before the first camera frame arrives. The
        ndarray is shared with the callback (no copy on the hot path) — the
        recorder must read/encode it before treating it as stable, which the
        single-assignment stash makes safe enough for diagnostic JPEGs.
        """
        return getattr(self, "_latest_frame", None)

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

        # Keyframe-capture stash (t_85fe2e5c — write-only observer): remember
        # the latest processed camera frame for the Tracker's KeyframeRecorder.
        # frame.copy() guarantees the recorder's background thread never sees a
        # buffer that the callback thread is about to reallocate (~1 ms cost,
        # off the inference path since the stash is post-processing).
        self._latest_frame = (frame.copy(), stamp.sec + stamp.nanosec / 1e9)

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
                        source=(
                            d["source"]
                            if "source" in d
                            else ("tile_?" if TILING_ENABLED else "full_frame")
                        ),
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
