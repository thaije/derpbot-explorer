"""
Detector — open-vocabulary object detection using YOLOE26-S (or YOLO-World-S fallback).

Runs inference on the RGB camera stream (/derpbot_0/rgbd/image) at ~5 Hz on GPU 0.
Detected bounding boxes + class names are posted to an internal thread-safe queue
consumed by tracker.py.

Usage:
  detector = Detector(node, targets=["fire extinguisher", "hazard sign"])
  detector.start()
  # detector.detections is a queue.Queue of DetectionResult items
"""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# cv_bridge for ROS Image → numpy
try:
    from cv_bridge import CvBridge
except ImportError:
    raise ImportError("cv_bridge not found — install ros-jazzy-cv-bridge")

# Ultralytics for YOLOE26 / YOLO-World
try:
    from ultralytics import YOLO
    _ULTRALYTICS_AVAILABLE = True
except ImportError:
    _ULTRALYTICS_AVAILABLE = False

CONFIDENCE_THRESHOLD = 0.30
PROCESS_EVERY_N_FRAMES = 2      # process every 2nd frame → ~5 Hz from 10 Hz stream
GPU_DEVICE = 0                  # GPU index for inference


@dataclass
class DetectionResult:
    class_name: str
    confidence: float
    # Bounding box in pixel coords (centre x, centre y, width, height)
    cx_px: float
    cy_px: float
    w_px: float
    h_px: float
    # ROS timestamp (rclpy.time.Time) of the source image
    stamp: object


def _load_model(targets: list[str]):
    """
    Load YOLOE26-S with open-vocabulary classes set to `targets`.
    Falls back to YOLO-World-S if YOLOE26 is unavailable.
    """
    if not _ULTRALYTICS_AVAILABLE:
        raise ImportError("ultralytics package not installed — pip install ultralytics")

    # Try YOLOE26-S first (2025, zero-inference-overhead open-vocab)
    try:
        model = YOLO("yoloe-11s-seg.pt")   # will auto-download on first use
        model.set_classes(targets)
        model.to(f"cuda:{GPU_DEVICE}")
        return model, "yoloe26-s"
    except Exception:
        pass

    # Fallback: YOLO-World-S
    try:
        model = YOLO("yolov8s-worldv2.pt")
        model.set_classes(targets)
        model.to(f"cuda:{GPU_DEVICE}")
        return model, "yolo-world-s"
    except Exception as exc:
        raise RuntimeError(
            f"Could not load YOLOE26-S or YOLO-World-S. "
            f"Ensure ultralytics>=8.2 is installed and GPU is available. "
            f"Original error: {exc}"
        ) from exc


class Detector:
    """
    Subscribes to the RGB image topic and runs open-vocabulary detection.

    All results are posted to self.detections (a thread-safe queue.Queue).
    """

    def __init__(self, node: Node, targets: list[str]):
        self._node = node
        self._targets = targets
        self._logger = node.get_logger()
        self._bridge = CvBridge()

        self.detections: queue.Queue[DetectionResult] = queue.Queue(maxsize=100)

        self._model = None
        self._model_name = None
        self._frame_count = 0
        self._lock = threading.Lock()

        self._logger.info("Detector: loading model for targets: %s", targets)
        try:
            self._model, self._model_name = _load_model(targets)
            self._logger.info("Detector: loaded %s on cuda:%d", self._model_name, GPU_DEVICE)
        except Exception as exc:
            self._logger.error("Detector: model load failed — %s", exc)
            return

        # Subscribe to RGB image
        node.create_subscription(
            Image,
            "/derpbot_0/rgbd/image",
            self._image_cb,
            rclpy.qos.QoSProfile(depth=5),
        )

    def start(self) -> None:
        """No-op: detection runs in the ROS subscription callback thread."""
        if self._model is None:
            self._logger.error("Detector: model not loaded — detection disabled.")
        else:
            self._logger.info("Detector: active, processing every %dth frame.", PROCESS_EVERY_N_FRAMES)

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        with self._lock:
            self._frame_count += 1
            if self._frame_count % PROCESS_EVERY_N_FRAMES != 0:
                return
            if self._model is None:
                return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._logger.warning("Detector: cv_bridge conversion failed — %s", exc)
            return

        # Run inference
        try:
            results = self._model.predict(
                source=frame,
                conf=CONFIDENCE_THRESHOLD,
                device=GPU_DEVICE,
                verbose=False,
            )
        except Exception as exc:
            self._logger.warning("Detector: inference error — %s", exc)
            return

        stamp = msg.header.stamp

        for result in results:
            if result.boxes is None:
                continue
            boxes = result.boxes
            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                if conf < CONFIDENCE_THRESHOLD:
                    continue
                cls_id = int(boxes.cls[i])
                class_name = self._targets[cls_id] if cls_id < len(self._targets) else str(cls_id)
                xywh = boxes.xywh[i].tolist()   # [cx, cy, w, h] in pixels
                det = DetectionResult(
                    class_name=class_name,
                    confidence=conf,
                    cx_px=xywh[0],
                    cy_px=xywh[1],
                    w_px=xywh[2],
                    h_px=xywh[3],
                    stamp=stamp,
                )
                try:
                    self.detections.put_nowait(det)
                except queue.Full:
                    # Drop oldest, add newest
                    try:
                        self.detections.get_nowait()
                        self.detections.put_nowait(det)
                    except queue.Empty:
                        pass
