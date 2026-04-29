"""
DepthProjector — converts a 2D bounding-box pixel coordinate into a world (x, y)
position using the depth image and TF2.

Camera model:
  - 640×480, 90° horizontal FOV
  - fx = fy = 320 / tan(45°) ≈ 320.0  (pinhole model)
  - cx = 320.0, cy = 240.0

Depth image:
  - Topic: /derpbot_0/rgbd/depth_image
  - Encoding: 32FC1 (metres)
  - Valid range: 0.3 m – 3.0 m (noisy / unreliable outside this)

Usage:
  projector = DepthProjector(node)
  world_pos = projector.project(det)   # returns (x, y) or None
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

try:
    from cv_bridge import CvBridge
except ImportError:
    raise ImportError("cv_bridge not found — install ros-jazzy-cv-bridge")

try:
    import tf2_ros
    import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform
except ImportError:
    raise ImportError("tf2_ros not found — install ros-jazzy-tf2-ros")

from detector import DetectionResult

# Camera intrinsics (computed from 90° HFOV + 640×480)
_H_FOV_DEG = 90.0
_WIDTH_PX = 640
_HEIGHT_PX = 480
_FX = _FY = (_WIDTH_PX / 2) / math.tan(math.radians(_H_FOV_DEG / 2))  # ≈ 320.0
_CX = _WIDTH_PX / 2.0    # 320.0
_CY = _HEIGHT_PX / 2.0   # 240.0

DEPTH_MIN = 0.30   # metres — discard closer than this
DEPTH_MAX = 5.00   # metres — sensor valid range up to 6 m; 5 m keeps noise manageable
DEPTH_SAMPLE_HALF = 2  # sample a (2*half+1)×(2*half+1) = 5×5 patch

# TF frames
WORLD_FRAME = "map"
CAMERA_FRAME = "camera_link"   # ← confirm from sim with ros2 run tf2_tools view_frames


class DepthProjector:
    """
    Maintains the latest depth image and projects detection bboxes to world coords.
    """

    def __init__(self, node: Node, create_subscriber: bool = True):
        self._node = node
        self._logger = node.get_logger()
        self._bridge = CvBridge()

        self._depth_image: Optional[np.ndarray] = None
        self._depth_stamp = None
        self._depth_lock = threading.Lock()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        if create_subscriber:
            _sensor_qos = QoSProfile(
                depth=1,  # Only keep latest to reduce queue overhead
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._last_cb_time = 0.0  # Wall-clock rate limiter
            self._cb_interval = 0.2  # 5 Hz callback rate limit
            node.create_subscription(
                Image,
                "/derpbot_0/rgbd/depth_image",
                self._depth_cb,
                _sensor_qos,
            )
        else:
            self._logger.info("DepthProjector: subscriber disabled for GIL probe.")

    # ------------------------------------------------------------------
    # Depth image callback
    # ------------------------------------------------------------------

    def _depth_cb(self, msg: Image) -> None:
        # Rate limiter: only process at ~5 Hz to reduce GIL contention
        import time
        now = time.monotonic()
        if now - self._last_cb_time < self._cb_interval:
            return
        self._last_cb_time = now

        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            self._logger.warning(f"DepthProjector: cv_bridge error — {exc}")
            return
        with self._depth_lock:
            self._depth_image = img
            self._depth_stamp = msg.header.stamp

    # ------------------------------------------------------------------
    # Projection
    # ------------------------------------------------------------------

    def project(self, det: DetectionResult) -> Optional[tuple[float, float]]:
        """
        Project the centre of `det`'s bounding box to world (x, y).

        Returns (world_x, world_y) or None if depth is invalid or TF lookup fails.
        """
        with self._depth_lock:
            depth_img = self._depth_image
            depth_stamp = self._depth_stamp

        if depth_img is None:
            return None

        # Bounding box centre in pixel coords
        u = int(round(det.cx_px))
        v = int(round(det.cy_px))

        # Sample a 5×5 patch; take median to reject noise
        h, w = depth_img.shape[:2]
        u_lo = max(0, u - DEPTH_SAMPLE_HALF)
        u_hi = min(w, u + DEPTH_SAMPLE_HALF + 1)
        v_lo = max(0, v - DEPTH_SAMPLE_HALF)
        v_hi = min(h, v + DEPTH_SAMPLE_HALF + 1)

        patch = depth_img[v_lo:v_hi, u_lo:u_hi]
        valid = patch[np.isfinite(patch) & (patch > DEPTH_MIN) & (patch < DEPTH_MAX)]

        if valid.size == 0:
            return None

        depth = float(np.median(valid))

        # Back-project to camera frame
        x_cam = (u - _CX) / _FX * depth
        y_cam = (v - _CY) / _FY * depth
        z_cam = depth

        # Build a PointStamped in camera frame
        pt_cam = PointStamped()
        pt_cam.header.frame_id = CAMERA_FRAME
        # Use latest-available TF (rclpy.time.Time() = time 0 = "latest").
        # Using the exact depth_stamp causes failures when sim-clock jitter
        # triggers a TF buffer clear and the stamp predates the earliest entry.
        pt_cam.header.stamp = rclpy.time.Time().to_msg()
        pt_cam.point.x = z_cam    # camera +Z is forward → world +X convention
        pt_cam.point.y = -x_cam   # camera +X is right → world -Y
        pt_cam.point.z = -y_cam   # camera +Y is down → world -Z (ignored for 2D)

        # Transform to world frame
        try:
            pt_world = self._tf_buffer.transform(
                pt_cam, WORLD_FRAME, timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ) as exc:
            self._logger.debug(f"DepthProjector: TF lookup failed — {exc}")
            return None

        return (pt_world.point.x, pt_world.point.y)
