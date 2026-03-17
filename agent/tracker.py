"""
Tracker — merges detections from Detector+DepthProjector into persistent tracked
objects, then publishes vision_msgs/Detection2DArray to /derpbot_0/detections.

Strategy:
  - Each tracked object has a persistent string ID ("track_1", "track_2", …)
  - New detection matched by class + world position within MATCH_RADIUS metres
  - Running-mean world position averaged across all sightings
  - Object published only after ≥ MIN_SIGHTINGS sightings from robot poses
    that are ≥ MIN_POSE_DISTANCE apart (reduces false positives)
  - target filter: only publish objects whose class_name is in the mission targets list

Detection2DArray format expected by scorer:
  - results[0].hypothesis.class_id  — target class string
  - id                              — "track_N" string
  - bbox.center.position.x/y       — world x/y (published in PoseWithCovariance slot)
  - results[0].pose.pose.position.x — world x
  - results[0].pose.pose.position.y — world y
"""

from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    import tf2_ros
except ImportError:
    raise ImportError("tf2_ros not found — install ros-jazzy-tf2-ros")

from detector import Detector, DetectionResult
from depth_projector import DepthProjector

MATCH_RADIUS = 1.5          # metres — same class within this → same object (was 1.0)
MIN_SIGHTINGS = 1           # single confirmed sighting is sufficient — OWLv2 at 0.20 threshold is precise
MIN_POSE_DISTANCE = 0.0     # no diversity requirement with MIN_SIGHTINGS=1
PUBLISH_RATE_HZ = 5.0       # rate to check for newly confirmed objects
REPUBLISH_SHIFT_M = 0.5     # republish if centroid moves more than this after first publish


@dataclass
class TrackedObject:
    track_id: str
    class_name: str
    # Accumulated world positions
    positions: list[tuple[float, float]] = field(default_factory=list)
    # Robot poses at time of each sighting
    robot_poses: list[tuple[float, float]] = field(default_factory=list)

    @property
    def world_x(self) -> float:
        return sum(p[0] for p in self.positions) / len(self.positions)

    @property
    def world_y(self) -> float:
        return sum(p[1] for p in self.positions) / len(self.positions)

    @property
    def sighting_count(self) -> int:
        return len(self.positions)

    def diverse_sightings(self) -> int:
        """Count sightings that are ≥ MIN_POSE_DISTANCE apart from each other."""
        if not self.robot_poses:
            return 0
        diverse: list[tuple[float, float]] = [self.robot_poses[0]]
        for px, py in self.robot_poses[1:]:
            far_enough = all(
                math.hypot(px - dx, py - dy) >= MIN_POSE_DISTANCE
                for dx, dy in diverse
            )
            if far_enough:
                diverse.append((px, py))
        return len(diverse)

    def is_confirmed(self) -> bool:
        return self.diverse_sightings() >= MIN_SIGHTINGS


class Tracker:
    """
    Consumes from Detector.detections queue, projects to world via DepthProjector,
    maintains tracked objects, and publishes confirmed detections.
    """

    def __init__(
        self,
        node: Node,
        detector: Detector,
        depth_projector: DepthProjector,
        targets: list[str],
    ):
        self._node = node
        self._detector = detector
        self._projector = depth_projector
        self._targets = set(targets)
        self._logger = node.get_logger()

        self._objects: list[TrackedObject] = []
        self._next_id = 1
        self._lock = threading.Lock()
        self._last_published_pos: dict[str, tuple[float, float]] = {}  # track_id → last published (x, y)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        self._pub = node.create_publisher(
            Detection2DArray, "/derpbot_0/detections", 10
        )

        # Periodic publish timer
        node.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_confirmed)

        # Detection processing thread
        self._running = True
        t = threading.Thread(target=self._process_loop, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    # Robot pose in map frame
    # ------------------------------------------------------------------

    def _get_robot_pose_in_map(self) -> Optional[tuple[float, float]]:
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ):
            return None

    # ------------------------------------------------------------------
    # Detection processing
    # ------------------------------------------------------------------

    def _process_loop(self) -> None:
        while self._running and rclpy.ok():
            try:
                det: DetectionResult = self._detector.detections.get(timeout=0.5)
            except queue.Empty:
                continue

            if det.class_name not in self._targets:
                continue

            world_pos = self._projector.project(det)
            if world_pos is None:
                continue

            robot_pos = self._get_robot_pose_in_map()
            if robot_pos is None:
                continue

            wx, wy = world_pos
            rx, ry = robot_pos

            with self._lock:
                obj = self._find_match(det.class_name, wx, wy)
                if obj is None:
                    obj = TrackedObject(
                        track_id=f"track_{self._next_id}",
                        class_name=det.class_name,
                    )
                    self._next_id += 1
                    self._objects.append(obj)
                obj.positions.append((wx, wy))
                obj.robot_poses.append((rx, ry))

    def _find_match(
        self, class_name: str, wx: float, wy: float
    ) -> Optional[TrackedObject]:
        for obj in self._objects:
            if obj.class_name != class_name:
                continue
            if math.hypot(obj.world_x - wx, obj.world_y - wy) < MATCH_RADIUS:
                return obj
        return None

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_confirmed(self) -> None:
        """Publish confirmed tracks on first confirmation, and again if centroid
        shifts more than REPUBLISH_SHIFT_M since the last publish."""
        with self._lock:
            to_publish = []
            for o in self._objects:
                if not o.is_confirmed():
                    continue
                cx, cy = o.world_x, o.world_y
                last = self._last_published_pos.get(o.track_id)
                if last is None or math.hypot(cx - last[0], cy - last[1]) >= REPUBLISH_SHIFT_M:
                    to_publish.append(o)

        if not to_publish:
            return

        msg = Detection2DArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for obj in to_publish:
            det2d = Detection2D()
            det2d.id = obj.track_id

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.class_name
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = obj.world_x
            hyp.pose.pose.position.y = obj.world_y
            hyp.pose.pose.position.z = 0.0
            hyp.pose.pose.orientation.w = 1.0

            det2d.results.append(hyp)
            msg.detections.append(det2d)
            self._last_published_pos[obj.track_id] = (obj.world_x, obj.world_y)

        self._pub.publish(msg)

    def stop(self) -> None:
        self._running = False
