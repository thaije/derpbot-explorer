"""
Tracker — merges detections into persistent tracked objects, publishes to /derpbot_0/detections.

Architecture (process-based to avoid GIL contention with Nav2/explorer):
  - Main process: relay thread reads (detection, world_pos, robot_pose) from
    the detector's relay, forwards to tracker_worker via mp.Queue.
  - tracker_worker (spawned process): receives detections + poses, runs
    tracking logic, sends confirmed detections back via output mp.Queue.
  - Main process: publisher thread reads confirmed detections from output
    queue and publishes to ROS.

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
import multiprocessing as mp
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from detector import DetectionResult

MATCH_RADIUS = 2.0          # metres — same class within this → same object
MIN_SIGHTINGS = 2           # require 2 sightings to filter single-frame noise
MIN_POSE_DISTANCE = 0.2     # robot must move ≥0.2m between diverse sightings
PUBLISH_RATE_HZ = 5.0       # rate to check for newly confirmed objects
REPUBLISH_SHIFT_M = 0.5     # republish if centroid moves more than this


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


# ---------------------------------------------------------------------------
# Worker process — runs in a separate Python process (no GIL shared with main)
# ---------------------------------------------------------------------------

def tracker_worker(
    targets: list[str],
    input_queue: mp.Queue,
    output_queue: mp.Queue,
    ready_event: mp.Event,
) -> None:
    """
    Runs in a spawned subprocess. Receives (class_name, world_pos, robot_pose)
    via input_queue, runs tracking logic, and sends confirmed detections back
    via output_queue.
    """
    import sys
    objects: list[TrackedObject] = []
    next_id = 1
    lock = threading.Lock()
    _count = 0

    def find_match(class_name: str, wx: float, wy: float) -> Optional[TrackedObject]:
        for obj in objects:
            if obj.class_name != class_name:
                continue
            if math.hypot(obj.world_x - wx, obj.world_y - wy) < MATCH_RADIUS:
                return obj
        return None

    # Track last-published centroid per track_id to avoid duplicate publishes
    last_published: dict[str, tuple[float, float]] = {}

    ready_event.set()
    sys.stderr.write("[tracker_worker] started, ready_event set\n")

    while True:
        try:
            item = input_queue.get(timeout=1.0)
        except Exception:
            continue
        if item is None:  # poison pill
            sys.stderr.write("[tracker_worker] received poison pill, exiting\n")
            break

        _count += 1
        class_name, world_pos, robot_pose = item[0], item[1], item[2]

        if class_name not in targets:
            continue

        wx, wy = world_pos
        rx, ry = robot_pose

        with lock:
            obj = find_match(class_name, wx, wy)
            if obj is None:
                obj = TrackedObject(
                    track_id=f"track_{next_id}",
                    class_name=class_name,
                )
                next_id += 1
                objects.append(obj)
            obj.positions.append((wx, wy))
            obj.robot_poses.append((rx, ry))

            # Only publish on first confirmation or centroid shift ≥ REPUBLISH_SHIFT_M
            if not obj.is_confirmed():
                continue
            cx, cy = obj.world_x, obj.world_y
            prev = last_published.get(obj.track_id)
            if prev is not None and math.hypot(cx - prev[0], cy - prev[1]) < REPUBLISH_SHIFT_M:
                continue

            last_published[obj.track_id] = (cx, cy)
            sys.stderr.write(
                f"[tracker_worker] confirmed {obj.class_name} "
                f"at ({cx:.1f}, {cy:.1f}) "
                f"sightings={obj.sighting_count}, diverse={obj.diverse_sightings()}\n"
            )
            try:
                output_queue.put_nowait({
                    "track_id": obj.track_id,
                    "class_name": obj.class_name,
                    "world_x": cx,
                    "world_y": cy,
                })
            except Exception:
                sys.stderr.write(
                    f"[tracker_worker] output_queue full, dropping {obj.class_name}\n"
                )


# ---------------------------------------------------------------------------
# Main-process Tracker class
# ---------------------------------------------------------------------------

class Tracker:
    """
    Spawns tracker_worker subprocess and manages the relay/publisher threads.

    Flow:
      detector relay → input_queue → tracker_worker (subprocess)
      tracker_worker → output_queue → publisher thread → ROS publish
    """

    def __init__(
        self,
        node: Node,
        detector: "Detector",       # forward-ref; resolved at runtime
        depth_projector: "DepthProjector",
        targets: list[str],
    ):
        self._node = node
        self._detector = detector
        self._projector = depth_projector
        self._targets = set(targets)
        self._logger = node.get_logger()

        self._last_published_pos: dict[str, tuple[float, float]] = {}

        # Queues for inter-process communication (spawn context)
        ctx = mp.get_context("spawn")
        self._input_queue: mp.Queue = ctx.Queue(maxsize=100)
        self._output_queue: mp.Queue = ctx.Queue(maxsize=100)
        self._ready_event: mp.Event = ctx.Event()

        # Spawn worker process
        self._worker = ctx.Process(
            target=tracker_worker,
            args=(targets, self._input_queue, self._output_queue, self._ready_event),
            daemon=True,
            name="tracker_worker",
        )
        self._worker.start()

        # Publisher thread: reads confirmed detections from output_queue, publishes to ROS
        self._running = True
        self._pub_thread = threading.Thread(
            target=self._publisher_loop, daemon=True, name="tracker_publisher"
        )
        self._pub_thread.start()

        self._pub = node.create_publisher(
            Detection2DArray, "/derpbot_0/detections", 10
        )

        # Relay thread: reads from detector's detections queue (via main-process
        # relay), projects to world, gets robot pose, forwards to worker.
        # This runs in a thread so the GIL is released during queue.get().
        self._relay_thread = threading.Thread(
            target=self._relay_loop, daemon=True, name="tracker_relay"
        )
        self._relay_thread.start()

        # Periodic status log (every 30s)
        node.create_timer(30.0, self._log_status)

        # Tracked objects snapshot for status logging (updated by publisher thread)
        self._published_objects: list[dict] = []
        self._published_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Relay: detector queue → project + robot pose → worker process
    # ------------------------------------------------------------------

    def _relay_loop(self) -> None:
        """Read from detector.detections, project to world, forward to worker."""
        import traceback
        _count = 0
        while self._running and rclpy.ok():
            try:
                det: DetectionResult = self._detector.detections.get(timeout=0.5)
            except queue.Empty:
                continue

            _count += 1
            self._logger.debug(f"Tracker relay: got detection #{_count}: {det.class_name}")

            # Project to world (uses TF2 — runs in main process, releases GIL
            # during the C++ TF2 lookup; GIL hold is just the Python math).
            world_pos = self._projector.project(det)
            if world_pos is None:
                self._logger.debug(f"Tracker relay: detection #{_count} projection failed")
                continue

            # Get robot pose (TF2 lookup in main process)
            try:
                from tf2_ros import Buffer, TransformListener
                if not hasattr(self, "_tf_buffer"):
                    self._tf_buffer = Buffer()
                    self._tf_listener = TransformListener(self._tf_buffer, self._node)
                t = self._tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time()
                )
                robot_pose = (t.transform.translation.x, t.transform.translation.y)
            except Exception as exc:
                self._logger.debug(f"Tracker relay: TF lookup failed for detection #{_count}: {exc}")
                continue

            # Forward to worker process
            try:
                self._input_queue.put_nowait(
                    (det.class_name, world_pos, robot_pose)
                )
                self._logger.debug(
                    f"Tracker relay: forwarded detection #{_count} "
                    f"({det.class_name} at {world_pos}) to worker"
                )
            except Exception:
                try:
                    self._input_queue.get_nowait()
                    self._input_queue.put_nowait(
                        (det.class_name, world_pos, robot_pose)
                    )
                    self._logger.debug(
                        f"Tracker relay: forwarded detection #{_count} (after dropping oldest)"
                    )
                except Exception as exc:
                    self._logger.warning(
                        f"Tracker relay: failed to forward detection #{_count}: {exc}"
                    )

    # ------------------------------------------------------------------
    # Publisher: worker output → ROS publish
    # ------------------------------------------------------------------

    def _publisher_loop(self) -> None:
        _count = 0
        while self._running and rclpy.ok():
            try:
                item = self._output_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            _count += 1
            self._logger.info(
                f"Tracker: publishing detection #{_count}: "
                f"{item['class_name']} at ({item['world_x']:.1f}, {item['world_y']:.1f})"
            )

            # Build and publish Detection2DArray
            msg = Detection2DArray()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            det2d = Detection2D()
            det2d.id = item["track_id"]

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = item["class_name"]
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = item["world_x"]
            hyp.pose.pose.position.y = item["world_y"]
            hyp.pose.pose.position.z = 0.0
            hyp.pose.pose.orientation.w = 1.0

            det2d.results.append(hyp)
            msg.detections.append(det2d)
            self._pub.publish(msg)

            self._last_published_pos[item["track_id"]] = (
                item["world_x"], item["world_y"]
            )

            with self._published_lock:
                self._published_objects.append(item)

    # ------------------------------------------------------------------
    # Status logging
    # ------------------------------------------------------------------

    def _log_status(self) -> None:
        with self._published_lock:
            published = list(self._published_objects)
        if published:
            self._logger.info(
                f"Tracker: confirmed={len(published)} "
                + ", ".join(
                    f"{o['class_name']}({o['world_x']:.1f},{o['world_y']:.1f})"
                    for o in published
                )
            )

    def stop(self) -> None:
        self._running = False
        try:
            self._input_queue.put_nowait(None)  # poison pill
        except Exception:
            pass
        if self._worker.is_alive():
            self._worker.join(timeout=5.0)
        # Clear diagnostics from frontier_explorer (no longer needed)
        # (diagnostics were added earlier in this session)
