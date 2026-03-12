#!/usr/bin/env python3
"""
Perception harness — Detector + DepthProjector + Tracker, no navigation.
Drive the robot manually; watch stdout for exactly what the pipeline sees.

Per-detection output format:
  [HH:MM:SS] ACTION  class_name       sim=0.45 world=(x.xx,y.yy) robot=(x.x,y.y) [track_N] status

ACTION:
  NEW   — first sighting of this object
  UPD   — update to existing track
  NOPOS — depth/TF lookup failed (no world position)
  SKIP  — class not in targets

Tracker state printed every 5 s.
Publishes confirmed detections to /derpbot_0/detections (same as full agent).

Usage:
    python3 agent/run_perception.py fire_extinguisher first_aid_kit hazard_sign
"""
from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time

import rclpy
from rclpy.node import Node
import tf2_ros
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from detector import Detector, DetectionResult
from depth_projector import DepthProjector

# ── Tracker parameters (mirror tracker.py) ────────────────────────────────────
MATCH_RADIUS = 1.0          # m — same class within this radius → same object
MIN_SIGHTINGS = 2           # diverse sightings before confirmed
MIN_POSE_DISTANCE = 0.5     # m — sightings must come from distinct robot poses


class _Track:
    def __init__(self, tid: str, cls: str, wx: float, wy: float, rx: float, ry: float):
        self.tid = tid
        self.cls = cls
        self.positions: list[tuple[float, float]] = [(wx, wy)]
        self.robot_poses: list[tuple[float, float]] = [(rx, ry)]

    @property
    def wx(self) -> float:
        return sum(p[0] for p in self.positions) / len(self.positions)

    @property
    def wy(self) -> float:
        return sum(p[1] for p in self.positions) / len(self.positions)

    def diverse_sightings(self) -> int:
        if not self.robot_poses:
            return 0
        diverse: list[tuple[float, float]] = [self.robot_poses[0]]
        for px, py in self.robot_poses[1:]:
            if all(math.hypot(px - dx, py - dy) >= MIN_POSE_DISTANCE for dx, dy in diverse):
                diverse.append((px, py))
        return len(diverse)

    def is_confirmed(self) -> bool:
        return self.diverse_sightings() >= MIN_SIGHTINGS

    def add(self, wx: float, wy: float, rx: float, ry: float) -> None:
        self.positions.append((wx, wy))
        self.robot_poses.append((rx, ry))


class PerceptionNode(Node):
    def __init__(self, targets: list[str]):
        super().__init__(
            "derpbot_perception",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
                )
            ],
        )
        self._targets = set(targets)
        self._tracks: list[_Track] = []
        self._next_id = 1
        self._lock = threading.Lock()

        # ── Perception pipeline ───────────────────────────────────────────────
        self._detector = Detector(self, targets=targets)
        self._projector = DepthProjector(self)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(
            Detection2DArray, "/derpbot_0/detections", 10
        )

        # ── Start ─────────────────────────────────────────────────────────────
        self._detector.start()

        t = threading.Thread(target=self._process_loop, daemon=True)
        t.start()

        self.create_timer(5.0, self._print_status)
        self.create_timer(1.0 / 5.0, self._publish_confirmed)

        print(
            f"\nTargets : {', '.join(targets)}\n"
            "Drive the robot — watching for detections...\n",
            flush=True,
        )
        self.get_logger().info(f"PerceptionNode ready. targets={targets}")

    # ── Robot pose ────────────────────────────────────────────────────────────

    def _robot_pose(self) -> tuple[float, float] | None:
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

    # ── Detection processing ──────────────────────────────────────────────────

    def _process_loop(self) -> None:
        while rclpy.ok():
            try:
                det: DetectionResult = self._detector.detections.get(timeout=0.5)
            except queue.Empty:
                continue

            ts = time.strftime("%H:%M:%S")

            if det.class_name not in self._targets:
                print(
                    f"[{ts}] SKIP   {det.class_name:25s}  sim={det.confidence:.3f}  "
                    f"(not a target)",
                    flush=True,
                )
                continue

            world_pos = self._projector.project(det)
            if world_pos is None:
                print(
                    f"[{ts}] NOPOS  {det.class_name:25s}  sim={det.confidence:.3f}  "
                    f"px=({det.cx_px:.0f},{det.cy_px:.0f})  "
                    f"bbox=({det.w_px:.0f}×{det.h_px:.0f})  — depth/TF fail",
                    flush=True,
                )
                continue

            wx, wy = world_pos
            robot_pos = self._robot_pose()
            if robot_pos is None:
                print(
                    f"[{ts}] NOROBOT {det.class_name:24s}  sim={det.confidence:.3f}  "
                    f"world=({wx:.2f},{wy:.2f})  — map TF not ready",
                    flush=True,
                )
                continue

            rx, ry = robot_pos

            with self._lock:
                track = self._find_match(det.class_name, wx, wy)
                if track is None:
                    track = _Track(
                        f"track_{self._next_id}", det.class_name, wx, wy, rx, ry
                    )
                    self._next_id += 1
                    self._tracks.append(track)
                    action = "NEW"
                else:
                    track.add(wx, wy, rx, ry)
                    action = "UPD"

                diverse = track.diverse_sightings()
                status = "✓ CONFIRMED" if track.is_confirmed() else f"({diverse}/{MIN_SIGHTINGS} diverse)"

            print(
                f"[{ts}] {action:3s}  {det.class_name:25s}  sim={det.confidence:.3f}  "
                f"world=({wx:.2f},{wy:.2f})  robot=({rx:.1f},{ry:.1f})  "
                f"[{track.tid}] {status}",
                flush=True,
            )

    def _find_match(self, cls: str, wx: float, wy: float) -> _Track | None:
        for t in self._tracks:
            if t.cls != cls:
                continue
            if math.hypot(t.wx - wx, t.wy - wy) < MATCH_RADIUS:
                return t
        return None

    # ── Status dump ───────────────────────────────────────────────────────────

    def _print_status(self) -> None:
        with self._lock:
            if not self._tracks:
                return
            snapshot = list(self._tracks)

        confirmed = [t for t in snapshot if t.is_confirmed()]
        print(
            f"\n{'─'*70}\n"
            f"Tracker: {len(snapshot)} tracks total, {len(confirmed)} confirmed\n",
            flush=True,
        )
        for t in snapshot:
            flag = "✓" if t.is_confirmed() else " "
            print(
                f"  [{flag}] {t.tid:10s}  {t.cls:25s}  @ ({t.wx:6.2f},{t.wy:6.2f})  "
                f"sightings={len(t.positions):3d}  diverse={t.diverse_sightings()}",
                flush=True,
            )
        print(f"{'─'*70}\n", flush=True)

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish_confirmed(self) -> None:
        with self._lock:
            confirmed = [t for t in self._tracks if t.is_confirmed()]

        if not confirmed:
            return

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for t in confirmed:
            d = Detection2D()
            d.id = t.tid
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = t.cls
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = t.wx
            hyp.pose.pose.position.y = t.wy
            hyp.pose.pose.position.z = 0.0
            hyp.pose.pose.orientation.w = 1.0
            d.results.append(hyp)
            msg.detections.append(d)

        self._pub.publish(msg)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._detector.stop()


def main():
    targets = sys.argv[1:] if len(sys.argv) > 1 else [
        "fire_extinguisher", "first_aid_kit", "hazard_sign",
    ]

    rclpy.init()
    node = PerceptionNode(targets)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
