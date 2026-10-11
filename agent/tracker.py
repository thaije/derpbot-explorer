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
  - Object confirmation requires (diverse sightings ≥ MIN_SIGHTINGS from robot
    poses ≥ MIN_POSE_DISTANCE apart) OR (FAST_CONFIRM_FRAMES close-range
    in-view frames — a parked robot can't produce diverse poses)
  - Anti-hallucination gate: before publishing, the track must clear
    OWL_CONF_THRESHOLD + CONFIDENCE_MARGIN on its best confidence and keep all
    sighting positions within STABILITY_RADIUS (rejects drifting low-margin
    OWLv2 hallucinations — FP backlog #40)
  - Cross-class position veto: different-class tracks within VETO_RADIUS are
    mutually-exclusive physical objects — both held back as ambiguous until
    centroids separate or one side's confidence wins by AMBIGUITY_MARGIN
    (prevents wrong-class submissions on real objects)
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
from pathlib import Path
import multiprocessing as mp
import queue
import threading
import time
import json
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from detector import DetectionResult, OWL_CONF_THRESHOLD

MATCH_RADIUS = 2.0          # metres — same class within this → same object
MIN_SIGHTINGS = 2           # require 2 sightings to filter single-frame noise
MIN_POSE_DISTANCE = 0.2     # robot must move ≥0.2m between diverse sightings
PUBLISH_RATE_HZ = 5.0       # rate to check for newly confirmed objects
REPUBLISH_SHIFT_M = 0.5     # republish if centroid moves more than this

# Sighting-stream recorder (t_451895af): default output when the agent is
# launched with --record-stream (no explicit path or auto mode). Relative to
# CWD — run_scenario-style invocations run from the explorer root, so this is
# run-adjacent; override with --record-stream <path>.
DEFAULT_STREAM_PATH = "results/streams/latest.jsonl"

# Anti-hallucination gate (FP backlog #40): OWLv2 hallucinations are low-margin
# detections whose projected position drifts as the robot moves (near-threshold
# bboxes re-anchored on different image patches via chained MATCH_RADIUS merges).
# A real object is re-sighted at (approximately) the same world position.
CONFIDENCE_MARGIN = 0.0     # (bisect t_9d9a4751) disabled: margin gate suppressed near-threshold real objects
STABILITY_RADIUS = 1.2      # max pairwise spread of sighting positions to confirm
                            # (real-object depth spread reaches ~1.2m at shallow
                            # view angles; 5/20 baseline FPs also sit 1.5–2.0m off,
                            # so this stays a march/re-anchor rejector, not tighter)

CANDIDATE_TIMEOUT_S = 180.0  # sim-seconds — discard pending candidate without 2nd sighting
CANDIDATE_PUBLISH_HZ = 2.0  # rate to send pending candidates to explorer

# Cross-class position veto (wrong-class submission repair): mutually-exclusive
# physical objects — two different-class tracks confirmed/pending within this
# radius means at most one is real (detector ran the wrong prompt on it).
# Both are flagged ambiguous and held back until evidence separates them.
VETO_RADIUS = 0.8           # metres — different-class tracks within this → ambiguous
AMBIGUITY_MARGIN = 0.10     # clear a veto by exceeding the other track's best conf by this

# Short-range fast-confirm: a robot parked in front of an object never produces
# two robot poses ≥ MIN_POSE_DISTANCE apart, so the diverse-sightings rule stalls
# forever at the detour goal (the "<1.0m candidates are skipped" filter prevents
# re-detouring). Confirm instead on N consecutive close-range in-view frames.
FAST_CONFIRM_DIST = 1.5     # metres — robot-object distance for in-view frames
FAST_CONFIRM_FRAMES = 3     # in-view frames needed (~0.5s at detector cadence)


@dataclass
class TrackedObject:
    track_id: str
    class_name: str
    # Accumulated world positions
    positions: list[tuple[float, float]] = field(default_factory=list)
    # Robot poses at time of each sighting
    robot_poses: list[tuple[float, float]] = field(default_factory=list)
    # Detector confidence per sighting (parallel to positions)
    confidences: list[float] = field(default_factory=list)
    # Cross-class veto: track_id of the mutually-exclusive track we're parked on
    ambiguous_with: Optional[str] = None
    # Consecutive close-range in-view frames (short-range fast-confirm, #3)
    in_view_frames: int = 0

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

    def max_confidence(self) -> float:
        """Best detector confidence seen on this track (0.0 if none recorded)."""
        return max(self.confidences) if self.confidences else 0.0

    def position_spread(self) -> float:
        """Max pairwise distance between sighting positions (0.0 for <2)."""
        if len(self.positions) < 2:
            return 0.0
        xs = [p[0] for p in self.positions]
        ys = [p[1] for p in self.positions]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    def passes_quality_gate(self, reference_conf: float = 0.0) -> bool:
        """
        Anti-hallucination gate applied at confirmation/publish time.

        Two generic checks against OWLv2 hallucination FPs (barely-above-threshold
        detections whose projected position drifts between sightings):
        1. margin — the track's best confidence must clear the detector threshold
           by CONFIDENCE_MARGIN (single-frame noise stays near threshold; real
           objects are seen well above it as the robot closes in);
        2. positional stability — sightings must cluster within STABILITY_RADIUS;
           a track that "marches" across the map via chained MATCH_RADIUS merges
           has a much larger spread.
        reference_conf is the other side's best confidence in a cross-class veto
        tie (0.0 = no comparison).
        """
        if self.max_confidence() < OWL_CONF_THRESHOLD + CONFIDENCE_MARGIN:
            return False
        if reference_conf > 0.0 and self.max_confidence() < reference_conf:
            return False
        if self.position_spread() > STABILITY_RADIUS:
            return False
        return True


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

    Also periodically sends "candidate" messages for objects with exactly 1
    sighting so the explorer can navigate toward them for re-detection.
    """
    import sys
    import time as _time
    objects: list[TrackedObject] = []
    next_id = 1
    lock = threading.Lock()
    _count = 0
    _last_candidate_send = _time.monotonic()
    CANDIDATE_SEND_INTERVAL = 1.0  # wall-seconds between candidate broadcasts

    def find_match(class_name: str, wx: float, wy: float) -> Optional[TrackedObject]:
        for obj in objects:
            if obj.class_name != class_name:
                continue
            if math.hypot(obj.world_x - wx, obj.world_y - wy) < MATCH_RADIUS:
                return obj
        return None

    def find_cross_class_neighbor(
        obj: TrackedObject,
    ) -> Optional[TrackedObject]:
        """Different-class track within VETO_RADIUS of obj's centroid, if any.

        Mutually-exclusive physical objects: two different-class tracks in the
        same spot means the detector fired the wrong prompt on one of them.
        """
        for other in objects:
            if other is obj or other.class_name == obj.class_name:
                continue
            if math.hypot(other.world_x - obj.world_x, other.world_y - obj.world_y) < VETO_RADIUS:
                return other
        return None

    def resolve_veto_on_append(
        obj: TrackedObject, neighbor: Optional[TrackedObject]
    ) -> None:
        """Update obj's ambiguity state from its current cross-class neighbor."""
        if neighbor is None:
            # No neighbor (never had one, or centroids separated) → ambiguous cleared
            if obj.ambiguous_with is not None:
                sys.stderr.write(
                    f"[tracker_worker] veto cleared {obj.track_id} "
                    f"(separated from {obj.ambiguous_with})\n"
                )
                obj.ambiguous_with = None
            return
        # Centroids still within veto radius — keep/refresh the flag pair.
        if obj.ambiguous_with != neighbor.track_id:
            sys.stderr.write(
                f"[tracker_worker] veto {obj.track_id}({obj.class_name}) "
                f"<-> {neighbor.track_id}({neighbor.class_name}) "
                f"at ({obj.world_x:.2f},{obj.world_y:.2f})\n"
            )
        obj.ambiguous_with = neighbor.track_id
        neighbor.ambiguous_with = obj.track_id

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
        confidence = item[3] if len(item) > 3 else 1.0

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
            obj.confidences.append(confidence)

            # Cross-class position veto (wrong-class submission repair):
            # flag/renew/clear ambiguity against the nearest different-class track.
            resolve_veto_on_append(obj, find_cross_class_neighbor(obj))

            # Short-range fast-confirm: while a close-range sighting arrives,
            # accumulate in-view frames (robot parked in front of the object
            # never yields two diverse poses at the detour goal).
            if (
                math.hypot(obj.world_x - rx, obj.world_y - ry) < FAST_CONFIRM_DIST
                and confidence >= OWL_CONF_THRESHOLD + CONFIDENCE_MARGIN
            ):
                obj.in_view_frames += 1

            # Confirmation: diverse sightings OR short-range in-view frames.
            # Only publish on first confirmation or centroid shift ≥ REPUBLISH_SHIFT_M
            # Anti-hallucination gate: confirmation quality checks must pass both on
            # first confirm and on republish (else a confirmed-then-drifting track
            # silently re-publishes a hallucinated centroid).
            if not (obj.is_confirmed() or obj.in_view_frames >= FAST_CONFIRM_FRAMES):
                continue
            neighbor = find_cross_class_neighbor(obj)
            if neighbor is not None:
                # Ambiguous pair: hold both back unless this side is clearly the
                # better explanation (higher best confidence by AMBIGUITY_MARGIN).
                reference = neighbor.max_confidence()
                if not obj.passes_quality_gate(reference + AMBIGUITY_MARGIN):
                    sys.stderr.write(
                        f"[tracker_worker] veto-held {obj.class_name} "
                        f"track={obj.track_id} vs {neighbor.track_id} "
                        f"max_conf={obj.max_confidence():.2f} "
                        f"ref={reference:.2f}\n"
                    )
                    continue
            if not obj.passes_quality_gate():
                sys.stderr.write(
                    f"[tracker_worker] gate-rejected {obj.class_name} "
                    f"track={obj.track_id} diversity={obj.diverse_sightings()} "
                    f"max_conf={obj.max_confidence():.2f} "
                    f"spread={obj.position_spread():.2f}m\n"
                )
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
                    "type": "confirmed",
                    "track_id": obj.track_id,
                    "class_name": obj.class_name,
                    "world_x": cx,
                    "world_y": cy,
                })
            except Exception:
                sys.stderr.write(
                    f"[tracker_worker] output_queue full, dropping {obj.class_name}\n"
                )

        # Periodically broadcast pending candidates (1 sighting, not yet confirmed)
        now_wall = _time.monotonic()
        if now_wall - _last_candidate_send >= CANDIDATE_SEND_INTERVAL:
            _last_candidate_send = now_wall
            with lock:
                for obj in objects:
                    if not obj.is_confirmed():
                        try:
                            output_queue.put_nowait({
                                "type": "candidate",
                                "track_id": obj.track_id,
                                "class_name": obj.class_name,
                                "world_x": obj.world_x,
                                "world_y": obj.world_y,
                                "sighting_count": obj.sighting_count,
                            })
                        except Exception:
                            pass


# ---------------------------------------------------------------------------
# Stream recorder (t_451895af) — write-only observer of the sighting stream
# ---------------------------------------------------------------------------

class StreamRecorder:
    """
    Records the tracker's raw sighting stream as JSONL (one line per sighting),
    for  offline replay  via  scripts/replay_tracker.py.

    Output format:
      line 1: {"type": "header", "spawn_pose": {...}, "map_pgm": ..., ...}
      n>1:    {"type": "sighting", "class_name", "confidence", "bbox":
               [cx, cy, w, h] (px), "x", "y" (map frame), "depth" (m, the
               median depth sample used for the projection, may be null),
               "rx", "ry", "ryaw", "t": sim-seconds}

    Recording is pure JSONL — no image dumps. write failures are swallowed
    so recording can never take the live pipeline down.
    """

    _WORLD_STATE = Path("/tmp/arst_worlds/world_state.json")

    def __init__(self, record_path: Optional[str], node: "Node"):
        self._logger = node.get_logger() if node is not None else None
        # Clock handle for sim-time stamps (weak-ish ref; node outlives recorder)
        self._clock_node = node
        self._fh = None
        self._n = 0
        if not record_path:
            return
        try:
            p = Path(record_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            self._fh = open(p, "a", buffering=1)  # line-buffered
            header = {
                "type": "header",
                "spawn_pose": self._read_spawn_pose(),
                "map_pgm": self._read_map_pgm(),
                "map_resolution": self._read_map_res(),
                "started_wall": time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()),
            }
            self._fh.write(json.dumps(header) + "\n")
            if self._logger is not None:
                self._logger.info(f"Tracker: recording sighting stream → {p}")
        except Exception as exc:
            self._fh = None
            if self._logger is not None:
                self._logger.warning(f"Tracker: stream recorder disabled ({exc})")

    @classmethod
    def _read_state(cls):
        try:
            return json.loads(cls._WORLD_STATE.read_text())
        except Exception:
            return {}

    @classmethod
    def _read_spawn_pose(cls) -> dict:
        return cls._read_state().get("spawn_pose", {"x": 0.0, "y": 0.0, "yaw": 0.0})

    @classmethod
    def _read_map_pgm(cls) -> str:
        return str(cls._read_state().get("map_pgm", ""))

    @classmethod
    def _read_map_res(cls) -> float:
        return float(cls._read_state().get("map_resolution", 0.5))

    @staticmethod
    def default_path() -> str:
        return DEFAULT_STREAM_PATH

    def stop(self) -> None:
        try:
            if self._fh is not None:
                self._fh.flush()
                self._fh.close()
                self._fh = None
        except Exception:
            pass

    def record(
        self,
        class_name: str,
        confidence: float,
        bbox: tuple[float, float, float, float],
        world_pos: tuple[float, float],
        robot_pose: tuple[float, float, float],
        depth: Optional[float] = None,
    ) -> None:
        if self._fh is None:
            return
        try:
            node = getattr(self, "_clock_node", None)
            sim_t = 0.0
            if node is not None:
                try:
                    sim_t = node.get_clock().now().nanoseconds / 1e9
                except Exception:
                    sim_t = time.time()
            cx, cy, w, h = bbox
            self._fh.write(json.dumps({
                "type": "sighting",
                "class_name": class_name,
                "confidence": round(float(confidence), 4),
                "bbox": [round(cx, 1), round(cy, 1), round(w, 1), round(h, 1)],
                "x": round(float(world_pos[0]), 3),
                "y": round(float(world_pos[1]), 3),
                "depth": None if depth is None else round(float(depth), 3),
                "rx": round(float(robot_pose[0]), 3),
                "ry": round(float(robot_pose[1]), 3),
                "ryaw": round(float(robot_pose[2]), 4),
                "t": round(sim_t, 3),
            }) + "\n")
            self._n += 1
            if self._n % 200 == 0 and self._logger is not None:
                self._logger.info(f"Tracker: stream recorded {self._n} sightings")
        except Exception:
            pass


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
        record_stream: "Optional[bool | str]" = None,  # None=auto, False=off, str=path
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

        # Stream recorder (write-only observer, t_451895af): JSONL of every
        # sighting the relay forwards to the tracker worker, plus a header line
        # with the environment data offline replay needs (spawn pose for the
        # scorer's map→world transform, PGM path for LOS checks). No behavior
        # change — the tracker consumes the same tuples as before.
        self._recorder = StreamRecorder(
            record_path=None if record_stream is False else (
                DEFAULT_STREAM_PATH if record_stream in (None, True) else str(record_stream)
            ),
            node=node,
        )

        # Tracked objects snapshot for status logging (updated by publisher thread)
        self._published_objects: list[dict] = []
        self._published_lock = threading.Lock()

        # Pending candidates: objects with < MIN_SIGHTINGS, exposed for explorer
        # Dict from track_id → {class_name, world_x, world_y, first_sim_t}
        self._pending_candidates: dict[str, dict] = {}
        self._pending_candidates_lock = threading.Lock()

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
            # project_with_depth() additionally returns the depth sample used —
            # needed by the recorder tap only; the worker receives the same
            # (class, world_pos, robot_pose) tuples as before (no behavior change).
            proj = self._projector.project_with_depth(det)
            if proj is None:
                self._logger.debug(f"Tracker relay: detection #{_count} projection failed")
                continue
            world_pos, depth_m = (proj[0], proj[1]), proj[2]

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
                robot_yaw = 2.0 * math.atan2(
                    t.transform.rotation.z, t.transform.rotation.w
                )
            except Exception as exc:
                self._logger.debug(f"Tracker relay: TF lookup failed for detection #{_count}: {exc}")
                continue

            # Stream recorder tap (write-only observer): log THIS sighting
            # exactly as the worker is about to receive it. Failures here must
            # never affect the live pipeline. project_with_depth() returns the
            # same world position plus the depth sample used (t_451895af).
            self._recorder.record(
                class_name=det.class_name,
                confidence=det.confidence,
                bbox=(det.cx_px, det.cy_px, det.w_px, det.h_px),
                world_pos=world_pos,
                robot_pose=(robot_pose[0], robot_pose[1], robot_yaw),
                depth=depth_m,
            )

            # Forward to worker process (confidence rides along for margin gating)
            try:
                self._input_queue.put_nowait(
                    (det.class_name, world_pos, robot_pose, det.confidence)
                )
                self._logger.debug(
                    f"Tracker relay: forwarded detection #{_count} "
                    f"({det.class_name} at {world_pos}) to worker"
                )
            except Exception:
                try:
                    self._input_queue.get_nowait()
                    self._input_queue.put_nowait(
                        (det.class_name, world_pos, robot_pose, det.confidence)
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

            msg_type = item.get("type", "confirmed")

            if msg_type == "candidate":
                with self._pending_candidates_lock:
                    tid = item["track_id"]
                    existing = self._pending_candidates.get(tid)
                    sim_t = self._node.get_clock().now().nanoseconds / 1e9
                    if existing is None:
                        self._pending_candidates[tid] = {
                            "class_name": item["class_name"],
                            "world_x": item["world_x"],
                            "world_y": item["world_y"],
                            "first_sim_t": sim_t,
                            "sighting_count": item["sighting_count"],
                        }
                        self._logger.info(
                            f"Tracker: NEW candidate {tid} {item['class_name']} "
                            f"at ({item['world_x']:.1f},{item['world_y']:.1f}) "
                            f"sightings={item['sighting_count']}"
                        )
                    else:
                        existing["world_x"] = item["world_x"]
                        existing["world_y"] = item["world_y"]
                        existing["sighting_count"] = item["sighting_count"]
                continue

            # confirmed message — publish to ROS
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

            # Remove from pending candidates if confirmed
            with self._pending_candidates_lock:
                removed = self._pending_candidates.pop(item["track_id"], None)
                if removed is not None:
                    self._logger.info(
                        f"Tracker: {item['track_id']} CONFIRMED (was candidate, now published)"
                    )

            with self._published_lock:
                self._published_objects.append(item)

    # ------------------------------------------------------------------
    # Status logging
    # ------------------------------------------------------------------

    def _log_status(self) -> None:
        with self._published_lock:
            published = list(self._published_objects)
        with self._pending_candidates_lock:
            candidates = dict(self._pending_candidates)
        cand_str = ""
        if candidates:
            cand_str = ", pending=" + ", ".join(
                f"{c['class_name']}({c['world_x']:.1f},{c['world_y']:.1f})"
                for c in candidates.values()
            )
        if published:
            self._logger.info(
                f"Tracker: confirmed={len(published)} "
                + ", ".join(
                    f"{o['class_name']}({o['world_x']:.1f},{o['world_y']:.1f})"
                    for o in published
                )
                + cand_str
            )

    def get_pending_candidates(self, sim_time: float) -> list[dict]:
        """
        Return pending candidates (1-sighting objects) that haven't timed out.

        Each candidate is a dict with: track_id, class_name, world_x, world_y.
        Candidates older than CANDIDATE_TIMEOUT_S sim-seconds are pruned.
        """
        with self._pending_candidates_lock:
            expired = [
                tid
                for tid, c in self._pending_candidates.items()
                if sim_time - c["first_sim_t"] > CANDIDATE_TIMEOUT_S
            ]
            for tid in expired:
                self._logger.info(
                    f"Tracker: candidate {tid} expired ("
                    f"age={sim_time - self._pending_candidates[tid]['first_sim_t']:.1f}s)"
                )
                del self._pending_candidates[tid]

            return [
                {
                    "track_id": tid,
                    "class_name": c["class_name"],
                    "world_x": c["world_x"],
                    "world_y": c["world_y"],
                }
                for tid, c in self._pending_candidates.items()
            ]

    def is_still_pending(self, track_id: str) -> bool:
        """Check whether a track_id is still a pending (unconfirmed) candidate."""
        with self._pending_candidates_lock:
            return track_id in self._pending_candidates

    def stop(self) -> None:
        self._running = False
        try:
            self._input_queue.put_nowait(None)  # poison pill
        except Exception:
            pass
        if self._worker.is_alive():
            self._worker.join(timeout=5.0)
        # Flush + close the sighting-stream recorder (t_451895af)
        self._recorder.stop()
        # Clear diagnostics from frontier_explorer (no longer needed)
        # (diagnostics were added earlier in this session)


# ---------------------------------------------------------------------------
# Policy-override bootstrap for OFFLINE REPLAY ONLY (scripts/replay_tracker.py,
# t_451895af). The worker runs in a spawn-context subprocess that re-imports
# this module with default constants; replay passes overrides through
# DERPBOT_POLICY_OVERRIDES ("NAME=VALUE,NAME=VALUE") so the spawned worker runs
# the requested policy. Env unset (live runs, tests) → nothing changes. Must
# sit AFTER every constant definition it patches — that's why it's at file end.
# ---------------------------------------------------------------------------
import os as _os
_overrides = _os.environ.get("DERPBOT_POLICY_OVERRIDES", "")
if _overrides:
    for _pair in _overrides.split(","):
        _pair = _pair.strip()
        if "=" in _pair and _pair.split("=", 1)[0] in globals():
            _name, _val = _pair.split("=", 1)
            _old = globals()[_name]
            if isinstance(_old, (int, float)) and not isinstance(_old, bool):
                _num = float(_val)
                globals()[_name] = (
                    int(_num)
                    if isinstance(_old, int) and _num == int(_num)
                    else _num
                )
