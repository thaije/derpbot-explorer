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
    (split-child tracks are quarantined to TIGHT_MATCH_RADIUS instead, so a
    real sibling object cannot re-merge the halves the split separated)
  - Spread-triggered split (t_ca2ea380): when a same-class track's pairwise
    spread exceeds STABILITY_RADIUS and both 2-means halves carry a
    corroborated core (≥2 re-sighted positions within 0.5 m), the chained
    chimera is divided into two fresh quarantined tracks — the stability /
    confidence gates stop receiving chimera inputs instead of being loosened
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
import os
from pathlib import Path
import multiprocessing as mp
import queue
import threading
import time
import json
from dataclasses import dataclass, field
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from detector import (
    DetectionResult,
    OWL_CONF_THRESHOLD,
    KEYFRAME_WIDTH,
    KEYFRAME_HEIGHT,
    KEYFRAME_QUALITY,
    KEYFRAME_ACTIVE_INTERVAL,
    KEYFRAME_IDLE_INTERVAL,
    KEYFRAME_MAX_PER_RUN,
)

MATCH_RADIUS = 2.0          # metres — same class within this → same object
TIGHT_MATCH_RADIUS = 0.75   # metres — SPLIT-CHILD association quarantine.
                            # Real distinct objects can sit 0.96–1.17 m apart
                            # (seed4 desk FEs, seed3 desk FEs) while same-object
                            # sighting scatter is ≤0.3 m post-projection-fix
                            # (projection forensics, t_fd80a2f9) — so a 0.75 m
                            # quarantine separates sibling objects without ever
                            # breaking a real object's own re-sightings.
MIN_SIGHTINGS = 2           # require 2 sightings to filter single-frame noise
MIN_POSE_DISTANCE = 0.2     # robot must move ≥0.2m between diverse sightings
PUBLISH_RATE_HZ = 5.0       # rate to check for newly confirmed objects
REPUBLISH_SHIFT_M = 0.5     # republish if centroid moves more than this

# Sighting-stream recorder (t_451895af): default output when the agent is
# launched with --record-stream (no explicit path or auto mode). Relative to
# CWD — run_scenario-style invocations run from the explorer root, so this is
# run-adjacent; override with --record-stream <path>.
#
# Per-run rotation (t_96cab082): when DERPBOT_STREAM_SCOPE is set (by
# start_stack.sh from the scenario/seed/run context), the default filename
# gains that scope + a wall-clock stamp — e.g.
#   stream_office_easy_001_seed1run2_20261011T120000Z.jsonl
# — so successive benchmark runs never append into one shared file. Plain
# manual runs (no scope, no explicit path) keep the legacy latest.jsonl name,
# and an existing file is .1-rotated rather than appended to.
DEFAULT_STREAM_PATH = "results/streams/latest.jsonl"


def _rotate_existing(p: Path) -> None:
    """Move p out of append range: latest.jsonl → latest.jsonl.1 (…2, …3)."""
    try:
        if not p.exists():
            return
        n = 1
        while Path(f"{p}.{n}").exists():
            n += 1
        p.rename(Path(f"{p}.{n}"))
    except Exception:
        pass  # write-only observer: rotation failure must never block the run


def default_stream_path() -> str:
    """Default stream path with per-run naming (t_96cab082).

    DERPBOT_STREAM_SCOPE (set by start_stack.sh) is a run identifier like
    "easy_seed1run2" — appended to the stem plus a wall-clock stamp so each
    benchmark run gets a unique file. Without the scope the legacy
    latest.jsonl name is used, rotated out of the way if it already exists.
    """
    scope = os.environ.get("DERPBOT_STREAM_SCOPE", "").strip()
    if scope and scope != "auto":
        import re as _re
        safe = _re.sub(r"[^A-Za-z0-9._-]+", "_", scope)
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        return f"results/streams/stream_{safe}_{stamp}.jsonl"
    p = Path(DEFAULT_STREAM_PATH)
    _rotate_existing(p)
    return DEFAULT_STREAM_PATH

def keyframes_dir_for(stream_path: str) -> str:
    """Per-run keyframe directory for a stream path (t_85fe2e5c).

    results/streams/frames/<stream-stem>/ — rotates with the per-run stream
    naming of t_96cab082 (each run's stream stem gets its own frames dir).
    """
    stem = Path(stream_path).stem or "stream"
    return str(Path("results") / "streams" / "frames" / stem)


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

# Spread-triggered split (t_ca2ea380): the stability / confirmation gates stay;
# instead the association layer stops feeding them chimera inputs. When a
# same-class track's pairwise spread exceeds STABILITY_RADIUS, try to divide it
# into the two real objects it chained together:
SPLIT_MIN_SIGHTINGS = 4     # split needs at least this many sightings
SPLIT_SEPARATION_M = 0.5    # 2-means centroids must be at least this far apart
SPLIT_CORE_MIN = 2          # each half needs at least this many sightings...
SPLIT_CORE_RADIUS = 0.5     # ...clustered within this (a re-sighted object core)


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
    # Split-child quarantine (t_ca2ea380): this track was carved out of a
    # chimera by the spread-triggered split; it only accepts new sightings
    # within TIGHT_MATCH_RADIUS of its own centroid, so a sibling object that
    # sits < MATCH_RADIUS away cannot re-merge the halves.
    quarantined: bool = False

    def _has_core(self, idxs: list[int]) -> bool:
        """True if some sighting in idxs has ≥ SPLIT_CORE_MIN-1 fellow
        half-sightings within SPLIT_CORE_RADIUS (a re-sighted object core)."""
        pos = self.positions
        for i in idxs:
            xi, yi = pos[i]
            n = sum(
                1 for j in idxs
                if j != i and math.hypot(pos[j][0] - xi,
                                         pos[j][1] - yi) < SPLIT_CORE_RADIUS
            )
            if n >= SPLIT_CORE_MIN - 1:
                return True
        return False

    # Split evidence bookkeeping (t_d479428a): parallel to positions, marks
    # sightings that entered the track via an EXTERNAL-VERIFIED source
    # (Tracker.inject_external_sighting passes the marker on the worker
    # tuple — detector relays never carry a 5th element, so flag False).
    injected_flags: list[bool] = field(default_factory=list)

    def _has_injected(self) -> bool:
        return any(self.injected_flags)

    def try_split(
        self,
    ) -> Optional[tuple["TrackedObject", "TrackedObject"]]:
        """
        Spread-triggered 2-means split of a chimera track (t_ca2ea380).

        DATA ASSOCIATION, not confirmation policy: a real near-duplicate pair
        (distinct objects 0.96–1.17 m apart, same class) chains into ONE track
        via MATCH_RADIUS 2.0 matching + running-mean centroid drift. The
        stability gate then (correctly) rejects the spread — and BOTH real
        objects are lost. Fix at the source: divide the chained sightings.

        Pre-conditions on this track: ≥ SPLIT_MIN_SIGHTINGS sightings,
        pairwise spread > STABILITY_RADIUS. 2-means clustering on sighting
        positions (extremes-seeded, mirroring the replay forensics variant
        that won the A/B). Split happens only when BOTH halves carry a
        corroborated core (≥ SPLIT_CORE_MIN sightings within SPLIT_CORE_RADIUS
        of each other = two re-sighted real objects), so diffuse hallucination
        scatter is never promoted into a fresh track.

        Returns (this-half, other-half) as two fresh TrackedObjects on success
        (children are `quarantined`; the caller replaces self in `objects`),
        else None (track unchanged).
        """
        if len(self.positions) < SPLIT_MIN_SIGHTINGS:
            return None
        if self.position_spread() <= STABILITY_RADIUS:
            return None
        pts = self.positions
        n = len(pts)
        # seed centers at the two farthest sightings
        best_pair, bd = (0, 1), -1.0
        for a in range(n):
            for b in range(a + 1, n):
                d = math.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1])
                if d > bd:
                    bd, best_pair = d, (a, b)
        cA, cB = pts[best_pair[0]], pts[best_pair[1]]
        assign = [0] * n
        for _ in range(10):  # Lloyd iterations (forensics-identical)
            for i, p in enumerate(pts):
                assign[i] = (
                    0 if math.hypot(p[0] - cA[0], p[1] - cA[1])
                    <= math.hypot(p[0] - cB[0], p[1] - cB[1]) else 1
                )
            nA, nB = assign.count(0), assign.count(1)
            if not nA or not nB:
                return None
            cA = (sum(pts[i][0] for i in range(n) if assign[i] == 0) / nA,
                  sum(pts[i][1] for i in range(n) if assign[i] == 0) / nA)
            cB = (sum(pts[i][0] for i in range(n) if assign[i] == 1) / nB,
                  sum(pts[i][1] for i in range(n) if assign[i] == 1) / nB)
        idxA = [i for i, a in enumerate(assign) if a == 0]
        idxB = [i for i, a in enumerate(assign) if a == 1]
        if not idxA or not idxB:
            return None
        if math.hypot(cA[0] - cB[0], cA[1] - cB[1]) < SPLIT_SEPARATION_M:
            return None
        # both halves must be re-sighted real objects, not diffuse scatter…
        # UNLESS the track carries an externally verified sighting
        # (t_d479428a, v13-cv-fu: the VLM verifier injects through the SAME
        # input queue at VLM confidence; `injected_flags` marks those tuples).
        # An outlier-ALIGNED chimera (one early MATCH_RADIUS misassociation
        # anchors the running mean off-object; the re-sighted core mass sits
        # in the OTHER half with the track spread exceeding STABILITY_RADIUS)
        # can NEVER satisfy the both-halves rule — the singleton half has no
        # fellow sightings by construction — so the stability gate rejects
        # the whole track forever and every later fused verified sighting
        # keeps polishing an unconvertible centroid (seed4 gate run: one
        # 0.25-conf FAK outlier at t=8.6 anchored track_2 1.47 m off the
        # object; 53 near-GT sightings incl. one 0.98-conf VLM injection
        # fused in; core check 1/23 every tick; pending all 900 s). With
        # verified evidence on the track, one corroborated core half
        # suffices: the verified sighting certifies the mass side, the
        # discarded corner is quarantined and left to starve.
        # Injection-conditional: with no external sightings (injected_flags
        # empty — detector relays never carry the 5th tuple element) this is
        # behavior-identical to the classic both-halves rule. NOT a global
        # confirm-bar lowering (the v4/v6 lesson: global lowering collapsed
        # recall AND FP discipline).
        coreA, coreB = self._has_core(idxA), self._has_core(idxB)
        if not (coreA or coreB):
            return None  # no re-sighted object anywhere → diffuse scatter
        if not (coreA and coreB) and not self._has_injected():
            return None  # classic rule, unless externally verified evidence

        def child(idxs: list[int], suffix: str) -> "TrackedObject":
            c = TrackedObject(track_id=f"{self.track_id}{suffix}",
                              class_name=self.class_name)
            c.positions = [self.positions[i] for i in idxs]
            c.robot_poses = [self.robot_poses[i] for i in idxs]
            c.confidences = [self.confidences[i] for i in idxs]
            c.injected_flags = [self.injected_flags[i] for i in idxs]
            c.quarantined = True
            # re-earn confirmation state: fast-confirm counts restart at 0
            c.in_view_frames = 0
            return c

        return child(idxA, "A"), child(idxB, "B")

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
            # Split-child quarantine (t_ca2ea380): a quarantined track only
            # accepts sightings within TIGHT_MATCH_RADIUS of its own centroid —
            # a real sibling object < MATCH_RADIUS away cannot re-merge the
            # halves the split just separated (same-object scatter is ≤0.3 m,
            # distinct objects observed 0.96–1.17 m apart).
            radius = TIGHT_MATCH_RADIUS if obj.quarantined else MATCH_RADIUS
            if math.hypot(obj.world_x - wx, obj.world_y - wy) < radius:
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
        # Externally verified sighting marker (t_d479428a): the VLM verifier
        # appends a 5th tuple element True via Tracker.inject_external_sighting
        # — detector relays always send 4-tuples (relay code, agent/tracker.py
        # _relay_loop), so flag False there.
        injected = bool(item[4]) if len(item) > 4 else False

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
            obj.injected_flags.append(injected)

            # Spread-triggered split (t_ca2ea380): if this sighting pushed a
            # same-class track beyond STABILITY_RADIUS spread, try to divide
            # the chained sightings into the two real objects before ANY gate
            # sees the chimera. Fires at most once per chimera build-up (the
            # halves are fresh, quarantined tracks well below the spread bound).
            split = obj.try_split()
            if split is not None:
                objA, objB = split
                objects.remove(obj)
                objects.append(objA)
                objects.append(objB)
                sys.stderr.write(
                    f"[tracker_worker] split chimera {obj.track_id} "
                    f"({obj.class_name}, {obj.sighting_count} sightings, "
                    f"spread={obj.position_spread():.2f}m) -> "
                    f"{objA.track_id} ({objA.sighting_count}) + "
                    f"{objB.track_id} ({objB.sighting_count}), "
                    f"quarantine {TIGHT_MATCH_RADIUS}m\n"
                )
                # continue processing with the half this sighting belongs to
                obj = objA if objA.positions[-1] == (wx, wy) else objB

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
              plus the per-run camera intrinsics object
              {"camera": {"fx", "fy", "cx", "cy", "width", "height",
               "source"}} (v2 migration, t_4aa1e7ff) — replay/forensics
              re-derive the camera model per run from this; streams without
              a "camera" key are pre-v2 and replay at 640×480/90° v1.
      n>1:    {"type": "sighting", "class_name", "confidence", "bbox":
               [cx, cy, w, h] (px), "x", "y" (map frame), "depth" (m, the
               median depth sample used for the projection, may be null),
               "rx", "ry", "ryaw", "t": sim-seconds}

    Recording is pure JSONL — no image dumps. write failures are swallowed
    so recording can never take the live pipeline down.
    """

    _WORLD_STATE = Path("/tmp/arst_worlds/world_state.json")

    def __init__(self, record_path: Optional[str], node: "Node",
                 camera_model: "Optional[dict | Callable[[], dict]]" = None):
        self._logger = node.get_logger() if node is not None else None
        # Clock handle for sim-time stamps (weak-ish ref; node outlives recorder)
        self._clock_node = node
        self._fh = None
        self._n = 0
        if not record_path:
            return
        # Lazy open: the header line is written on the FIRST record() call, not
        # at construction (t_4aa1e7ff). At recorder construction the executor
        # hasn't spun yet, so a camera_info-driven model can't have arrived —
        # every run 2.0.0+ would otherwise get a 'v1_fallback' header even in
        # v2 scenes. Written-on-first-sighting the snapshot reflects the live
        # model (first sighting is always ≥ seconds after camera_info; runs
        # without camera_info keep the loud DepthProjector fallback warning).
        # The header remains the physically first line of the file.
        self._p = Path(record_path)
        self._camera_model = camera_model
        try:
            self._p.parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self._fh = None
            if self._logger is not None:
                self._logger.warning(f"Tracker: stream recorder disabled ({exc})")

    def _open_and_write_header(self) -> None:
        """Materialise the file + header line (deferred from __init__)."""
        try:
            self._fh = open(self._p, "a", buffering=1)  # line-buffered
            header = {
                "type": "header",
                "spawn_pose": self._read_spawn_pose(),
                "map_pgm": self._read_map_pgm(),
                "map_resolution": self._read_map_res(),
                "started_wall": time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()),
            }
            # Per-run camera intrinsics (v2 migration, t_4aa1e7ff): snapshot of
            # the projector's live model so replay/forensics can always
            # re-derive the camera model for THIS run. A zero-arg callable is
            # evaluated HERE (first sighting, executor live); a dict is used
            # as-is (pre-v2 callers/tests). "source" records a fallback if the
            # model is still the static v1 pinhole at this point.
            if self._camera_model is not None:
                model = (self._camera_model() if callable(self._camera_model)
                         else dict(self._camera_model))
                if model is not None:
                    header["camera"] = dict(model)
            self._fh.write(json.dumps(header) + "\n")
            if self._logger is not None:
                self._logger.info(f"Tracker: recording sighting stream → {self._p}")
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
        return default_stream_path()

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
            if getattr(self, "_open_failed", False) or not hasattr(self, "_p"):
                return  # recording off (no path) or open already failed once
            self._open_and_write_header()
            if self._fh is None:
                self._open_failed = True
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


class KeyframeRecorder:
    """
    Trigger-gated keyframe capture (t_85fe2e5c) — write-only observer.

    Trigger set (t_85fe2e5c + orchestrator addendum):
      - pending-candidate active: >=1 unconfirmed candidate in the tracker's
        pending dict (read-only peek at the SAME signal frontier_explorer
        uses via tracker.get_pending_candidates() — no tracker policy change)
        -> at most 1 frame per second;
      - candidate-driven navigation phases (detour toward / hold near a
        candidate, signalled read-only by the explorer via
        frontier_explorer.set_capture_hook()) -> at most 1 frame per second;
      - idle (none of the above): sparse context, at most 1 frame per 10s.
        Kept (vs skipping entirely) because a 320x240 q70 JPEG is ~25 KB ->
        2.5 KB/s worst case and idle frames let offline forensics align
        "in-FOV-but-undetected" windows with the surrounding exploration
        phase and robot pose.

    While any trigger is active the phase is keyframed CONTINUOUSLY at the
    1 Hz ceiling (not just when the pending list is non-empty at poll time),
    so "the moment the robot parks wrong" is captured even if the candidate
    gets confirmed/expired mid-poll. Idle frames stop once the run is within
    10 frames of the hard cap so context frames can never crowd out the
    candidate-triggered budget. Hard cap per run (KEYFRAME_MAX_PER_RUN).

    Output naming (addendum): results/streams/frames/<stream-stem>/
      f_<simtime>.jpg — sim-time so filenames cross-reference sighting-stream
      lines ("t" field) and result-JSON timelines exactly. Per-run dir rotates
      with the per-run stream naming of t_96cab082.

    Zero effect on detection latency: the camera frame is a single tuple
    reference stashed by Detector._image_cb (detector.latest_frame()); all
    downscale/JPEG/disk work happens on THIS recorder's background thread,
    never on the inference hot path. Every failure is swallowed — recording
    can never take the live pipeline down.
    """

    def __init__(
        self,
        frames_dir: str,
        detector: "Detector",
        pending_provider,   # Callable[[], list[dict]] — read-only pending candidates
        clock_node: "Node",
        extra_trigger_provider=None,  # Callable[[], bool] — explorer detour/hold phase
    ):
        self._frames_dir = Path(frames_dir)
        self._detector = detector
        self._pending_provider = pending_provider
        self._extra_trigger_provider = extra_trigger_provider
        self._clock_node = clock_node
        self._logger = clock_node.get_logger() if clock_node is not None else None
        self._count = 0
        self._last_write_monotonic: Optional[float] = None
        self._last_written_sim: Optional[float] = None
        self._running = True
        try:
            self._frames_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            self._running = False
            if self._logger is not None:
                self._logger.warning("Tracker: keyframe recorder disabled (mkdir failed)")

        self._thread: Optional[threading.Thread] = None
        self._cv2 = None
        if self._running:
            try:
                import cv2
                self._cv2 = cv2  # fail fast before starting the thread
                self._thread = threading.Thread(
                    target=self._loop, daemon=True, name="keyframe_recorder"
                )
                self._thread.start()
            except ImportError:
                self._running = False
                if self._logger is not None:
                    self._logger.warning(
                        "Tracker: keyframe recorder disabled (cv2 unavailable in agent process)"
                    )
            except Exception as exc:
                self._running = False
                if self._logger is not None:
                    self._logger.warning(f"Tracker: keyframe recorder disabled ({exc})")

    def _sim_time(self) -> float:
        node = self._clock_node
        if node is None:
            return time.time()
        try:
            return node.get_clock().now().nanoseconds / 1e9
        except Exception:
            return time.time()

    def _loop(self) -> None:
        while self._running:
            if self._count >= KEYFRAME_MAX_PER_RUN:
                break  # hard cap reached — stop the thread entirely
            try:
                pending = self._pending_provider() or []
            except Exception:
                pending = []
            extra = False
            if self._extra_trigger_provider is not None:
                try:
                    extra = bool(self._extra_trigger_provider())
                except Exception:
                    extra = False
            active = bool(pending) or extra
            interval = KEYFRAME_ACTIVE_INTERVAL if active else KEYFRAME_IDLE_INTERVAL

            now = time.monotonic()
            due = (
                self._last_write_monotonic is None
                or (now - self._last_write_monotonic) >= interval
            )
            if due:
                # Idle context frames stop near the cap so they can never
                # crowd out candidate-triggered keyframes.
                if active or self._count < KEYFRAME_MAX_PER_RUN - 10:
                    self._last_write_monotonic = now
                    self._write_one(sim_time=self._sim_time())
            time.sleep(0.2)

    def _write_one(self, sim_time: float) -> None:
        try:
            frame_item = self._detector.latest_frame()
            if frame_item is None:
                return  # no camera frame yet
            frame, cam_t = frame_item
            # Sim-time name collision guard (addendum): filename must
            # cross-reference the stream "t" field exactly — skip the rare
            # <=100 ms repeat rather than overwrite an earlier frame.
            if (
                self._last_written_sim is not None
                and abs(sim_time - self._last_written_sim) < 0.1
            ):
                return
            h, w = frame.shape[:2]
            # Full-res option (t_b0b71441): sandbox v2's 1080p hypothesis is
            # invisible in the 320-wide keyframe corpus (the resolution is
            # discarded before the bake-off instrument runs), so an env flag
            # lets instrument runs store detector-input-resolution frames.
            small = frame
            if os.environ.get("DERPBOT_KEYFRAME_FULLRES") != "1":
                scale = min(KEYFRAME_WIDTH / w, KEYFRAME_HEIGHT / h, 1.0)
                if scale < 1.0:
                    small = self._cv2.resize(
                        frame, (int(w * scale), int(h * scale)),
                        interpolation=self._cv2.INTER_AREA,
                    )
            ok, buf = self._cv2.imencode(
                ".jpg", small,
                [int(self._cv2.IMWRITE_JPEG_QUALITY), KEYFRAME_QUALITY],
            )
            if not ok:
                return
            out = self._frames_dir / f"f_{sim_time:.3f}.jpg"
            buf.tofile(str(out))  # single write, off the hot path
            self._count += 1
            self._last_written_sim = sim_time
            if self._count == 1 and self._logger is not None:
                self._logger.info(
                    f"Tracker: keyframe capture active (1Hz while candidates "
                    f"pending or in candidate detour/hold, {KEYFRAME_IDLE_INTERVAL:.0f}s "
                    f"idle, cap {KEYFRAME_MAX_PER_RUN}) -> {self._frames_dir}"
                )
        except Exception:
            pass  # write-only observer: never take the live pipeline down

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)


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
        # t_96cab082: the default path is per-run rotated (scoped filename
        # under the benchmark stack, .N-rotated legacy name for manual runs).
        self._recorder = StreamRecorder(
            record_path=None if record_stream is False else (
                default_stream_path() if record_stream in (None, True) else str(record_stream)
            ),
            node=node,
            # Per-run intrinsics header (v2 migration, t_4aa1e7ff): the
            # recorder snapshots the projector's CURRENT camera model when it
            # writes the header line (deferred to the FIRST sighting — at
            # construction the executor hasn't spun, so camera_info could not
            # have arrived yet; a construction-time snapshot would always be
            # the v1 fallback). The supplier keeps the model live to that
            # moment; its "source" field records a fallback if it is still one.
            camera_model=(
                self._projector.get_camera_model
                if hasattr(self._projector, "get_camera_model") else None
            ),
        )

        # Pending candidates: objects with < MIN_SIGHTINGS, exposed for explorer
        # Dict from track_id → {class_name, world_x, world_y, first_sim_t}
        # (initialized BEFORE the keyframe recorder below — its background
        # thread starts polling _peek_pending_candidates immediately.)
        self._pending_candidates: dict[str, dict] = {}
        self._pending_candidates_lock = threading.Lock()

        # Keyframe-capture trigger extensions (t_85fe2e5c addendum): the
        # explorer registers a read-only callback here that returns True while
        # the robot is navigating toward / holding at a candidate detour goal.
        # Write-only observer: a callback returning False/True never changes
        # explorer decisions.
        self._keyframe_extra_trigger = None

        # Keyframe capture (t_85fe2e5c — write-only observer): trigger-gated
        # downscaled JPEGs. Trigger = >=1 pending candidate (read-only peek at
        # _pending_candidates, same signal get_pending_candidates() exposes —
        # NO policy change) OR explorer-set candidate detour/hold phase
        # (addendum); idle -> 1 frame/10s sparse context. Frames dir rotates
        # with the per-run stream naming. Zero effect on detection latency:
        # encode/write runs on the recorder's own background thread.
        _record = record_stream
        if _record is None or _record is True:
            _stream_path = default_stream_path()
        elif _record is False:
            _stream_path = DEFAULT_STREAM_PATH
        else:
            _stream_path = str(_record)
        self._keyframe_recorder: Optional[KeyframeRecorder] = None
        if _record is not False:
            self._keyframe_recorder = KeyframeRecorder(
                frames_dir=keyframes_dir_for(_stream_path),
                detector=detector,
                pending_provider=self._peek_pending_candidates,
                clock_node=node,
            )

        # Tracked objects snapshot for status logging (updated by publisher thread)
        self._published_objects: list[dict] = []
        self._published_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Keyframe-capture trigger plumbing (read-only)
    # ------------------------------------------------------------------

    def _peek_pending_candidates(self) -> list[dict]:
        """Read-only snapshot of the pending-candidate dict.

        Same signal get_pending_candidates() exposes to the explorer, minus
        the expiry pruning side effect (capture must never mutate tracker
        state). Never blocks the publisher thread for long: lock held only
        for the dict copy.
        """
        with self._pending_candidates_lock:
            return list(self._pending_candidates.values())

    def set_keyframe_extra_trigger(self, provider) -> None:
        """Register explorer detour/hold hook (t_85fe2e5c addendum).

        provider() -> True while a candidate-driven navigation phase (detour
        toward / hold near a candidate) is active. Read-only from the
        explorer's side: the recorder only CALLS it.
        """
        self._keyframe_extra_trigger = provider
        if self._keyframe_recorder is not None:
            self._keyframe_recorder._extra_trigger_provider = provider

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

    def has_pending_candidates(self) -> bool:
        """Read-only: any unconfirmed candidate right now (no expiry pruning)."""
        with self._pending_candidates_lock:
            return bool(self._pending_candidates)

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
        # Stop the keyframe-capture thread (t_85fe2e5c)
        if self._keyframe_recorder is not None:
            self._keyframe_recorder.stop()

    # ------------------------------------------------------------------
    # External sighting injection (v13-cv VLM verifier, t_8ed4eb97)
    # ------------------------------------------------------------------

    def inject_external_sighting(
        self,
        class_name: str,
        world_pos: tuple[float, float],
        robot_pose: tuple[float, float, float],
        confidence: float,
        source: str = "vlm_verify",
    ) -> bool:
        """Inject ONE sighting from a non-detector source into the SAME
        pipeline the detector relay feeds (all existing gates apply).

        Used by agent/vlm_verifier.py to publish a verifier-gated sighting at
        VLM confidence (v13-cv): the sighting rides the standard input_queue
        → tracker_worker path, so association, split, veto, stability and
        confirm gates treat it exactly like a detector sighting. The stream
        recorder taps it too (provenance `source` kept for stream forensics).

        Returns True when the tuple was enqueued.
        """
        rx, ry, ryaw = robot_pose
        try:
            self._recorder.record(
                class_name=class_name,
                confidence=confidence,
                bbox=(0.0, 0.0, 0.0, 0.0),
                world_pos=world_pos,
                robot_pose=(rx, ry, ryaw),
                depth=None,
            )
        except Exception:
            pass  # recorder is a write-only observer
        try:
            # t_d479428a: 5th tuple element marks this as an externally
            # verified sighting (the tracker's split-evidence gate reads it;
            # detector-relay tuples have length 4 → not injected).
            self._input_queue.put_nowait(
                (class_name, world_pos, (rx, ry), confidence, True)
            )
            return True
        except Exception:
            try:
                self._input_queue.get_nowait()
                self._input_queue.put_nowait(
                    (class_name, world_pos, (rx, ry), confidence, True)
                )
                return True
            except Exception:
                return False


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
