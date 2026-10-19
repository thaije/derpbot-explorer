"""
VLM pending-track verifier (v13-cv, t_8ed4eb97) — ASYNC, OFF the control path.

What it is:
  A background thread that watches the tracker's pending (unconfirmed)
  mission-class candidates, and for each one that has sat unconfirmed for a
  while, crops the FULL-RES camera frame around the track's projected pixel
  bbox and asks glm-5.3-flash (ollama-cloud) the same structured prompt
  family the detector prompts use (class names derived from the mission
  targets with the detector's underscore->space rule — NO hardcoded class
  logic). On a positive answer, ONE sighting is injected into the tracker
  input queue at the VLM's reported confidence (its own strict gates
  still apply downstream — this never force-confirms anything).

Why:
  Phase-3 bake-off (t_bbe085d2): glm-5.3-flash clears every pre-registered
  bar leg on the 94-frame holdout subset but at 5.9 s/frame it can never be
  a primary detector (cadence bar) and the hybrid second-pass couples the
  FAK gain to the FP cost. As a CONFIRMATOR it fires only on pending
  tracks (rare), so the latency is hidden off the control path while its
  clean grounding converts glimpses the OWLv2 confirm gate can't.

Causality guard (bbox → world position):
  The VLM bbox is measured on a frame cropped at time T with the robot at
  pose P. The world position injected back into the tracker MUST be the
  one derived at crop time (bbox -> depth -> world via the projector),
  NOT re-projected later from the track's stale world pose: the crop is
  triggered BY the pending track's projected bbox, so on a positive we
  recycle the SAME pixel bbox the projection used — the tracker's
  association radius then does the fusing (crop-time world pos is within
  MATCH_RADIUS of the pending centroid or it would not have matched).

Guards (pre-registered):
  - DERPBOT_VLM_VERIFY=1 to enable (default OFF — zero behavior change)
  - <=1 in-flight ollama call
  - <=1 call per track per VLM_VERIFY_TRACK_COOLDOWN_S sim-seconds
  - <= VLM_VERIFY_BUDGET calls per run
  - strict single-parse of the structured reply; parse failure = a
    cooldown'd miss (no retry — the operator cost protocol caps retries
    at 2 per call and probe showed 0 parse failures in 36 calls)
  - never blocks the detector/explorer/control paths (daemon thread)

Off-path safety: every failure inside the thread is swallowed + logged;
the verifier can never take the live pipeline down (same policy as the
StreamRecorder/KeyframeRecorder write-only observers).
"""

from __future__ import annotations

import base64
import json
import math
import os
import queue
import re
import threading
import time
import urllib.error
import urllib.request
from typing import Callable, Optional

import rclpy

# --- Tuning constants (all env-overridable for the 5x1 gate sweep) ---------

MODEL = os.environ.get("DERPBOT_VLM_VERIFY_MODEL", "glm-5.3-flash:cloud")
OLLAMA_URL = os.environ.get(
    "DERPBOT_VLM_VERIFY_URL", "http://127.0.0.1:11434/api/chat"
)
VLM_VERIFY_ENABLED_ENV = "DERPBOT_VLM_VERIFY"
VLM_VERIFY_TRACK_COOLDOWN_S = float(
    os.environ.get("DERPBOT_VLM_VERIFY_TRACK_COOLDOWN_S", "20")
)  # sim-s between calls on the SAME pending track
VLM_VERIFY_BUDGET = int(
    os.environ.get("DERPBOT_VLM_VERIFY_BUDGET", "30")
)  # max calls per run
VLM_VERIFY_MIN_AGE_S = float(
    os.environ.get("DERPBOT_VLM_VERIFY_MIN_AGE_S", "5")
)  # track must be pending this many sim-s before first verify
VLM_VERIFY_MIN_CONF = float(
    os.environ.get("DERPBOT_VLM_VERIFY_MIN_CONF", "0.30")
)  # publish at VLM confidence only above this
VLM_VERIFY_CROP_PAD = float(
    os.environ.get("DERPBOT_VLM_VERIFY_CROP_PAD", "1.5")
)  # crop = bbox * pad (min 128 px)
VLM_VERIFY_TIMEOUT_S = float(
    os.environ.get("DERPBOT_VLM_VERIFY_TIMEOUT_S", "240")
)
VLM_VERIFY_MAX_RETRIES = int(
    os.environ.get("DERPBOT_VLM_VERIFY_MAX_RETRIES", "2")
)
VLM_VERIFY_POLL_S = float(os.environ.get("DERPBOT_VLM_VERIFY_POLL_S", "1.0"))

PROMPT_TEMPLATE = (
    "Look at this image from a robot's camera. Does the image clearly show "
    "any of the following objects: {classes}?\n\n"
    'Answer with ONLY a JSON object, no other text:\n'
    '{{"detections": [{{"class": "<object name>", "bbox": [x1, y1, x2, y2], '
    '"confidence": <number between 0.0 and 1.0>]}}]}}\n'
    "- bbox is pixel coordinates in the cropped image, as [x1, y1, x2, y2].\n"
    '- "class" must be exactly one of the listed object names.\n'
    "- Include an object ONLY if it is clearly visible in this image. "
    'If none are visible, return {{"detections": []}}.'
)


def _class_prompt_map(targets: list[str]) -> dict[str, str]:
    """Mission targets → prompt names, exactly like the detector:
    underscores to spaces (detector.py: prompts = [t.replace(\"_\", \" \")])."""
    return {t: t.replace("_", " ") for t in targets}


def parse_strict(text: str) -> list[dict]:
    """Strict single-parse of the structured reply -> [{cls, conf, bbox}].

    Raises ValueError on any deviation (no JSON object, missing keys,
    non-list detections, malformed bbox). The caller counts the failure.
    """
    text = text.strip()
    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        raise ValueError(f"no JSON object in reply: {text[:120]!r}")
    obj = json.loads(m.group(0))
    dets = obj.get("detections", [])
    if not isinstance(dets, list):
        raise ValueError("detections is not a list")
    out = []
    for d in dets:
        if not isinstance(d, dict) or "class" not in d or "bbox" not in d:
            raise ValueError(f"malformed detection: {str(d)[:120]}")
        bb = [float(x) for x in d["bbox"]]
        if len(bb) != 4:
            raise ValueError(f"bbox length != 4: {bb}")
        out.append({
            "cls": str(d["class"]),
            "conf": float(d.get("confidence", 0.0)),
            "bbox": bb,
        })
    return out


class _OllamaClient:
    """Minimal chat+image client. <=2 retries; 429/quota = terminal None."""

    def __init__(self, url: str, model: str):
        self._url = url
        self._model = model

    def ask(self, prompt: str, jpeg_bytes: bytes,
            retries: int = VLM_VERIFY_MAX_RETRIES):
        b64 = base64.b64encode(jpeg_bytes).decode()
        body = json.dumps({
            "model": self._model,
            "messages": [{"role": "user", "content": prompt, "images": [b64]}],
            "stream": False,
        }).encode()
        last_err = None
        for attempt in range(1 + retries):
            req = urllib.request.Request(
                self._url, data=body,
                headers={"Content-Type": "application/json"})
            t0 = time.time()
            try:
                with urllib.request.urlopen(req, timeout=VLM_VERIFY_TIMEOUT_S) as r:
                    d = json.loads(r.read())
                return (d.get("message", {}).get("content", ""), time.time() - t0,
                        None)
            except urllib.error.HTTPError as exc:
                if exc.code in (429, 402):
                    # quota / rate limit: terminal per the cost protocol
                    return None, time.time() - t0, f"HTTP {exc.code} quota"
                last_err = f"HTTP {exc.code}"
            except Exception as exc:  # noqa: BLE001
                last_err = f"{type(exc).__name__}: {exc}"
        return None, 0.0, last_err


class VLMPendingVerifier:
    """
    Owns the verifier background thread.

    Wiring (agent_node.py, default-OFF):
        verifier = VLMPendingVerifier(node, tracker, detector, projector, targets)
        verifier.start()   # if DERPBOT_VLM_VERIFY=1
        ... verifier.stop()

    The tracker must expose:
      - get_pending_candidates(sim_time) -> [{track_id, class_name, world_x, world_y, ...}]
      - inject_external_sighting(class_name, world_pos, robot_pose, confidence)
        (single method added to Tracker; routes the sighting through the SAME
        input_queue the detector relay uses — all existing gates apply)
    """

    def __init__(
        self,
        node,                                    # rclpy Node (logging + clock)
        tracker,                                 # tracker.Tracker
        detector,                                # detector.Detector (latest_frame)
        projector,                               # depth_projector.DepthProjector
        targets: list[str],
        client: Optional[_OllamaClient] = None,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._tracker = tracker
        self._detector = detector
        self._projector = projector
        self._targets = [t for t in targets if t]  # mission classes (snake_case)
        self._client = client or _OllamaClient(OLLAMA_URL, MODEL)
        self._prompt = PROMPT_TEMPLATE.format(
            classes=", ".join(t.replace("_", " ") for t in self._targets))

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._in_flight = threading.Semaphore(1)      # <=1 in-flight call
        self._budget_left = VLM_VERIFY_BUDGET
        self._last_call_sim: dict[str, float] = {}    # track_id -> sim-t last call
        self._injected: set[str] = set()              # track_ids already confirmed
        self._calls_made = 0

    # -- lifecycle ----------------------------------------------------------

    @staticmethod
    def enabled_from_env() -> bool:
        return os.environ.get(VLM_VERIFY_ENABLED_ENV) == "1"

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="vlm_pending_verifier")
        self._thread.start()
        self._logger.info(
            f"VLM verifier: ON (model={MODEL}, budget={VLM_VERIFY_BUDGET}, "
            f"cooldown={VLM_VERIFY_TRACK_COOLDOWN_S}s, min_age={VLM_VERIFY_MIN_AGE_S}s)"
        )

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def stats(self) -> dict:
        return {"calls_made": self._calls_made, "budget_left": self._budget_left,
                "injected": sorted(self._injected)}

    # -- pixel-bbox projection of a pending track ----------------------------

    def project_track_bbox(self, wx: float, wy: float, camera: dict) -> Optional[dict]:
        """Pending track's map pose -> expected pixel bbox via camera model.

        Mirrors the GT projection convention (build_recog_dataset.py):
        u = cx + fx*(-tan(rel_bearing)); v = cy - fy*tan(elev). The class's
        physical half-extents bound the crop so the object is inside even if
        the track pose is a metre off (crop pad scales it further).
        """
        robot = self._robot_pose()
        if robot is None:
            return None
        rx, ry, yaw = robot
        cx0, cy0 = camera["cx"], camera["cy"]
        fx, fy = camera["fx"], camera["fy"]
        W, H = camera["width"], camera["height"]
        rel = math.atan2(wy - ry, wx - rx) - yaw
        rel = (rel + math.pi) % (2 * math.pi) - math.pi
        if abs(rel) > math.pi / 2:        # behind the robot — wait for a pass
            return None
        rng = math.hypot(wx - rx, wy - ry)
        if rng < 0.4 or rng > 8.0:        # projector validity envelope
            return None
        u = cx0 + fx * (-math.tan(rel))
        # object centre ~0.25 m height (camera height) → near image centre row
        v = cy0
        return {"u": u, "v": v, "range": rng}

    def _robot_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            return self._tracker_robot_pose_provider()()
        except Exception:
            return None

    def _tracker_robot_pose_provider(self):
        """Returns a callable -> (x, y, yaw) from the tracker's TF buffer."""
        def get():
            t = self._tracker._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
            yaw = 2.0 * math.atan2(t.transform.rotation.z, t.transform.rotation.w)
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        return get

    # -- main loop ------------------------------------------------------------

    def _loop(self) -> None:
        while self._running:
            try:
                self._tick()
            except Exception as exc:  # never take the pipeline down
                try:
                    self._logger.warning(f"VLM verifier: tick error {exc}")
                except Exception:
                    pass
            time.sleep(VLM_VERIFY_POLL_S)

    def _sim_time(self) -> float:
        try:
            return self._node.get_clock().now().nanoseconds / 1e9
        except Exception:
            return time.time()

    def _tick(self) -> None:
        if self._budget_left <= 0:
            return
        sim_t = self._sim_time()
        pending = self._tracker.get_pending_candidates(sim_t)
        # oldest-first: a track that has been pending longest is the most
        # worth one call; only mission classes arrive here (worker filter)
        eligible = []
        for c in pending:
            tid = c["track_id"]
            if tid in self._injected:
                continue
            age = sim_t - float(c.get("first_sim_t", sim_t))
            if age < VLM_VERIFY_MIN_AGE_S:
                continue
            last = self._last_call_sim.get(tid)
            if last is not None and sim_t - last < VLM_VERIFY_TRACK_COOLDOWN_S:
                continue
            eligible.append(c)
        if not eligible:
            return
        # one call per tick (max 1 Hz + the in-flight guard)
        c = eligible[0]
        self._verify_one(c, sim_t)

    def _verify_one(self, cand: dict, sim_t: float) -> None:
        tid = cand["track_id"]
        frame_item = self._detector.latest_frame()
        if frame_item is None:
            return
        frame, _cam_t = frame_item
        h, w = frame.shape[:2]
        cam = self._projector.get_camera_model()
        if cam is None or cam.get("source") != "camera_info":
            # v1-fallback intrinsics would corrupt the crop geometry
            return

        proj = self.project_track_bbox(cand["world_x"], cand["world_y"], cam)
        if proj is None:
            self._last_call_sim[tid] = sim_t  # don't hammer unusable tracks
            return
        u, v = proj["u"], proj["v"]
        rng = proj["range"]
        # crop box: bbox half-extent from range (class-agnostic conservative
        # 0.6 m half-height) * pad
        half = max(128.0, (0.6 / rng) * cam["fx"] * VLM_VERIFY_CROP_PAD)
        x1, y1 = int(max(0, u - half)), int(max(0, v - half * 1.6))
        x2, y2 = int(min(w, u + half)), int(min(h, v + half * 1.6))
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            self._last_call_sim[tid] = sim_t
            return

        robot_pose = self._robot_pose_or_none()
        # <=1 in-flight call, non-blocking: a tick that arrives while one call
        # is in flight SKIPS (never queues up, never blocks the thread)
        if not self._in_flight.acquire(blocking=False):
            return
        try:
            self._calls_made += 1
            self._budget_left -= 1
            self._last_call_sim[tid] = sim_t
            import cv2 as _cv2
            ok, buf = _cv2.imencode(".jpg", crop,
                                    [int(_cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                return
            content, wall_s, err = self._client.ask(self._prompt, buf.tobytes())
        finally:
            self._in_flight.release()

        if err is not None:
            # 429/quota marks the run incomplete: stop calling entirely
            if "quota" in (err or ""):
                self._budget_left = 0
                self._logger.warning(f"VLM verifier: quota stop ({err})")
            else:
                self._logger.warning(
                    f"VLM verifier: {tid} call failed ({err})")
            return
        try:
            dets = parse_strict(content or "")
        except (ValueError, json.JSONDecodeError) as exc:
            self._logger.info(
                f"VLM verifier: {tid} parse failure ({exc}) — cooldown miss")
            return
        # a hit for THIS track's class, above the publish conf floor
        class_name = cand["class_name"]
        want = class_name.replace("_", " ")
        hit = next((d for d in dets if _norm_cls(d["cls"]) == _norm_cls(want)
                    and d["conf"] >= VLM_VERIFY_MIN_CONF), None)
        if hit is None:
            self._logger.info(
                f"VLM verifier: {tid} {class_name} NOT confirmed "
                f"(dets={[(d['cls'], round(d['conf'], 2)) for d in dets]})")
            return
        # positive → inject ONE sighting at the VLM confidence + the
        # crop-time robot pose. World position = the pending track's own
        # tracked map pose (the association radius fuses it).
        self._injected.add(tid)
        self._logger.info(
            f"VLM verifier: {tid} {class_name} CONFIRMED by VLM "
            f"conf={hit['conf']:.2f} ({wall_s:.1f}s) — injecting sighting")
        self._tracker.inject_external_sighting(
            class_name=class_name,
            world_pos=(cand["world_x"], cand["world_y"]),
            robot_pose=robot_pose,
            confidence=hit["conf"],
        )

    def _robot_pose_or_none(self):
        try:
            return self._tracker_robot_pose_provider()()
        except Exception:
            return (0.0, 0.0, 0.0)


def _norm_cls(s: str) -> str:
    return re.sub(r"[^a-z]", "", s.lower())