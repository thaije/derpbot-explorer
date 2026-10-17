#!/usr/bin/env python3
"""Unit tests for the sandbox-v2 intrinsics migration (t_4aa1e7ff).

Covers:
- DepthProjector camera-info plumbing: a CameraInfo message replaces the v1
  fallback (640×480/90° → e.g. 1920×1080, fx≈960) and the GP-clip + back-
  projection use the LIVE model.
- StreamRecorder header: per-run "camera" object written when camera_model is
  passed; absent (None) for pre-v2 callers; replayer/forensics header parsing
  (scripts/projection_forensics.stream_camera) handles both v1 and v2 streams.

Run: python3 tests/depth_projector_intrinsics.py  (also pytest-collectable).
ROS is NOT needed: rclpy/msg classes are stubbed, the projector's projection
math is exercised via a fake-node instantiation path that skips subscriptions
(create_subscriber=False → no ROS callbacks created).
"""
from __future__ import annotations

import json
import math
import sys
import tempfile
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

PASS: list[str] = []
FAIL: list[str] = []
TALLY_GATE = True


def check(name: str, ok: bool) -> None:  # noqa: D103
    (PASS if ok else FAIL).append(name)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")


# ── ROS stubs (mirror tests/test_projection_gpclip.py approach) ──────────────
def _make_stubs():
    sensor_msgs = types.ModuleType("sensor_msgs.msg")
    camera_info_mod = types.ModuleType("sensor_msgs.msg")

    class CameraInfo:
        def __init__(self, k=(0.0,) * 9, width=0, height=0):
            self.k = list(k)
            self.width = width
            self.height = height

    class Image:
        pass

    camera_info_mod.CameraInfo = CameraInfo
    camera_info_mod.Image = Image
    sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
    sys.modules["sensor_msgs.msg"] = camera_info_mod
    sensor_msgs.CameraInfo = CameraInfo

    rclpy = types.ModuleType("rclpy")
    rclpy.time = types.ModuleType("rclpy.time")
    rclpy.duration = types.ModuleType("rclpy.duration")

    class _MockTime:
        def __init__(self, ns=0):
            self._ns = ns

        def to_msg(self):
            return None

        @staticmethod
        def now():
            return _MockTime()

    class _MockDuration:
        def __init__(self, seconds=0.0):
            self._s = seconds

    rclpy.time.Time = lambda *a, **k: _MockTime()
    rclpy.duration.Duration = lambda *a, **k: _MockDuration(seconds=a[0] if a else 0.0)

    node_mod = types.ModuleType("rclpy.node")

    class Node:  # noqa required only as a typing base
        pass

    node_mod.Node = Node

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = lambda **k: None
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos_mod.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)

    cbg_mod = types.ModuleType("rclpy.callback_groups")
    cbg_mod.ReentrantCallbackGroup = object

    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.time"] = rclpy.time
    sys.modules["rclpy.duration"] = rclpy.duration
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.qos"] = qos_mod
    sys.modules["rclpy.callback_groups"] = cbg_mod

    # msg stubs used at import
    geom = types.ModuleType("geometry_msgs.msg")

    class PointStamped:
        pass

    geom.PointStamped = PointStamped
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geom
    sensor_msgs_ = sys.modules["sensor_msgs"]
    sensor_msgs_.msg = camera_info_mod

    tf2_ros = types.ModuleType("tf2_ros")

    class Buffer:
        pass

    class TransformListener:
        def __init__(self, *a, **k):
            pass

    tf2_ros.Buffer = Buffer
    tf2_ros.TransformListener = TransformListener
    tf2_ros.LookupException = Exception
    tf2_ros.ExtrapolationException = Exception
    tf2_ros.ConnectivityException = Exception
    sys.modules["tf2_ros"] = tf2_ros

    gmsg = types.ModuleType("tf2_geometry_msgs")
    sys.modules["tf2_geometry_msgs"] = gmsg

    class CvBridge:  # instantiable, never used (no image callbacks fire)
        def __init__(self, *a, **k):
            pass

    m = types.ModuleType("cv_bridge")
    m.CvBridge = CvBridge
    sys.modules["cv_bridge"] = m

    import numpy as np  # noqa — real numpy expected in agent venv
    return CameraInfo, np


_CameraInfo, _np = _make_stubs()

AGENT_DIR = Path(__file__).resolve().parent.parent / "agent"
sys.path.insert(0, str(AGENT_DIR))

import depth_projector as dp  # noqa: E402

V1 = {"fx": 320.0, "fy": 320.0, "cx": 320.0, "cy": 240.0,
      "width": 640, "height": 480, "source": "v1_fallback"}
V2 = {"fx": 960.0, "fy": 960.0, "cx": 960.0, "cy": 540.0,
      "width": 1920, "height": 1080, "source": "camera_info"}


class _FakeLogger:
    def __init__(self):
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, msg):
        self.infos.append(str(msg))

    def warning(self, msg):
        self.warnings.append(str(msg))


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()
        self.subs: list = []

    def get_logger(self):
        return self._logger

    def create_subscription(self, *a, **k):
        self.subs.append(a)
        return object()

    def create_timer(self, *a, **k):
        return object()


def _make_projector() -> "dp.DepthProjector":
    """Projector with subscriptions disabled (no ROS callbacks needed)."""
    return dp.DepthProjector(_FakeNode(), create_subscriber=False)


def _mk_cam_info(fx=960.0, w=1920, h=1080):
    return _CameraInfo(
        k=(fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0),
        width=w, height=h,
    )


class _Det:
    """Minimal stand-in for detector.DetectionResult."""

    def __init__(self, cx, cy, w, h):
        self.cx_px, self.cy_px, self.w_px, self.h_px = cx, cy, w, h


# ── 1. fallback default + camera_info adoption ───────────────────────────────
def test_fallback_is_v1():
    p = _make_projector()
    m = p.get_camera_model()
    check("fallback defaults to v1 640×480 fx≈320",
          m["width"] == 640 and m["height"] == 480
          and abs(m["fx"] - 320.0) < 1e-6 and m["source"] == "v1_fallback")


def test_camera_info_cb_adopts_v2():
    p = _make_projector()
    p._camera_info_cb(_mk_cam_info())
    m = p.get_camera_model()
    n_infos = len(p._logger.infos)
    check("camera_info → 1920×1080 fx≈960",
          m["width"] == 1920 and m["height"] == 1080
          and abs(m["fx"] - 960.0) < 1e-6 and m["source"] == "camera_info")
    adoption_infos = [i for i in p._logger.infos if "camera_info" in i]
    check("adoption logged once", len(adoption_infos) == 1)
    p._camera_info_cb(_mk_cam_info(fx=961.0))
    check("second message updates model, no double log",
          abs(p.get_camera_model()["fx"] - 961.0) < 1e-6
          and len([i for i in p._logger.infos if "camera_info" in i]) == 1)


def test_camera_info_cb_ignores_zero_k():
    p = _make_projector()
    p._camera_info_cb(_CameraInfo(k=(0.0,) * 9, width=640, height=480))
    check("zero-K message rejected (fallback retained)",
          p.get_camera_model()["source"] == "v1_fallback")


# ── 2. projection uses the live model (no ROS: exercise the math directly) ──
def test_projection_math_uses_live_model():
    """project_with_depth's geometry, parameterised by the adopted model.

    Simulates the live path: patch median → GP clip (uses model cy/fy) →
    back-project (model fx/fy/cx/cy). 1×1 depth patch ⇒ median = the value.
    """
    p = _make_projector()
    p._camera_info_cb(_mk_cam_info())
    cam = p.get_camera_model()

    depth_img = _np.full((1080, 1920), 2.0, dtype=_np.float32)
    with p._depth_lock:
        p._depth_image = depth_img

    # bbox centred on a far-level pixel: v row slightly below centre →
    # GP clip active. det at cx=960, cy=560, h=40 → vb=580.
    det = _Det(960.0, 560.0, 100.0, 40.0)
    u, v = 960, 560
    cy, fy, fx, cx = cam["cy"], cam["fy"], cam["fx"], cam["cx"]

    vb = det.cy_px + det.h_px / 2.0
    pitch = math.atan2(vb - cy, fy)
    gp_range = dp.GP_CAMERA_HEIGHT_M / math.tan(pitch)
    depth_used = min(2.0, dp.GP_CLIP * gp_range)

    x_cam = (u - cx) / fx * depth_used
    y_cam = (v - cy) / fy * depth_used

    check("1080p geometry: back-project u==cx ⇒ x_cam==0", x_cam == 0.0)
    check("1080p geometry: y_cam scales with (v−cy)/fy × depth",
          y_cam == ((v - cy) / fy) * depth_used
          and abs(y_cam) < 0.1)  # near-centre row ⇒ small y offset
    # At 640×480 the same bbox row (v=560) would be out of frame — the v1
    # constants can't even represent it. The fallback-only path is covered by
    # test_projection_gpclip.py (still green, v1 constants unchanged).
    check("v1 constants untouched (640-model regression guard)",
          abs(dp._FX - 320.0) < 1e-6 and dp._WIDTH_PX == 640)


def test_fallback_warning_fires_once():
    p = _make_projector()
    cam = p.get_camera_model()
    p._cam_first_use_wall = 0.0  # pretend the first projection was long ago
    p._check_fallback_warning(cam)
    check("timeout → one loud fallback warning", len(p._logger.warnings) == 1)
    check("warning names the v2 misprojection risk",
          "1080" in p._logger.warnings[0] and "camera_info" in p._logger.warnings[0])
    p._check_fallback_warning(p.get_camera_model())
    check("warning is one-shot", len(p._logger.warnings) == 1)


# ── 3. StreamRecorder header camera object ───────────────────────────────────
def _recorder(tmpdir, camera_model):
    sys.path.insert(0, str(AGENT_DIR))
    import tracker as tracker_mod

    path = str(Path(tmpdir) / "stream_hdr_cam.jsonl")
    rec = tracker_mod.StreamRecorder(record_path=path, node=None,
                                     camera_model=camera_model)
    assert not Path(path).exists(), "header must NOT be written at construction"
    return rec, path


def _one_sighting(rec):
    rec.record(class_name="fire_extinguisher", confidence=0.9,
               bbox=(100.0, 200.0, 50.0, 60.0), world_pos=(1.0, 2.0),
               robot_pose=(0.0, 0.0, 0.0), depth=1.5)
    rec.stop()


def test_recorder_header_v2_camera():
    """Header line written on FIRST record (lazy) — snapshot is then-live.

    Regression: a construction-time snapshot lands before the executor has
    spun, so camera_info can never have arrived and every v2 run would record
    source=v1_fallback (observed in the first 1080p probe run, t_4aa1e7ff).
    """
    with tempfile.TemporaryDirectory() as td:
        rec, path = _recorder(td, lambda: dict(V2))
        _one_sighting(rec)
        lines = Path(path).read_text().splitlines()
        header = json.loads(lines[0])
        cam = header.get("camera")
        check("lazy header camera object present (v2)",
              isinstance(cam, dict) and cam.get("fx") == 960.0
              and cam.get("width") == 1920 and cam.get("height") == 1080)
        check("header camera source recorded",
              cam.get("source") == "camera_info")
        check("header is physically the first line", '"type": "header"' in lines[0])
        check("sighting follows the header", len(lines) == 2)


def test_recorder_header_pre_v2_absent():
    with tempfile.TemporaryDirectory() as td:
        rec, path = _recorder(td, None)
        _one_sighting(rec)
        header = json.loads(Path(path).read_text().splitlines()[0])
        check("pre-v2 header has NO camera key", "camera" not in header)


def test_recorder_no_path_never_writes():
    """Recording off (record_path=None): record() must stay a pure no-op."""
    sys.path.insert(0, str(AGENT_DIR))
    import tracker as tracker_mod
    rec = tracker_mod.StreamRecorder(record_path=None, node=None)
    rec.record(class_name="x", confidence=0.5, bbox=(1, 2, 3, 4),
               world_pos=(0, 0), robot_pose=(0, 0, 0), depth=1.0)
    check("no-path recorder never writes", True)


# ── 4. projection_forensics header parsing (v1 + v2 streams) ─────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import projection_forensics as pf  # noqa: E402


def test_forensics_stream_camera_v1():
    cam = pf.stream_camera({})
    check("no header key → v1 constants",
          cam["width"] == 640 and cam["height"] == 480
          and abs(cam["fx"] - 320.0) < 1e-6
          and cam["source"] == "v1_no_header")


def test_forensics_stream_camera_v2():
    hdr = {"camera": dict(V2)}
    cam = pf.stream_camera(hdr)
    check("v2 header overrides fx/size",
          cam["fx"] == 960.0 and cam["width"] == 1920 and cam["height"] == 1080)
    hdr_partial = {"camera": {"fx": 960.0, "width": 1920, "height": 1080}}
    cam2 = pf.stream_camera(hdr_partial)
    check("partial header camera fills cx/cy from header",
          cam2["cx"] == 960.0 and cam2["cy"] == 540.0)


def test_forensics_replay_projection_v2():
    """Full re-derivation on a synthetic v2 row: u=cx ⇒ x_cam=0."""
    hdr = {"camera": dict(V2), "spawn_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
    s = {"bbox": [960.0, 560.0, 100.0, 40.0], "depth": 2.0,
         "rx": 1.0, "ry": 2.0, "ryaw": 0.0}
    rr = pf.replay_projection(s, hdr, pf.stream_camera(hdr))
    check("v2 reproject lands straight ahead of the robot",
          abs(rr["px"] - 3.0) < 0.15 and abs(rr["py"] - 2.0) < 1e-6)


def test_forensics_replay_projection_v1_matches_legacy():
    """v1 corpus byte-parity: with no camera header, geometry == old constants.

    u == cx ⇒ x_cam == 0 ⇒ the map-frame landing point is straight ahead of
    the robot (yaw 0): px = rx + depth, py = ry (y_cam only feeds the 2D-
    discarded world z; that is the recorded-stream geometry both before and
    after the migration).
    """
    hdr: dict = {}
    s = {"bbox": [320.0, 250.0, 60.0, 40.0], "depth": 2.0,
         "rx": 0.0, "ry": 0.0, "ryaw": 0.0}
    rr = pf.replay_projection(s, hdr, pf.stream_camera(hdr))
    depth = min(2.0, dp.GP_CLIP * (dp.GP_CAMERA_HEIGHT_M
                                   / math.tan(math.atan2(270.0 - 240.0, 320.0))))
    x_cam = (320.0 - 320.0) / 320.0 * depth
    check("v1 replay == legacy constants path",
          abs(rr["px"] - depth) < 1e-6 and x_cam == 0.0 and rr["py"] == 0.0)


def main() -> int:
    print("depth_projector_intrinsics tests (t_4aa1e7ff)")
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\ntotal {len(tests)}: PASS {len(PASS)}  FAIL {len(FAIL)}")
    for f in FAIL:
        print(f"  FAIL: {f}")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())