"""Offline unit checks for the VLM pending-track verifier (v13-cv, t_8ed4eb97).
No ROS needed, no sim, no network (the ollama client is stubbed).

Covers:
  1. Env gate: verifier disabled by default (no DERPBOT_VLM_VERIFY) —
     agent_node wiring is import-gated on the env flag.
  2. Strict single-parse: valid reply / no-JSON / non-list detections /
     malformed det / wrong bbox arity.
  3. Guards: budget cap, <=1 in-flight call, per-track cooldown, min age,
     already-injected tracks never re-verified.
  4. Parse-failure path: a parse failure consumes budget + sets cooldown,
     injects nothing.
  5. Positive path: hit >= VLM_VERIFY_MIN_CONF → exactly ONE injected
     sighting at the VLM confidence, through Tracker.inject_external_sighting.
  6. Negative path: VLM "none" → no injection, cooldown recorded.
  7. inject_external_sighting: routes through input_queue with confidence
     riding along (tracker replay-visible).
  8. Quota (429) path: budget zeroed → no further calls.

Run: python3.12 tests/test_vlm_verifier.py -q   (repo-canonical tallied mode)
Also: python3.12 -m pytest tests/test_vlm_verifier.py -q   (ROS env)

TALLY_GATE = True (conftest.py session guard): this module's tally is green
in BOTH modes, so pytest counts may not silently drift from the tally.
"""

import os
import sys
import threading
import time
import queue as queue_mod
from unittest.mock import Mock

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "agent"))

try:
    import vlm_verifier as vv
    import tracker as tracker_mod
except ImportError as exc:
    pytest.skip(f"vlm_verifier import failed ({exc}) — run on ROS env",
                allow_module_level=True)

PASS = []
FAIL = []

# conftest.py session guard: the tally is green in both pytest and the
# repo-canonical tallied __main__ run — pytest MUST fail if this module's
# check() tally records FAILs again (the exact drift hazard that left this
# module red through a green pytest run in the v13-cv round-1 review).
TALLY_GATE = True


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(("PASS" if cond else "FAIL"), name, detail)


# ---------------------------------------------------------------- parse_strict

def test_parse_valid():
    dets = vv.parse_strict(
        '{"detections": [{"class": "first aid kit", "bbox": [1, 2, 3, 4], "confidence": 0.92}]}')
    check("parse: valid single det", dets == [
        {"cls": "first aid kit", "conf": 0.92, "bbox": [1.0, 2.0, 3.0, 4.0]}])


def test_parse_prose_wrapped():
    dets = vv.parse_strict(
        'Here you go:\n{"detections": []}\nThanks!')
    check("parse: prose-wrapped empty", dets == [])


def test_parse_garbage_raises():
    for bad in ["no json here", '{"wrong": 1}', ""]:
        try:
            vv.parse_strict(bad)
            check(f"parse: rejects {bad!r}", False)
        except (ValueError, Exception):
            check(f"parse: rejects {bad!r}", True)


def test_parse_malformed_det():
    for bad in ['{"detections": {"a": 1}}',
                '{"detections": ["nope"]}',
                '{"detections": [{"class": "x"}]}',
                '{"detections": [{"class": "x", "bbox": [1, 2, 3]}]}']:
        try:
            vv.parse_strict(bad)
            check(f"parse: rejects {bad[:40]!r}", False)
        except (ValueError, KeyError, TypeError):
            check(f"parse: rejects {bad[:40]!r}", True)


# ---------------------------------------------------------------- fake tracker

class FakeTracker:
    def __init__(self):
        self.pending = []
        self.injected = []
        self._q = queue_mod.Queue()

    def get_pending_candidates(self, sim_time):
        return list(self.pending)

    def inject_external_sighting(self, class_name, world_pos, robot_pose,
                                 confidence, source="vlm_verify"):
        self.injected.append((class_name, world_pos, robot_pose, confidence))
        self._q.put((class_name, world_pos, robot_pose, confidence))
        return True


class FakeNode:
    class _Clock:
        def now(self):
            # 60 sim-s fixed
            class T:
                nanoseconds = int(60e9)
            return T()

    def __init__(self):
        self._clock = self._Clock()

    def get_clock(self):
        return self._clock

    class _Log:
        def info(self, m): pass
        def warning(self, m): pass
        def error(self, m): pass

    def get_logger(self):
        return self._Log()


class FakeProjector:
    @staticmethod
    def get_camera_model():
        return {"fx": 960.0, "fy": 960.0, "cx": 960.0, "cy": 540.0,
                "width": 1920, "height": 1080, "source": "camera_info"}


class FakeClient:
    """Scripted client: pops replies; records calls."""
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = []

    def ask(self, prompt, jpeg_bytes, retries=2):
        self.calls.append(len(self.calls))
        if not self.replies:
            return ("", 0.1, "scripted-out")
        item = self.replies.pop(0)
        if isinstance(item, tuple) and item[0] == "ERROR":
            return None, 0.1, item[1]
        return (item, 0.1, None)


class FakeFrame:
    def __init__(self):
        import numpy as np
        # realistic v2 frame size (1920x1080) — the camera model in
        # FakeProjector must match the frame or the crop degenerates
        self.arr = np.zeros((1080, 1920, 3), dtype=np.uint8)

    @property
    def shape(self):
        return self.arr.shape

    def __getitem__(self, key):
        return self.arr[key]


class FakeDetector:
    _f = FakeFrame()

    @staticmethod
    def latest_frame():
        return (FakeDetector._f, 123.0)


def make_verifier(replies, targets=("first_aid_kit",), budget=10,
                  min_age=0.0, cooldown=0.0):
    node = FakeNode()
    tracker = FakeTracker()
    v = vv.VLMPendingVerifier(
        node=node, tracker=tracker, detector=FakeDetector,
        projector=FakeProjector(), targets=list(targets),
        client=FakeClient(replies))
    v._budget_left = budget
    # robot pose fixed at origin facing +x
    v._robot_pose = lambda: (0.0, 0.0, 0.0)
    # default: every pending() track has already been seen (age ≥ min age) so
    # individual ticks exercise the later guards; the min-age test uses a
    # fresh track id with NO pre-seed (a pre-seed defeats its own first check)
    v._first_seen_sim["track_1"] = 30.0
    return v, tracker


def pending(tid="track_1", cls="first_aid_kit", wx=3.0, wy=0.4, first=40.0):
    return {"track_id": tid, "class_name": cls, "world_x": wx, "world_y": wy,
            "first_sim_t": first}


ENV_KEYS = ("DERPBOT_VLM_VERIFY_MODEL", "DERPBOT_VLM_VERIFY_URL",
            "DERPBOT_VLM_VERIFY_TRACK_COOLDOWN_S", "DERPBOT_VLM_VERIFY_BUDGET",
            "DERPBOT_VLM_VERIFY_MIN_AGE_S", "DERPBOT_VLM_VERIFY_MIN_CONF",
            "DERPBOT_VLM_VERIFY_TIMEOUT_S", "DERPBOT_VLM_VERIFY_MAX_RETRIES",
            "DERPBOT_VLM_VERIFY_POLL_S")


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    for k in ENV_KEYS:
        monkeypatch.delenv(k, raising=False)
    monkeypatch.delenv(vv.VLM_VERIFY_ENABLED_ENV, raising=False)
    yield


# ---------------------------------------------------------------------- tests

def test_env_gate_default_off():
    check("env gate: default OFF", not vv.VLMPendingVerifier.enabled_from_env())


def test_env_gate_on():
    os.environ[vv.VLM_VERIFY_ENABLED_ENV] = "1"
    check("env gate: DERPBOT_VLM_VERIFY=1 enables",
          vv.VLMPendingVerifier.enabled_from_env())


def test_positive_injects_one_sighting():
    v, tracker = make_verifier(
        ['{"detections": [{"class": "first aid kit", "bbox": [10, 10, 50, 50], "confidence": 0.9}]}'])
    tracker.pending = [pending()]
    v._tick()
    check("positive: exactly one injection", len(tracker.injected) == 1,
          str(tracker.injected))
    conf = tracker.injected[0][3]
    check("positive: publishes at VLM confidence", abs(conf - 0.9) < 1e-6,
          f"conf={conf}")
    check("positive: world pos = tracked map pose",
          tracker.injected[0][1] == (3.0, 0.4))
    check("positive: track marked injected", "track_1" in v._injected)


def test_negative_no_injection():
    v, tracker = make_verifier(['{"detections": []}'])
    tracker.pending = [pending()]
    v._tick()
    check("negative: no injection", len(tracker.injected) == 0)
    check("negative: cooldown recorded", "track_1" in v._last_call_sim)


def test_below_min_conf_not_published():
    v, tracker = make_verifier(
        ['{"detections": [{"class": "first aid kit", "bbox": [1, 1, 9, 9], "confidence": 0.29}]}'])
    tracker.pending = [pending()]
    v._tick()
    check("below VLM_VERIFY_MIN_CONF: not injected", len(tracker.injected) == 0)


def test_wrong_class_hit_ignored():
    v, tracker = make_verifier(
        ['{"detections": [{"class": "office chair", "bbox": [1, 1, 9, 9], "confidence": 0.9}]}'])
    tracker.pending = [pending(cls="first_aid_kit")]
    v._tick()
    check("other-class det: no injection", len(tracker.injected) == 0)


def test_parse_failure_consumes_budget_no_inject():
    v, tracker = make_verifier(["this is not json at all"])
    tracker.pending = [pending()]
    v._tick()
    check("parse failure: no injection", len(tracker.injected) == 0)
    check("parse failure: budget consumed", v._budget_left == 9)
    check("parse failure: cooldown recorded", "track_1" in v._last_call_sim)
    check("parse failure: track not marked injected", "track_1" not in v._injected)


def test_budget_cap():
    v, tracker = make_verifier([pending() and "x"] * 5)
    v._budget_left = 0
    tracker.pending = [pending()]
    v._tick()
    check("budget 0: tick is a no-op", len(tracker.injected) == 0
          and v._calls_made == 0)


def test_in_flight_guard():
    # simulate an ongoing call via the semaphore, then a tick must skip
    v, tracker = make_verifier(['{"detections": []}'])
    tracker.pending = [pending()]
    got = v._in_flight.acquire(blocking=False)
    v._tick()
    check("in-flight guard: second call blocked while one holds the slot",
          len(tracker.injected) == 0 and v._calls_made == 0)
    if got:
        v._in_flight.release()
    # tick with the slot free works
    os.environ["DERPBOT_VLM_VERIFY_MAX_RETRIES"] = "0"
    v2, tracker2 = make_verifier(['{"detections": []}'])
    tracker2.pending = [pending()]
    v2._tick()
    check("in-flight guard: call proceeds when slot free", v2._calls_made == 1)


def test_track_cooldown():
    os.environ["DERPBOT_VLM_VERIFY_TRACK_COOLDOWN_S"] = "20"
    import importlib
    importlib.reload(vv)
    v, tracker = make_verifier(['{"detections": []}', '{"detections": []}'],
                               cooldown=20.0)
    tracker.pending = [pending(first=40.0)]
    v._first_seen_sim["track_1"] = 30.0  # seen 30 sim-s ago → eligible now
    v._tick()                     # first call at sim 60
    v._tick()                     # 0 s later — must be skipped
    check("cooldown: only 1 call within 20 sim-s", len(v._client.calls) == 1,
          f"calls={v._client.calls}")
    os.environ.pop("DERPBOT_VLM_VERIFY_TRACK_COOLDOWN_S", None)
    importlib.reload(vv)


def test_min_age():
    os.environ["DERPBOT_VLM_VERIFY_MIN_AGE_S"] = "5"
    import importlib
    importlib.reload(vv)
    v, tracker = make_verifier(['{"detections": []}'])
    # age is tracked by the verifier's OWN first-seen map (get_pending_candidates
    # strips first_sim_t): a track first seen "now" is too young, ages into
    # eligibility on later ticks. Fresh track id — make_verifier's track_1
    # pre-seed (age 30 ≥ 5) would defeat this check's "not called" assertion.
    tracker.pending = [pending(tid="track_fresh", first=58.0)]  # age field stale
    v._tick()
    check("min age: first-sighting tick not called", len(v._client.calls) == 0)
    check("min age: fresh track seeded into first-seen map",
          "track_fresh" in v._first_seen_sim)
    v._first_seen_sim["track_fresh"] = v._sim_time() - 10.0  # seen 10 sim-s ago
    v._tick()
    check("min age: aged track called", len(v._client.calls) == 1)
    os.environ.pop("DERPBOT_VLM_VERIFY_MIN_AGE_S", None)
    importlib.reload(vv)


def test_already_injected_never_recalled():
    v, tracker = make_verifier(['{"detections": []}'])
    tracker.pending = [pending()]
    v._injected.add("track_1")
    v._tick()
    check("injected set: no second call", len(v._client.calls) == 0)


def test_first_seen_map_tracks_new_ids():
    v, tracker = make_verifier(['{"detections": []}'])
    tracker.pending = [pending(tid="track_7")]
    v._tick()   # first sighting of track_7 → age 0, must NOT call
    check("new track id: first tick not called", len(v._client.calls) == 0)
    v._tick()   # age still ~0 (poll interval) — still not called
    check("new track id: second tick still too young", len(v._client.calls) == 0)


def test_quota_stops_budget():
    v, tracker = make_verifier([("ERROR", "HTTP 429 quota")])
    tracker.pending = [pending()]
    v._tick()
    check("quota: budget zeroed on 429", v._budget_left == 0)
    check("quota: no injection", len(tracker.injected) == 0)
    tracker2_pending = [pending("track_2")]
    tracker.pending = tracker2_pending
    v._tick()
    check("quota: no further calls", len(v._client.calls) == 1)


def test_unrenderable_track_no_call():
    # track behind the robot (yaw 0, object at negative x) → no usable crop
    v, tracker = make_verifier(['{"detections": []}'])
    tracker.pending = [pending(wx=-3.0, wy=0.0)]
    v._tick()
    check("behind-robot track: no call, cooldown set", len(v._client.calls) == 0
          and "track_1" in v._last_call_sim)


def test_inject_external_sighting_routes_queue():
    # real Tracker method behavior via a stub of its internals
    class FakeRec:
        def record(self, **kw): self.rec = kw
        def stop(self): pass
    class T:
        _recorder = FakeRec()
        _input_queue = queue_mod.Queue(maxsize=100)
        inject_external_sighting = tracker_mod.Tracker.inject_external_sighting
    t = T()
    ok = t.inject_external_sighting("first_aid_kit", (3.0, 0.4),
                                    (0.0, 0.0, 0.0), 0.9)
    check("inject: returns True", ok is True)
    item = t._input_queue.get_nowait()
    # t_d479428a: 5-tuple with the externally-verified marker True (worker
    # reads item[4]; detector relays send 4-tuples → flag False there)
    check("inject: tuple shape (cls, pos, pose, conf, injected=True)",
          item == ("first_aid_kit", (3.0, 0.4), (0.0, 0.0), 0.9, True), str(item))
    check("inject: stream recorder tapped",
          t._recorder.rec["class_name"] == "first_aid_kit"
          and t._recorder.rec["confidence"] == 0.9)
    check("inject: recorder never gets non-tuple bbox",
          t._recorder.rec["bbox"] == (0.0, 0.0, 0.0, 0.0))


def test_prompt_derived_from_targets():
    # all three mission targets: the check asserts on all three prompts
    v, tracker = make_verifier(
        ['{"detections": []}'],
        targets=("first_aid_kit", "fire_extinguisher", "person"))
    check("prompt: class names via underscore->space rule",
          "first aid kit" in v._prompt and "fire extinguisher" in v._prompt
          and "person" in v._prompt)
    check("prompt: no snake_case leakage into prompt text",
          "first_aid_kit" not in v._prompt)


def test_no_class_logic_hardcoded():
    # a mission class outside the usual three still produces a correct prompt
    v, _ = make_verifier(['{"detections": []}'], targets=("power_drill",))
    check("prompt: arbitrary mission class lands via same rule",
          "power drill" in v._prompt)


def print_tally():
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        print("FAILED:", FAIL)
    return len(FAIL)


if __name__ == "__main__":
    import inspect
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and inspect.isfunction(fn):
            fn()
    sys.exit(1 if print_tally() else 0)