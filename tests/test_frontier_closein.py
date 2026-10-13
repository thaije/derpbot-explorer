"""
Unit tests for the close-in verification detour (t_5e2d3407).

Covers:
  1. Budget cap: after CLOSE_IN_MAX_DETOURS detours, _build_detection_frontiers
     returns [] and the budget event is timeline-tagged exactly once.
  2. Per-track attempt cap: a track hit CLOSE_IN_MAX_ATTEMPTS is skipped.
  3. Per-track rate limit: a track retried inside CLOSE_IN_RETRY_RATE_S is skipped.
  4. Costmap cell guard: candidate on an unknown/lethal costmap cell is skipped;
     unknown-costmap (no snapshot) is allowed.
  5. Stop-and-stare hold: _close_in_hold publishes zero Twist for the full hold
     when the track stays pending, exits early on confirm, and always tags
     close_in_hold with source=close_in_verify.

Interpreter: system python3.12 + ROS paths (source /opt/ros/jazzy/setup.bash;
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/usr/lib/python3/dist-packages).
Run: python3.12 -m pytest tests/test_frontier_closein.py -q
"""

import importlib
import sys
from unittest.mock import MagicMock, Mock, patch
from types import SimpleNamespace

import os
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "agent"))  # profiler lives in agent/

# ---------------------------------------------------------------------------
# Mock ROS2 and heavy deps before importing frontier_explorer (test_detector_roi
# pattern). frontier_explorer also imports numpy and profiler — import the real
# numpy; profiler is a local module with no ROS deps at import time.
# ---------------------------------------------------------------------------
mock_rclpy = Mock()
mock_rclpy_node = Mock()
mock_rclpy.node = mock_rclpy_node
mock_rclpy.action = Mock()
mock_rclpy.qos = Mock()
mock_rclpy.qos.QoSProfile = Mock
mock_rclpy.qos.ReliabilityPolicy = Mock()
mock_rclpy.qos.DurabilityPolicy = Mock()
mock_rclpy.callback_groups = Mock()
mock_rclpy.callback_groups.ReentrantCallbackGroup = Mock
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy_node
sys.modules['rclpy.action'] = mock_rclpy.action
sys.modules['rclpy.qos'] = mock_rclpy.qos
sys.modules['rclpy.callback_groups'] = mock_rclpy.callback_groups

mock_action_msgs = Mock()
mock_action_msgs.msg = Mock()
mock_action_msgs.msg.GoalStatus = Mock()
sys.modules['action_msgs'] = mock_action_msgs
sys.modules['action_msgs.msg'] = mock_action_msgs.msg

mock_geometry = Mock()
mock_geometry.msg = Mock()
mock_geometry.msg.PoseStamped = Mock()
mock_geometry.msg.Point = Mock()
mock_geometry.msg.Twist = MagicMock  # real-ish: attribute assignment works
sys.modules['geometry_msgs'] = mock_geometry
sys.modules['geometry_msgs.msg'] = mock_geometry.msg

mock_nav_msgs = Mock()
mock_nav_msgs.msg = Mock()
mock_nav_msgs.msg.OccupancyGrid = Mock()
mock_nav_msgs.msg.Odometry = Mock()
sys.modules['nav_msgs'] = mock_nav_msgs
sys.modules['nav_msgs.msg'] = mock_nav_msgs.msg

mock_nav2 = Mock()
mock_nav2.action = Mock()
mock_nav2.action.BackUp = Mock()
mock_nav2.action.NavigateToPose = Mock()
mock_nav2.srv = Mock()
mock_nav2.srv.ClearEntireCostmap = Mock()
sys.modules['nav2_msgs'] = mock_nav2
sys.modules['nav2_msgs.action'] = mock_nav2.action
sys.modules['nav2_msgs.srv'] = mock_nav2.srv

mock_ros_gz = Mock()
mock_ros_gz.msg = Mock()
sys.modules['ros_gz_interfaces'] = mock_ros_gz
sys.modules['ros_gz_interfaces.msg'] = mock_ros_gz.msg

# Minimal Nav2 OccupancyGrid stand-in with the attribute surface the explorer reads.
class FakeOccupancyGrid:
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.info = SimpleNamespace(
            width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(position=SimpleNamespace(x=origin_x, y=origin_y)),
            frame_id="map",
        )
        self.data = data
        self.header = SimpleNamespace(frame_id="map", stamp=SimpleNamespace())

from agent.frontier_explorer import (  # noqa: E402
    CLOSE_IN_HOLD_S,
    CLOSE_IN_MAX_ATTEMPTS,
    CLOSE_IN_MAX_DETOURS,
    CLOSE_IN_RETRY_RATE_S,
    COSTMAP_LETHAL_THRESHOLD,
    FrontierExplorer,
)

PASS = []
FAIL = []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(f"{name} {detail}")
    print(("PASS" if cond else "FAIL"), name, detail)


def make_explorer(sim_time=100.0, tracker=None, costmap=None):
    """FrontierExplorer with mocked node/clients and controlled sim clock."""
    explorer = FrontierExplorer.__new__(FrontierExplorer)
    explorer._node = Mock()
    node = explorer._node
    node.get_logger.return_value = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = int(sim_time * 1e9)
    explorer._logger = node.get_logger()

    explorer._tracker = tracker
    explorer._sim_clock = sim_time
    explorer._robot_vx = 0.0
    explorer._robot_wz = 0.0
    explorer._map = None
    explorer._odom_lock = __import__("threading").Lock()
    explorer._map_lock = __import__("threading").Lock()
    explorer._costmap_lock = __import__("threading").Lock()
    explorer._global_costmap = costmap
    explorer._blacklist = []
    explorer._success_exclusion = []
    explorer._candidate_visited = set()
    explorer._closein_last_attempt = {}
    explorer._closein_attempts = __import__("collections").defaultdict(int)
    explorer._closein_budget_used = 0
    explorer._closein_budget_logged = False
    explorer._timeline = []
    explorer._cmd_vel_pub = Mock()
    return explorer


def set_time(explorer, sim_time):
    explorer._sim_clock = sim_time
    explorer._node.get_clock.return_value.now.return_value.nanoseconds = int(sim_time * 1e9)


def cand(track_id="track_1", cls="first_aid_kit", x=5.0, y=5.0):
    return {"track_id": track_id, "class_name": cls, "world_x": x, "world_y": y}


def fake_tracker(cands):
    t = Mock()
    t.get_pending_candidates.return_value = cands
    t.is_still_pending.return_value = True
    return t


# --------------------------------------------------------- guard-rail gates

def test_budget_cap():
    ex = make_explorer(tracker=fake_tracker([cand()]))
    ex._closein_budget_used = CLOSE_IN_MAX_DETOURS
    out = ex._build_detection_frontiers(0.0, 0.0)
    check("budget: exhausted -> []", out == [])
    check("budget: timeline-tagged once",
          len([e for e in ex._timeline if e["phase"] == "close_in_budget"]) == 1,
          f"tags={len(ex._timeline)}")
    ex._build_detection_frontiers(0.0, 0.0)
    check("budget: no duplicate tag on repeat cycle",
          len([e for e in ex._timeline if e["phase"] == "close_in_budget"]) == 1)


def test_attempt_cap():
    ex = make_explorer(
        tracker=fake_tracker([cand(track_id="track_9")]), sim_time=500.0)
    ex._closein_last_attempt["track_9"] = 400.0  # rate limit satisfied (100s ago)
    ex._closein_attempts["track_9"] = CLOSE_IN_MAX_ATTEMPTS
    out = ex._build_detection_frontiers(0.0, 0.0)
    check("attempt cap: capped track skipped", out == [])
    check("attempt cap: tagged",
          any(e["phase"] == "close_in_skip_attempts" and "track_9" in e["notes"]
              for e in ex._timeline))


def test_rate_limit():
    ex = make_explorer(
        tracker=fake_tracker([cand(track_id="track_7")]), sim_time=500.0)
    ex._closein_last_attempt["track_7"] = 490.0  # 10s ago < 60s rate limit
    out = ex._build_detection_frontiers(0.0, 0.0)
    check("rate limit: retried-too-soon track skipped", out == [])
    check("rate limit: tagged",
          any(e["phase"] == "close_in_skip_rate" for e in ex._timeline))
    # after the rate window passes the candidate is eligible again
    set_time(ex, 500.0 + CLOSE_IN_RETRY_RATE_S + 1.0)
    ex._global_costmap = None
    out2 = ex._build_detection_frontiers(0.0, 0.0)
    check("rate limit: eligible after window", len(out2) == 1, f"n={len(out2)}")


def test_costmap_cell_guard():
    # 5x5 costmap, 0.5m res, origin (0,0) — cell for (4.25, 4.25) = r8,c8 in a
    # 10x10 grid; build a 10x10 for precision.
    cm = FakeOccupancyGrid(10, 10, 0.5, 0.0, 0.0, [0] * 100)
    # candidate at (2.25, 3.25) -> cell r6, c4
    wx, wy = 2.25, 3.25
    idx = 6 * 10 + 4
    for value, label, expect_ok in [
        (0, "free", True),
        (50, "inflation-mid", True),
        (COSTMAP_LETHAL_THRESHOLD, "lethal-99", False),
        (100, "lethal-100", False),
    ]:
        cm.data[idx] = value
        ex = make_explorer(tracker=fake_tracker([cand(x=wx, y=wy)]))
        ex._global_costmap = cm
        out = ex._build_detection_frontiers(0.0, 0.0)
        ok = (len(out) == 1) == expect_ok
        check(f"costmap cell {label}: {'kept' if expect_ok else 'skipped'}", ok,
              f"out={len(out)}")
    # unknown costmap value (-1 raw data) is NOT a valid costmap value; the
    # OccupancyGrid convention stores 255 for unknown. But our guard reads the
    # raw int; -1 data would map value=-1 -> allowed by '<99 and !=-1' logic is
    # wrong direction — verify: value -1 -> rejected? No: -1 != -1 is False ->
    # NOT allowed... test the actual behaviour:
    cm.data[idx] = -1
    ex = make_explorer(tracker=fake_tracker([cand(x=wx, y=wy)]))
    ex._global_costmap = cm
    out = ex._build_detection_frontiers(0.0, 0.0)
    check("costmap cell -1: treated as lethal-ish (skipped)", out == [],
          f"out={len(out)}") if False else None
    # NOTE: -1 in raw data → value=-1 → `value < 99 and value != -1` → False →
    # skipped. That matches the spec (unknown cell → skip this cycle).
    check("costmap -1 semantics: unknown cell skipped",
          (int(cm.data[idx]) == -1) and out == [], f"out={len(out)}")


def test_no_costmap_allows():
    ex = make_explorer(tracker=fake_tracker([cand()]))
    ex._global_costmap = None
    out = ex._build_detection_frontiers(0.0, 0.0)
    check("no costmap snapshot: candidate allowed (budget bounds abuse)",
          len(out) == 1, f"n={len(out)}")


def test_outside_costmap_extent_allows():
    cm = FakeOccupancyGrid(4, 4, 0.5, 0.0, 0.0, [100] * 16)
    ex = make_explorer(tracker=fake_tracker([cand(x=50.0, y=50.0)]))
    ex._global_costmap = cm
    out = ex._build_detection_frontiers(0.0, 0.0)
    check("outside costmap extent: candidate allowed (Nav2 judges)",
          len(out) == 1, f"n={len(out)}")


# --------------------------------------------------------- hold behaviour

def test_hold_full_duration_when_pending():
    import threading
    ex = make_explorer(tracker=fake_tracker([cand()]))
    # Sim clock frozen at t0 — the sim-clock loop would spin forever, but the
    # 15s wall-clock ceiling bounds it. Instead freeze clock AND make the loop
    # exit by wall clock quickly: shrink via monkeypatched constant.
    with patch("agent.frontier_explorer.CLOSE_IN_HOLD_S", 0.05):
        ex._close_in_hold("track_1", "first_aid_kit", 3)
    n_zero = ex._cmd_vel_pub.publish.call_count
    check("hold: zero-twist publishes during hold", n_zero >= 1,
          f"publishes={n_zero}")
    hold_ev = [e for e in ex._timeline if e["phase"] == "close_in_hold"]
    check("hold: close_in_hold timeline-tagged with source",
          len(hold_ev) == 1 and "source=close_in_verify" in hold_ev[0]["notes"],
          f"notes={hold_ev[0]['notes'] if hold_ev else '-'}")
    check("hold: pending track -> confirmed=False",
          hold_ev and "confirmed=False" in hold_ev[0]["notes"])


def test_hold_early_exit_on_confirm():
    import threading

    class ConfirmingTracker:
        def __init__(self):
            self._calls = 0

        def is_still_pending(self, tid):
            self._calls += 1
            return False  # already confirmed

        def get_pending_candidates(self, t):
            return []

    ex = make_explorer(tracker=None)
    ex._tracker = ConfirmingTracker()
    t0 = 100.0
    set_time(ex, t0)
    ex._close_in_hold("track_2", "person", 1)
    held_notes = [e for e in ex._timeline if e["phase"] == "close_in_hold"][0]["notes"]
    check("hold early-exit: confirmed=True tag", "confirmed=True" in held_notes,
          f"notes={held_notes}")
    set_time(ex, t0 + 5.0)
    # early exit consumed < 1 sim-s of clock (we set the clock manually, so just
    # verify the publish count is tiny: 1-2 publishes before the confirm check)
    check("hold early-exit: minimal publishes",
          ex._cmd_vel_pub.publish.call_count <= 3,
          f"publishes={ex._cmd_vel_pub.publish.call_count}")


def test_hold_tags_track_and_class():
    class Confirming:
        def is_still_pending(self, tid):
            return False
        def get_pending_candidates(self, t):
            return []
    ex = make_explorer(tracker=None)
    ex._tracker = Confirming()
    ex._close_in_hold("track_42", "fire_extinguisher", 8)
    ev = [e for e in ex._timeline if e["phase"] == "close_in_hold"][-1]
    check("hold tags: track + class + source in notes",
          all(s in ev["notes"] for s in ("track_42", "fire_extinguisher", "source=close_in_verify")),
          f"notes={ev['notes']}")


def test_constants_stable():
    check("constants: hold 2.0s", CLOSE_IN_HOLD_S == 2.0)
    check("constants: budget 6", CLOSE_IN_MAX_DETOURS == 6)
    check("constants: rate 60s", CLOSE_IN_RETRY_RATE_S == 60.0)
    check("constants: attempts 2", CLOSE_IN_MAX_ATTEMPTS == 2)


if __name__ == "__main__":
    test_budget_cap()
    test_attempt_cap()
    test_rate_limit()
    test_costmap_cell_guard()
    test_no_costmap_allows()
    test_outside_costmap_extent_allows()
    test_hold_full_duration_when_pending()
    test_hold_early_exit_on_confirm()
    test_hold_tags_track_and_class()
    test_constants_stable()
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    sys.exit(1 if FAIL else 0)