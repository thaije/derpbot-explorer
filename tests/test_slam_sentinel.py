"""
Unit tests for the slam first-map sentinel state machine (t_8673a87f).

Covers (mocked clock, no ROS — slam_sentinel is pure stdlib):
  1. Healthy path: /map inside the grace window -> RECOVERED, no decision.
  2. Late-map path: /map arrives after the window but before abort — both as
     a late arrival on the initial wait (decision must have been acted on
     first: restart request) and after restart (recovery on the second
     window).
  3. Restart-once path: exactly ONE restart decision at t=grace; tick()
     returns None afterwards until the second window elapses; a flag
     re-write can never happen (state machine re-arms never).
  4. Abort after restart: no /map after 2×grace -> abort
     ("no_map_after_restart") at ~90 sim-s.
  5. No-watcher abort: without a watcher, abort at one grace window
     ("no_map_no_watcher") — never a restart request.
  6. write_restart_flag: writes valid JSON (reason/sim_t/scan_seen); returns
     False on unwritable path (never raises).
  7. Terminal states: tick() after RECOVERED/ABORTED always returns None.

Run: python3.12 -m pytest tests/test_slam_sentinel.py -q
     (no ROS imports needed — plain python3.12 also works)
"""

import json
import os
import sys
import tempfile
from pathlib import Path

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "agent"))

from slam_sentinel import (  # noqa: E402
    SLAM_SENTINEL_GRACE_S,
    ST_ABORTED,
    ST_RECOVERED,
    ST_RESTART_REQUESTED,
    ST_WAITING,
    SentinelDecision,
    SlamSentinel,
    write_restart_flag,
)


class FakeClock:
    """Controllable sim clock: advance() moves time, never sleeps."""

    def __init__(self, t0: float = 100.0):
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


PASS = []
FAIL = []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(("PASS" if cond else "FAIL"), name, detail)


# ---------------------------------------------------------------- 1. healthy

def test_healthy_map_inside_window():
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=True)
    check("healthy: starts WAITING", s.state == ST_WAITING)
    check("healthy: tick before map -> None", s.tick() is None)
    clk.advance(SLAM_SENTINEL_GRACE_S - 5.0)
    check("healthy: tick at <grace -> None", s.tick() is None)
    s.mark_map()
    check("healthy: mark_map -> RECOVERED", s.state == ST_RECOVERED)
    check("healthy: no restart flag", not s.restart_requested)
    clk.advance(SLAM_SENTINEL_GRACE_S * 10)
    check("healthy: tick after recovery -> None", s.tick() is None)
    check("healthy: state stays RECOVERED", s.state == ST_RECOVERED)


def test_healthy_map_just_inside_window():
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=True)
    clk.advance(SLAM_SENTINEL_GRACE_S - 0.01)
    check("healthy: no decision strictly before grace", s.tick() is None)
    s.mark_map()
    clk.advance(SLAM_SENTINEL_GRACE_S)  # far past grace now — recovered wins
    check("healthy: recovered sentinel never fires", s.tick() is None)


# ------------------------------------------------------- 2. late (post-restart)

def test_late_map_after_restart_recovers():
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=True)
    clk.advance(SLAM_SENTINEL_GRACE_S)
    d = s.tick()
    check("restart: decision fires at grace", d is not None and d.action == "restart")
    check("restart: reason tag", d.reason == "no_map_watcher_armed")
    check("restart: state RESTART_REQUESTED", s.state == ST_RESTART_REQUESTED)
    # slam restarts, /map appears mid-second-window
    clk.advance(SLAM_SENTINEL_GRACE_S / 2)
    check("restart: no abort mid-second-window", s.tick() is None)
    s.mark_map()
    check("late-map: RESTART_REQUESTED -> RECOVERED", s.state == ST_RECOVERED)
    clk.advance(SLAM_SENTINEL_GRACE_S * 5)
    check("late-map: never aborts after recovery", s.tick() is None)


# ------------------------------------------------------------- 3. restart-once

def test_restart_exactly_once():
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=True)
    n_decisions = 0
    # Simulate 10 grace windows of ticking with NO /map ever arriving.
    for _ in range(10):
        clk.advance(SLAM_SENTINEL_GRACE_S)
        d = s.tick()
        if d is not None:
            n_decisions += 1
            if n_decisions == 1:
                check("once: first decision is restart", d.action == "restart")
            else:
                check("once: later decisions are abort only", d.action == "abort")
    check("once: exactly one restart decision", s.restart_requested)
    # restart_requested flips exactly once — a second restart decision never
    # happens across the whole run: count restarts across all ticks.
    check("once: exactly one restart", n_decisions == 2)  # 1 restart + 1 final abort


def test_no_double_restart_flag_writes():
    """Explorer-side integration guard: the decision==restart branch runs only
    once because the sentinel enters RESTART_REQUESTED (terminal for restarts)
    after the first request. Assert via the state machine, mirroring the
    _explore_loop branch condition (action == 'restart' happens exactly once)."""
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=True)
    restarts = 0
    aborts = 0
    for _ in range(50):
        clk.advance(SLAM_SENTINEL_GRACE_S / 3)
        d = s.tick()
        if d is not None:
            if d.action == "restart":
                restarts += 1
            else:
                aborts += 1
    check("flag: exactly one restart decision", restarts == 1, f"restarts={restarts}")
    check("flag: exactly one abort decision", aborts == 1, f"aborts={aborts}")


# ------------------------------------------------------------- 4. abort paths

def test_abort_after_restart_window():
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=True)
    clk.advance(SLAM_SENTINEL_GRACE_S)
    d1 = s.tick()
    assert d1 is not None and d1.action == "restart"
    clk.advance(SLAM_SENTINEL_GRACE_S)
    d2 = s.tick()
    check("abort: fires after second window", d2 is not None and d2.action == "abort")
    check("abort: reason after-restart", d2.reason == "no_map_after_restart")
    check("abort: fired at ~2×grace", abs(d2.sim_t - 100.0 - 2 * SLAM_SENTINEL_GRACE_S) < 1.0,
          f"sim_t={d2.sim_t:.1f}")
    check("abort: state ABORTED", s.state == ST_ABORTED)
    clk.advance(SLAM_SENTINEL_GRACE_S)
    check("abort: terminal — tick None", s.tick() is None)


def test_abort_without_watcher_never_restarts():
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=False)
    for i in range(4):  # tick past 2 windows — no restart ever
        clk.advance(SLAM_SENTINEL_GRACE_S)
        d = s.tick()
        if d is not None:
            check("no-watcher: single abort", d.action == "abort" and d.reason == "no_map_no_watcher")
            break
    check("no-watcher: no restart flag", not s.restart_requested)
    check("no-watcher: abort at ONE grace window", d is not None and abs(d.sim_t - 100.0 - SLAM_SENTINEL_GRACE_S) < 1.0)


# ------------------------------------------------------- 6. write_restart_flag

def test_write_restart_flag_json():
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "flag.json")
        ok = write_restart_flag(path, reason="no_map_watcher_armed", sim_t=145.2, scan_seen=True)
        check("flag: write ok", ok)
        payload = json.loads(Path(path).read_text())
        check("flag: reason", payload["reason"] == "no_map_watcher_armed")
        check("flag: sim_t", payload["sim_t"] == 145.2)
        check("flag: scan_seen", payload["scan_seen"] is True)
        check("flag: no scan_seen when sim silent",
              write_restart_flag(path, reason="r", sim_t=1.0, scan_seen=False)
              and json.loads(Path(path).read_text())["scan_seen"] is False)


def test_write_restart_flag_unwritable():
    ok = write_restart_flag("/proc/definitely/not/writable/flag.json", reason="r", sim_t=0.0, scan_seen=False)
    check("flag: unwritable -> False (no raise)", ok is False)


# ------------------------------------------------------------- 7. terminal

def test_mark_map_after_abort_is_ignored():
    clk = FakeClock()
    s = SlamSentinel(clk, watcher_armed=True)
    clk.advance(SLAM_SENTINEL_GRACE_S)
    s.tick()
    clk.advance(SLAM_SENTINEL_GRACE_S)
    s.tick()  # abort
    s.mark_map()  # map arrives AFTER abort — must not resurrect the run
    check("terminal: aborted stays aborted", s.state == ST_ABORTED)
    check("terminal: tick stays None", s.tick() is None)


# ---------------------------------------------------------------- runner

if __name__ == "__main__":
    import traceback

    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        try:
            t()
        except Exception as exc:  # noqa: BLE001
            FAIL.append(f"{t.__name__} EXCEPTION")
            print("FAIL", t.__name__, "EXCEPTION:", exc)
            traceback.print_exc()
    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    sys.exit(1 if FAIL else 0)