"""
SlamSentinel — SLAM first-map startup watchdog with one-restart recovery.

Background (t_8673a87f; s3r2 collapse diagnosis in results/s3r2_collapse_diagnosis.md):
async_slam_toolbox can silently activate without producing its first /map
(zero scans processed — s3r2 easy_seed3_run2 burned the full 901 s mission
budget on the STARTUP_MAP_TIMEOUT=120 abort path). This sentinel watches the
explorer's map-wait window:

  WAITING             /map absent — tick() is a no-op until the grace window
                      elapses (happy path: healthy runs see /map in 3.6–19.5
                      sim-s over a 116-run sample, p50 13.4 / p95 19.4; the
                      sentinel never fires there).
  → RESTART_REQUESTED watcher armed: emit the slam-restart flag file ONCE
                      (never twice) for scripts/slam_restart_watch.sh, which
                      kills and relaunches the slam tmux session. The agent
                      keeps spinning (unchanged) and waits one more full grace
                      window.
  → ABORTED           watcher not armed (manual run, nothing to restart), or
                      /map still absent after the post-restart window: abort
                      exactly like today's timeout, ~90 sim-s worst case
                      instead of 120.

Score-neutral by construction: no decision is made while /map is missing for
less than one grace window, and 45 sim-s is >2.3x the worst observed healthy
startup (19.5 s).

Restart contract: the agent CANNOT restart slam_toolbox itself (only the
outer stack script may restart nodes without duplicating them). It writes the
flag file; the watcher — armed by scripts/start_stack.sh in its own tmux
session — performs exactly one kill/relaunch. The flag is cleared by
start_stack.sh at every stack start, so a stale flag can never trigger a
restart of a fresh stack.

Pure stdlib: NO rclpy import. The state machine is unit-tested with a fake
clock (tests/test_slam_sentinel.py); ROS integration lives in
frontier_explorer._explore_loop.

Tunables (module constants, no per-run config):
  SLAM_SENTINEL_GRACE_S          grace window (sim-s) before restart request
                                 and again before the final abort. 45.0.
  SLAM_RESTART_FLAG_DEFAULT      flag file path. Overridable via
                                 DERPBOT_SLAM_RESTART_FLAG in start_stack.sh.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Callable, Optional

SLAM_SENTINEL_GRACE_S = 45.0
SLAM_RESTART_FLAG_DEFAULT = "/tmp/derpbot_slam_restart_request"

# State names (kept strings so logs/timeline tags are readable).
ST_WAITING = "WAITING"
ST_RESTART_REQUESTED = "RESTART_REQUESTED"
ST_RECOVERED = "RECOVERED"
ST_ABORTED = "ABORTED"


@dataclass
class SentinelDecision:
    """One terminal-or-once action returned by tick()."""

    action: str  # "restart" | "abort"
    reason: str  # "no_map_watcher_armed" | "no_map_after_restart" | "no_map_no_watcher"
    sim_t: float  # sim-time (from the injected clock) the decision fired at


class SlamSentinel:
    """Pure-logic SLAM first-map sentinel. All time comes from `clock`.

    Parameters
    ----------
    clock : Callable[[], float]
        Sim-clock provider (frontier_explorer._sim_time); injected so tests
        control time without sleeps.
    watcher_armed : bool
        True iff a restart watcher can actually consume the flag (env
        DERPBOT_SLAM_WATCHER_ARMED=1 set by start_stack.sh). When False the
        sentinel never requests a restart — requesting one would only waste a
        second grace window before the same abort.
    grace_s : float
        Window before the restart request, and again before the final abort.
    """

    def __init__(
        self,
        clock: Callable[[], float],
        watcher_armed: bool,
        grace_s: float = SLAM_SENTINEL_GRACE_S,
    ):
        self.state = ST_WAITING
        self._clock = clock
        self._watcher_armed = watcher_armed
        self._grace_s = grace_s
        self._t_start = float(clock())
        self._t_restart: Optional[float] = None
        self._restart_requested = False

    @property
    def restart_requested(self) -> bool:
        return self._restart_requested

    def mark_map(self) -> None:
        """First /map arrived (any state other than ABORTED → recovered)."""
        if self.state in (ST_WAITING, ST_RESTART_REQUESTED):
            self.state = ST_RECOVERED

    def tick(self) -> Optional[SentinelDecision]:
        """Advance the state machine. Returns a decision exactly when one
        must be acted on (a restart request ONCE, or the final abort); None
        otherwise. After RECOVERED/ABORTED, always None."""
        if self.state in (ST_RECOVERED, ST_ABORTED):
            return None
        now = float(self._clock())
        if self.state == ST_WAITING:
            if now - self._t_start < self._grace_s:
                return None
            if not self._watcher_armed:
                self.state = ST_ABORTED
                return SentinelDecision("abort", "no_map_no_watcher", now)
            self.state = ST_RESTART_REQUESTED
            self._t_restart = now
            self._restart_requested = True
            return SentinelDecision("restart", "no_map_watcher_armed", now)
        # ST_RESTART_REQUESTED: wait one full grace window for the restarted
        # slam to push its first /map, then abort like today.
        if self._t_restart is not None and now - self._t_restart < self._grace_s:
            return None
        self.state = ST_ABORTED
        return SentinelDecision("abort", "no_map_after_restart", now)


def write_restart_flag(
    path: str, *, reason: str, sim_t: float, scan_seen: bool,
    extra: Optional[dict] = None,
) -> bool:
    """Emit the slam-restart request flag. Returns False (never raises) if the
    path is unwritable — the sentinel then simply continues to the abort path
    like a watcher-less run.

    Content is diagnostic JSON: reason, sim clock at request time, and whether
    the sim was publishing /derpbot_0/scan at all (a dead sim cannot be fixed
    by restarting slam — the flag content makes that visible post-run)."""
    payload = {"reason": reason, "sim_t": sim_t, "scan_seen": scan_seen}
    if extra:
        payload.update(extra)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f)
    except OSError:
        return False
    return True