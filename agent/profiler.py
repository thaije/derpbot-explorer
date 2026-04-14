"""
Run profiler — GoalStats dataclass + per-goal timing table + markdown timeline writer.

Extracted from frontier_explorer.py to keep that module under the ~500 LOC guideline.
Called from FrontierExplorer._log_timing_table / _write_timeline thin wrappers.
"""

from __future__ import annotations

import json
import math
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass
class GoalStats:
    """Per-goal timing breakdown. All times in sim-seconds; NaN = not applicable."""
    goal_num: int
    is_patrol: bool
    result: Optional[bool]          # True=success, False=fail/stuck, None=rejected
    bfs_and_selection_s: float      # BFS + scoring + inter-goal overhead
    accept_latency_s: float         # send_goal_async → goal accepted (NaN if not accepted)
    time_to_first_move_s: float     # accepted → first 0.15 m displacement (NaN if never moved)
    nav_time_s: float               # accepted → result (0.0 if never accepted)
    post_goal_pause_s: float        # recovery sleeps in explore_loop after dispatch returns


def log_timing_table(logger, goal_stats: list[GoalStats]) -> None:
    """Log a per-phase timing summary for the completed exploration run."""
    stats = goal_stats
    if not stats:
        return

    def _vals(attr: str) -> list[float]:
        return [getattr(s, attr) for s in stats if not math.isnan(getattr(s, attr))]

    nav_stats = [s for s in stats if not math.isnan(s.accept_latency_s)]
    travel_vals = [
        s.nav_time_s - s.time_to_first_move_s
        for s in nav_stats
        if not math.isnan(s.time_to_first_move_s)
    ]

    phases: list[tuple[str, list[float]]] = [
        ("BFS + selection",    _vals("bfs_and_selection_s")),
        ("Nav2 accept",        _vals("accept_latency_s")),
        ("Rotation / spin-up", _vals("time_to_first_move_s")),
        ("Travel to goal",     travel_vals),
        ("Post-goal pause",    _vals("post_goal_pause_s")),
    ]

    total_s = sum(
        s.bfs_and_selection_s + s.nav_time_s + s.post_goal_pause_s
        for s in stats
    )
    n_ok   = sum(1 for s in stats if s.result is True)
    n_fail = sum(1 for s in stats if s.result is False)
    n_rej  = sum(1 for s in stats if s.result is None)

    sep = "=" * 72
    lines = [
        "",
        sep,
        "TIMING TABLE — per-goal navigation stack breakdown (sim-seconds)",
        sep,
        f"{'Phase':<24} {'N':>4} {'Mean':>7} {'Median':>7} {'Max':>7} {'% total':>8}",
        "-" * 56,
    ]
    for name, vals in phases:
        if not vals:
            lines.append(f"  {name:<22} {'—':>4}")
            continue
        mean   = float(np.mean(vals))
        median = float(np.median(vals))
        mx     = float(np.max(vals))
        pct    = 100.0 * sum(vals) / total_s if total_s > 0 else 0.0
        lines.append(
            f"  {name:<22} {len(vals):>4} {mean:>6.1f}s {median:>6.1f}s"
            f" {mx:>6.1f}s {pct:>7.1f}%"
        )
    lines += [
        "-" * 56,
        f"  Goals dispatched: {len(stats)} | success={n_ok} fail={n_fail} rejected={n_rej}",
        f"  Total tracked sim-time: {total_s:.0f}s",
        sep,
    ]
    for line in lines:
        logger.info(line)


def write_timeline(
    logger,
    timeline: list[dict],
    t_end: float,
    meters_traveled: float,
    phase_dist: dict[str, float],
    phase_rot: dict[str, float],
    phase_sample_time: dict[str, float],
) -> None:
    """Write a self-contained markdown profile: summary table + raw timeline."""
    entries = timeline
    if not entries:
        return

    t0 = entries[0]["t"]
    total_time = t_end - t0
    if total_time <= 0:
        return

    # Compute duration per entry (gap to next; last entry → t_end)
    for i in range(len(entries)):
        t_next = entries[i + 1]["t"] if i + 1 < len(entries) else t_end
        entries[i]["dur"] = t_next - entries[i]["t"]

    # Summary: group by phase
    summary: dict[str, dict] = defaultdict(lambda: {"count": 0, "total": 0.0})
    for e in entries:
        s = summary[e["phase"]]
        s["count"] += 1
        s["total"] += e["dur"]

    traveling_time = summary["traveling"]["total"] if "traveling" in summary else 0.0

    # Compare profiled budget against the sandbox mission time (if available)
    # to surface any unmeasurable startup gap — time spent BEFORE _explore_loop
    # runs (mission fetch, OWLv2 detector spawn, AgentNode init).
    unmeasured_gap_str = "n/a (no recent sandbox result found)"
    scenario_str = "unknown"
    seed_str = "unknown"
    try:
        sandbox_results = Path.home() / "Projects" / "robot-sandbox" / "results"
        if sandbox_results.is_dir():
            candidates = sorted(
                sandbox_results.glob("*.json"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            if candidates:
                latest = candidates[0]
                age_s = datetime.now().timestamp() - latest.stat().st_mtime
                # Window must cover: sandbox write → bailout → node shutdown
                # → profile write. Observed ~190s; 15 min gives headroom
                # without risking cross-run confusion (most-recent wins).
                if age_s < 900.0:
                    data = json.loads(latest.read_text())
                    raw = data.get("raw_metrics", {}) or {}
                    scenario_str = str(data.get("scenario_name") or "unknown")
                    seed_str = str(data.get("random_seed") or "unknown")
                    mission_time = float(
                        raw.get("task_completion_time")
                        or data.get("elapsed_seconds")
                        or 0.0
                    )
                    if mission_time > 0:
                        gap = mission_time - total_time
                        # Negative gap = profile window exceeds mission
                        # (shutdown overhead after timeout). Means no
                        # measurable pre-_explore_loop gap exists.
                        gap_fmt = (
                            f"{gap:.1f}s" if gap > 1.0
                            else f"~0s (profile ≥ mission)"
                        )
                        unmeasured_gap_str = (
                            f"{gap_fmt}"
                            f" (mission={mission_time:.1f}s,"
                            f" profiled={total_time:.1f}s;"
                            f" source={latest.name})"
                        )
    except Exception as exc:
        logger.debug(f"Unmeasured-gap lookup failed: {exc}")
        unmeasured_gap_str = f"n/a ({exc})"

    lines: list[str] = []
    ts = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    lines.append(f"# Run Profile — {ts}")
    lines.append("")
    lines.append(f"- **Scenario:** {scenario_str}  ·  **Seed:** {seed_str}")
    lines.append(f"- **Budget:** {total_time:.1f} sim-s"
                 f" (from first `_tl` call to end — includes `startup` phase)")
    lines.append(f"- **Unmeasured startup:** {unmeasured_gap_str}")
    lines.append(f"- **Distance:** {meters_traveled:.1f} m (odom-integrated)")
    lines.append(f"- **Effective speed:** {meters_traveled / total_time:.3f} m/s"
                 f" ({meters_traveled / total_time * 3.6:.2f} km/h)")
    if traveling_time > 0:
        lines.append(f"- **Moving speed:** {meters_traveled / traveling_time:.3f} m/s"
                     f" (during 'traveling' phases only)")
        lines.append(f"- **Moving fraction:** {traveling_time / total_time * 100:.1f}%"
                     f" ({traveling_time:.1f} / {total_time:.1f} s)")
    lines.append("")

    # Summary table — sorted by total time descending
    lines.append("## Summary")
    lines.append("")
    lines.append("| Phase | Count | Total (s) | Mean (s) | % budget |"
                 " Avg vx (m/s) | Avg wz (rad/s) |")
    lines.append("|---|---|---|---|---|---|---|")
    for phase, s in sorted(summary.items(), key=lambda x: -x[1]["total"]):
        mean = s["total"] / s["count"]
        pct = 100.0 * s["total"] / total_time
        # Divide integrators by actual sample time (Σdt during this phase)
        # — not timeline duration — so dropped callbacks don't bias avgs.
        sample_t = phase_sample_time.get(phase, 0.0)
        avg_vx = phase_dist.get(phase, 0.0) / sample_t if sample_t > 0 else 0.0
        avg_wz = phase_rot.get(phase, 0.0) / sample_t if sample_t > 0 else 0.0
        lines.append(f"| {phase} | {s['count']} | {s['total']:.1f} |"
                     f" {mean:.1f} | {pct:.1f}% | {avg_vx:.3f} | {avg_wz:.3f} |")
    lines.append(f"| **Total** | | **{total_time:.1f}** | | **100%** | | |")
    lines.append("")
    lines.append("> ⚠️ **`rotating` / `waiting` phase durations and `Avg wz` are unreliable.**"
                 " The sub-phase classifier in `_send_goal_and_wait` is sticky: when the"
                 " polling loop is GIL-starved under Nav2 contention, it stops re-evaluating,"
                 " so a single `rotating` entry can span the full nav window (active rotation"
                 " + idle gaps). The time-weighted `Avg wz` therefore reads much lower than the"
                 " bot's actual commanded/peak wz. Do **not** infer \"rotation is slow\" from"
                 " this number — instrument `cmd_vel` directly. See #16 closing notes.")
    lines.append("")

    # Per-goal summary
    goal_nums = sorted({e["goal"] for e in entries if e["goal"] > 0})
    if goal_nums:
        lines.append("## Per-goal breakdown")
        lines.append("")
        lines.append("| Goal | Nav (s) | Waiting (s) | Rotating (s) |"
                     " Traveling (s) | BFS+Sel (s) | Result |")
        lines.append("|---|---|---|---|---|---|---|")
        for gn in goal_nums:
            ge = [e for e in entries if e["goal"] == gn]
            nav_t = sum(e["dur"] for e in ge
                        if e["phase"] in ("waiting", "rotating", "traveling",
                                          "nav2_accepted"))
            wait_t = sum(e["dur"] for e in ge if e["phase"] == "waiting")
            rot_t  = sum(e["dur"] for e in ge if e["phase"] == "rotating")
            trav_t = sum(e["dur"] for e in ge if e["phase"] == "traveling")
            bfs_t  = sum(e["dur"] for e in ge
                         if e["phase"] in ("bfs_detect", "frontier_select"))
            result_phases = [e["phase"] for e in ge
                             if e["phase"] in ("goal_reached", "goal_failed", "goal_stuck")]
            result_str = result_phases[-1] if result_phases else "?"
            lines.append(f"| {gn} | {nav_t:.1f} | {wait_t:.1f} | {rot_t:.1f} |"
                         f" {trav_t:.1f} | {bfs_t:.1f} | {result_str} |")
        lines.append("")

    # Raw timeline
    lines.append("## Timeline")
    lines.append("")
    lines.append("| sim_t | dur_s | phase | goal | vx | wz | notes |")
    lines.append("|---|---|---|---|---|---|---|")
    for e in entries:
        t_rel = e["t"] - t0
        lines.append(f"| {t_rel:.1f} | {e['dur']:.1f} | {e['phase']} |"
                     f" {e['goal']} | {e['vx']:.2f} | {e['wz']:.2f} |"
                     f" {e.get('notes', '')} |")
    lines.append("")

    # Write file
    out_dir = Path(__file__).resolve().parent.parent / "results"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / f"profile_{datetime.now().strftime('%Y%m%dT%H%M%S')}.md"
    out_path.write_text("\n".join(lines) + "\n")
    logger.info(f"FrontierExplorer: profile written to {out_path}")
