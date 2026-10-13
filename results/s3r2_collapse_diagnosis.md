# s3r2 collapse diagnosis (v6-fastconfirm easy_seed3_run2, 2026-10-11)

Verdict: **NOT watchdog, NOT torch-oom, NOT silent-partial detector fire.**
The 0/6 run is a **slam_toolbox dead-silence startup abort** (#26-class
"detector silent" symptom, mechanism = no `/map` → STARTUP_MAP_TIMEOUT
explorer abort), combined with **empty agent main-process logs**.

## Evidence chain (all zero-sim, from surviving artifacts)

Stream `results/streams/stream_easy_seed3_run2_20261011T133320Z.jsonl`:
- 131 sightings, ALL `person`, sim t = 11.728 → 128.04 s, then nothing for the
  remaining 770 sim-s (mission ran 901.5 s).
- All 131 records have robot pose (0.0, 0.0) — the robot never moved (raw
  metrics confirm: meters_traveled 0.0, exploration_coverage 24.2 % [lidar-only
  initial map], collision 0).
- bbox constant (397, 234), confidence constant 0.2535 — the detector's
  inference subprocess was ALIVE and firing on a static FOV containing a
  person. So: detector fine, camera fine, relay fine.

Result JSON `robot-sandbox/results/submissions/derpbot-explorer-v6-fastconfirm/easy_seed3_run2.json`:
found_ratio 0.0, task_completion_time 902.6 (full budget elapsed),
detection_count 0 — tracker never confirmed anything (person GT13 at 4.89 m
min range was seen 131× but stalled at 1 fixed robot pose: no diverse poses,
no fast-confirm; consistent with the tracker-side bucket (b/d) note, not a new
mechanism).

Profile `results/profile_20261011T153422.md` (seed 3, mission 901.5 s,
profiled 126.9 s):
- ONE timeline entry: `startup` at sim_t 0 lasting 126.9 s, nothing after.
- startup > 120 s ⇒ matches `STARTUP_MAP_TIMEOUT=120` abort
  (frontier_explorer.py:661-670): no `/map` within 120 sim-s → publish stop,
  `_exploring=False`, `_done_callback()`. Robot sits still forever (the
  stop_cmd_vel + no goals explains meters_traveled = 0), mission timer later
  kills the run at 900 s.

Agent main-process log: the three python logs of the 13:37-13-40 stack window
(`~/.ros/log/python3_16305…`, `python3.12_16342/16343…`) are **all 0 bytes** —
the agent node's stdout/stderr produced nothing to the ROS logger file for
that run (tmux `agent` session pane is gone; stack was torn down by
cleanup.sh). The detector-side inference counts we see in the stream are the
subprocess worker (alive across the whole 128 s — relay kept getting results).
No watchdog restart, no `inference error`, no stall messages anywhere — the
watchdog never fired because inference kept producing results.

slam_toolbox log for the window (`async_slam_toolbox_node_16903_1791718633980.log`,
303 bytes): "Configuring / Using solver plugin / CeresSolver: Using SCHUR_JACOBI
preconditioner / Activating" and nothing more — the node activated but
**published no `/map` in the next 126 sim-s**. bt_navigator (16924): normal
activation, no goals ever (explorer aborted).

## Root cause (mechanism, one sentence)

slam_toolbox activated but never produced its first `/map` inside the 120-s
startup window of easy_seed3_run2 — same family as the STATE.md "startup
timeout (slam_toolbox dead)" invariant but this time with a *silent* partial
activation — the robot sat at spawn, the mission timed out at 900 s, and the
stream shows only the static-FOV person sightings from a robot that never
moved.

## Classification vs the three candidate causes

| cause | observed | verdict |
|---|---|---|
| #26-class watchdog silent/partial-fire | watchdog never fired (inference results kept flowing, no restart logged) | NO |
| env (PTHONPATH / pkg1 pin / GIL) | detector subprocess ran 128 s continuously and delivered 131 boxes; no import errors | NO |
| torch-oom / cuda hang | alloc/reserved steady; no vram errors | NO |
| slam_toolbox silent-scan → no /map in 120 s | slam log shows activation but zero scans processed; profile shows exact 126.9 s = startup abort; result = full-budget 900.6 s timeout | **YES** |

## Recommended follow-up (NOT in this task's scope)

File as its own mechanism candidate: a **slam-toolbox first-scan
sentinel** — when `async_slam_toolbox` activates, verify the sim is actually
publishing `/derpbot_0/scan` (it was: the run's sim session was healthy) and
that slam received ≥1 scan / has ≥1 keyframe within ~15 s wall; if not,
restart slam once rather than burning the whole mission budget on the
STARTUP_MAP_TIMEOUT abort path (which currently costs ~780 sim-s = a full run
when it fires).