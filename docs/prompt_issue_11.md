# Agent prompt — issue #11: verify MPPI cascade is resolved

**Task: verify issue #11 is resolved (MPPI "Failed to make progress" cascade)**

**Context.** During Task 2 verification (2026-04-09), goal #4 triggered a cascade: MPPI "Failed to make progress" → local costmap clear → "Costmap timed out" → "Resulting plan has 0 poses" loop. Task 4 then landed footprint-aware MPPI (`consider_footprint: true`, tighter sampling `vx_std: 0.15`, `wz_std: 0.3`, `cost_scaling_factor: 5.0`). The seed=42 Task 4 verification run showed zero "Failed to make progress" events, but only one seed was tested. This issue is open purely because multi-seed confirmation was never done. No code change is expected.

**Work.** Run two scored runs — seed=43 and seed=44 — using the `arst-runner` agent with the current config (perception ON, speed=2). Never start the stack manually. For each run, scan the agent logs for any of:
- `"Failed to make progress"`
- `"Costmap timed out"`
- `"Resulting plan has 0 poses"`

Also check whether any goals triggered the backup+clear recovery BT more than once in a row (cascade indicator).

**DoD.** If both seeds are clean (zero cascade events), close issue #11 with a comment: "Verified clean on seeds 43 and 44 — Task 4 MPPI tuning resolved the cascade. Zero 'Failed to make progress' events across both runs." No benchmark entry needed unless a regression is found. If the cascade reappears, do not fix it — file the reproduction details as a comment on #11 and stop.
