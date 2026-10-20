# v13-cv-fu re-gate bars (t_d479428a) — pre-registered 2026-10-20 BEFORE the 5×1

## Bars (SAME as the t_8ed4eb97 gate — committed here before any run)

- **PASS ⇔ mean found ≥ 4.9 AND FP/run ≤ 4.0 AND ≥ 3/5 completions → proceed (sibling 5×3 card t_6124e5bb runs v13-cv-fu @ HEAD).**
- **REJECT ⇔ mean found ≤ 4.5 OR FP/run > 5.0 → halt (sibling 5×3 card runs cb4ce47 as-is per its pre-registered selection rule).**
- **Between → attribution + halt (sibling 5×3 card runs cb4ce47 as-is).**

Escalation guards (reference): dup ≤ 7.6, collisions ≤ 1.0 — not gate legs.

## Protocol

Agent derpbot-explorer-v13-cv-fu @ HEAD (a0b640f — cb4ce47 + the injection-conditional
split-evidence fix; only delta vs the t_8ed4eb97 gate agent). DERPBOT_VLM_VERIFY=1 is the
ONLY env behavior delta; everything else off. Easy 5×1, one run per seed (seeds 1–5),
900 s sim-time, speed 2, pkg1 pin (cores 10-19,30-39), port 7400 pre-flight + cleanup,
one sim at a time, 5/5 runs required with 0 failures — any failed run = quota/retry rules
(≤2 retries/call, budget 30 calls/run, 429 = quota-stop for the verifier; failed SIM runs
are retried once then the halt rule applies as in t_8ed4eb97's round-1 void).

Par basis: v1-human-derived (easy par 6.00/6) + v2 human re-baseline PENDING — same-sandbox
(v2.0.0) agent deltas only; absolute levels carry the v2-shift caveat.