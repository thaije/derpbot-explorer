# CLAUDE.md — derpbot-explorer

## Session start

Read in order before doing anything:
1. `agent-scripts/AGENTS.MD` — working style + tool catalogue
2. `docs/AGENT_HANDOFF.md` — what's built, gotchas, current performance
3. `docs/approach1_classical_pipeline.md` — current plan + open tasks

---

## Hard rules

**Generic robot — no shortcuts**
The goal is a general-purpose agent that works for any object type, environment, and scenario tier.
- No hardcoded class names, object-specific thresholds, or mission-specific logic.
- No oracle mode, ground-truth positions, or other dev/cheat mechanisms in scored code.
- If a solution bypasses the stated goal, call it out explicitly before using it.
- See `docs/AUTONOMOUS_AGENT_GUIDE.md` for the full task spec and grading criteria.

**Research before building**
Web-search first for any nav, perception, ROS 2, or ML approach. Quote exact errors and prefer 2024–2025 sources. Don't invent APIs or configs from memory.

**Root-cause fixes only**
No band-aids or workarounds that mask the real problem. If the same approach fails twice: stop, summarise what you know, list ≥ 3 alternative approaches, and ask Tjalling before continuing.

**Environment is changeable**
The sim, launch files, config, and hardware setup are all fair game. If something is a fundamental bottleneck (CPU, GPU, broken config, sim bug), flag it explicitly — don't silently work around it. Hardware issues (e.g. thermal throttling on NUMA node 0) should be escalated to Tjalling.

---

## Docs

- Lessons learned, gotchas, architecture decisions → `docs/AGENT_HANDOFF.md`. Keep new entries ≤ 3 lines each.
- No new doc files without asking. No duplicating content across docs.
- Update `AGENT_HANDOFF.md` whenever behaviour, config, or APIs change. Don't ship without a doc update.
- Keep all doc entries brief and telegraph-style (noun phrases ok, drop filler).

---

## Code quality

- Files approaching ~500 LOC: split or refactor before adding more code.
- After any major addition: check whether existing code can be simplified, merged, or removed.
- Prefer end-to-end verification over unit tests in isolation; if blocked, say what's missing.

---

## Git

- Commit after any significant improvement using `committer` (Conventional Commits: `feat|fix|refactor|docs|perf|chore`).
- Don't push unless Tjalling asks.
- Keep commits small and reviewable; no repo-wide search/replace scripts.
