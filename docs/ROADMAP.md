# ROADMAP — derpbot-explorer

Table of contents for the issue tracker. Each active task gets a short entry here with a link to the GitHub issue where the full plan, root cause, and discussion live. Do not copy issue content into this file.

Current state lives in [`STATE.md`](STATE.md). History lives in closed issues + commits + [`benchmark_results.md`](benchmark_results.md).

---

## Ground rules (apply to all upcoming work)

- **Preserve Task 4's nav baseline:** 0 collisions, ≥ 70% coverage, 5+/6 goals on easy/seed=42/`--no-perception`. Any change that regresses these without a compensating gain gets reverted.
- **No dead-end retries.** `gh issue list --state closed --label dead-end` before proposing a change in an area that has prior attempts.
- **No hardcoded class names, scenario-specific logic, or oracle mode.** Generic robot goal — see [`../CLAUDE.md`](../CLAUDE.md).

---

## Next

---

## Open backlog

Known issues not currently prioritized. Full details in the linked issues; check before starting related work in case one is already tracked.

- [#36](https://github.com/thaije/derpbot-explorer/issues/36) — Annotated camera stream with detection bboxes

Run `gh issue list --state open --label backlog` for the live list.

---

## Potential future extensions

Not tracked as issues — reconsider only if triggered.

---

## Workflow

- **Starting a task:** read `STATE.md`, `ROADMAP.md`, and the task's issue. Check closed dead-ends in the same area.
- **During a task:** log findings and decisions as comments on the issue, not in this doc.
- **New finding:** `gh issue create` with `task` / `bug` / `dead-end` / `backlog` / `upstream` label. Cross-link related issues.
- **Completing a task:** close the issue with a final comment (outcome + commit SHA). Delete the task entry from the "Next" section here. Update `STATE.md` only if a new *invariant* came out of it.
- **Commits:** reference the issue, e.g. `feat(nav2): tighter MPPI sampling (#4)`. GitHub auto-links.