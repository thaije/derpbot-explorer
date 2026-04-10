# CLAUDE.md — derpbot-explorer

## Session start

Tjalling owns this. Start: say hi + 1 motivating line.

Read in order before doing anything:
1. This doc — working style 
2. `docs/STATE.md` — what's built, current performance, invariants
3. `docs/ROADMAP.md` — next work, TOC of the issue tracker
Do NOT skip reading these files!

Open and closed work lives in GitHub issues (`gh issue list`). Before proposing a change in a previously-touched area, check closed issues: `gh issue list --state closed --label dead-end` and `gh issue list --state closed --label task`.
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
The sim, launch files, config, and hardware setup are all fair game. If something is a fundamental bottleneck (CPU, GPU, broken config, sim bug), flag it explicitly — don't silently work around it. Hardware issues should be escalated to Tjalling.

---

## Git

- Commit after any significant improvement using `committer` (Conventional Commits: `feat|fix|refactor|docs|perf|chore`).
- Don't push unless Tjalling asks.
- Keep commits small and reviewable; no repo-wide search/replace scripts.

---

## Docs

**One fact, one home.** Don't duplicate content across files.

- **`docs/STATE.md`** — invariants + current architecture/config. Keep entries ≤ 3 lines each, telegraph-style. Update only when a behaviour/config/API change produces a *new invariant* that future agents need in-context.
- **`docs/ROADMAP.md`** — short TOC of the issue tracker, one entry per active task with a link. Full plans live in the GitHub issue, not here.
- **GitHub issues** — anything with a lifecycle: tasks, bugs, dead-ends, backlog, upstream-asks. Labels: `task`, `bug`, `dead-end`, `backlog`, `upstream`. Cross-link related issues.
- **Commit messages** — the "why" of code changes. Reference the issue (`feat(nav2): X (#4)`). Commits + closed issues = project history.
- **`docs/benchmark_results.md`** — append-only metric log.
- No new doc files without asking.

---

## Code quality

- Files approaching ~500 LOC: split or refactor before adding more code.
- After any major addition: check whether existing code can be simplified, merged, or removed.
- Prefer end-to-end verification over unit tests in isolation; if blocked, say what's missing.

---

## Way of working
- Style: telegraph; noun-phrases ok; drop grammar; min tokens.
- Web: search early; quote exact errors; prefer 2024–2025 sources.
- Unsure: read more code; if still stuck, ask w/ short options.
- Conflicts: call out; pick safer path.
- Leave breadcrumb notes in thread.


## Tools
CLI tools available on Tjalling's machines. Use these for agentic tasks.

### gh
- GitHub CLI for PRs/CI/releases. Given issue/PR URL (or `/pull/5`): use `gh`, not web search.
- Examples: `gh issue view <url> --comments -R owner/repo`, `gh pr view <url> --comments --files -R owner/repo`.

### tmux
- Use for persistent/interactive tasks (debugger/server).
- Start session: `tmux new -s <name> -d '<command>'   # detached`
- View output (non-interactive): `tmux capture-pane -t <name> -p -S -30   # last 30 lines`
- Manage sessions:
    - `tmux ls                              # list all sessions`
    - `tmux attach -t <name>                # attach to session`
    - `tmux kill-session -t <name>          # kill session`
```

### Serena MCP (symbol navigation)
- **Use for Python**: `find_symbol`, `find_referencing_symbols`, `replace_symbol_body` instead of Read+Grep when you know the symbol name.
- **Skip for**: YAML, SDF, URDF, shell scripts, launch files — no language server benefit.
- Prefer `find_symbol` over reading whole files; prefer `find_referencing_symbols` over grep for cross-file traces.
- First run in a session downloads language server via uvx — normal, wait it out.

### ast-grep
- Structural code search: `ast-grep --lang python -p '$FUNC($$$)' src/`
- Use over grep when matching code structure matters (e.g. all callers of a method, all class definitions).

