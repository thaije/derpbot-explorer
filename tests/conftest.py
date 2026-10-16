"""
Session guard: several test modules in this repo use an internal tally
(``check()`` appending to module-level PASS/FAIL lists) instead of asserts,
so a plain pytest run reported green even when a module's tally had FAILs —
only the direct ``python3.12 tests/<file>.py`` (__main__) run exits nonzero.
This bit us: an always-failing check in test_slam_sentinel_wiring.py stayed
invisible to pytest for several gates while the tallied run was red.

t_1779f975: after pytest finishes running every collected test module,
inspect each test module that opts in via a module-level ``TALLY_GATE = True``
marker (set it once the module's tally is green under pytest) and fail the
session if any such module recorded tally failures, so pytest counts cannot
silently drift from the tallied reality:

    before:  python3.12 -m pytest tests/ -q   ->  green despite red tally
    after:   same command -> "check() tally failures" errors + nonzero exit
"""
import sys

import pytest


def _mod_tally_failures():
    """Return [(module_name, [check names])] for every imported test module
    that opted in (TALLY_GATE = True) and whose check() tally has FAILs."""
    problems = []
    for name, mod in sorted(sys.modules.items()):
        if not name.startswith("test_"):
            continue
        if not getattr(mod, "TALLY_GATE", False):
            continue
        fail = getattr(mod, "FAIL", None)
        if isinstance(fail, list) and fail:
            problems.append((name, list(fail)))
    return problems


def pytest_sessionfinish(session, exitstatus):
    problems = _mod_tally_failures()
    if not problems:
        return
    print("\nERROR check() tally failures (module FAIL lists non-empty):")
    for mod_name, fail_names in problems:
        print(f"  {mod_name}: x{len(fail_names)}: {fail_names}")
    # Session guard: recorded tally failures must fail the pytest run too.
    # (Modules without the opt-in keep today's behavior until their pytest-mode
    # tally is genuinely green; then set TALLY_GATE = True in that module.)
    session.exitstatus = pytest.ExitCode.TESTS_FAILED