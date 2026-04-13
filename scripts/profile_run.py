#!/usr/bin/env python3.12
"""
Read a profile markdown file (written by FrontierExplorer._write_timeline)
and print the summary table to stdout.

Usage:
    python3.12 scripts/profile_run.py results/profile_20260410T212000.md
    python3.12 scripts/profile_run.py results/profile_*.md   # compare multiple
"""

import re
import sys
from pathlib import Path


def extract_summary(path: Path) -> str:
    """Extract header + Summary + Per-goal sections from a profile markdown file."""
    text = path.read_text()
    lines = text.splitlines()
    out: list[str] = []
    include = True  # start including (header + stats)

    for line in lines:
        # Include everything up to and including "## Per-goal breakdown" table
        if line.startswith("## Timeline"):
            include = False  # skip the raw timeline
        if include:
            out.append(line)

    return "\n".join(out).rstrip()


def main() -> None:
    if len(sys.argv) < 2:
        # Default: find most recent profile in results/
        results = Path(__file__).resolve().parent.parent / "results"
        profiles = sorted(results.glob("profile_*.md"))
        if not profiles:
            print("No profile files found. Run a scenario first.", file=sys.stderr)
            sys.exit(1)
        paths = [profiles[-1]]
        print(f"(using most recent: {paths[0].name})\n")
    else:
        paths = [Path(a) for a in sys.argv[1:]]

    for p in paths:
        if not p.exists():
            print(f"ERROR: {p} not found", file=sys.stderr)
            continue
        if len(paths) > 1:
            print(f"{'=' * 60}")
            print(f"  {p.name}")
            print(f"{'=' * 60}")
        print(extract_summary(p))
        if len(paths) > 1:
            print()


if __name__ == "__main__":
    main()
