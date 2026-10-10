"""Offline replay-fixture test (t_451895af) — no ROS, no sim.

Builds a tiny hand-written sighting-stream fixture, replays it through
scripts/replay_tracker.py's loader + tracker_worker with two parameter sets,
and asserts the scored verdict flips as expected.

Run:  python3.12 tests/test_replay_fixture.py
(needs rclpy/vision_msgs on sys.path — same interpreter rule as
tests/test_tracker_gates.py)
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "agent"))

try:
    import rclpy  # noqa: F401
    import vision_msgs  # noqa: F401
    import tracker as tracker_mod  # noqa: F401
except ImportError:
    sys.exit("skipped: rclpy/vision_msgs unavailable on this box — run on ROS env")

PASS = []
FAIL = []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(f"{name} {detail}")
    print(("PASS" if cond else "FAIL"), name, detail)


# ── Fixture ──────────────────────────────────────────────────────────────────
# Spawn offset: map frame origin at world (10.0, 10.0), yaw = 90° → the map→world
# transform rotates map coords. We put the real object at a known map position
# and derive the GT world position via the scorer's transform, so the fixture
# exercises the spawn-offset code path too.
SPAWN = {"x": 10.0, "y": 10.0, "yaw": 1.5707963267948966}  # 90°


def map_to_world(mx, my, spawn=SPAWN):
    import math
    s, c = math.sin(spawn["yaw"]), math.cos(spawn["yaw"])
    return spawn["x"] + c * mx - s * my, spawn["y"] + s * mx + c * my


# Object A confirmed via 2 diverse far sightings at map (3.0, 3.0), conf 0.6
# Object B (different class) parked 0.5 m away in map coords, same robot poses —
# only passes under configs without the veto gate.
A_MAP = (3.0, 3.0)
A_WORLD = map_to_world(*A_MAP)
B_MAP = (3.5, 3.0)
B_WORLD = map_to_world(*B_MAP)

SIGHTINGS = [
    # class, conf, x, y, rx, ry, t
    ("cup",    0.6,  A_MAP[0],       A_MAP[1],       1.0, 1.0, 1.0),
    ("cup",    0.6,  A_MAP[0]+0.02,  A_MAP[1]-0.02,  1.0, 1.5, 2.0),  # diverse
    ("bottle", 0.45, B_MAP[0],       B_MAP[1],       2.0, 2.0, 3.0),
    ("bottle", 0.45, B_MAP[0]+0.02,  B_MAP[1],       2.0, 2.5, 4.0),

    # near-threshold single sighting 40 m away → marching FP candidate only
    # confirms under margin=0 only if stable; it's a single sighting so it can
    # never confirm (needs 2 diverse or 3 fast frames). Add 3 fast frames to
    # exercise fast-confirm: robot parked 0.5 m away, 3 sightings 0.1 s apart.
    ("cup",    0.7,  20.0, 20.0, 19.8, 20.0, 5.0),
    ("cup",    0.7,  20.0, 20.0, 19.8, 20.0, 5.1),
    ("cup",    0.7,  20.0, 20.0, 19.8, 20.0, 5.2),
    ("cup",    0.7,  20.0, 20.0, 19.8, 20.0, 5.3),
]


def write_fixture(dirpath: Path) -> tuple[Path, Path]:
    stream = dirpath / "fixture_stream.jsonl"
    gt = dirpath / "fixture_gt.json"
    header = {
        "type": "header",
        "spawn_pose": SPAWN,
        "map_pgm": "",
        "map_resolution": 0.5,
    }
    with open(stream, "w") as fh:
        fh.write(json.dumps(header) + "\n")
        for cls, conf, x, y, rx, ry, t in SIGHTINGS:
            fh.write(json.dumps({
                "type": "sighting",
                "class_name": cls,
                "confidence": conf,
                "bbox": [320.0, 240.0, 40.0, 30.0],
                "x": x, "y": y, "rx": rx, "ry": ry, "ryaw": 0.0,
                "t": t,
            }) + "\n")

    result = {
        "raw_metrics": {
            "ground_truth_objects": {
                "1": {"type": "cup", "instance": 0, "x": A_WORLD[0], "y": A_WORLD[1],
                      "mission_target": True},
                "2": {"type": "bottle", "instance": 0, "x": B_WORLD[0], "y": B_WORLD[1],
                      "mission_target": False},
            }
        }
    }
    gt.write_text(json.dumps(result))
    return stream, gt


def run_replay(stream: Path, gt: Path, sets: list[str]) -> dict:
    """Import the replayer as a module, monkeypatch policy, return score dict."""
    import importlib
    import replay_tracker as rt
    importlib.reload(rt)

    header, sightings = rt.load_stream(str(stream))
    gt_data = json.loads(gt.read_text())["raw_metrics"]["ground_truth_objects"]
    rt.monkeypatch_policy(sets)  # single call — env must carry the FULL set
    confirmed = rt.run_tracker(sightings)
    found, tp, fp, dp, tp_events, log = rt.score(confirmed, gt_data, header["spawn_pose"])
    return {
        "found": found, "tp": tp, "fp": fp, "dp": dp,
        "events": tp_events, "log": log, "published": confirmed,
        "gt_types": sorted({e["type"] for e in gt_data.values()}),
    }


def main() -> int:
    with tempfile.TemporaryDirectory() as td:
        stream, gt = write_fixture(Path(td))

        # ── Config A: baseline, all gates effectively off ────────────────────
        base_sets = [
            "CONFIDENCE_MARGIN=0.0",
            "STABILITY_RADIUS=100.0",   # disable positional stability
            "VETO_RADIUS=0.0",          # disable cross-class veto
        ]
        rA = run_replay(stream, gt, base_sets)

        # Cup track A: 2 diverse sightings, 0.6 ≥ 0.15+0.0 → must publish.
        # The parked 0.5 m cup cluster also confirms (fast-confirm, 4 frames ≥ 3)
        # at map (20,20) — that one has NO GT within 1.5 m → 1 FP.
        # Bottle track confirms (2 diverse, 0.45 ≥ 0.15) → GT key 2 → TP.
        check("configA: cup@GT1 is TP", any(
            e["label_key"] == "1" for e in rA["events"]), str(rA["events"]))
        check("configA: bottle@GT2 is TP", any(
            e["label_key"] == "2" for e in rA["events"]), str(rA["events"]))
        check("configA: 1 positional FP (cup@20,20)", rA["fp"] == 1,
              f"fp={rA['fp']}")
        check("configA: found=2/2", rA["found"] == 2, f"found={rA['found']}")

        # ── Config B: margin + stability restored (master defaults semantics) ─
        confB_sets = [
            "CONFIDENCE_MARGIN=0.05",   # bottle 0.45 < 0.15+0.05 = 0.20? No: passes.
            "STABILITY_RADIUS=1.2",
            "VETO_RADIUS=0.0",
        ]
        # Bottle conf 0.45 clears 0.20 — use a margin that rejects it instead.
        # Margin semantics: max_conf < OWL_CONF_THRESHOLD + MARGIN. Rejected when
        # 0.45 < 0.15+MARGIN → MARGIN > 0.30. Use 0.32 (also drops the parked
        # 0.7 conf? No: 0.7 ≥ 0.47 → still publishes; the 20,20 cluster then
        # adds 1 TP-less FP? It has no GT → still 1 FP).
        confB2_sets = [
            "CONFIDENCE_MARGIN=0.32",   # rejects bottle (0.45 < 0.47)
            "STABILITY_RADIUS=100.0",
            "VETO_RADIUS=0.0",
        ]
        rB = run_replay(stream, gt, confB2_sets)
        check("configB: bottle rejected by margin (no GT2 TP)", not any(
            e["label_key"] == "2" for e in rB["events"]), str(rB["events"]))
        check("configB: cup@GT1 still TP (0.6 ≥ 0.47)", any(
            e["label_key"] == "1" for e in rB["events"]), str(rB["events"]))

        # ── Config C: verdict flips across configs ───────────────────────────
        check("verdict flips: bottle found in A, not in B",
              any(e["label_key"] == "2" for e in rA["events"])
              and not any(e["label_key"] == "2" for e in rB["events"]))

        # ── Config D: veto holds the bottle when a different-class track is near ─
        # Put a cup track 0.1 m from the bottle (B), high conf → veto (0.8 m
        # default) holds bottle unless it out-confirms the cup by AMBIGUITY_MARGIN.
        veto_sets = ["STABILITY_RADIUS=100.0", "CONFIDENCE_MARGIN=0.0",
                     "VETO_RADIUS=0.8", "AMBIGUITY_MARGIN=0.10"]
        # restore margin first (B config changed it on this module instance)
        import importlib as _il
        _il.reload(sys.modules["tracker"])
        # fixture with a high-conf cup near bottle B
        stream2, gt2 = write_fixture(Path(td))
        with open(stream2, "a") as fh:
            fh.write(json.dumps({
                "type": "sighting", "class_name": "cup", "confidence": 0.9,
                "bbox": [320, 240, 40, 30],
                "x": B_MAP[0] - 0.05, "y": B_MAP[1], "rx": 3.0, "ry": 3.0,
                "ryaw": 0.0, "t": 6.0,
            }) + "\n")
            fh.write(json.dumps({
                "type": "sighting", "class_name": "cup", "confidence": 0.9,
                "bbox": [320, 240, 40, 30],
                "x": B_MAP[0] - 0.05, "y": B_MAP[1] + 0.02, "rx": 3.0, "ry": 3.5,
                "ryaw": 0.0, "t": 7.0,
            }) + "\n")
        rD = run_replay(stream2, gt2, veto_sets)
        # Bottle B (0.45) is now veto-held by the 0.9 cup 0.10 m away →
        # no GT2 TP in config D.
        check("veto holds bottle (no GT2 TP)", not any(
            e["label_key"] == "2" for e in rD["events"]), str(rD["events"]))

        # ── CLI smoke test ───────────────────────────────────────────────────
        proc = subprocess.run(
            [sys.executable, str(REPO_ROOT / "scripts" / "replay_tracker.py"),
             "--stream", str(stream), "--gt", str(gt),
             "--set", "CONFIDENCE_MARGIN=0.0", "--set",
             "STABILITY_RADIUS=100.0,VETO_RADIUS=0.0"],
            capture_output=True, text=True, timeout=120,
        )
        check("CLI exits 0", proc.returncode == 0, proc.stderr[-200:])
        check("CLI verdict line", proc.stdout.startswith("found="),
              proc.stdout.splitlines()[:1])

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()