#!/usr/bin/env python3
"""Recover per-object yaw + exact z for every benchmark GT run (t_fe1459a9, review r2 item 3).

The result-JSON GT carries only (type, x, y): no yaw, no z. The sim's object
placer assigns yaw ∈ {0, π/2, π, 3π/2} at placement time and z at spawn, and
the WorldGenerator that ran at scenario-launch time is DETERMINISTIC in
(random_seed, scenario YAML) — replaying it reproduces the run's GT
coordinates exactly (verified: 368/368 objects across all v10-assoc /
stream-corpus / v2holdout easy runs ≤ 0.002 m; stress seeds 6–7 ≤ 0.001 m —
generated_gt was built with the same tool).

The generated SDF embeds each object as a <model> with a 6-DOF pose, so the
yaw and z ARE recoverable offline, with no sim:

    <model name="person_0"><pose>x y z 0 0 yaw</pose>

Pairing: label_map (placement order, 1..N) ↔ GT JSON keys (1..N, same order)
verified by type + x/y equality; the SDF model name is f"{type}_{instance}".

Output: results/recog_benchmark/generated_gt_with_yaw/<result-json-stem>.json
  {key: {type, x, y, z, yaw, instance, mission_target}}
plus <name> for the stress runs (keyed by stream run name, mirroring
generated_gt/). A coverage report prints any run that fails to replay.

Run:  .venv/bin/python scripts/replay_object_yaws.py
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

EXPLORER_ROOT = Path("/home/plip/Projects/derpbot-explorer")
SANDBOX_ROOT = Path("/home/plip/Projects/robot-sandbox")
sys.path.insert(0, str(SANDBOX_ROOT / "src"))
from world_manager.world_generator import WorldGenerator  # noqa: E402

OUT_DIR = EXPLORER_ROOT / "results" / "recog_benchmark" / "generated_gt_with_yaw"
SCENARIO_YAMLS = {
    "easy": SANDBOX_ROOT / "config/scenarios/office_explore_detect/easy.yaml",
    "perception_stress": SANDBOX_ROOT /
    "config/scenarios/office_explore_detect/perception_stress.yaml",
}
GT_DIRS = [
    SANDBOX_ROOT / "results/submissions/derpbot-explorer-v10-assoc",
    SANDBOX_ROOT / "results/submissions/derpbot-explorer-stream-corpus",
    SANDBOX_ROOT / "results/submissions/derpbot-explorer-v2holdout",
]
GEN_GT_DIR = EXPLORER_ROOT / "results/recog_benchmark/generated_gt"


def sdf_object_poses(sdf_path: Path):
    """label-ordered [(type, instance, x, y, z, yaw)] from the generated SDF.

    The WorldGenerator appends placed objects (in placement order, which is
    exactly the label_map order) as trailing <model> elements whose names are
    f"{type}_{instance}" — skip the template furniture by name pattern.
    """
    root = ET.parse(sdf_path).getroot()
    out = []
    for model in root.iter("model"):
        pose = model.find("pose")
        name = model.get("name", "")
        if pose is None or "_" not in name:
            continue
        parts = pose.text.split()
        if len(parts) != 6:
            continue
        mtype, _, inst = name.rpartition("_")
        if not inst.isdigit():
            continue
        x, y, z, _rx, _ry, yaw = (float(v) for v in parts)
        out.append((mtype, int(inst), x, y, z, yaw))
    return out


def replay(seed: int, difficulty: str):
    cfg = yaml.safe_load(open(SCENARIO_YAMLS[difficulty]))
    cfg["scenario"]["random_seed"] = seed
    gen = WorldGenerator(output_dir="/tmp/arst_replay_test", project_root=str(SANDBOX_ROOT))
    sdf_path = gen.generate(cfg)
    label_map = gen.label_map
    poses = sdf_object_poses(sdf_path)
    if len(poses) < len(label_map):
        raise RuntimeError(f"only {len(poses)} SDF object poses for {len(label_map)} labels")
    # label_map order == placement order == trailing model order
    by_type_inst = {}
    for (mtype, inst, x, y, z, yaw) in poses:
        by_type_inst[(mtype, inst)] = (x, y, z, yaw)
    out = {}
    for k, v in label_map.items():
        key = (v["type"], v["instance"])
        if key not in by_type_inst:
            raise RuntimeError(f"label {k} type/instance {key} not in SDF")
        x, y, z, yaw = by_type_inst[key]
        out[k] = {**v, "z": round(z, 4), "yaw": round(yaw, 4)}
    return out


def verify(gt: dict, replayed: dict, tol: float = 0.002) -> tuple[int, int]:
    ok = 0
    for k, v in replayed.items():
        g = gt.get(k)
        if g is None or g.get("type") != v["type"]:
            continue
        if abs(g["x"] - v["x"]) <= tol and abs(g["y"] - v["y"]) <= tol:
            ok += 1
    return ok, len(replayed)


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    n_runs = n_ok = 0
    failures = []

    # easy runs: result JSONs carry the stored seed
    for d in GT_DIRS:
        for p in sorted(d.glob("easy_seed*_run*.json")):
            res = json.load(open(p))
            gt = res["raw_metrics"]["ground_truth_objects"]
            seed = int(res["random_seed"])
            try:
                replayed = replay(seed, "easy")
                n_runs += 1
            except Exception as e:  # noqa: BLE001
                failures.append((p.stem, str(e)))
                continue
            ok, n = verify(gt, replayed)
            if ok != n:
                failures.append((p.stem, f"{ok}/{n} coordinate match"))
                continue
            # preserve mission_target from the actual run's GT (replay resets
            # target_pool; values checked equal in verify via type+coords —
            # carry the run's own flags)
            for k, v in replayed.items():
                v["mission_target"] = bool(gt[k].get("mission_target", False))
            (OUT_DIR / f"{p.stem}.json").write_text(json.dumps(replayed))
            n_ok += 1
            print(f"== {p.stem}: seed {seed}, {n} objects, yaw recovered")

    # stress runs: generated_gt/<run_key>.json (mission flags already correct)
    for p in sorted(GEN_GT_DIR.glob("stream_perception_stress_*.json")):
        gt = json.load(open(p))
        m = p.stem.split("_")
        seed = int(m[3].removeprefix("seed"))
        try:
            replayed = replay(seed, "perception_stress")
            n_runs += 1
        except Exception as e:  # noqa: BLE001
            failures.append((p.stem, str(e)))
            continue
        ok, n = verify(gt, replayed)
        if ok != n:
            failures.append((p.stem, f"{ok}/{n} coordinate match"))
            continue
        for k, v in replayed.items():
            v["mission_target"] = bool(gt[k].get("mission_target", False))
            # carry the generated-gt per-instance spawn z (desk placements)
            v["z"] = float(gt[k].get("z", v["z"]))
        (OUT_DIR / f"{p.stem}.json").write_text(json.dumps(replayed))
        n_ok += 1
        print(f"== {p.stem}: seed {seed}, {n} objects, yaw recovered")

    print(f"\nreplayed {n_ok}/{n_runs} runs; failures: {failures}")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())