#!/usr/bin/env python3.12
"""Per-run miss attribution for the v6-fastconfirm batch (task t_81ea7a00)."""
import json, glob, sys
sys.path.insert(0, "/home/plip/Projects/derpbot-explorer/scripts")
sys.path.insert(0, "/home/plip/Projects/derpbot-explorer/agent")
import replay_tracker as rt

SUB = '/home/plip/Projects/robot-sandbox/results/submissions/derpbot-explorer-v6-fastconfirm/'
RUNS = [(s, r) for s in range(1, 6) for r in range(1, 4)]
for s, r in RUNS:
    res = json.load(open(f'{SUB}easy_seed{s}_run{r}.json'))['raw_metrics']
    sl = res.get('submission_log', [])
    tps = {x.get('nearest_gt_key') for x in sl if x['outcome'] == 'TP'}
    gt = {k: v for k, v in res['ground_truth_objects'].items() if v.get('mission_target')}
    n_miss = len(gt) - len(tps & set(gt))
    if n_miss == 0:
        continue
    st = sorted(glob.glob(f'/home/plip/Projects/derpbot-explorer/results/streams/stream_easy_seed{s}_run{r}_*.jsonl'))
    if not st:
        print(f"--- s{s}r{r}: {n_miss} misses, NO STREAM")
        continue
    header, sightings = rt.load_stream(st[0])
    spawn = header.get('spawn_pose') or {"x": 0.0, "y": 0.0, "yaw": 0.0}
    print(f"--- s{s}r{r} ({n_miss} mission-target misses, stream {st[0].split('/')[-1]}, {len(sightings)} sightings)")
    for line in rt.diagnose_misses(sightings, gt, spawn, tps):
        print(line)