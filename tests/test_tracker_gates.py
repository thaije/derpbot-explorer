"""Offline unit checks for the tracker confirmation gates (no ROS needed).

Run: python3 tests/test_tracker_gates.py
Covers: anti-hallucination gate (margin + stability), cross-class veto,
short-range fast-confirm. Feeds tracker_worker via mp queues directly.
"""
import multiprocessing as mp
import sys
import time

sys.path.insert(0, "agent")
from tracker import tracker_worker  # noqa: E402

# tracker.py imports rclpy/vision_msgs — skip gracefully if missing
try:
    import rclpy  # noqa: F401
    from vision_msgs.msg import Detection2D  # noqa: F401
except ImportError:
    sys.exit("skipped: rclpy/vision_msgs unavailable on this box — run on ROS env")

PASS = []
FAIL = []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(f"{name} {detail}")
    print(("PASS" if cond else "FAIL"), name, detail)


def run_worker(sightings, wait_s=2.5):
    """Feed (class, wx, wy, rx, ry, conf[, injected]) tuples; return messages.

    Optional 7th element True marks an externally verified sighting (the
    5-tuple Tracker.inject_external_sighting enqueues, t_d479428a)."""
    ctx = mp.get_context("spawn")
    iq, oq, ev = ctx.Queue(), ctx.Queue(), ctx.Event()
    p = ctx.Process(target=tracker_worker, args=(["cup", "bottle"], iq, oq, ev))
    p.start()
    ev.wait(timeout=5.0)
    for s in sightings:
        tup = (s[0], (s[1], s[2]), (s[3], s[4]), s[5])
        if len(s) > 6:
            tup = tup + (bool(s[6]),)
        iq.put(tup)
        time.sleep(0.05)
    time.sleep(wait_s)
    iq.put(None)
    p.join(timeout=5.0)
    out = []
    try:
        while True:
            out.append(oq.get_nowait())
    except Exception:
        pass
    return out


def conf_msgs(out):
    return [m for m in out if m.get("type") == "confirmed"]


def main():
    # 0. Causal note: a track that confirms BEFORE its cross-class neighbor
    # exists publishes (nothing to veto yet); already-published submissions
    # cannot be retracted. The veto governs tracks that coexist.

    # 1. Two diverse far sightings, good confidence → publishes
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.5),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.6),   # robot moved 0.45m ≥0.2 → diverse
    ])
    msgs = conf_msgs(out)
    check("diverse-2 publishes", len(msgs) == 1 and msgs[0]["class_name"] == "cup",
          str([(m["class_name"], m["world_x"], m["world_y"]) for m in msgs]))

    # 2. Marching hallucination: sightings drift far beyond the map → gate rejects
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.28),
        ("cup", 4.0, 3.8, 1.0, 1.5, 0.28),
        ("cup", 5.0, 4.6, 1.0, 2.0, 0.28),
        ("cup", 6.0, 4.0, 1.0, 2.5, 0.28),
    ])
    check("marching track rejected", len(conf_msgs(out)) == 0)

    # 3. Margin disabled (bisect, margin=0.0): 0.16 ≥ 0.15+0.0 passes the margin
    # check → real track near threshold now publishes (recall restored).
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.16),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.16),
        ("cup", 2.95, 3.05, 1.0, 1.9, 0.16),
    ])
    msgs = conf_msgs(out)
    check("margin=0.0: near-threshold track publishes", len(msgs) == 1
          and msgs[0]["class_name"] == "cup", str(len(msgs)))

    # 4. Cross-class veto, interleaved so BOTH tracks exist before either
    # confirms: cup + bottle within 0.1m, near-equal conf → both held forever
    # (no disambiguating evidence).
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.6),
        ("bottle", 3.1, 3.0, 1.2, 1.2, 0.55),   # 0.10m from cup, own track
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.6),    # cup confirmable now — but vetoed
        ("bottle", 3.15, 2.9, 1.2, 1.7, 0.55),  # bottle confirmable — also vetoed
    ])
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("cross-class veto holds both", klass == set(),
          str([(m["class_name"], round(m["world_x"], 1)) for m in conf_msgs(out)]))

    # 5. Veto resolved by clear margin winner (interleaved): cup 0.9 vs bottle 0.4
    # → cup clears the other's conf+margin, publishes; bottle stays held.
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.9),
        ("bottle", 3.1, 3.0, 1.2, 1.2, 0.4),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.9),
        ("bottle", 3.15, 2.9, 1.2, 1.7, 0.4),
    ])
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("veto releases clear winner", klass == {"cup"},
          str(klass))

    # 6. Fast-confirm: 3 close-range frames, robot stationary → publishes
    out = run_worker([
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.7),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.7),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.7),
    ])
    msgs = conf_msgs(out)
    check("fast-confirm publishes", len(msgs) == 1 and msgs[0]["class_name"] == "cup",
          str([(m["class_name"]) for m in msgs]))

    # 7. Fast-confirm with margin=0.0: 0.16 ≥ 0.15+0.0 → 3 close frames publish
    # (margin gate disabled by bisect; stability/veto coverage unchanged below).
    out = run_worker([
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.16),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.16),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.16),
    ])
    msgs = conf_msgs(out)
    check("fast-confirm at threshold (margin=0.0)", len(msgs) == 1
          and msgs[0]["class_name"] == "cup", str(len(msgs)))

    # 8. Veto cleared on separation: the pair is held while within VETO_RADIUS;
    # once the bottle track's centroid drifts beyond it, the next cup sighting
    # finds no neighbor → cup publishes. (Bottle, single-sighting-far, never
    # confirms on its own.)
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.9),
        ("bottle", 3.1, 3.0, 1.2, 1.2, 0.4),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.9),   # cup diverse — held (bottle 0.05m away)
        ("bottle", 5.0, 3.0, 1.2, 1.7, 0.4),   # bottle centroid now ≥0.95m from cup
        ("cup", 3.1, 3.1, 1.0, 1.9, 0.9),      # cup re-checked: neighbor gone → cleared
    ])
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("veto cleared on separation", klass == {"cup"},
          str(klass))

    # 9. Outlier-aligned chimera with an externally verified sighting
    #    (t_d479428a, v13-cv-fu): one early sighting anchors the track's
    #    running mean off-object; a corroborated core mass forms elsewhere
    #    (spread > STABILITY_RADIUS) and a high-trust VERIFIER INJECTION
    #    fuses in — the track was pending forever before the fix (core check
    #    failed on the singleton anchor half every tick). After: the split
    #    fires on the injected evidence, the mass half converts, the corner
    #    is quarantined off. Geometry mirrors the live seed4 case: anchor
    #    1.47m from the core, core sightings pairwise-tight.
    #    Tuple order in run_worker: (class, wx, wy, rx, ry, conf).
    #    First 3 corner sightings + 3 core sightings (robot moves 0.45m
    #    apart within the core half so it re-sights densely):
    out = run_worker([
        ("cup", 10.0, 10.0, 0.0, 0.0, 0.25),   # corner anchor (off-object)
        ("cup", 6.0, 6.5, 1.0, 1.0, 0.28),
        ("cup", 6.1, 6.4, 1.45, 1.0, 0.22),
        ("cup", 6.0, 6.6, 1.9, 1.0, 0.30),
        ("cup", 6.05, 6.45, 2.35, 1.0, 0.24),
        ("cup", 5.95, 6.55, 2.8, 1.0, 0.26),
        ("cup", 6.0, 6.5, 3.25, 1.0, 0.98, True),  # VLM injection, marker set
    ])
    msgs = conf_msgs(out)
    check("injected chimera: corner split off, mass half converts",
          len(msgs) >= 1 and msgs[0]["class_name"] == "cup"
          and abs(msgs[0]["world_x"] - 6.0) < 0.3
          and abs(msgs[0]["world_y"] - 6.5) < 0.3,
          str([(m["world_x"], m["world_y"]) for m in msgs]))

    # 10. Same track WITHOUT the injected marker: the classic rule holds —
    #     no split evidence → no outlier-aligned split → the chimera stays
    #     rejected (the no-injection-behavior-unchanged guarantee). Geometry
    #     = the live seed4 track_2 case (corner 1.47m from the mass, mass
    #     pairwise-tight at GT, track spread 2.08m > STABILITY_RADIUS —
    #     replay-proven pending-forever on master before this fix).
    out = run_worker([
        ("cup", 8.6, 5.13, 0.18, 0.29, 0.25),      # corner anchor (seed4 t=8.6)
        ("cup", 10.0, 4.76, 0.18, 0.29, 0.26),
        ("cup", 10.0, 4.74, 0.18, 0.29, 0.22),
        ("cup", 10.03, 4.83, 0.18, 0.29, 0.29),
        ("cup", 9.92, 4.82, 0.18, 0.29, 0.27),
        ("cup", 10.03, 4.84, 0.18, 0.29, 0.28),
        ("cup", 10.07, 4.79, 0.18, 0.29, 0.30),
        ("bottle", 60.0, 60.0, 2.0, 2.0, 0.6),     # far bottle: no cross-talk
        ("cup", 9.53, 3.79, 0.25, 0.48, 0.22),     # mass core at GT (seed4 1.53,3.80)
        ("cup", 9.53, 3.79, 0.29, 0.55, 0.24),
        ("cup", 9.53, 3.81, 0.32, 0.61, 0.18),
        ("cup", 9.52, 3.79, 0.39, 0.79, 0.16),
        ("cup", 9.52, 3.79, 0.43, 0.87, 0.19),
        ("cup", 9.51, 3.80, 0.49, 0.96, 0.18),
    ])
    msgs = conf_msgs(out)
    check("seed4-shaped chimera, no injection marker: stays rejected",
          len(msgs) == 0, str([(m["world_x"], m["world_y"]) for m in msgs]))

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()