"""Offline unit checks for the tracker confirmation gates (no ROS needed).

Run: python3 tests/test_tracker_gates.py   (needs rclpy/vision_msgs → python3.12)

Semantics after the bcfc adoption (sweep t_451895af): CONFIDENCE_MARGIN=0.0,
MIN_SIGHTINGS=2 diverse + fast-confirm intact; STABILITY_RADIUS=100.0 and
VETO_RADIUS=0.0 make the positional-stability gate and the cross-class veto
INERT at as-code defaults. Coverage layout:
  - default-semantics tests assert the inert behavior (a marching track and a
    close cross-class pair now publish — the documented recall/FP trade-off);
  - mechanism tests re-activate the gates per-worker via
    DERPBOT_POLICY_OVERRIDES (the same override channel the offline replay and
    sweep use) and assert the old hold/reject behavior still works when enabled.
Feeds tracker_worker via mp queues directly.
"""
import multiprocessing as mp
import os
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


def run_worker(sightings, wait_s=2.5, overrides=None):
    """Feed (class, wx, wy, rx, ry, conf) tuples; return published messages.

    overrides: optional policy override string ("NAME=VALUE,NAME=VALUE") —
    exported as DERPBOT_POLICY_OVERRIDES so the spawned worker re-imports
    tracker.py with those constants (OFFLINE-replay bootstrap), then restored.
    """
    env_kept = os.environ.get("DERPBOT_POLICY_OVERRIDES")
    if overrides:
        os.environ["DERPBOT_POLICY_OVERRIDES"] = overrides
    try:
        ctx = mp.get_context("spawn")
        iq, oq, ev = ctx.Queue(), ctx.Queue(), ctx.Event()
        p = ctx.Process(target=tracker_worker, args=(["cup", "bottle"], iq, oq, ev))
        p.start()
        ev.wait(timeout=5.0)
        for s in sightings:
            iq.put((s[0], (s[1], s[2]), (s[3], s[4]), s[5]))
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
    finally:
        if env_kept is None:
            os.environ.pop("DERPBOT_POLICY_OVERRIDES", None)
        else:
            os.environ["DERPBOT_POLICY_OVERRIDES"] = env_kept


def conf_msgs(out):
    return [m for m in out if m.get("type") == "confirmed"]


def main():
    # 0. Causal note: a track that confirms BEFORE its cross-class neighbor
    # exists publishes (nothing to veto yet); already-published submissions
    # cannot be retracted. With the veto INERT (default) it governs nothing.

    # 1. Two diverse far sightings, good confidence → publishes
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.5),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.6),   # robot moved 0.45m ≥0.2 → diverse
    ])
    msgs = conf_msgs(out)
    check("diverse-2 publishes", len(msgs) == 1 and msgs[0]["class_name"] == "cup",
          str([(m["class_name"], m["world_x"], m["world_y"]) for m in msgs]))

    # 2. Marching hallucination under INERT stability gate (as-code bcfc): the
    # track spreads 3.4m > old 1.2m radius but STABILITY_RADIUS=100.0 never
    # fires → it now publishes (documented recall/FP trade-off). The centroid
    # re-publishes as it marches (shift ≥ REPUBLISH_SHIFT_M), so expect ≥1
    # message, all from ONE track id.
    MARCH = [
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.28),
        ("cup", 4.0, 3.8, 1.0, 1.5, 0.28),
        ("cup", 5.0, 4.6, 1.0, 2.0, 0.28),
        ("cup", 6.0, 4.0, 1.0, 2.5, 0.28),
    ]
    out = run_worker(MARCH)
    msgs = conf_msgs(out)
    check("stability inert: marching track publishes from one track",
          len(msgs) >= 1 and len({m["track_id"] for m in msgs}) == 1
          and all(m["class_name"] == "cup" for m in msgs),
          str([(m["track_id"], m["world_x"], m["world_y"]) for m in msgs]))

    # 2b. Mechanism test: stability gate re-enabled via policy override → the
    # same marching track is rejected (spread 3.4m > 1.2m at every check).
    out = run_worker(MARCH, overrides="STABILITY_RADIUS=1.2")
    check("stability gate (override 1.2): marching track rejected",
          len(conf_msgs(out)) == 0, str(conf_msgs(out)))

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

    # 4. Cross-class pair with the veto INERT (as-code bcfc): cup + bottle
    # 0.1m apart, near-equal conf → both tracks confirm and publish
    # (documented: wrong-class-submission protection is off by default).
    PAIR = [
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.6),
        ("bottle", 3.1, 3.0, 1.2, 1.2, 0.55),   # 0.10m from cup, own track
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.6),
        ("bottle", 3.15, 2.9, 1.2, 1.7, 0.55),
    ]
    out = run_worker(PAIR)
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("veto inert: both cross-class tracks publish", klass == {"cup", "bottle"},
          str(klass))

    # 4b. Mechanism test: veto re-enabled (0.8m) → both held forever: cup 0.6
    # does not clear bottle 0.55 by AMBIGUITY_MARGIN 0.10 (needs ≥0.65), and
    # bottle 0.55 does not clear cup 0.6 (needs ≥0.70). Interleaved so BOTH
    # tracks exist before either confirms.
    out = run_worker(PAIR, overrides="VETO_RADIUS=0.8")
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("veto (override 0.8) holds both", klass == set(),
          str([(m["class_name"], round(m["world_x"], 1)) for m in conf_msgs(out)]))

    # 5. Mechanism test: veto (0.8m) resolved by clear margin winner: cup 0.9
    # vs bottle 0.4 → cup clears the other's conf+margin, publishes; bottle
    # stays held.
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.9),
        ("bottle", 3.1, 3.0, 1.2, 1.2, 0.4),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.9),
        ("bottle", 3.15, 2.9, 1.2, 1.7, 0.4),
    ], overrides="VETO_RADIUS=0.8")
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("veto (override 0.8) releases clear winner", klass == {"cup"},
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
    # (margin gate disabled by bisect; stability/veto inert below).
    out = run_worker([
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.16),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.16),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.16),
    ])
    msgs = conf_msgs(out)
    check("fast-confirm at threshold (margin=0.0)", len(msgs) == 1
          and msgs[0]["class_name"] == "cup", str(len(msgs)))

    # 8. Cross-class pair, second bottle sighting far away, veto INERT
    # (as-code bcfc): nothing holds either track → both publish (the bottle
    # centroid lands between its two sightings, (4.05, 3.0)).
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.9),
        ("bottle", 3.1, 3.0, 1.2, 1.2, 0.4),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.9),   # cup confirms, publishes
        ("bottle", 5.0, 3.0, 1.2, 1.7, 0.4),   # merges bottle track, confirms
        ("cup", 3.1, 3.1, 1.0, 1.9, 0.9),      # tiny shift → no republish
    ])
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("veto inert: separated-pair scenario publishes both", klass == {"cup", "bottle"},
          str(klass))

    # 8b. Mechanism test: veto re-enabled (0.8m) → cup 0.9 out-confirms bottle
    # 0.4 by the margin and publishes; bottle 0.4 stays held on every
    # confirmation attempt (cup's best conf 0.9 + 0.10 is out of reach, and the
    # pair's centroids stay within the veto radius).
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.9),
        ("bottle", 3.1, 3.0, 1.2, 1.2, 0.4),
        ("cup", 3.05, 2.95, 1.0, 1.45, 0.9),   # cup confirmable — wins the pair
        ("bottle", 3.2, 3.0, 1.2, 1.7, 0.4),   # bottle confirmable — veto-held
        ("cup", 3.1, 3.1, 1.0, 1.9, 0.9),      # tiny shift → no republish
    ], overrides="VETO_RADIUS=0.8")
    klass = {(m["class_name"]) for m in conf_msgs(out)}
    check("veto (override 0.8): winner publishes, loser held", klass == {"cup"},
          str(klass))

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()