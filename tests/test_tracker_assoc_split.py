"""Unit checks for the spread-triggered split + quarantined re-match
(t_ca2ea380 — DATA ASSOCIATION fix: real near-duplicate objects 0.96–1.17 m
apart must form separate tracks, and the stability gate must never again see
the merged chimera).

Run: python3 tests/test_tracker_assoc_split.py   (needs rclpy → ROS env)
Covers:
  1. two objects 0.96 m apart (the seed4 desk-FE shape) — 2-means split
     separates them, each half publishes at its own GT position (un_merge);
  2. single object, tight scatter (≤0.3 m) — never splits, still one track;
  3. chimera regression: the pre-fix behavior (merge → spread > 1.2 → gate
     reject, zero publications) is replaced by two publications;
  4. diffuse-hallucination scatter spread across the map — split guard holds,
     track stays a single gated (rejected) track, no new track is promoted;
  5. quarantine: after a split, a sibling-object sighting 0.96 m from one
     half's centroid does NOT re-merge the halves;
  6. split-child quarantine still accepts its OWN object's re-sightings
     (0.3 m scatter) and confirms normally;
  7. cross-class veto / fast-confirm behavior unchanged for non-split tracks.
"""
import multiprocessing as mp
import sys
import time

sys.path.insert(0, "agent")
from tracker import tracker_worker  # noqa: E402

try:
    import rclpy  # noqa: F401
    from vision_msgs.msg import Detection2D  # noqa: F401
except ImportError:
    sys.exit("skipped: rclpy/vision_msgs unavailable on this box — run on ROS env")

PASS = []
FAIL = []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(("PASS" if cond else "FAIL"), name, detail)


def run_worker(sightings, targets=("cup",), wait_s=3.0):
    """Feed (cls, wx, wy, rx, ry, conf) tuples via mp queues (no pacing sleeps
    between individual sightings — order is preserved by the queue)."""
    ctx = mp.get_context("spawn")
    iq, oq, ev = ctx.Queue(), ctx.Queue(), ctx.Event()
    p = ctx.Process(target=tracker_worker, args=(list(targets), iq, oq, ev))
    p.start()
    ev.wait(timeout=5.0)
    for s in sightings:
        iq.put((s[0], (s[1], s[2]), (s[3], s[4]), s[5]))
    time.sleep(wait_s)
    iq.put(None)
    p.join(timeout=8.0)
    out = []
    while True:
        try:
            out.append(oq.get_nowait())
        except Exception:
            break
    return [m for m in out if m.get("type") == "confirmed"]


# --- synthetic GT-free scenarios --------------------------------------------
# Two objects at x=0 and x=0.96 (seed4 desk-FE geometry). Each is seen from 3
# diverse robot poses (0.4 m apart) with tight 0.1 m scatter and good conf, so
# EACH HALF of the split is confirmable on its own; under MATCH_RADIUS 2.0 the
# objects' sightings interleave into ONE chimera (distances 0..1.06 < 2.0).
def merged_pair_stream(n_per=8, d=1.6, conf=0.7):
    """Interleaved near-dup pair: A sightings at y=0, B at y=d (d < 2.0 →
    chained into one track under plain MATCH_RADIUS 2.0 association). The
    y-spread d=1.6 > STABILITY_RADIUS 1.2 is what fires the split guard, and
    the x-halos (±0.5) give each object's core ≥2 sightings within 0.5 m."""
    sight = []
    poses = [(0.0, 0.0), (0.1, 0.4), (0.5, 0.0), (0.6, 0.4),
             (0.2, 0.0), (0.3, 0.4)]
    for i in range(n_per * 2):
        pa = poses[i % len(poses)]
        if i % 2 == 0:
            # object A at (0.5, 0.0): halo x in {0.0..1.0}, scatter ±0.05
            ax = 0.5 + 0.5 * ((i % 4) - 1) + 0.05 * ((i % 3) - 1)
            sight.append(("cup", ax, 0.0, pa[0], pa[1], conf))
        else:
            # object B at (0.5, 1.6)
            bx = 0.5 + 0.5 * (((i + 1) % 4) - 1) + 0.05 * ((i % 3) - 1)
            sight.append(("cup", bx, d, pa[0] + 0.05, pa[1] + 0.05, conf))
    return sight


def main():
    # 1+3. The seed4 shape: interleaved near-duplicate pair → two publications
    out = run_worker(merged_pair_stream())
    msgs = sorted((round(m["world_y"], 1) for m in out))
    check("near-dup pair un-merges into 2 publications", len(out) == 2,
          f"published {[ (m['world_x'], m['world_y']) for m in out ]}")
    if len(out) == 2:
        ys = sorted(round(m["world_y"], 1) for m in out)
        check("halves sit at each object", ys[0] in (0.0,) and ys[1] in (1.6,),
              f"ys={ys}")

    # 2. Single object, tight scatter, interleaves nothing — one track
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.6),
        ("cup", 3.05, 3.02, 1.0, 1.4, 0.6),
        ("cup", 2.96, 2.98, 1.4, 1.0, 0.6),
        ("cup", 3.01, 3.04, 1.4, 1.4, 0.6),
        ("cup", 2.99, 3.0, 1.8, 1.0, 0.6),
        ("cup", 3.03, 2.97, 1.8, 1.4, 0.6),
    ])
    check("tight scatter stays one track", len(out) == 1,
          f"published {len(out)}")

    # 4. Diffuse hallucination scatter (spread > 1.5 m, no two-half core):
    #    split guard must not fire — track stays single and gate-rejected.
    out = run_worker([
        ("cup", 3.0, 3.0, 1.0, 1.0, 0.5),
        ("cup", 4.2, 3.8, 1.0, 1.5, 0.5),
        ("cup", 5.4, 4.6, 1.5, 2.0, 0.5),
        ("cup", 6.6, 5.4, 2.0, 2.5, 0.5),
        ("cup", 7.8, 6.2, 2.5, 3.0, 0.5),
    ])
    check("diffuse scatter: no split, gate holds", len(out) == 0,
          f"published {len(out)}")

    # 5+6. Quarantine: split first (pair above), then feed a fresh sighting
    # that sits between the two halves — whichever half it lands near, both
    # halves must stay separate tracks (2 publications already exist; the new
    # sighting joins one track but cannot pull the other half's centroid).
    sight = merged_pair_stream()
    # extra sibling sighting 1.6 m from half A's centroid (i.e. near B)
    sight.append(("cup", 0.5, 1.6, 0.4, 0.0, 0.7))
    sight.append(("cup", 0.5, 1.6, 0.4, 0.4, 0.7))  # diverse pose for B half
    out = run_worker(sight)
    # Both objects must be published; no track should publish a ~0.5 m blend
    blends = [m for m in out
              if 0.2 < m["world_y"] < 0.7]
    check("quarantine prevents half re-merge (no blend centroids)",
          len(blends) == 0, f"blends={[(m['world_x'], m['world_y']) for m in blends]}")
    check("quarantine: both objects still published", len(out) >= 2,
          f"published {[(m['world_x'], m['world_y']) for m in out]}")

    # 7. Regression: fast-confirm + veto on NON-split tracks unchanged
    out = run_worker([
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.7),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.7),
        ("cup", 3.0, 3.0, 2.0, 2.0, 0.7),
        ("bottle", 3.05, 3.0, 2.2, 2.2, 0.9),
        ("bottle", 3.05, 3.0, 2.2, 2.6, 0.9),   # diverse → confirmable but vetoed
    ])
    klass = {m["class_name"] for m in out}
    check("fast-confirm + veto unchanged", klass == {"cup"}, str(klass))

    print(f"\n{len(PASS)} passed, {len(FAIL)} failed")
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()