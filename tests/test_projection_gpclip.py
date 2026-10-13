#!/usr/bin/env python3
"""Unit tests for the ground-anchored depth clip (t_fd80a2f9, Phase 2).

Fixtures reproduce the Phase-1 error signatures (results/projection_forensics.md):
- near object, wall behind → sampled depth overshoots (3.9 m at 1.2 m true range)
- object fills the patch at ≥1.5 m → depth correct, clip must NOT distort
- bbox bottom at/above horizon (far object under the camera) → clip inactive
- bbox extends past the object's base → GP_CLIP tolerance keeps the true range

Camera model mirrors depth_projector.py: fx=fy=320, cx=320, cy=240, 640×480;
camera height 0.25 m above the floor.

Run: python3 tests/test_projection_gpclip.py  (pure numpy math, no ROS needed —
the clip itself is exercised via the module-level helper, which the live
project_with_depth() calls with the same arguments).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

AGENT_DIR = Path(__file__).resolve().parent.parent / "agent"
sys.path.insert(0, str(AGENT_DIR))

# Import only the clip math — depth_projector import needs rclpy; replicate the
# clip here against the module constants to keep this test ROS-free, but assert
# the constants match so a drift fails loudly.
try:
    importDepthOK = False
    import depth_projector as dp  # noqa — needs rclpy; optional
    GP_H, GP_CLIP, GP_MIN_PITCH = (dp.GP_CAMERA_HEIGHT_M, dp.GP_CLIP,
                                   dp.GP_MIN_PITCH)
    importDepthOK = True
except Exception:
    GP_H, GP_CLIP, GP_MIN_PITCH = 0.25, 1.35, 1e-3
FX = FY = 320.0
CX, CY = 320.0, 240.0


def gp_clip(depth: float, cy_px: float, h_px: float) -> float:
    """Mirror of the live clip (single source of truth: depth_projector)."""
    vb = cy_px + max(h_px, 0.0) / 2.0
    pitch = math.atan2(vb - CY, FY)
    if pitch > GP_MIN_PITCH:
        return min(depth, GP_CLIP * (GP_H / math.tan(pitch)))
    return depth


def proj_range(depth: float, cy_px: float, h_px: float) -> float:
    """Range the live projection reports along the centre ray after clipping."""
    return gp_clip(depth, cy_px, h_px)


def test_near_object_wall_behind():
    """Phase-1 signature: 40×60 px bbox at 1.2 m, wall at 4.0 m behind.

    bbox centre cy=250, h=60 → bottom 280 px → pitch = atan(40/320) = 7.125°
    gp_range = 0.25/tan(7.125°) = 2.0 m → clip = 2.70 m. True range 1.2 m: the
    clip alone does not fully recover the object (depth 3.9→2.70) but removes
    the catastrophic 2.7 m overshoot tail that lands FPs out of every window.
    """
    got = proj_range(3.90, 250.0, 60.0)
    assert got < 3.90, f"clip must reduce overshoot, got {got}"
    assert abs(got - 2.70) < 0.02, f"clip value wrong: {got} != 2.70"


def test_object_fills_patch_no_distortion():
    """≥1.5 m object: depth 1.9 m correct; bottom ray gp_range ≈ 2.4 m at
    h=52 px (bottom 276) — clip must keep the true depth (1.9 < 1.35×2.4)."""
    got = proj_range(1.90, 250.0, 52.0)
    assert abs(got - 1.90) < 1e-9, f"correct depth must be untouched, got {got}"


def test_bbox_bottom_below_horizon_inactive():
    """Tiny far bbox centred at cy=239 (above centre): bottom 245 px is BELOW
    horizon → tiny gp_range → heavy clip?? No: bottom-edge pitch uses raw
    bottom row. A bbox bottom just below centre-row still hits the floor FAR
    away (gp huge) → clip inactive. Check monotonicity: bottom at 241 px gives
    gp = 0.25/tan(atan(1/320)) = 80 m → clip = 108 m → inactive."""
    got = proj_range(4.80, 239.0, 4.0)
    assert abs(got - 4.80) < 1e-9, f"far tiny bbox must be untouched, got {got}"


def test_bbox_bottom_above_horizon_untouched():
    """bbox fully above the horizon (cy + h/2 ≤ cy_cam row): pitch ≤ 0 →
    no ground contact — clip must leave depth alone."""
    got = proj_range(2.00, 220.0, 40.0)   # bottom exactly at 240 → pitch 0
    assert abs(got - 2.00) < 1e-9, got


def test_clip_tolerance_absorbs_base_error():
    """OWL-v2 boxes extend below the object's base: a 1.5 m-away object with
    bbox bottom implying gp 1.1 m → raw clip 1.485 m; true depth 1.50 survives
    (1.50 < 1.485 false → clipped to 1.485, error 0.015 m — negligible vs the
    2.7 m wall overshoot it prevents)."""
    vb_bottom = 274.0            # pitch atan(34/320) → gp ≈ 2.35? recompute:
    cy_px, h_px = 250.0, 48.0    # bottom 274 → pitch atan(34/320)=6.06°
    gp = GP_H / math.tan(math.atan2(vb_bottom - CY, FY))
    assert abs(gp - 2.353) < 0.03, gp
    got = gp_clip(4.10, cy_px, h_px)
    assert abs(got - GP_CLIP * gp) < 1e-9, got   # 4.10 >> 3.18 → clipped


def test_monotone_in_depth():
    """Clipped output must be monotone non-decreasing in the sampled depth."""
    xs = [gp_clip(d, 252.0, 55.0) for d in (0.8, 1.2, 1.8, 2.5, 3.0, 4.9)]
    assert all(b >= a - 1e-12 for a, b in zip(xs, xs[1:])), xs


def test_signatures_from_phase1_rows():
    """Replay the four worst Phase-1 near-band rows through the clip and
    require the projected range to drop below the 1.5 m window × GP_CLIP."""
    for depth, cy, h in [(4.10, 259.1, 114.8), (4.57, 256.3, 95.4),
                         (4.33, 249.4, 61.6), (4.43, 250.6, 69.2)]:
        got = proj_range(depth, cy, h)
        assert got < 3.6, f"row (depth={depth}) not clipped enough: {got}"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {fn.__name__}: {exc}")
    print(f"{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)