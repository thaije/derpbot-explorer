# DerpBot — Approach 1: Classical Modular Pipeline

> Current implementation. Full architecture survey: [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)

## Status

Core pipeline is built and running. Remaining work:

| Item | Status | Notes |
|------|--------|-------|
| slam_toolbox + Nav2 + frontier exploration | ✅ running | Robot navigating, map building |
| YOLOE26-S detector | ✅ running | GPU 0, ~5 Hz |
| Depth projector + tracker → `/derpbot_0/detections` | ⚠️ untested | cv_bridge fixed, needs end-to-end verify |
| End-to-end scenario score | ❌ not tested | Run `easy.yaml --seed 42 --timeout 300` |
| `medium`/`hard` tier tuning | ❌ not started | May need SLAM loop-closure, smarter frontier scoring |

## Potential future extensions

### Detector backend upgrade
Current pipeline (YOLOE+CLIP) works for the simple sim but has two weaknesses: hardcoded visual anchor classes, and CLIP similarity at the floor (0.20) for low-poly meshes. Three viable upgrades, in order of effort:

| Option | Speed on RTX 2070S | VRAM | Notes |
|---|---|---|---|
| **YOLO-World M/L** (`yolov8m/l-worldv2`) | ~25 / ~15 FPS | ~2 GB | Drop-in swap; no CLIP stage needed; open-vocab via `set_classes()`; M/L meaningfully better than current S. Already wired as fallback in `detector.py`. |
| **OWL-v2 base** (`google/owlv2-base`) | ~10–15 FPS | ~2.5 GB | Text-to-bbox in one pass; no anchors needed; `transformers` install required. Better cluttered-scene performance than YOLO-World. |
| **Grounding DINO tiny** (`Swin-T`) | ~6–10 FPS | ~2.5 GB | Best open-vocab detection quality; handles realistic cluttered scenes well; `groundingdino` package required. Marginal at 5 Hz on RTX 2070S — test before committing. |

Grounding DINO base (Swin-B) is too slow (~3–5 FPS) and too heavy (~4 GB VRAM) for this hardware.

Test script: [`agent/test_detector_live.py`](../agent/test_detector_live.py) — run while driving in sim to compare backends.

### IMU-fused odometry (robot_localization EKF)
If odometry angular accuracy degrades again (e.g. different sim versions or real hardware), add a `robot_localization` EKF node that fuses `/derpbot_0/imu` (100 Hz gyro, accurate angular rate) with `/derpbot_0/odom` (accurate linear, unreliable angular). The EKF publishes `/odom_fused`; point `slam_toolbox`'s `odom_frame` at it. This eliminates systematic wheel-baseline angular error at the source without touching SLAM params. Config: ~20 lines of YAML (`ekf.yaml`) + one launch node.

### Other fixes
- map takes 2min to become available. Find a way to speed this up.