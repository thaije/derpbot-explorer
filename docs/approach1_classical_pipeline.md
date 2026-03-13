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

### IMU-fused odometry (robot_localization EKF)
If odometry angular accuracy degrades again (e.g. different sim versions or real hardware), add a `robot_localization` EKF node that fuses `/derpbot_0/imu` (100 Hz gyro, accurate angular rate) with `/derpbot_0/odom` (accurate linear, unreliable angular). The EKF publishes `/odom_fused`; point `slam_toolbox`'s `odom_frame` at it. This eliminates systematic wheel-baseline angular error at the source without touching SLAM params. Config: ~20 lines of YAML (`ekf.yaml`) + one launch node.

### Other fixes
- map takes 2min to become available. Find a way to speed this up.