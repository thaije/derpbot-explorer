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
