# Agent Handoff — autonomous derpbot explorer

Full plan + remaining work: [`docs/approach1_classical_pipeline.md`](approach1_classical_pipeline.md)
Architecture survey (all 5 approaches): [`docs/five approaches to robot autonomy.md`](five%20approaches%20to%20robot%20autonomy.md)
Task + robot spec: [`docs/AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md)

---

## Current Implementation: Approach 1 — Classical Modular Pipeline

**slam_toolbox + Nav2 + frontier explorer + YOLOE26-S detector**

All code is in `agent/`. Config in `config/`. Launch file in `launch/`.

---

## Key Gotchas

### TF frame names — MUST CONFIRM FROM SIM

The camera TF frame is hardcoded as `camera_link` in [agent/depth_projector.py](../agent/depth_projector.py). Before first run:

```bash
ros2 run tf2_tools view_frames  # generates /tmp/frames.pdf
```

Update `CAMERA_FRAME` in `depth_projector.py` if it differs.

Also confirm `base_link` is the correct robot base frame (used throughout Nav2 params).

### slam_toolbox async mode

Map updates lag by ~1 scan under CPU load — this is expected and safe. The frontier explorer re-reads `/map` on each planning cycle so it always uses the latest available data.

### Nav2 cmd_vel routing

The `collision_monitor` node receives `cmd_vel_smoothed` and publishes to `/derpbot_0/cmd_vel`. If the sim doesn't see velocity commands, check this remapping in `config/derpbot_nav2_params.yaml` (`cmd_vel_in_topic` / `cmd_vel_out_topic`).

### Nav2 action server name

FrontierExplorer connects to `navigate_to_pose` (Nav2 default). Verify with:
```bash
ros2 action list
```

### Detection publishing format

Scorer expects:
- `Detection2D.id` — string e.g. `"track_1"`
- `results[0].hypothesis.class_id` — target class string
- `results[0].pose.pose.position.x/y` — world position in `map` frame
