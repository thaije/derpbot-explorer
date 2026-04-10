# derpbot-explorer

Autonomous robot exploration agent for the [Autonomous Robotics Simulation Testbed (ARST)](docs/AUTONOMOUS_AGENT_GUIDE.md). Explores unknown indoor environments, builds a map, detects target objects, and reports findings within a time limit.

Architecture: slam_toolbox + Nav2 + frontier explorer + YOLOE26-S open-vocabulary detector.
Agent state / invariants: [`docs/STATE.md`](docs/STATE.md) · Next work: [`docs/ROADMAP.md`](docs/ROADMAP.md) · Tracker: GitHub issues

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy
- **uv** — `pip install uv` or [docs.astral.sh/uv](https://docs.astral.sh/uv)

```bash
# ROS packages
sudo apt install \
    ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller \
    ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-vision-msgs

# Python venv
uv venv
source .venv/bin/activate
uv pip install -r requirements.txt
uv pip install "numpy<2"   # cv_bridge requires numpy 1.x
```

## Run

Start the ARST simulator first, then:

```bash
# Terminal 1 — SLAM
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$(pwd)/config/slam_toolbox_params.yaml use_sim_time:=true

# Terminal 2 — Nav2
ros2 launch $(pwd)/launch/navigation_launch.py \
    params_file:=$(pwd)/config/derpbot_nav2_params.yaml use_sim_time:=true

# Terminal 3 — Agent
cd agent && python3 agent_node.py

# Nav-only benchmarking (skips OWLv2 detector / tracker):
cd agent && python3 agent_node.py --no-perception
```

Or use the combined launch file (starts all three with a 5 s delay between Nav2 and agent):

```bash
ros2 launch launch/derpbot_autonomy.launch.py
```


```bash
# Terminal 4 — RViz (optional, for visualisation)
rviz2 -d config/derpbot_rviz.rviz
```

Shows: occupancy map, global costmap, lidar scan, robot model, TF frames, planned path, goal pose, RGB camera feed. Use the **Nav2** panel or **2D Goal Pose** tool to send goals manually.


## Cleanup between runs

The stack uses tmux sessions (`fds`, `sim`, `slam`, `nav2`, `agent`). Leftover processes from a previous run will corrupt the next one (stale TF, inactive Nav2 lifecycle, etc.).

```bash
# List running tmux sessions
tmux ls

# Kill all stack sessions
for s in agent nav2 slam sim fds; do tmux kill-session -t "$s" 2>/dev/null; done

# Nuclear option — kill all stack processes by name + reset ROS2 daemon
pkill -9 -f "gz sim|parameter_bridge|scenario_runner|run_scenario"
pkill -9 -f "slam_toolbox|controller_server|planner_server|bt_navigator"
pkill -9 -f "behavior_server|velocity_smoother|collision_monitor|lifecycle_manager"
pkill -9 -f "smoother_server|agent_node|fastdds discovery"
ros2 daemon stop && ros2 daemon start

# Verify nothing is left
tmux ls                          # should say "no server running"
ps aux | grep -E "gz sim|slam_toolbox|bt_navigator" | grep -v grep
```

`start_stack.sh` does this cleanup automatically before each launch. It also accepts `--no-perception` to run the agent with the OWLv2 detector / depth projector / tracker disabled — useful for isolating Nav2 behaviour from GPU load (found_ratio will be 0 in this mode).

## Known issues

- Old TF frames from a killed sim can corrupt a new run — always kill the ROS2 daemon between runs (see above).
