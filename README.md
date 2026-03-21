# derpbot-explorer

Autonomous robot exploration agent for the [Autonomous Robotics Simulation Testbed (ARST)](docs/AUTONOMOUS_AGENT_GUIDE.md). Explores unknown indoor environments, builds a map, detects target objects, and reports findings within a time limit.

Architecture: slam_toolbox + Nav2 + frontier explorer + YOLOE26-S open-vocabulary detector.
Agent state / implementation notes: [`docs/AGENT_HANDOFF.md`](docs/AGENT_HANDOFF.md)

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


Bugs: 
- Can we prevent old TF frames from reaching nav2 or something? 