#!/usr/bin/env bash
# Start the full DerpBot autonomy stack with correct timing.
#
# Usage:
#   ./scripts/start_stack.sh [--speed N] [--seed N] [--scenario TIER] [--no-agent]
#
# Starts: fastdds discovery server → sim → (wait ready) → SLAM + Nav2 → agent.
# Each component runs in its own tmux session: fds, sim, slam, nav2, agent.
#
# The --no-agent flag skips the agent (useful for benchmarking SLAM+Nav2 RTF).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPLORER_ROOT="$(dirname "$SCRIPT_DIR")"
SANDBOX_ROOT="$HOME/Projects/robot-sandbox"

# Defaults
SPEED=2
SEED=42
SCENARIO="easy"
START_AGENT=1

# Parse args
while [[ $# -gt 0 ]]; do
    case $1 in
        --speed)   SPEED="$2"; shift 2 ;;
        --seed)    SEED="$2"; shift 2 ;;
        --scenario) SCENARIO="$2"; shift 2 ;;
        --no-agent) START_AGENT=0; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

SCENARIO_FILE="config/scenarios/office_explore_detect/${SCENARIO}.yaml"

# FastDDS discovery server settings
FDS_HOST="127.0.0.1"
FDS_PORT="11811"

# Env vars injected into every ROS2 session.
# ROS_SUPER_CLIENT makes CLI tools (ros2 topic/node list) work correctly
# when discovery server is active.
ROS_ENV="export ROS_DISCOVERY_SERVER=${FDS_HOST}:${FDS_PORT}; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export ROS_SUPER_CLIENT=1;"

echo "=== DerpBot Stack Launcher ==="
echo "  speed=$SPEED  seed=$SEED  scenario=$SCENARIO  agent=$START_AGENT"
echo ""

# --- Step 0: Kill everything from previous runs ---
echo "[0/5] Cleaning up old processes..."
for sess in agent nav2 slam sim fds; do
    tmux kill-session -t "$sess" 2>/dev/null || true
done
sleep 1

# Kill sim/bridge/discovery processes by name — two rounds to catch respawns
for _round in 1 2; do
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "scenario_runner" 2>/dev/null || true
    pkill -9 -f "run_scenario" 2>/dev/null || true
    pkill -9 -f "slam_toolbox" 2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "planner_server" 2>/dev/null || true
    pkill -9 -f "bt_navigator" 2>/dev/null || true
    pkill -9 -f "behavior_server" 2>/dev/null || true
    pkill -9 -f "velocity_smoother" 2>/dev/null || true
    pkill -9 -f "collision_monitor" 2>/dev/null || true
    pkill -9 -f "lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "waypoint_follower" 2>/dev/null || true
    pkill -9 -f "smoother_server" 2>/dev/null || true
    pkill -9 -f "map_publisher" 2>/dev/null || true
    pkill -9 -f "agent_node" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "fastdds discovery" 2>/dev/null || true
    sleep 1
done
fuser -k 7400/tcp 2>/dev/null || true
fuser -k "${FDS_PORT}/udp" 2>/dev/null || true
sleep 1

# --- Step 1: Start FastDDS discovery server ---
# Must start before the ROS2 daemon so the daemon can register with it.
echo "[1/5] Starting FastDDS discovery server (port ${FDS_PORT})..."
tmux new -s fds -d "fastdds discovery -i 0 -l ${FDS_HOST} -p ${FDS_PORT}"

# Wait for discovery server port to open (max 5s)
FDS_WAITED=0
while [[ $FDS_WAITED -lt 5 ]]; do
    if ss -ulnp 2>/dev/null | grep -q ":${FDS_PORT}"; then
        break
    fi
    sleep 1
    FDS_WAITED=$((FDS_WAITED + 1))
done
if [[ $FDS_WAITED -ge 5 ]]; then
    echo "WARNING: FastDDS discovery server port ${FDS_PORT} not open after 5s — continuing anyway"
else
    echo "       Discovery server ready after ${FDS_WAITED}s."
fi

# --- Step 2: Restart ROS2 daemon with discovery env vars ---
# The daemon must be restarted AFTER the discovery server starts and AFTER
# ROS_DISCOVERY_SERVER is set so that it registers as a SuperClient.
echo "[2/5] Restarting ROS2 daemon with discovery env vars..."
export ROS_DISCOVERY_SERVER="${FDS_HOST}:${FDS_PORT}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=1
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true
sleep 1

# --- Step 3: Start simulation ---
echo "[3/5] Starting simulation (speed=$SPEED, seed=$SEED)..."
tmux new -s sim -d "${ROS_ENV} cd $SANDBOX_ROOT && ./scripts/run_scenario.sh $SCENARIO_FILE --headless --seed $SEED --speed $SPEED"

# Wait for sim ready (poll tmux output for "Simulation ready")
echo "       Waiting for sim ready..."
MAX_WAIT=60
WAITED=0
while [[ $WAITED -lt $MAX_WAIT ]]; do
    if tmux capture-pane -t sim -p -S -50 2>/dev/null | grep -q "Simulation ready"; then
        break
    fi
    sleep 1
    WAITED=$((WAITED + 1))
done

if [[ $WAITED -ge $MAX_WAIT ]]; then
    echo "ERROR: Sim did not become ready within ${MAX_WAIT}s"
    exit 1
fi
# Brief pause: let the ros_gz_bridge publish its first /clock messages before
# Nav2 starts — the lifecycle manager's bond check fires immediately at startup
# and needs sim clock to be flowing or it hits the 4s wall-clock timeout.
echo "       Sim ready after ${WAITED}s. Waiting 3s for clock bridge to settle..."
sleep 3
echo "       Starting SLAM + Nav2..."

# --- Step 4: Start SLAM (must be within 5s of sim ready) ---
echo "[4/5] Starting SLAM toolbox..."
tmux new -s slam -d "${ROS_ENV} cd $EXPLORER_ROOT && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$(cd $EXPLORER_ROOT && pwd)/config/slam_toolbox_params.yaml use_sim_time:=true"

# Start Nav2 immediately after SLAM
echo "      Starting Nav2..."
tmux new -s nav2 -d "${ROS_ENV} cd $EXPLORER_ROOT && ros2 launch $(cd $EXPLORER_ROOT && pwd)/launch/navigation_launch.py params_file:=$(cd $EXPLORER_ROOT && pwd)/config/derpbot_nav2_params.yaml use_sim_time:=true"

# Wait briefly for Nav2 lifecycle to finish activating
sleep 5

# --- Step 5: Start agent (optional) ---
if [[ $START_AGENT -eq 1 ]]; then
    echo "[5/5] Starting agent..."
    tmux new -s agent -d "${ROS_ENV} cd $EXPLORER_ROOT && python3.12 agent/agent_node.py"
else
    echo "[5/5] Skipping agent (--no-agent)"
fi

echo ""
echo "=== Stack launched ==="
echo "  tmux sessions: $(tmux ls 2>/dev/null | cut -d: -f1 | tr '\n' ' ')"
echo ""
echo "  Monitor RTF:    cd $SANDBOX_ROOT && python3.12 scripts/rtf_monitor.py --once"
echo "  World state:    cd $SANDBOX_ROOT && python3.12 scripts/world_state.py"
echo "  Agent logs:     tmux capture-pane -t agent -p -S -50"
echo "  Sim logs:       tmux capture-pane -t sim -p -S -30"
echo "  FDS logs:       tmux capture-pane -t fds -p -S -20"
