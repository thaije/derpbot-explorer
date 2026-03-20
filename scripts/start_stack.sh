#!/usr/bin/env bash
# Start the full DerpBot autonomy stack with correct timing.
#
# Usage:
#   ./scripts/start_stack.sh [--speed N] [--seed N] [--scenario TIER] [--no-agent]
#
# Starts: sim → (wait ready) → SLAM + Nav2 → agent (within 5s of sim ready).
# Each component runs in its own tmux session: sim, slam, nav2, agent.
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

echo "=== DerpBot Stack Launcher ==="
echo "  speed=$SPEED  seed=$SEED  scenario=$SCENARIO  agent=$START_AGENT"
echo ""

# --- Step 0: Kill everything from previous runs ---
echo "[0/4] Cleaning up old processes..."
for sess in agent nav2 slam sim; do
    tmux kill-session -t "$sess" 2>/dev/null || true
done
sleep 1

# Kill sim/bridge processes by name — two rounds to catch respawns
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
    sleep 1
done
fuser -k 7400/tcp 2>/dev/null || true
sleep 1

# Restart ROS2 daemon to clear cached TF
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null || true
sleep 1

echo "[1/4] Starting simulation (speed=$SPEED, seed=$SEED)..."
tmux new -s sim -d "cd $SANDBOX_ROOT && numactl --cpunodebind=1 --membind=1 ./scripts/run_scenario.sh $SCENARIO_FILE --headless --seed $SEED --speed $SPEED"

# Wait for sim ready (poll world_state.py or check for "Simulation ready" in tmux)
echo "       Waiting for sim ready..."
MAX_WAIT=30
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
echo "       Sim ready after ${WAITED}s. Starting SLAM + Nav2 immediately..."

# --- Step 2: Start SLAM (must be within 5s of sim ready) ---
echo "[2/4] Starting SLAM toolbox..."
tmux new -s slam -d "cd $EXPLORER_ROOT && numactl --cpunodebind=1 --membind=1 ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$(cd $EXPLORER_ROOT && pwd)/config/slam_toolbox_params.yaml use_sim_time:=true"

# --- Step 3: Start Nav2 (immediately after SLAM) ---
echo "[3/4] Starting Nav2..."
tmux new -s nav2 -d "cd $EXPLORER_ROOT && numactl --cpunodebind=1 --membind=1 ros2 launch $(cd $EXPLORER_ROOT && pwd)/launch/navigation_launch.py params_file:=$(cd $EXPLORER_ROOT && pwd)/config/derpbot_nav2_params.yaml use_sim_time:=true"

# Wait briefly for Nav2 lifecycle to finish activating
sleep 5

# --- Step 4: Start agent (optional) ---
if [[ $START_AGENT -eq 1 ]]; then
    echo "[4/4] Starting agent..."
    tmux new -s agent -d "cd $EXPLORER_ROOT && numactl --cpunodebind=1 --membind=1 python3.12 agent/agent_node.py"
else
    echo "[4/4] Skipping agent (--no-agent)"
fi

echo ""
echo "=== Stack launched ==="
echo "  tmux sessions: $(tmux ls 2>/dev/null | cut -d: -f1 | tr '\n' ' ')"
echo ""
echo "  Monitor RTF:    cd $SANDBOX_ROOT && python3.12 scripts/rtf_monitor.py --samples 5"
echo "  World state:    cd $SANDBOX_ROOT && python3.12 scripts/world_state.py"
echo "  Agent logs:     tmux capture-pane -t agent -p -S -50"
echo "  Sim logs:       tmux capture-pane -t sim -p -S -30"
