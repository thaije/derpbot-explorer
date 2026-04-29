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
NO_PERCEPTION=0
NO_SUBSCRIBERS=0

# Parse args
while [[ $# -gt 0 ]]; do
    case $1 in
        --speed)   SPEED="$2"; shift 2 ;;
        --seed)    SEED="$2"; shift 2 ;;
        --scenario) SCENARIO="$2"; shift 2 ;;
        --no-agent) START_AGENT=0; shift ;;
        --no-perception) NO_PERCEPTION=1; shift ;;
        --no-subscribers) NO_SUBSCRIBERS=1; shift ;;
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
echo "  speed=$SPEED  seed=$SEED  scenario=$SCENARIO  agent=$START_AGENT  no_perception=$NO_PERCEPTION"
echo ""

# --- Step 0: Kill everything from previous runs ---
echo "[0/5] Cleaning up old processes..."
FDS_PORT="${FDS_PORT}" "$SCRIPT_DIR/cleanup.sh"

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
# we touch the control service. 1s is enough — we no longer need to wait
# for Nav2 here because Gazebo will be paused before Nav2 starts.
echo "       Sim ready after ${WAITED}s. Waiting 1s for clock bridge to settle..."
sleep 1

# --- #18 fix: pause Gazebo so wall-time stack bringup burns zero sim-s ---
# Without this, ~15–25 wall-s of slam+nav2+agent cold-launch is multiplied by
# RTF on the sim clock, consuming ~40 sim-s of the 300 s mission budget before
# the agent ever processes its first /map.
#
# Discover the world control service dynamically so this works for any tier.
WORLD_CONTROL="$(gz service -l 2>/dev/null | grep -oE '^/world/[^/]+/control$' | head -1 || true)"
if [[ -z "$WORLD_CONTROL" ]]; then
    echo "       WARNING: could not discover /world/<name>/control service — skipping pause."
    PAUSED=0
else
    echo "       Pausing sim via ${WORLD_CONTROL}..."
    if gz service -s "$WORLD_CONTROL" \
        --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
        --timeout 2000 --req 'pause: true' >/dev/null 2>&1; then
        PAUSED=1
        # Safety: unpause on script exit (success or failure) so a crashed
        # launch can never leave Gazebo paused forever.
        trap 'if [[ "${PAUSED:-0}" == "1" && -n "${WORLD_CONTROL:-}" ]]; then
                gz service -s "$WORLD_CONTROL" \
                    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
                    --timeout 2000 --req "pause: false" >/dev/null 2>&1 || true
              fi' EXIT
    else
        echo "       WARNING: pause request failed — continuing without pause."
        PAUSED=0
    fi
fi

# Clear any stale agent-ready flag from a previous run.
READY_FLAG="${DERPBOT_READY_FLAG:-/tmp/derpbot_agent_ready}"
rm -f "$READY_FLAG"

echo "       Starting SLAM + Nav2..."

# --- Step 4: Start SLAM (must be within 5s of sim ready) ---
echo "[4/5] Starting SLAM toolbox..."
tmux new -s slam -d "${ROS_ENV} cd $EXPLORER_ROOT && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$(cd $EXPLORER_ROOT && pwd)/config/slam_toolbox_params.yaml use_sim_time:=true"

# Start Nav2 immediately after SLAM
echo "      Starting Nav2..."
tmux new -s nav2 -d "${ROS_ENV} cd $EXPLORER_ROOT && ros2 launch $(cd $EXPLORER_ROOT && pwd)/launch/navigation_launch.py params_file:=$(cd $EXPLORER_ROOT && pwd)/config/derpbot_nav2_params.yaml use_sim_time:=true"

# Nav2 lifecycle activation happens wall-clock, not sim-clock, so it proceeds
# normally while Gazebo is paused. No wall-sleep needed here — the agent's
# own wait_for_server will block until the nav2 action server is discovered.

# --- Step 5: Start agent (optional) ---
if [[ $START_AGENT -eq 1 ]]; then
    AGENT_FLAGS=""
    [[ $NO_PERCEPTION -eq 1 ]] && AGENT_FLAGS="--no-perception"
    [[ $NO_SUBSCRIBERS -eq 1 ]] && AGENT_FLAGS="$AGENT_FLAGS --no-subscribers"
    echo "[5/5] Starting agent (flags: ${AGENT_FLAGS:-none})..."
    # Pass through DERPBOT_FRONTIER_DEBUG (issue #10 diagnostic dumps) if set.
    FRONTIER_DEBUG_FWD=""
    [[ -n "${DERPBOT_FRONTIER_DEBUG:-}" ]] && FRONTIER_DEBUG_FWD="DERPBOT_FRONTIER_DEBUG='${DERPBOT_FRONTIER_DEBUG}'"
    tmux new -s agent -d "${ROS_ENV} cd $EXPLORER_ROOT && DERPBOT_READY_FLAG='$READY_FLAG' $FRONTIER_DEBUG_FWD $EXPLORER_ROOT/.venv/bin/python agent/agent_node.py ${AGENT_FLAGS}"

    if [[ "${PAUSED:-0}" == "1" ]]; then
        # Poll for the agent's ready flag. The flag is created at the top of
        # _explore_loop in frontier_explorer.py — after AgentNode.__init__
        # has completed all wall-time work (imports, OWLv2 load, Nav2 client
        # construction). See #18.
        echo "       Waiting for agent ready flag (${READY_FLAG})..."
        READY_WAITED=0
        READY_MAX=60  # wall-s; OWLv2 weight load can take 20-30 wall-s on cold cache
        while [[ $READY_WAITED -lt $READY_MAX ]]; do
            if [[ -f "$READY_FLAG" ]]; then
                break
            fi
            sleep 1
            READY_WAITED=$((READY_WAITED + 1))
        done
        if [[ ! -f "$READY_FLAG" ]]; then
            echo "       WARNING: agent did not signal ready within ${READY_MAX}s — unpausing anyway."
        else
            echo "       Agent ready after ${READY_WAITED}s wall — unpausing sim."
        fi
        gz service -s "$WORLD_CONTROL" \
            --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
            --timeout 2000 --req 'pause: false' >/dev/null 2>&1 || true
        PAUSED=0
    fi
else
    echo "[5/5] Skipping agent (--no-agent)"
    if [[ "${PAUSED:-0}" == "1" ]]; then
        gz service -s "$WORLD_CONTROL" \
            --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
            --timeout 2000 --req 'pause: false' >/dev/null 2>&1 || true
        PAUSED=0
    fi
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
