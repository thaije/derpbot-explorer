#!/usr/bin/env bash
# slam_restart_watch.sh — one-shot slam_toolbox restart watcher (t_8673a87f).
#
# Contract: the agent CANNOT restart slam_toolbox itself (only the outer stack
# script may restart nodes without duplicating them). The agent writes
# /tmp/derpbot_slam_restart_request (diagnostic JSON) when no /map arrives
# within one 45-s grace window; this watcher — launched by start_stack.sh in
# its own tmux session — performs exactly ONE clean kill + relaunch of the
# slam session, then exits.
#
# Exactly-once guarantees:
#   - The agent writes the flag at most once (sentinel state machine).
#   - The watcher consumes (deletes) the flag before restarting and never
#     re-arms after its single restart.
#   - start_stack.sh deletes any stale flag before arming the watcher, so a
#     leftover flag from a previous run can never restart a fresh slam.
#
# The relaunch command must stay byte-identical to the original slam launch in
# start_stack.sh (same params, use_sim_time, discovery env).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPLORER_ROOT="$(dirname "$SCRIPT_DIR")"

FLAG="${DERPBOT_SLAM_RESTART_FLAG:-/tmp/derpbot_slam_restart_request}"
ARM_TIMEOUT="${DERPBOT_SLAM_WATCHER_TIMEOUT:-300}"   # wall-s: give up watching after this
GRACE_POLL="1"

# ROS discovery env so the relaunched slam finds the sim (start_stack.sh env
# is inherited via the tmux session, but be explicit for the slam command).
ROS_ENV="export ROS_DISCOVERY_SERVER=127.0.0.1:11811; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export ROS_SUPER_CLIENT=1;"

# IMPORTANT (forced-sim-test finding, t_8673a87f): the watcher's tmux session
# does NOT reliably inherit a ROS-sourced environment — new tmux sessions get
# the CLIENT shell's env merged over the server env, and a watcher armed from
# a non-ROS shell launches `ros2: command not found`. Source ROS explicitly in
# the relaunch command so restart works from any arming context.
SLAM_LAUNCH="source /opt/ros/jazzy/setup.bash; export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/usr/lib/python3/dist-packages; ${ROS_ENV} cd $EXPLORER_ROOT && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$EXPLORER_ROOT/config/slam_toolbox_params.yaml use_sim_time:=true"

echo "[slam-restart-watch] watching ${FLAG} (exits after one restart or ${ARM_TIMEOUT}s)"

WAITED=0
while [[ $WAITED -lt $ARM_TIMEOUT ]]; do
    if [[ -f "$FLAG" ]]; then
        echo "[slam-restart-watch] restart flag detected after ${WAITED}s"
        cat "$FLAG" || true
        rm -f "$FLAG"   # consume first: a crash mid-restart can't double-fire
        break
    fi
    sleep "$GRACE_POLL"
    WAITED=$((WAITED + GRACE_POLL))
done

if [[ $WAITED -ge $ARM_TIMEOUT ]]; then
    echo "[slam-restart-watch] no flag within ${ARM_TIMEOUT}s — exiting (slam was healthy)"
    exit 0
fi

# --- Single restart -------------------------------------------------------
# Kill the slam tmux session and any lingering slam processes, then relaunch
# with the exact original command from start_stack.sh.
echo "[slam-restart-watch] killing slam session..."
tmux kill-session -t slam 2>/dev/null || true
pkill -9 -f "slam_toolbox" 2>/dev/null || true
sleep 2

echo "[slam-restart-watch] relaunching slam_toolbox (once)..."
tmux new -s slam -d "${SLAM_LAUNCH}"

echo "[slam-restart-watch] restart complete — watcher exiting (never restarts again)"
exit 0