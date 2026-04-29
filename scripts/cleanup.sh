#!/usr/bin/env bash

# FDS_PORT defaults to 11811 if not set (from start_stack.sh)
FDS_PORT="${FDS_PORT:-11811}"

# Gracefully stop agent_node first so it can flush its profile.
# SIGKILL (from tmux kill-session) skips Python finally blocks; SIGINT raises
# KeyboardInterrupt in the main thread → except/finally → shutdown() → _write_timeline().
pkill -INT -f "agent_node" 2>/dev/null || true
sleep 3

echo "Cleaning up tmux sessions..."
for sess in agent nav2 slam sim fds; do
    tmux kill-session -t "$sess" 2>/dev/null || true
done

echo "Killing remaining processes..."
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

echo "Freeing ports..."
fuser -k 7400/tcp 2>/dev/null || true
fuser -k 11811/udp 2>/dev/null || true

echo "cleanup done"
