#!/usr/bin/env bash
# Source this before running any ROS 2 CLI commands or monitoring scripts
# when the FastDDS discovery server is active (started by start_stack.sh).
#
# Usage:
#   . ~/Projects/derpbot-explorer/scripts/ros_env.sh
#   source ~/Projects/derpbot-explorer/scripts/ros_env.sh
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=1
