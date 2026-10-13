#!/usr/bin/env bash
# Launch Webots (Windows host) and the ROS 2 stack (WSL2). Usage: bash scripts/sim.sh [rviz:=true]
set -e
source /opt/ros/jazzy/setup.bash
source ~/quad_ws/install/setup.bash
# Webots installed per-user on Windows; webots_ros2 looks for it here from WSL.
export WEBOTS_HOME="${WEBOTS_HOME:-/mnt/c/Users/PC/AppData/Local/Programs/Webots}"
exec ros2 launch quad_webots sim.launch.py "$@"
