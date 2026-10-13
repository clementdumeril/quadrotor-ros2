#!/usr/bin/env bash
# Run one ROS 2 command with the workspace sourced, e.g.  bash scripts/ros.sh ros2 topic list
source /opt/ros/jazzy/setup.bash
source ~/quad_ws/install/setup.bash
export WEBOTS_HOME="${WEBOTS_HOME:-/mnt/c/Users/PC/AppData/Local/Programs/Webots}"
exec "$@"
