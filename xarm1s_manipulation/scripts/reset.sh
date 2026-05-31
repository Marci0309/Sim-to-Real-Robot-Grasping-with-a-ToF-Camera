#!/usr/bin/env bash
# Reset the four objects to their start positions (run between demo runs).
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/reset_objects.py"
