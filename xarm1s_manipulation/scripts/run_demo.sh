#!/usr/bin/env bash
# Run the manipulation node against the running sim. Sets ROS_LOCALHOST_ONLY=1
# (must match start_sim.sh, or the nodes won't discover each other).
# Default = ground-truth demo. Override with launch args, e.g.:
#   bash run_demo.sh target_source:=perception run_tag:=sim_clean n_trials:=20
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash
ros2 launch xarm1s_manipulation manipulation.launch.py "$@"
