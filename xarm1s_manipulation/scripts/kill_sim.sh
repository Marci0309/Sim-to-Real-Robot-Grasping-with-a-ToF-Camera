#!/usr/bin/env bash
# Kill a stale/half-dead Gazebo sim + ros2_control bringup. Run as a file
# (`bash .../scripts/kill_sim.sh`) so the script's own cmdline does not match
# the pkill -f patterns (which would otherwise kill this shell).
# Kill the launch orchestrators first so they stop re-spawning children.
pkill -9 -f sim_pose_estimation
pkill -9 -f noise_injector
pkill -9 -f manipulation.launch
pkill -9 -f 'ros2 launch'
sleep 1
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f 'controller_manager/spawner'
pkill -9 -f robot_state_publisher
pkill -9 -f pose_estimator
pkill -9 -f grasp_executor
sleep 2
echo "remaining gazebo/spawner procs:"
pgrep -af gzserver || true
pgrep -af gzclient || true
pgrep -af 'controller_manager/spawner' || true
echo CLEAN_DONE
