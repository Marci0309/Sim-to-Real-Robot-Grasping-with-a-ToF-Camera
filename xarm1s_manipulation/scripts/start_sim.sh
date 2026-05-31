#!/usr/bin/env bash
# Reliable sim bringup. The Humble gazebo_ros2_control controller_manager
# sometimes comes up slowly/wedged, leaving the joint_state_broadcaster spawner
# hung. This wrapper does a clean restart, launches the sim, waits for the
# controllers to actually go ACTIVE, and retries the whole thing if they don't.
# Keeps the sim (and Gazebo window) alive until you Ctrl-C.
# ROS_LOCALHOST_ONLY=1: the controller_manager hang was DDS discovery failing
# over WSL's Hyper-V network (service registers but is uncontactable). Forcing
# ROS traffic to loopback fixes it (everything runs on one machine here).
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LAUNCH_PID=""
kill_launch() { [ -n "$LAUNCH_PID" ] && kill -9 "$LAUNCH_PID" 2>/dev/null; }
cleanup() { kill_launch; bash "$SCRIPT_DIR/kill_sim.sh" >/dev/null 2>&1; }
trap cleanup INT TERM

for attempt in 1 2 3 4 5 6; do
  echo "=== bringup attempt $attempt ==="
  bash "$SCRIPT_DIR/kill_sim.sh" >/dev/null 2>&1
  ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
  sleep 1
  ros2 launch tof_perception sim_pose_estimation.launch.py \
      > /tmp/thesis_sim_bringup.log 2>&1 &
  LAUNCH_PID=$!
  for i in $(seq 1 35); do          # up to ~70 s per attempt
    # timeout 5: when the controller_manager loses the init race it is
    # uncontactable and this call would otherwise hang forever (no retry).
    if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
      echo ""
      echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
      echo ">>> SIM READY  -- controllers active, Gazebo window is open."
      echo ">>> In ANOTHER Ubuntu terminal run the demo:"
      echo ">>>   bash $SCRIPT_DIR/run_demo.sh"
      echo ">>> Ctrl-C HERE stops the sim when you are done."
      echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
      trap cleanup EXIT INT TERM
      wait "$LAUNCH_PID"
      exit 0
    fi
    [ $((i % 5)) -eq 0 ] && echo "  ...waiting for controllers to activate ($((i*2))s) — normal, hang tight"
    sleep 2
  done
  echo "attempt $attempt: controllers not active after ~70 s; killing and retrying..."
  kill_launch; LAUNCH_PID=""
done
echo "FAILED to bring up controllers after 6 attempts. See /tmp/thesis_sim_bringup.log"
cleanup
exit 1
