#!/usr/bin/env bash
# =============================================================================
# Phase 4A LIVE GUI demo -- watch the arm leave coke + banana behind.
#
# Brings up the sim WITH the Gazebo window (gui:=true), clean depth (no noise),
# then runs ONE perception-driven pick-and-place pass. Expected on screen
# (post banana re-center, 2026-06-05 -- all biases are now 0):
#   coke_can  -> jaws close on it, MISS (1.5 mm tol, size limit), arm lifts EMPTY, stays
#   banana    -> CAPTURE (re-centered onto its link origin), carried to the green plate
#   golf_ball -> CAPTURE, carried to the green plate
#   blue_box  -> CAPTURE, carried to the green plate
# The console prints, per object:  capture_err=X.Xmm tol=Y.Ymm -> CAPTURE/MISS
#
# The real results CSV is moved aside first so this demo never pollutes it.
# The sim is LEFT RUNNING (~20 min) so you can pan around; kill_sim.sh ends it.
# =============================================================================
set -u
export ROS_LOCALHOST_ONLY=1
set +u; source /opt/ros/humble/setup.bash; cd "$HOME/thesis_ws"; source install/setup.bash; set -u

SD="$HOME/thesis_ws/src/xarm1s_manipulation/scripts"
CSV="$HOME/thesis_ws/src/evaluation/results/manip_results.csv"

bash "$SD/kill_sim.sh" >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 1

echo ">>> bringing up Gazebo GUI sim (clean depth) ..."
ros2 launch tof_perception sim_pose_estimation.launch.py gui:=true \
    > /tmp/demo_sim.log 2>&1 &

ctrl_active(){ timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; }
dets_live(){ timeout 8 ros2 topic echo /detected_objects --once >/dev/null 2>&1; }
for i in $(seq 1 50); do ctrl_active && break; sleep 2; done
for i in $(seq 1 30); do dets_live && break; sleep 2; done
if ! dets_live; then echo "!! bringup failed (no /detected_objects) -- see /tmp/demo_sim.log"; fi
echo ">>> SIM READY"

# protect the 480-row results CSV: move it aside, let the demo write a throwaway
[ -f "$CSV" ] && mv "$CSV" "$CSV.realbak"

echo ">>> RUNNING DEMO GRASP -- WATCH THE GAZEBO WINDOW NOW"
timeout 300 ros2 launch xarm1s_manipulation manipulation.launch.py \
    target_source:=perception run_tag:=demo_gui n_trials:=1 \
    execute_motion:=true bias_correction:=false > /tmp/demo_grasp.log 2>&1
echo ">>> DEMO GRASP DONE"

# restore the real results CSV (drop the throwaway demo rows)
rm -f "$CSV"
[ -f "$CSV.realbak" ] && mv "$CSV.realbak" "$CSV"
echo ">>> results CSV restored: $(wc -l < "$CSV" 2>/dev/null || echo 0) lines"

echo ">>> per-object outcome:"
grep -E 'capture_err=|CAPTURE|MISS' /tmp/demo_grasp.log | grep -E 'trial 0' || true

echo ">>> SIM LEFT UP for inspection (~20 min). Run kill_sim.sh to end early."
sleep 1200
bash "$SD/kill_sim.sh" >/dev/null 2>&1
