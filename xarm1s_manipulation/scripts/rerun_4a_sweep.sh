#!/usr/bin/env bash
# =============================================================================
# Phase 4A CLEAN re-run sweep  (post-55mm-gripper, 2026-06).
#
# Runs ALL SIX perception conditions x N trials into a FRESH manip_results.csv.
# The old (append-accumulated, polluted) CSV is archived first, so the new file
# contains only this run's rows.
#
# Uses execute_motion:=false  -> pick-success is the ANALYTIC capture test
# (perceived-vs-true within capture_tol); no arm motion, ~2 min/condition.
#
# Each condition = a fresh sim bringup with perception on the right depth stream:
#   sim_clean         clean depth, no injector
#   sim_noisy_oakd    noise-injected depth, axial_scale = 1   (measured OAK-D)
#   sim_noisy_oakd_xK noise-injected depth, axial_scale = K   (ablation)
#
# Usage:
#   bash rerun_4a_sweep.sh [N_TRIALS]      # default 100
#
# *** UNTESTED end-to-end here (no Gazebo on this machine). VALIDATE FIRST with a
#     short run, inspect the CSV, THEN do the full 100:
#         bash rerun_4a_sweep.sh 10
# =============================================================================
set -u
export ROS_LOCALHOST_ONLY=1          # WSL DDS fix (matches start_sim.sh)
# ROS/ament setup scripts reference unset vars -> relax `set -u` around sources.
set +u; source /opt/ros/humble/setup.bash; set -u
cd "$HOME/thesis_ws"

SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
N="${1:-100}"
CSV="$HOME/thesis_ws/src/evaluation/results/manip_results.csv"

echo ">>> Building (description, manipulation, noise_injection, tof_perception) ..."
colcon build --packages-select xarm1s_description xarm1s_gazebo xarm1s_manipulation \
        noise_injection tof_perception > /tmp/sweep_build.log 2>&1 \
    || { echo "BUILD FAILED -- see /tmp/sweep_build.log"; exit 1; }
set +u; source install/setup.bash; set -u
INJ_YAML="$(ros2 pkg prefix noise_injection 2>/dev/null)/share/noise_injection/config/oakd_measured.yaml"
[ -f "$INJ_YAML" ] || INJ_YAML="$HOME/thesis_ws/install/noise_injection/share/noise_injection/config/oakd_measured.yaml"
[ -f "$INJ_YAML" ] || INJ_YAML="$HOME/thesis_ws/src/noise_injection/config/oakd_measured.yaml"

# name | depth_topic | axial_scale  (empty scale => clean, no injector)
CONDS=(
  "sim_clean|/overhead_depth/depth/image_raw|"
  "sim_noisy_oakd|/overhead_depth/depth/image_noisy|1.0"
  "sim_noisy_oakd_x2|/overhead_depth/depth/image_noisy|2.0"
  "sim_noisy_oakd_x5|/overhead_depth/depth/image_noisy|5.0"
  "sim_noisy_oakd_x10|/overhead_depth/depth/image_noisy|10.0"
  "sim_noisy_oakd_x20|/overhead_depth/depth/image_noisy|20.0"
)

# ---- archive the old (polluted) CSV so we start clean -----------------------
if [ -f "$CSV" ]; then
  BK="${CSV%.csv}.polluted_$(date +%Y%m%d_%H%M%S).csv"
  mv "$CSV" "$BK"; echo "archived old CSV -> $BK"
fi

SIM_PID=""; INJ_PID=""
teardown() {
  [ -n "$INJ_PID" ] && kill -9 "$INJ_PID" 2>/dev/null
  [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null
  bash "$SD/kill_sim.sh" >/dev/null 2>&1
  SIM_PID=""; INJ_PID=""
}
trap 'echo; echo "interrupted"; teardown; exit 130' INT TERM

ctrl_active() { timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; }
dets_live()   { timeout 8 ros2 topic echo /detected_objects --once >/dev/null 2>&1; }

bringup() {  # $1 depth_topic   $2 axial_scale ('' => clean)
  local depth="$1" scale="$2" attempt i
  for attempt in 1 2 3; do
    teardown
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 1
    ros2 launch tof_perception sim_pose_estimation.launch.py \
        depth_topic:="$depth" > /tmp/sweep_sim.log 2>&1 &
    SIM_PID=$!
    if [ -n "$scale" ]; then
      ros2 run noise_injection depth_noise_injector --ros-args \
          --params-file "$INJ_YAML" -p axial_scale:="$scale" -p use_sim_time:=true \
          > /tmp/sweep_inj.log 2>&1 &
      INJ_PID=$!
    fi
    for i in $(seq 1 40); do ctrl_active && break; sleep 2; done   # controllers ~80 s
    for i in $(seq 1 25); do dets_live  && break; sleep 2; done    # /detected_objects ~50 s
    if dets_live; then return 0; fi
    echo "  bringup attempt $attempt failed (no detections); retrying ..."
  done
  return 1
}

# ---- the sweep --------------------------------------------------------------
for cond in "${CONDS[@]}"; do
  IFS='|' read -r tag depth scale <<< "$cond"
  echo "==================== $tag  (axial_scale=${scale:-clean}) ===================="
  if ! bringup "$depth" "$scale"; then
    echo "  !! $tag bringup failed -- SKIPPING"; teardown; continue
  fi
  echo "  sim ready -> running $N varied-placement trials/object (analytic, no motion) ..."
  timeout 900 ros2 launch xarm1s_manipulation manipulation.launch.py \
      target_source:=perception run_tag:="$tag" n_trials:="$N" \
      execute_motion:=false carry_object:=false \
      vary_placement:=true bias_correction:=false > "/tmp/sweep_grasp_$tag.log" 2>&1
  rows=$(( $(wc -l < "$CSV" 2>/dev/null || echo 1) - 1 ))
  echo "  $tag done  (total rows in fresh CSV: $rows)"
  teardown
done

# ---- summary ----------------------------------------------------------------
echo; echo "######################## SUMMARY ########################"
python3 - "$CSV" <<'PY'
import csv, sys, collections
try:
    rows=list(csv.DictReader(open(sys.argv[1])))
except FileNotFoundError:
    print("no CSV produced -- every condition failed; check /tmp/sweep_*.log"); sys.exit(0)
order=['sim_clean','sim_noisy_oakd','sim_noisy_oakd_x2','sim_noisy_oakd_x5',
       'sim_noisy_oakd_x10','sim_noisy_oakd_x20']
objs=['coke_can','banana','golf_ball','blue_box']
G=collections.defaultdict(list)
for r in rows: G[(r['tag'],r['object'])].append(r)
print('%-20s %8s   %s'%('condition','overall','per-object pick rate'))
for tag in order:
    tot=k=0; cells=[]
    for o in objs:
        rs=G.get((tag,o),[])
        if not rs: continue
        n=len(rs); s=sum(int(r.get('grasp_feasible', r.get('pick_success', 0))) for r in rs); tot+=n; k+=s
        cells.append('%s %3.0f%%'%(o[:4], 100*s/n))
    if tot:
        print('%-20s %4d/%-4d %5.1f%%   %s'%(tag,k,tot,100*k/tot,'  '.join(cells)))
print('\nfresh CSV ->', sys.argv[1])
PY
echo "logs: /tmp/sweep_grasp_<condition>.log"
