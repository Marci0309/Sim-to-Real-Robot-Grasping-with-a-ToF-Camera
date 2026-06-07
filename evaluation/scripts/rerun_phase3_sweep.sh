#!/usr/bin/env bash
# =============================================================================
# Phase 3 CLEAN re-run sweep  (post model-swap: slim 250 ml coke + re-centred
# solid-yellow banana, 2026-06).
#
# Regenerates ALL pose-accuracy conditions into a FRESH run.csv. The previous
# run.csv was APPEND-accumulated across many runs AND predates the model swap,
# so its coke (~80 mm) / banana (~38 mm) rows are a stale mesh artefact from the
# old thick 330 ml can + off-centre banana. We archive it, regenerate the sim
# rows against the CURRENT models + perception, and carry the Phase-4B `real`
# rows forward untouched (compute_sim_to_real_gap.py reads them from here).
#
# Per condition = a fresh headless sim bringup with perception on the right
# depth stream, then pose_evaluator logs /detected_objects-vs-/model_states
# position error for DURATION seconds, tagged with run_tag:
#   sim_clean           clean Gazebo depth, no injector
#   sim_noisy_nguyen    injector, Nguyen-Kinect literature YAML  (axial_scale 1)
#   sim_noisy_oakd      injector, measured OAK-D YAML            (axial_scale 1)
#   sim_noisy_oakd_xK   injector, measured OAK-D YAML, axial_scale = K (ablation)
#
# STATIC-scene methodology (objects at fixed spawn poses) -- matches the original
# Phase-3 design; varied placement is Phase-4A's separate contribution.
#
# Usage:
#   bash rerun_phase3_sweep.sh [DURATION_SEC] [ONLY_TAG_SUBSTR]
#       DURATION_SEC     seconds of logging per condition (default 60)
#       ONLY_TAG_SUBSTR  if set, only run conditions whose tag contains it
#   env:
#       SKIP_BUILD=1     skip the colcon build
#       CSV_OUT=<path>   write to a different CSV (use a throwaway for validation)
#
# Validate first on ONE condition into a throwaway CSV, then do the full run:
#   CSV_OUT=/tmp/run_validate.csv SKIP_BUILD=1 bash rerun_phase3_sweep.sh 15 sim_clean
#   bash rerun_phase3_sweep.sh 60
# =============================================================================
set -u
export ROS_LOCALHOST_ONLY=1                       # WSL DDS fix (matches start_sim.sh)
set +u; source /opt/ros/humble/setup.bash; set -u
cd "$HOME/thesis_ws"

MANIP_SCRIPTS="$HOME/thesis_ws/src/xarm1s_manipulation/scripts"   # reuse kill_sim.sh
DUR="${1:-60}"
ONLY="${2:-}"
CSV="${CSV_OUT:-$HOME/thesis_ws/src/evaluation/results/run.csv}"

if [ "${SKIP_BUILD:-0}" != "1" ]; then
  echo ">>> Building perception packages (description, gazebo, noise, tof, evaluation) ..."
  colcon build --packages-select xarm1s_description xarm1s_gazebo \
          noise_injection tof_perception evaluation > /tmp/p3_build.log 2>&1 \
      || { echo "BUILD FAILED -- see /tmp/p3_build.log"; tail -20 /tmp/p3_build.log; exit 1; }
fi
set +u; source install/setup.bash; set -u

share="$(ros2 pkg prefix noise_injection 2>/dev/null)/share/noise_injection/config"
OAKD_YAML="$share/oakd_measured.yaml";   [ -f "$OAKD_YAML" ]   || OAKD_YAML="$HOME/thesis_ws/src/noise_injection/config/oakd_measured.yaml"
NGUYEN_YAML="$share/nguyen_kinect.yaml"; [ -f "$NGUYEN_YAML" ] || NGUYEN_YAML="$HOME/thesis_ws/src/noise_injection/config/nguyen_kinect.yaml"

# tag | depth_topic | params_yaml | axial_scale   (empty yaml => clean, no injector)
CONDS=(
  "sim_clean|/overhead_depth/depth/image_raw||"
  "sim_noisy_nguyen|/overhead_depth/depth/image_noisy|$NGUYEN_YAML|1.0"
  "sim_noisy_oakd|/overhead_depth/depth/image_noisy|$OAKD_YAML|1.0"
  "sim_noisy_oakd_x2|/overhead_depth/depth/image_noisy|$OAKD_YAML|2.0"
  "sim_noisy_oakd_x5|/overhead_depth/depth/image_noisy|$OAKD_YAML|5.0"
  "sim_noisy_oakd_x10|/overhead_depth/depth/image_noisy|$OAKD_YAML|10.0"
  "sim_noisy_oakd_x20|/overhead_depth/depth/image_noisy|$OAKD_YAML|20.0"
)

# ---- archive old CSV, preserve Phase-4B `real` rows -------------------------
# SKIP_ARCHIVE=1 appends to an existing fresh CSV (for resuming a chunked sweep);
# REAL_TMP from the archiving invocation persists in /tmp across chunks.
REAL_TMP="/tmp/p3_real_rows.csv"
if [ "${SKIP_ARCHIVE:-0}" != "1" ]; then
  : > "$REAL_TMP"
  if [ -f "$CSV" ]; then
    BK="${CSV%.csv}.polluted_$(date +%Y%m%d_%H%M%S).csv"
    cp "$CSV" "$BK"
    grep '^real,' "$CSV" > "$REAL_TMP" 2>/dev/null || true
    rm -f "$CSV"
    echo "archived old CSV -> $BK ; preserved $(wc -l < "$REAL_TMP") real row(s)"
  fi
else
  echo "SKIP_ARCHIVE=1 -> appending to existing $CSV"
fi

SIM_PID=""; INJ_PID=""
teardown() {
  [ -n "$INJ_PID" ] && kill -9 "$INJ_PID" 2>/dev/null
  [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null
  bash "$MANIP_SCRIPTS/kill_sim.sh" >/dev/null 2>&1
  SIM_PID=""; INJ_PID=""
}
trap 'echo; echo "interrupted"; teardown; exit 130' INT TERM

dets_live() { timeout 8 ros2 topic echo /detected_objects --once >/dev/null 2>&1; }

bringup() {  # $1 depth  $2 yaml  $3 scale
  local depth="$1" yaml="$2" scale="$3" attempt i
  for attempt in 1 2 3; do
    teardown
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 1
    ros2 launch tof_perception sim_pose_estimation.launch.py \
        gui:=false depth_topic:="$depth" > /tmp/p3_sim.log 2>&1 &
    SIM_PID=$!
    if [ -n "$yaml" ]; then
      ros2 run noise_injection depth_noise_injector --ros-args \
          --params-file "$yaml" -p axial_scale:="$scale" -p use_sim_time:=true \
          > /tmp/p3_inj.log 2>&1 &
      INJ_PID=$!
    fi
    for i in $(seq 1 30); do dets_live && break; sleep 3; done   # detections ~30-90 s
    if dets_live; then return 0; fi
    echo "  bringup attempt $attempt failed (no detections); retrying ..."
  done
  return 1
}

# ---- the sweep --------------------------------------------------------------
for cond in "${CONDS[@]}"; do
  IFS='|' read -r tag depth yaml scale <<< "$cond"
  # ONLY = comma-separated list of EXACT tags to run (empty => all)
  if [ -n "$ONLY" ]; then case ",$ONLY," in *",$tag,"*) ;; *) continue ;; esac; fi
  echo "==================== $tag  (scale=${scale:-clean}) ===================="
  if ! bringup "$depth" "$yaml" "$scale"; then
    echo "  !! $tag bringup failed -- SKIPPING"; teardown; continue
  fi
  echo "  sim ready -> logging pose error for ${DUR}s ..."
  # timeout sends SIGINT at DUR (-> pose_evaluator.summarise + clean CSV close),
  # then SIGKILL 5 s later as a backstop.
  timeout -k 5 -s INT "$DUR" ros2 run evaluation pose_evaluator --ros-args \
      -p run_tag:="$tag" -p use_sim_time:=true -p output_csv:="$CSV" \
      > "/tmp/p3_eval_$tag.log" 2>&1 || true
  rows=$(( $(wc -l < "$CSV" 2>/dev/null || echo 1) - 1 ))
  echo "  $tag done  (rows so far: $rows)"
  grep -E 'RMSE|detection rate' "/tmp/p3_eval_$tag.log" | tail -6 | sed 's/^/      /'
  teardown
done

# ---- carry the Phase-4B real rows back (idempotent across chunks) ------------
if [ -s "$REAL_TMP" ] && [ -f "$CSV" ] && ! grep -q '^real,' "$CSV" 2>/dev/null; then
  cat "$REAL_TMP" >> "$CSV"
  echo "re-appended $(wc -l < "$REAL_TMP") real row(s) to fresh run.csv"
fi

# ---- summary ----------------------------------------------------------------
echo; echo "############## SUMMARY  (mean planar / 3D-norm per object, mm) ##############"
python3 - "$CSV" <<'PY'
import csv, sys, math, collections
rows=collections.defaultdict(list)
try:
    for r in csv.DictReader(open(sys.argv[1])): rows[r['tag']].append(r)
except FileNotFoundError:
    print("no CSV produced -- every condition failed; check /tmp/p3_*.log"); sys.exit(0)
order=['sim_clean','sim_noisy_nguyen','sim_noisy_oakd','sim_noisy_oakd_x2',
       'sim_noisy_oakd_x5','sim_noisy_oakd_x10','sim_noisy_oakd_x20','real']
objs=['coke_can','banana','golf_ball','blue_box']
print('%-20s %6s   %s'%('tag','n','per-object  planar / 3Dnorm (mm)'))
for tag in order:
    rs=rows.get(tag,[])
    if not rs: continue
    cells=[]
    for o in objs:
        pl=[]; nm=[]
        for r in rs:
            if r.get(o+'_norm','') in ('','nan'): continue
            pl.append(1000*math.hypot(float(r[o+'_dx']),float(r[o+'_dy'])))
            nm.append(1000*float(r[o+'_norm']))
        cells.append('%-4s %5.1f/%-5.1f'%(o[:4], (sum(pl)/len(pl) if pl else 0),
                                          (sum(nm)/len(nm) if nm else 0)))
    print('%-20s %6d   %s'%(tag,len(rs),'  '.join(cells)))
print('\nfresh CSV ->', sys.argv[1])
PY
echo "logs: /tmp/p3_eval_<tag>.log ; sim /tmp/p3_sim.log ; inj /tmp/p3_inj.log"
