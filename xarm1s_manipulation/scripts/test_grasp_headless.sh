#!/usr/bin/env bash
# Headless smoke test for the contact-triggered + effort-controlled grasp
# (see ../GRASP_REALISM_NOTES.md). Brings up gzserver with NO GUI, waits for the
# effort gripper_controller to go active, runs a ground-truth pick-and-place over
# the given objects, then reports: the grasp plugin's events (each weld should be
# "contact-triggered", not "FALLBACK"), and each object's before/after pose
# (carried to its plate spot = PASS; left at origin = grasp missed; off the table
# = FAIL). Usage:
#   bash test_grasp_headless.sh ['blue_box coke_can golf_ball banana']
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash
OBJS="${1:-blue_box coke_can golf_ball banana}"
SIMLOG=/tmp/grasp_test_sim.log
DEMOLOG=/tmp/grasp_test_demo.log
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MON_PID=""
cleanup(){ [ -n "$MON_PID" ] && kill -9 "$MON_PID" 2>/dev/null; bash "$SCRIPTS/kill_sim.sh" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM

SIM_PID=""
bringup(){
  bash "$SCRIPTS/kill_sim.sh" >/dev/null 2>&1
  ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
  sleep 1
  ros2 launch xarm1s_description spawn_robot.launch.py gui:=false > "$SIMLOG" 2>&1 &
  SIM_PID=$!
  for i in $(seq 1 45); do          # up to ~90 s
    if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "gripper_controller.*active" \
       && timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

ok=0
for attempt in 1 2 3; do
  echo "=== bringup attempt $attempt (headless) ==="
  if bringup; then ok=1; break; fi
  echo "  controllers not active; retrying"
  [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null
done
if [ "$ok" -ne 1 ]; then echo "SIM FAILED TO BRING UP"; tail -40 "$SIMLOG"; exit 1; fi

echo ">>> controllers:"; ros2 control list_controllers
echo ">>> plugin ready:"; grep -i "link_attacher ready" "$SIMLOG" | tail -1
sleep 3   # let objects settle on the table

python3 - <<'PY'
import json, time, rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
rclpy.init(); n=Node('s'); got={}
def cb(m):
    for name,p in zip(m.name,m.pose):
        got[name]=[round(p.position.x,3),round(p.position.y,3),round(p.position.z,3)]
n.create_subscription(ModelStates,'/model_states',cb,10)
t=time.time()
while time.time()-t<5:
    rclpy.spin_once(n,timeout_sec=0.3)
    if got: break
json.dump(got, open('/tmp/grasp_baseline.json','w'))
print(">>> BASELINE:", json.dumps(got)); rclpy.shutdown()
PY

# continuous pose monitor (min-z = sink check, excursion = fling check)
python3 "$SCRIPTS/grasp_monitor.py" &
MON_PID=$!

OBJ_LIST=$(echo "$OBJS" | sed 's/ /, /g')
echo ">>> running ground-truth demo for: [$OBJ_LIST]"
ros2 run xarm1s_manipulation grasp_executor --ros-args \
  -p use_sim_time:=true -p target_source:=ground_truth -p execute_motion:=true \
  -p "objects:=[$OBJ_LIST]" -p n_trials:=1 > "$DEMOLOG" 2>&1
echo ">>> demo exit=$?"

echo ">>> grasp plugin events:"
grep -iE "armed|grasped|detached|FALLBACK" "$SIMLOG" | tail -30

kill -9 "$MON_PID" 2>/dev/null; MON_PID=""
echo ">>> min z over the whole run (sink check; tabletop top = 0.750):"
python3 - <<'PY'
import csv
mn = {}
for r in csv.DictReader(open('/tmp/grasp_track.csv')):
    o, z = r['obj'], float(r['z'])
    if o not in mn or z < mn[o]:
        mn[o] = z
for o in ('coke_can', 'banana', 'golf_ball', 'blue_box'):
    if o in mn:
        flag = ('OK (never under table)' if mn[o] > 0.735
                else ('DIPPED under table' if mn[o] > 0.5 else 'FELL THROUGH/flung'))
        print(f"   {o:10} min_z = {mn[o]:.3f}  [{flag}]")
PY

python3 - <<'PY'
import json, time, rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
base=json.load(open('/tmp/grasp_baseline.json'))
PLATE={'coke_can':(0.27,0.07),'banana':(0.27,-0.07),'golf_ball':(0.19,0.05),'blue_box':(0.19,-0.05)}
rclpy.init(); n=Node('s'); got={}
def cb(m):
    for name,p in zip(m.name,m.pose):
        got[name]=[round(p.position.x,3),round(p.position.y,3),round(p.position.z,3)]
n.create_subscription(ModelStates,'/model_states',cb,10)
t=time.time()
while time.time()-t<5:
    rclpy.spin_once(n,timeout_sec=0.3)
    if got: break
print(">>> FINAL vs plate spot (placed = near spot, resting ~z0.75-0.80):")
for o,tgt in PLATE.items():
    if o not in got:
        print(f"   {o:10} MISSING from /model_states"); continue
    x,y,z=got[o]; b=base.get(o,[None]*3)
    d=((x-tgt[0])**2+(y-tgt[1])**2)**0.5
    sane = 0.72 < z < 0.86
    flag = ("OK placed" if (sane and d<0.08)
            else ("FLUNG/UNSANE" if not sane else f"off-target ({d*1000:.0f}mm)"))
    print(f"   {o:10} base={b} -> final={x,y,z}  dist_to_spot={d*1000:5.0f}mm  [{flag}]")
rclpy.shutdown()
PY
echo ">>> done"
