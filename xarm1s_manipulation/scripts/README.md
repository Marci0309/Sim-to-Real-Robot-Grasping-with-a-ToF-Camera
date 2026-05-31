# xarm1s_manipulation / scripts

Operational helpers for the sim manipulation demo. All set `ROS_LOCALHOST_ONLY=1`
(fixes the WSL DDS controller_manager hang — see project notes) and source ROS +
the workspace. Run them as files, e.g. `bash ~/thesis_ws/src/xarm1s_manipulation/scripts/start_sim.sh`.

| Script | What it does |
|---|---|
| `start_sim.sh` | Reliable sim bringup: clean restart → launch `tof_perception sim_pose_estimation` → **wait until the controllers go ACTIVE, auto-retrying** (up to 6×) the flaky gazebo_ros2_control bringup. Prints `SIM READY`; keeps the sim alive until Ctrl-C. Run this in Terminal 1. |
| `run_demo.sh` | Runs `manipulation.launch.py` against the running sim. Default = ground-truth pick-and-place demo. Pass launch args, e.g. `bash run_demo.sh target_source:=perception n_trials:=20`. Run this in Terminal 2. |
| `reset.sh` | Reset the four objects to their start poses (run between demo runs; GT mode leaves objects at the drop spot). Wraps `reset_objects.py`. |
| `kill_sim.sh` | Tear down a stale/half-dead sim (kills the launch first, then gazebo/spawners; written to avoid pkill self-match). |
| `reset_objects.py` | rclpy one-shot: `set_entity_state` each object to its canonical world pose. |
| `post_check.py` | rclpy one-shot: print current world positions of the four objects (debug). |
| `venv_check.py` | Report which thesis Python deps import in the running interpreter (run with `~/thesis_ws/.venv/bin/python`). |

**Typical session**
```
# Terminal 1
bash ~/thesis_ws/src/xarm1s_manipulation/scripts/start_sim.sh    # wait for SIM READY
# Terminal 2
bash ~/thesis_ws/src/xarm1s_manipulation/scripts/run_demo.sh     # watch the arm
```
