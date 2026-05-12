# Running the Experiment

End-to-end recipe for the simulator side of the thesis: spawn the xArm 1S
+ overhead RGB-D in Gazebo, run perception under either *Sim-clean* or
*Sim-noisy* conditions, log per-frame pose error vs. Gazebo ground truth.

## Scene

`tabletop.world` spawns four objects on a 0.8 m square table at z = 0.75 m
(top surface), arranged in a 2×2 grid 0.30 m apart:

| Object | World pose (x, y) | Notes |
|---|---|---|
| `coke_can`  | (+0.25, +0.15) | textured Sketchfab mesh, lying on its side |
| `banana`    | (+0.25, −0.15) | textured Sketchfab mesh, lying flat |
| `golf_ball` | (−0.25, +0.15) | magenta sphere primitive (HSV-distinct from red/yellow/blue) |
| `blue_box`  | (−0.25, −0.15) | inline 60×40×30 mm box primitive |

Camera is mounted top-down 0.68 m above the table surface (matches the
real-world calibration distance used in Phase 2).

## Build

If anything in `src/` changed, rebuild first:

```bash
cd ~/thesis_ws
colcon build --symlink-install
source install/setup.bash
```

Always `source install/setup.bash` in **every new terminal** before running
ROS commands. (The `setup.bash` exports the ament index — without it,
`ros2 launch` can't find packages.)

## Pre-flight: kill stale Gazebo

If a previous launch crashed or you `Ctrl-C`'d in a weird state, leftover
`gzserver`/`gzclient` processes will block the next launch silently:

```bash
pkill -9 -f gazebo; pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f spawner
ros2 daemon stop && ros2 daemon start
```

(Safe to run even if nothing is stale — `pkill` returns nonzero but does
nothing harmful.)

## Terminals

Open **4 terminals**. In each one, first source the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash
```

### Terminal 1 — Simulation + clean perception

```bash
ros2 launch tof_perception sim_pose_estimation.launch.py
```

This launches:
- `gzserver` + `gzclient` with the tabletop world. `GAZEBO_MODEL_PATH` and
  `GAZEBO_PLUGIN_PATH` are injected by the launch (see
  `xarm1s_description/launch/spawn_robot.launch.py`); without those the
  banana / golf_ball includes silently fail and gazebo_ros2_control loads
  the wrong build.
- `robot_state_publisher` with the URDF.
- `static_transform_publisher` for `world → xarm_base_link` at z = 0.76 m.
- `spawn_entity.py` to insert the xArm into Gazebo (90 s timeout).
- `controller_manager` spawners for `joint_state_broadcaster`, then
  `arm_controller` and `gripper_controller`. These are chained off
  `OnProcessExit` so they only fire after the previous step succeeds — no
  more time-based race.
- `pose_estimator` node (default depth topic = clean Gazebo stream).

You should see Gazebo load with the table, the four objects, and the xArm.
A `Successfully spawned entity 'xarm1s'` line confirms the spawn worked,
and `[joint_state_broadcaster] Configured and activated` confirms the
controllers are alive.

If Gazebo loads but objects are missing → `GAZEBO_MODEL_PATH` problem;
rebuild and re-source. If spawn times out → kill stale gzserver and retry.

### Terminal 2 — Noise injector (only for the *Sim-noisy* condition)

```bash
ros2 launch noise_injection noise_injector.launch.py
```

Loads measured OAK-D parameters from
`noise_injection/config/oakd_measured.yaml` (Phase 2 calibration result).

To compare against the original Nguyen 2012 Kinect defaults:

```bash
ros2 launch noise_injection noise_injector.launch.py \
    params_file:=$(ros2 pkg prefix noise_injection)/share/noise_injection/config/nguyen_kinect.yaml
```

Per-parameter ablation overrides on top of either YAML:

```bash
ros2 launch noise_injection noise_injector.launch.py --ros-args -p axial_scale:=2.0
```

Subscribes: `/overhead_depth/depth/image_raw`
Publishes:  `/overhead_depth/depth/image_noisy`

### Terminal 3 — Point perception at the noisy depth (only for *Sim-noisy*)

```bash
ros2 run tof_perception pose_estimator --ros-args \
    -p use_sim_time:=true \
    -p depth_topic:=/overhead_depth/depth/image_noisy
```

This is a **second** pose_estimator running alongside the one launched by
Terminal 1. Both publish to `/detected_objects` — for cleanest evaluation,
either kill the Terminal 1 pose_estimator first (it's started by the
launch but can be killed individually) or accept that the noisy run
overwrites the clean output.

If you only want *Sim-noisy*, the cleanest workflow is to set
`depth_topic` directly via the launch file's parameter, but for now the
override-at-runtime pattern above works.

### Terminal 4 — Evaluation (writes CSV)

```bash
ros2 run evaluation pose_evaluator --ros-args \
    -p use_sim_time:=true \
    -p run_tag:=sim_clean
```

`run_tag` becomes the first column of `run.csv` so multiple runs
(`sim_clean`, `sim_noisy_oakd`, `sim_noisy_nguyen`, `real`) live in one
file and pivot cleanly in pandas/Excel.

On `Ctrl-C` it prints per-object MAE / RMSE / detection rate and appends
to `~/thesis_ws/src/evaluation/results/run.csv`.

## Phase-3 experiment recipe

Run each condition for ~60 seconds with the four objects static.

```bash
# 1. Sim-clean baseline (only Terminals 1 + 4)
#    Terminal 4: -p run_tag:=sim_clean

# 2. Sim-noisy with measured OAK-D params (all 4 terminals; Terminal 2 = oakd_measured)
#    Terminal 4: -p run_tag:=sim_noisy_oakd

# 3. Sim-noisy with Nguyen Kinect params (Terminal 2 with params_file:=nguyen_kinect.yaml)
#    Terminal 4: -p run_tag:=sim_noisy_nguyen
```

The three tagged runs in `run.csv` are the Phase-3 deliverable — quantifies
how much the *Sim-clean* → *Sim-noisy* shift differs between a
literature-default noise model and one fitted to our actual hardware.

## Shutdown

`Ctrl-C` in reverse order: 4 → 3 → 2 → 1. If Gazebo windows linger after
Terminal 1 quits, run the pre-flight kill block above.
