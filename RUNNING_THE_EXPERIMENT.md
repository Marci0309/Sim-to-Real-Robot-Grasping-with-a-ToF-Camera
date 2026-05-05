# Running the Experiment

Open **4 terminals**. In each one, first run:

```bash
source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash
```

If you changed any code, build first:

```bash
cd ~/thesis_ws && colcon build --symlink-install && source install/setup.bash
```

---

### Terminal 1 — Simulation + clean perception

```bash
ros2 launch tof_perception sim_pose_estimation.launch.py
```

Wait until Gazebo shows the cube + cylinder on the table.

### Terminal 2 — Noise injector

```bash
ros2 launch noise_injection noise_injector.launch.py
```

### Terminal 3 — Point perception at the noisy depth

```bash
ros2 run tof_perception pose_estimator --ros-args \
    -p use_sim_time:=true \
    -p depth_topic:=/image_noisy
```

### Terminal 4 — Evaluation (writes CSV)

```bash
ros2 run evaluation pose_evaluator --ros-args \
    -p use_sim_time:=true \
    -p run_tag:=test_run
```

On `Ctrl-C` it prints MAE / RMSE / detection rate, and appends to
`~/thesis_ws/src/evaluation/results/run.csv`.

---

**Shutdown:** `Ctrl-C` in reverse order (4 → 3 → 2 → 1).
