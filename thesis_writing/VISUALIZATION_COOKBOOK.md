# Visualization Cookbook — pictures for the presentation

Every command that produces a screenshot-able window or an image file, verified against the code.
Environment split: **camera = Windows `py -3.14`**, **simulation + analysis = WSL `python3`**.
PowerShell setup (once per terminal): `$SRC = "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src"`

---

## ⭐ THE ONE YOU ASKED FOR — live table view: red object blobs + grasp-X

Two live windows from the overhead OAK-D: **"1) FILTER"** = the scene with the segmented object in **RED**;
**"2) CENTERS"** = a **green X** at each object centre + a **magenta line** = the jaw-close (grasp) axis +
the table-frame XY in mm + a cyan reachable-zone box. **Press `S`** to save a depth frame, **`Q`** to quit.

```powershell
# Windows PowerShell — needs the OAK-D powered + connected (PoE @ 169.254.1.222)
$SRC = "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src"
py -3.14 "$SRC\tof_perception\scripts\live_perception_view.py" --median 15
```
Flags: `--median 15` (running-median frames; raise to ~25 for a steadier picture), `--scale 1.4`
(window zoom), `--no-roi` (hide the cyan zone box), `--camera-ip 169.254.1.222`.
**Put an object on the table, let it settle, then screenshot both windows.**

---

## 1 · Live camera (Windows `py -3.14`, needs the OAK-D)

| What you get | Command |
|---|---|
| **Red blobs + grasp-X** (above) | `py -3.14 "$SRC\tof_perception\scripts\live_perception_view.py" --median 15` |
| **Raw depth stream** (JET colormap, near=red→far=blue, crosshair) | `py -3.14 "$SRC\tof_perception\scripts\tof_depth.py"` |
| **Depth + amplitude** during a capture (saves `<name>_depth.npy/_amp.png`) | `py -3.14 "$SRC\tof_perception\scripts\capture_scene_4b.py" --name demo --table-distance 0.68` |
| **Sharp RGB** (what ArUco runs on; saves `<name>_rgb.png`) | `py -3.14 "$SRC\tof_perception\scripts\capture_rgb_4b.py" --name demo` |

All open live cv2 windows you can screenshot; press `Q` to stop.

---

## 2 · Clean vs noisy simulated depth (WSL) — "one with noise, one without"

A synthetic clean tabletop + the **measured OAK-D noise model** (the same 4-term pipeline the experiments use),
shown as **clean / +noise ×1 / +noise ×10**. Already generated; re-run to tweak:

```bash
# WSL terminal
cd ~/thesis_ws/src/thesis_writing && python3 make_noise_demo.py
# -> evaluation/results/clean_vs_noisy_depth.png
```
(From PowerShell instead: `wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/thesis_ws/src/thesis_writing && python3 make_noise_demo.py"`.)
Edit the object heights / `axial_scale` in the script to change the scene. This image is on **deck slide 5**.

---

## 3 · Simulation views (WSL + ROS 2) — Gazebo scene, sim camera, sim grasp

⚠ These need the **ROS 2 workspace built** (`source ~/thesis_ws/install/setup.bash`) and a **display**
(WSLg / X server) for the GUI windows. After launching, confirm exact topic names with `ros2 topic list`.
Run each block in its **own WSL terminal**.

```bash
# (a) Gazebo tabletop scene: xArm 1S + 4 objects + overhead camera  -> screenshot the Gazebo window
source ~/thesis_ws/install/setup.bash
ros2 launch tof_perception sim_pose_estimation.launch.py

# (b) what the overhead camera sees (clean sim RGB / depth)
source ~/thesis_ws/install/setup.bash
python3 ~/thesis_ws/src/tof_perception/scripts/view_debug.py /overhead_rgb/image_raw
python3 ~/thesis_ws/src/tof_perception/scripts/view_debug.py /overhead_depth/depth/image_raw

# (c) the perception debug overlay (segmentation masks + centroids + grasp axis), if published
python3 ~/thesis_ws/src/tof_perception/scripts/view_debug.py /pose_estimator/debug_view

# (d) sim depth WITH injected noise — run alongside (a), then view the noisy topic
source ~/thesis_ws/install/setup.bash
ros2 launch noise_injection noise_injector.launch.py            # publishes /overhead_depth/depth/image_noisy
python3 ~/thesis_ws/src/tof_perception/scripts/view_debug.py /overhead_depth/depth/image_noisy

# (e) a grasp in simulation (arm reaches, closes, lifts) — screenshot/record the Gazebo window
source ~/thesis_ws/install/setup.bash
ros2 launch xarm1s_manipulation manipulation.launch.py target_source:=perception n_trials:=1 execute_motion:=true
```
`view_debug.py` falls back to writing `/tmp/debug_view.png` if there's no display (pass `--png`).
Gazebo's own **File → Screenshot** also works once the GUI is up.

---

## 4 · Regenerate any result figure (WSL `python3`, offline — no hardware)

```bash
cd ~/thesis_ws/src

# Phase 2 — axial sigma/bias vs distance  -> fit_results_truly_raw/{sigma_vs_z,bias_vs_z}.png
python3 tof_perception/scripts/fit_noise_model.py --in-dir ./tof_perception/scripts/captures_truly_raw --out-dir ./tof_perception/scripts/fit_results_truly_raw
# Phase 2 — per-object edge flying-pixel overlays  -> object_results/*_overlay.png
python3 tof_perception/scripts/analyze_object_noise.py --in-dir ./tof_perception/scripts/object_captures --out-dir ./tof_perception/scripts/object_results
# Phase 4B — ChArUco extrinsic overlay  -> board_4b/extrinsics_overlay.png
python3 tof_perception/scripts/solve_table_extrinsics.py --name board

# Phase 3 — pose RMSE vs noise            -> evaluation/results/phase3_pose_vs_noise.png
python3 evaluation/scripts/summarize_phase3.py
# Phase 4A — grasp rate vs noise          -> evaluation/results/phase4a_rate_vs_axial_scale.png
python3 xarm1s_manipulation/scripts/summarize_4a_rates.py evaluation/results/manip_results.csv
# Phase 4A — perceived error vs jaw rings -> evaluation/results/phase4a_perception_error.png
python3 xarm1s_manipulation/scripts/plot_perception_error.py sim_clean
# Phase 4B — sim-to-real pose gap         -> evaluation/results/phase4b_sim_to_real_gap.png
python3 evaluation/scripts/compute_sim_to_real_gap.py
# Phase 4C — real vs predicted            -> evaluation/results/phase4c_real_vs_pred.png
python3 evaluation/scripts/summarize_phase4c.py --csv evaluation/results/real_manip_4c_formal.csv --out-dir evaluation/results
```

---

## Quick picks for tomorrow
- **Live perception (the wow shot):** §0 `live_perception_view.py` → screenshot both windows.
- **Noise model:** §2 `clean_vs_noisy_depth.png` (already on slide 5).
- **Sim scene + sim camera:** §3 (a)+(b) if you have time to bring the sim up.
- **Any result plot:** §4 — all already embedded in the deck, regenerate only if you change data.
