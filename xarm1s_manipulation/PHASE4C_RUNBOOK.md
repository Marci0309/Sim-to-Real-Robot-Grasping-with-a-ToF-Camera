# Phase 4C — Real Perceived Grasp: how it works + how to run it

End-to-end: an **overhead OAK-D** sees an object, the **depth** is segmented into the
object's table-frame position + orientation, that pose is transformed into the **arm
base frame**, analytical **IK** plans the waypoints, and the **xArm 1S** reaches in and
grasps it. No hand-placing.

---

## 1. The pipeline (how an object gets detected and picked up)

```
 overhead OAK-D ToF  ──depth image──►  SEGMENTATION (perceive_scene_4c)
                                          │  table plane = median depth
                                          │  "object" = ≥8 mm above the TRUE ChArUco
                                          │             plane AND inside the ROI
                                          │  connected components → 1 blob / object
                                          │  median back-projected centroid → TABLE xy
                                          │  PCA on the blob → minor axis (jaw direction)
                                          ▼
                                   object pose in the TABLE frame  (x, y, yaw)
                                          │  × arm_base_extrinsics:  arm = scale·R·table + t
                                          ▼
                                   object pose in the ARM BASE frame
                                          │  arm_ik: steepest feasible top-down approach
                                          │  wrist-roll J2 aligned to the (rotated) minor axis
                                          ▼
                                   joint waypoints: HOME → hover → grasp → lift → place
                                          │  servo_calib: radians → servo units
                                          ▼
                                   xArm 1S executes (USB-HID)
```

### Three coordinate frames (the key idea)
- **Camera/ToF frame** — raw depth pixels.
- **Table frame** — defined by the ChArUco board (`board_4b/table_extrinsics.json`).
  Perception outputs object poses here.
- **Arm base frame** — where `arm_ik` works. Linked to the table frame by
  `config/arm_base_extrinsics.json` (the calibration solved by touching markers).

> **Note (important):** the real arm reaches ~28% farther than the URDF model, so the
> table→arm map is a **similarity** (`scale ≈ 0.78`), not a rigid transform. Because the
> same FK/IK runs end-to-end, applying the scale before IK still lands the real fingertip
> on the true target. The `scale` lives in `arm_base_extrinsics.json`.

---

## 2. Environments (which Python, where)

| task | host / interpreter | why |
|---|---|---|
| **arm** (grasp, home, calibrate) | **Windows `py -3.11`** | `xarm` USB-HID lib |
| **camera** (capture, live view) | **Windows `py -3.14`** | `depthai` |
| **perception / analysis** | **WSL `python3`** | OpenCV + numpy |

Scripts live in the WSL tree; Windows runs them over the UNC path. Shortcuts used below:
```
WSL  src = /home/szokedencsim/thesis_ws/src
UNC  src = \\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src
```

---

## 3. One-time calibration (already DONE — redo only if hardware moves)

1. **Servo calibration** → `config/servo_calib.json` (joint angle ↔ servo unit).
   `py -3.11 ...\xarm1s_manipulation\scripts\calibrate_servos.py`  *(don't rerun unless the arm changes)*
2. **Camera → table** → `board_4b/table_extrinsics.json`. Valid as long as the camera
   doesn't move relative to the table (ours is bolted to the table, so it's fixed).
   To redo: put the ChArUco board flat in view, then
   - `py -3.14 ...\tof_perception\scripts\capture_rgb_4b.py   --name board --out-dir ...\tof_perception\captures_4b`
   - `py -3.14 ...\tof_perception\scripts\capture_scene_4b.py --name board --table-distance 0.68 --out-dir ...\tof_perception\captures_4b`
   - `(WSL) python3 solve_table_extrinsics.py --name board`  → check reproj < 1 px.
3. **Table → arm (scale-aware)** → `config/arm_base_extrinsics.json`. This is the one we
   solved by touching the 4 object markers. To redo:
   - `py -3.14 capture_rgb_4b.py --name armcal --out-dir ...\captures_4b`   (glue the markers down!)
   - `(WSL)` detect the markers' table-frame centres (see session notes), then
   - `py -3.11 ...\scripts\solve_arm_base_extrinsics.py`  (touch each marker centre, type its table xy; logs units)
   - `(WSL) python3 ...\scripts\fit_arm_base_extrinsics.py`  → similarity fit + outlier rejection; check residual ≲ 8 mm.

---

## 4. Run ONE perceived grasp (standard flow)

Place a **single object** in the reachable zone (**~12–30 cm from the base**, table
x ≈ 0.27–0.43; objects farther out are unreachable). Then:

```powershell
# 0) home the arm (clear of the camera view)            [py -3.11]
py -3.11 "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src\xarm1s_manipulation\scripts\phase4c_manual_grasp.py" --home

# 1) capture a depth frame                              [py -3.14]
py -3.14 "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src\tof_perception\scripts\capture_scene_4b.py" --name g1 --table-distance 0.68 --out-dir "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src\tof_perception\captures_4c"
```
```bash
# 2) perceive the object (table xy + PCA jaw axis)      [WSL]
cd /home/szokedencsim/thesis_ws/src/tof_perception/scripts
python3 perceive_scene_4c.py --name g1 --all                       # see all blobs (optional)
python3 perceive_scene_4c.py --name g1 --object banana             # pick it (add --hint-x/-y if several)
```
```powershell
# 3) grasp it (perceive → transform → IK → arm)         [py -3.11]
#    --pick-only = lift & hold (skip place); --close-mm = firm pinch (well past object width)
py -3.11 "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src\xarm1s_manipulation\scripts\real_grasp_executor.py" --object banana --scene g1 --pick-only --close-mm 15
```
The executor prints the table→arm transform, prompts **`lift-and-hold success? [y/N]`**
(run it in YOUR terminal so the y/N is recorded), and logs to
`evaluation/results/real_manip_results.csv`.

```powershell
# 4) set the object back down after a --pick-only hold (it stays clamped otherwise)
#    use the arm_x / arm_y the executor printed:
py -3.11 "...\phase4c_manual_grasp.py" --x <arm_x> --y <arm_y> --z 0.82   # lower
py -3.11 "...\phase4c_manual_grasp.py" --gripper-mm 55                    # open / release
py -3.11 "...\phase4c_manual_grasp.py" --home                            # home empty
```

> Drop `--pick-only` for the **full pick-and-place** (it releases at the object's drop spot
> automatically — that's what the 60-trial protocol uses).

---

## 5. Watch the camera DURING the pickup (live window)

The grasp uses only the arm, not the camera — so the live perception viewer can stay open
and show the pickup. Use **two terminals**:

```powershell
# Terminal A  [py -3.14]  — live camera window (object mask + perceived X + jaw axis + ROI)
py -3.14 "\\wsl.localhost\Ubuntu-22.04\home\szokedencsim\thesis_ws\src\tof_perception\scripts\live_perception_view.py" --name g1
#   → place the object, then press  S  in the window to SAVE this depth as scene 'g1'
#   → keep this window open; it shows the camera the whole time, including the grasp
```
```bash
# Terminal B  [WSL]  — perceive the saved frame
cd /home/szokedencsim/thesis_ws/src/tof_perception/scripts
python3 perceive_scene_4c.py --name g1 --object banana
```
```powershell
# Terminal B  [py -3.11]  — grasp (watch it happen in Terminal A's camera window)
py -3.11 "...\real_grasp_executor.py" --object banana --scene g1 --pick-only --close-mm 15
```
Keys in the viewer: **S** = save current depth as `<name>`, **Q** = quit. (Only one process
can own the camera, so `capture_scene_4b.py` and the viewer can't run at the same time —
the viewer's `S` replaces the separate capture.)

---

## 6. Tuning knobs

| knob | where | meaning |
|---|---|---|
| `--close-mm` | `real_grasp_executor` | absolute jaw width at grasp; command **well past** the object width for a firm pinch (15 ≈ firm; lower = tighter) |
| `--grasp-z` | `real_grasp_executor` | fingertip height (default **0.82**; raise ~0.83–0.84 for a more central grip on short objects) |
| `--object` / `--hint-x/-y` | `perceive_scene_4c` | which object + which blob to pick |
| `ROI_TABLE_XY`, z-band, base disc | `ingest_real_scene` | what counts as "object": reachable patch, height band [8 mm, 0.12 m], arm-base exclusion |
| `--no-roi` | `perceive_scene_4c`, viewer | segment the whole frame (debug) |

---

## 7. Gotchas (things we hit)

- **Object out of reach** → executor says `IK infeasible`. Move it into the 12–30 cm band.
- **Arm shows up in perception** → home it first; the ROI's z-ceiling drops the tall homed arm.
- **`X_LINK_DEVICE_NOT_FOUND`** → the camera is held by another process (e.g. the live viewer
  is open during a `capture_scene_4b`). Close the other one.
- **`--pick-only` keeps holding the object** → that's intended (lift-and-hold); set it down with §4 step 4.
- **Grip too loose** → lower `--close-mm`. **Jaws misaligned** → the PCA yaw needs the object
  elongated enough (ratio ≳ 1.2); near-spherical objects (golf) have no well-defined axis.

---

## Key files
- `tof_perception/scripts/perceive_scene_4c.py` — depth → table-frame pose + PCA yaw
- `tof_perception/scripts/ingest_real_scene.py` — segmentation + ROI (`perceived_blobs_table`, `roi_filter_above`)
- `tof_perception/scripts/live_perception_view.py` — live camera/segmentation viewer (S=save, Q=quit)
- `xarm1s_manipulation/scripts/real_grasp_executor.py` — perceive → transform → IK → grasp
- `xarm1s_manipulation/scripts/fit_arm_base_extrinsics.py` — table→arm similarity fit
- `xarm1s_manipulation/xarm1s_manipulation/{arm_ik,grasp_common,servo_calib,real_arm_driver}.py`
- `config/{servo_calib,arm_base_extrinsics}.json`, `board_4b/table_extrinsics.json`
