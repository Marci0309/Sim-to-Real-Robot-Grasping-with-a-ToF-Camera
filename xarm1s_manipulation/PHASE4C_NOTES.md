# Phase 4C — Real Manipulation (notes)

Closes the thesis loop: run the pick-and-place on the **real Hiwonder xArm 1S** and
measure **physical** grasp success — the one thing the sim's analytic metric and the
perception-only Phase 4B could not test (SQ3, real). Companion to `PHASE4A_NOTES.md`.

## Status (2026-06-07)
**Software stack COMPLETE and dry-run validated; physical run pending hardware setup.**
Every piece below runs end-to-end with the arm unplugged (`--dry-run`); the only
remaining work is hardware-gated (calibrate, then the 60-trial run).

## Control decision
The xArm 1S is a **USB-HID** device (VID `0x0483` / PID `0x5750`), driven by
`pip install xarm` (servo IDs 1–6, position units 0–1000 ≈ ±125°, move-with-duration).
We drive it with a **lightweight, ROS-free Python bridge on Windows** (co-located with
DepthAI), NOT a `ros2_control` hardware interface:
- the proposal explicitly permits a "predefined grasp strategy";
- the analytical IK (`arm_ik.py`) is already pure and returns the 5 joint angles, reused verbatim;
- USB-HID + DepthAI are both simplest on Windows (WSL HID passthrough via usbipd is brittle);
- a full `ros2_control` `SystemInterface` for a hobby servo board is weeks of middleware for zero scientific payoff.
WSL stays the analysis side (perception, calibration solves, CSV/figures), exactly as Phase 4B.

## Software stack (all committed; `--dry-run` validated)
| File | Role |
|---|---|
| `xarm1s_manipulation/grasp_common.py` | **single source of truth** for the per-object grasp data, gripper jaw model, and pick-and-place waypoint geometry (shared by the sim node and the real executor; ROS-free). `grasp_executor.py` now imports it. |
| `xarm1s_manipulation/servo_calib.py` | radian↔servo-unit model (per-servo zero/slope/dir/limits + gripper width map); loads `config/servo_calib.json`. |
| `xarm1s_manipulation/real_arm_driver.py` | `RealArm` — the ONLY module importing `xarm`; `move_joints`/`set_gripper`/`read_joints`/`relax`; `--dry-run` prints unit commands without the device. |
| `scripts/calibrate_servos.py` | two-point per-servo calibration → `config/servo_calib.json` (hardware). |
| `scripts/solve_arm_base_extrinsics.py` | touch ≥4 known table points + 2D Kabsch → `config/arm_base_extrinsics.json` (+ residual) (hardware). |
| `tof_perception/scripts/perceive_scene_4c.py` | reuses Phase-4B `perceived_blobs_table` → perceived object pose in the **table frame** (no marker on the object) → `captures_4c/<scene>_perceived.json`. |
| `scripts/real_grasp_executor.py` | the orchestrator: perceived pose → table→arm transform → IK waypoints (`grasp_common.plan_pick_place`) → `RealArm` HOME→pre→grasp→close→lift→place→open→retreat → operator lift-score → `real_manip_results.csv`. |
| `evaluation/scripts/summarize_phase4c.py` | Results 5.5: per-object real lift rate + **Wilson 95% CI**, joined with the sim-feasible (4A) and real pose-error (4B) predictors → `phase4c_real_table.{csv,txt}` + `phase4c_real_vs_pred.png`. |

Dry-run evidence: `real_arm_driver.py --dry-run` and `real_grasp_executor.py --dry-run`
both produce a full perceive→IK→servo-command trace (units in range); the summarizer
builds the table+figure from a synthetic CSV; Kabsch recovers a known transform to 0 mm.

## Calibration (hardware)
1. **Per-servo** (`calibrate_servos.py`): relax the arm; for each joint read the unit at the
   URDF-zero pose (HOME, straight up) and at a protractor-verified +90° → `(zero_unit, units_per_rad, dir)`;
   derive safe unit limits from `arm_ik.LIMITS`; fit the gripper open(55 mm)/closed reference.
   **VERIFY** with a ruler (commanded vs measured fingertip XY, few-mm) BEFORE any multi-joint move.
2. **Arm-base ↔ table** (`solve_arm_base_extrinsics.py`): the perceived pose is in the table
   (ChArUco) frame; the IK is in the arm base frame (they coincide only in sim). Touch the
   fingertip to ≥4 points of known table-frame XY (ChArUco corners at k·0.1 m), read joints → FK
   → arm-frame XY, 2D Kabsch → `arm_base_extrinsics.json` + RMS residual. Fallback: clamp the
   board at a measured fixed offset.

## Metric & honest comparison
Real success = **physical lift-and-hold** (arm closes, lifts clear, holds ≥3 s without dropping),
operator-scored. This differs in kind from the sim's analytic feasibility, so Results 5.5 places
three columns side by side as **model-linked, not identical**:
1. sim analytic grasp-feasible % (Phase 4A) — the necessary-condition *ceiling*;
2. real planar pose error mm (Phase 4B) vs the jaw capture tolerance — the *predictor*;
3. real physical lift % (this phase) — the *achieved* result.
Predicted story (from current 4A/4B numbers): **coke ~0%** (9.4 mm err ≫ 1.5 mm tol — the size
wall), **golf likely** (4.7 < 6.0), **banana/blue_box marginal** (12.1 vs 10.0; 7.4 vs 7.5). Do
**not** claim sim predicts the real *rate* numerically — extra real losses (extrinsic residual,
backlash, slip, roll) explain the gap; the 4B pose error is the per-object bridge.

## Experiment protocol
4 objects × **5 reachable placements** (r ∈ 0.12–0.30 m, validated by `test/check_reachable_placements.py`)
× **3 repeats** = **60 trials**, one object per scene (no marker on the object during the grasp).
Open-loop (no live feedback). Per placement: `capture_scene_4b.py` + `capture_rgb_4b.py` →
`perceive_scene_4c.py --object X` → `real_grasp_executor.py --object X --scene S` → score.

## Hardware-run checklist (turnkey)
0. **Smoke test (Windows):** `pip install xarm hidapi`; confirm VID 0483/PID 5750; `xarm.Controller('USB')` + `getPosition(1..6)`.
1. `python real_arm_driver.py --dry-run` (sanity), then `calibrate_servos.py` → ruler-verify.
2. `solve_arm_base_extrinsics.py` (touch points) → check residual ≲ 5 mm.
3. One cautious powered golf_ball grasp (`real_grasp_executor.py`, watch closely).
4. Run the 60-trial protocol; then `summarize_phase4c.py` → Results 5.5.

## Numbers to fill after the run
- servo_calib zero/slope/dir per joint; gripper open/closed units.
- arm-base extrinsic RMS residual (mm).
- ruler-verification residual (mm).
- per-object real lift k/N + CI; overall rate.
