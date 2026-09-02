# Phase 4C — Beating the 4 cm working zone (multi-agent investigation, 2026-06-13)

Output of a 19-agent research+redesign workflow (research → digest → 5 redesigns → 3 critics → PM
synthesis). The PM verified every load-bearing claim against the live code/data.

## Root cause (verified, not guessed)
The 4 cm zone is an **AIMING** failure from an **unidentifiable, position-dependent kinematic-model error** —
NOT a hardware-repeatability floor, NOT a clean per-servo gain error.
- Real arm over-reaches the URDF/`arm_ik` model by ~27% (2D-similarity scale ≈ 0.789). The current
  `arm = scale·R@table + t` hack leaves **14.1 mm RMS / 24.9 mm max on all 6 touch points** — the published
  7.94 mm only survives after dropping a point to n=5. 14.1 mm > the box's ±7.5 mm tolerance ⇒ only a narrow band works.
- The error is **irreducibly position-dependent** (FK puts 6 points that are physically on ONE flat table
  across a 72.8 mm z-spread; 11.5% distance-ratio scatter). A single scalar can't capture it, and **6 points
  cannot identify a 13-parameter kinematic model** (overfits, no holdout). The "fit per-servo gain first"
  idea was **falsified**: gain-only fits leave 27–37 mm, *worse* than the hack.
- The "approach-tilt" limit is **edge-only**: vertical (~88°) holds from r=0.13–0.19 m; only past r≈0.20 m does
  it tilt (85→75→65° at r=0.22–0.24). So most of the reach CAN be grasped vertically; aiming is the problem.

⇒ Any open-loop calibration is capped at ~8–14 mm and cannot clear ±7.5 mm workspace-wide. **Close the loop.**

## The fix
**PRIMARY — closed-loop visual nudge.** Tape one ArUco marker (same DICT_4X4_50 as the object markers) flat on
the gripper so the overhead OAK-D sees it. Before descending, iterate:
`err = obj_table_xy − (gripper_marker_xy + fixed_tip_offset)`; nudge the IK target by `K·err` (K≈0.6) through the
existing `table_to_arm → ik.feasible_approach → arm.move_joints` path; repeat 2–4× until `|err| < 3 mm`; then
descend + close. Because `err` is a **difference of two points seen through the same camera in the same frame,
the wrong FK and the 0.789 scale cancel as common-mode bias** — they only need to be good enough to converge.
Grasp accuracy becomes **camera-bound (~1–4 mm), workspace-wide, independent of the broken model.**

**COARSE / citable replacement for the hack — dense correction field.** Replace the 4-point similarity with a
camera-measured 5×5–7×7 grid (25–49 pts) + a thin-plate-spline residual field
(`scipy.interpolate.RBFInterpolator`), leave-one-out validated, wired into the single `table_to_arm` hook.
Cite **REPLAB (ICRA 2019, arXiv:1905.07447)** — a ~$2k hobby arm + fixed overhead camera doing exactly this.
This is the principled, reviewer-credible version of the 0.777 hack.

**KEEP** the open-loop FK path as the coarse pre-position; the field makes it good, the loop makes it precise.

## Expected outcome
- **~3–4 mm at grasp, workspace-wide** (camera-bound). Inside box ±7.5, golf ~6, banana ~10 mm tolerances.
- Working zone grows from **~4 cm → most of the ~20 cm reach**; far-edge tilt past r≈0.20 m reported as honest
  separable physics.
- Clean **ablation = the Results 5.5 figure + reviewer rebuttal**: similarity-hack (4 cm) → dense field
  (open-loop, most of reach) → + closed-loop (fine, workspace-wide).

## Steps (time-box ~3–5 days; manipulation is a BASELINE, not the contribution)
0. **Mechanical wins + repeatability sanity (1–2 hr):** rigidly clamp the arm base + OAK-D; add a *unidirectional*
   final approach (overshoot+settle) to collapse ~3 mm backlash → ~1 mm; command 5 configs, return 10× each,
   measure fingertip XY by camera — expect 1–3 mm spread (⇒ accuracy not repeatability).
1. **Gripper marker (0.5 hr):** print DICT_4X4_50 (~25–30 mm), tape flat upward-facing; measure marker→fingertip
   offset once with a ruler at near-vertical approach.
2. **`detect_gripper_xy()` helper (~½ day):** reuse the existing `table_extrinsics.json` + the proven
   `detect_markers → marker_pose(solvePnP) → T_table_marker` path in `ingest_real_scene.py` (lines 117–271);
   use ToF depth for marker Z. Returns gripper table-XY + tip offset.
3. **`visual_servo_align()` loop (~1 day):** in `real_grasp_executor.run_one`, between `move_joints(wps['pre'])`
   and `wps['grasp']` (≈ line 184), at a HOVER height: loop ≤4×, nudge by K·err, 3 mm deadband, iteration cap,
   fall back to open-loop landing + LOG on non-convergence. **Dry-run first** with a synthetic gripper-marker
   injecting a known 28% bias; confirm <3 mm in 2–4 steps before hardware. (Pattern: hello-robot/stretch_visual_servoing.)
4. **Dense correction field (~1 day, parallel):** drive the arm over a 5×5–7×7 table lattice (incl. failing
   corners) + ~10 held-out poses; record commanded vs camera-measured XY; fit Kabsch (rigid baseline) then a TPS
   residual field; leave-one-out + held-out RMS/max; write to a NEW `config/arm_base_corrfield.json` (reversible,
   delta=0 fallback). Clamp/flag targets outside the sampled convex hull (TPS extrapolates badly).
5. **Before/after sweep (~½ day):** re-run the box-grasp sweep at a placement grid in 3 conditions (hack / +field /
   +closed-loop) → residual-vs-position heatmap + success-zone map. **This is the figure.**
6. **Coke/golf re-try + honest framing (~½ day):** closed-loop ON + compliant pads + per-object close. Document
   coke (52 mm in 55 mm jaw) as a mechanical **size wall**, not an aiming bug — don't over-claim 4/4.

## Keep unchanged (important)
The whole perception stack (ChArUco extrinsic 0.33 px, OAK-D ToF + ArUco path, `perceive_scene_4c`). **Do NOT
touch the `arm_ik` constants (A1/A2/A3/FINGERTIP_DZ)** — 6 points can't identify a better model AND those globals
are shared with `grasp_common.plan_pick_place` and the DONE/committed sim Phases 2–4A (changing them retro-pollutes
finished results). Keep the single `table_to_arm` hook, dry-run discipline, fingertip pads, per-object close.

## Fallback
If the gripper marker is occluded at some poses → the Step-4 dense field is a standalone open-loop fix that already
beats the 4 cm zone (~1–3 mm RMS, LOO-validated, REPLAB-cited). If the loop oscillates → lower K (0.6→0.4), keep the
cap + deadband, rely on the unidirectional approach. **Do NOT fall back to full kinematic ID** — the data can't
support it and it risks the committed sim phases.

## Honest thesis framing (this is a STRENGTH)
"A ~€200 hobby-grade 5-DOF arm has no reliable kinematic model, and the available calibration data (6 points, one plane)
cannot identify one — shown directly (global-scale residual 14.1 mm, 11.5% position-dependent scatter, > the 7.5 mm
grasp tolerance). Rather than chase an unidentifiable open-loop model, I close the loop visually: the same overhead
ToF camera that perceives the object also verifies the gripper is on target before the jaws close, making grasp
accuracy independent of the kinematic error." Ties straight to the sim-to-real theme; same idea as the reference
chess-arm (verify against known points, not a perfect model). Two honest limits stated plainly: far-edge approach
tilt (real 5-DOF physics past r≈0.20 m) and coke as a size wall.

## Decision points
1. **Scope:** closed-loop nudge only (~2–3 days, MVP) or + dense field (~4–5 days, strongest story + best fallback)?
2. **Marker visibility (biggest risk):** can a gripper marker stay camera-visible through the full reach? If not,
   the dense field becomes primary.
3. **Claim:** report "near-vertical grasp across most of the reach + documented tilted far-edge" (credible) vs "100%".
4. **Objects:** 3/4 (coke = size wall) vs force a hardware grip aid for coke.
5. **Bench time:** next session long enough for the Step-5 verification sweep?

## Key references
- Published DH model for this exact arm (cross-check `arm_ik`): ResearchGate 369694399 (5-DOF LewanSoul/Hiwonder, 2023).
- REPLAB (grid correction on a cheap arm + overhead camera): arXiv:1905.07447.
- Kinematic-cal lib (if ever needed): github.com/PhilNad/robot-arm-kinematic-calibration.
- LeArm FK/IK cross-checks: github.com/eric-rosen/LeArmKinematics, github.com/thomazach/LeArm, github.com/andrewda/learm_ros2.
- Homography direct pixel→robot map (alt coarse stage): ResearchGate 374284646.
- Visual-servo loop pattern: github.com/hello-robot/stretch_visual_servoing.
