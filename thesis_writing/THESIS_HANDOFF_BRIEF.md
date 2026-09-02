# Thesis Handoff Brief
## *Investigating the Sim-to-Real Gap in RGB-D Based Tabletop Pose Estimation and Manipulation*

**Author:** Marcell Mor Szokedencsi (s5173191) — BSc Artificial Intelligence, University of Groningen
**Supervisor:** Dr. Hamidreza Kasaei (IRL Lab, Bernoulli Institute, FSE) · Course WBAI901-15 (15 ECTS) · written in Overleaf
**Document purpose:** A complete, self-contained record of the project — research questions, methods, every experiment with exact numbers, the full struggle/solution journey (especially Phase 4C, which exists nowhere else), findings, limitations, the thesis structure, and the reference list (including citations that must **not** be used). Hand this to a fresh assistant to write the thesis without re-deriving anything.

> **How to read this:** Parts 0–2 = the thesis as it stood before today (Phases 1–4B). Part 3 = today's work (Phase 4C real manipulation), in full narrative detail. Parts 4–7 = synthesis, the writing plan, references, and where every file lives.

---

# PART 0 — THE THESIS AT A GLANCE

**Core contribution (the study):** a careful, quantified study of *how realistic Time-of-Flight (ToF) depth-sensor noise affects the sim-to-real gap* in RGB-D tabletop pose estimation and manipulation. The headline is an **honest negative result**.

**Baseline (explicitly NOT the contribution):** the manipulation pick-and-place is the proposal's authorised *"predefined grasp strategy for known object types"* — a downstream test of whether the pose-null propagates to grasping. All Phase-4C real-manipulation engineering (the closed-loop visual servo, calibration fixes, the USB-HID bridge) is **Methods/Appendix material, not a results contribution.** The thesis must not present the manipulation system as the novel deliverable.

### Research questions (verbatim from `PROJECT_STATUS_REPORT.md` / `SUPERVISOR_MEETING.md`)
- **Main RQ:** *"How does modelling realistic Time-of-Flight (ToF) sensor noise affect the sim-to-real gap in RGB-D tabletop pose estimation and manipulation?"*
- **SQ1 (noise statistics):** *"What is the real ToF sensor's noise behaviour, and how well do literature models (Nguyen 2012, Kinect-class) describe it?"* → **Phase 2**
- **SQ2 (pose accuracy):** *"Does injecting measured noise into simulation change pose-estimation accuracy relative to clean simulation?"* → **Phase 3**
- **SQ3 (manipulation propagation):** *"To what extent do pose estimation errors propagate to manipulation performance (pick success rate and placement accuracy)?"* → **Phases 4A (sim) / 4B (real perception) / 4C (real manipulation)**
- **Proposal hypothesis (verbatim):** *"Injecting measured ToF noise into simulation will reduce the sim-to-real gap."* — **NOT confirmed** (this is the negative result; write it as a strength).

### The four headline findings
1. **Negative result:** for a depth-segmented **median-centroid** estimator at OAK-D-measured noise, injecting measured ToF noise has **no measurable effect on pose RMSE (Δ < 0.1 mm vs clean)** and none on grasp success. The per-axis median discards the edge flying-pixel outliers that constitute the real noise.
2. **Pose threshold ≈ ×10:** accuracy only degrades when axial noise is scaled to ≈10× measured (σ ≈ 9 mm) — an order of magnitude above reality.
3. **Manipulation collapse ≈ ×20:** sim grasp success is flat clean→×10 and collapses only at ×20 (the extra factor exists because grasping is gated by object-vs-gripper tolerance, not perception alone).
4. **Real confirms sim:** the measured real-vs-sim planar pose gap is **−1.4 to +3.6 mm across all four objects** (Phase 4B); and the closed-loop real manipulation (Phase 4C, today) gives **blue_box 11/15 = 73 % (100 % in the near workspace)** vs **100 % sim-feasible** — a concrete SQ3 datapoint.

**Why the negative result is valid (not a broken pipeline):** the injector is *verified* — scaling the axial term up **does** eventually break perception (the ×20 collapse). Realistic noise simply sits below the estimator's noise floor.

---

# PART 1 — SYSTEM & METHODS

### Hardware
- **Arm — Hiwonder xArm 1S** (a.k.a. LewanSoul/LeArm), a ~€200 hobby arm. Modelled as a **5-DOF** kinematic chain + parallel-jaw gripper: base-yaw (servo 6) + a **planar 3R** (servos 5/4/3, shared parallel pitch axes) + wrist-roll (servo 2); gripper fixed, tip +0.09 m. URDF from GrabCAD. ⚠ *The status report's file inventory mislabels it "6-DoF"; the IK is 5-DOF — verify against `xarm.urdf` before writing Methods.*
  - Because the three pitch joints form a planar 3R arm, position is solved in **closed-form analytic IK** (`arm_ik.py`, pure NumPy, no MoveIt/KDL except optional FK cross-check) → every grasp error is attributable to perception, not solver convergence. Cite **Craig, *Introduction to Robotics*, 3rd ed., ch. 4**.
  - **Geometry quirk:** the arm is table-mounted, so a purely vertical top-down grasp is infeasible at most reaches — `feasible_approach` searches the *steepest reachable* slanted approach within joint limits (this becomes important in Phase 4C: the approach tilt degrades from ~88° near the base to ~70–80° at far reach).
  - **Real gripper opens to 55 mm** (the idealized URDF opened to 72 mm — a disclosed sim-to-real gap source; tightens coke's capture tolerance to 1.5 mm).
  - **Real control (Phase 4C):** USB-HID (**VID 0x0483 / PID 0x5750**), driven by `pip install xarm` + `hidapi`, servo IDs 1–6 (units 0–1000 ≈ ±125°). Lightweight **ROS-free Python bridge on Windows** (not ros2_control — USB-HID in WSL needs brittle usbipd). Needs a USB-A↔USB-B cable + DC power on. The real-arm FK was found ~28 % off (URDF link lengths ≠ real) — worked around with a **similarity-scale (~0.777) table→arm extrinsic**. *(This 28 % error is the seed of the entire Phase-4C story.)*
- **Camera — Luxonis OAK-D SR PoE (ToF):** fixed **overhead/top-down at 0.68 m** (real mount height carried into sim → eliminates eye-in-hand drift, isolating the depth axis). RGB-D 640×480, ~60° FOV. Multi-frequency phase-unwrapping → unusually quiet bulk depth (sub-mm axial σ, flat across range). DepthAI v3.5. **Factory intrinsics used (no Zhang calibration run).** The PoE link (`169.254.1.222`) is unreachable from WSL's NAT, so it is driven Windows-side.

### Software / middleware
- **Gazebo Classic 11 + ROS 2 Humble.** `ros2_control` (JTC + effort/PID gripper) via a vendored `gazebo_ros2_control`. Arm anchored to `world` by a fixed joint at z = 0.76. Custom MIT `xarm1s_grasp_plugin` (contact-triggered fixed-joint attach, PAL/IFRA pattern) — **for visualisation only; measured data come from the analytic grasp-feasible layer.** ⚠ Gazebo Classic is end-of-life (Jan 2025) — stable but unsupported.
- **Windows/WSL split:** authoritative ROS 2 workspace is in **WSL2 Ubuntu-22.04** at `/home/szokedencsim/thesis_ws`. For Phase 4C: **arm control = Windows `py -3.11`** (xarm/hidapi), **camera/depthai = Windows `py -3.14`**, **perception + analysis = WSL `python3`**.

### Perception pipeline (`pose_estimator.py`) — classical/geometric by design
1. RGB→HSV per-object masks used **only to label** which blob is which (never for geometry).
2. **Depth-based table removal:** pixels ≥ 5 mm closer than the **median** valid depth are "object" (deliberately replaces RANSAC plane fitting — the support plane is fronto-parallel and known).
3. `cv2.connectedComponents` → one blob per object.
4. **Per-axis median centroid** of the back-projected blob → table-frame position. **Translation-only (3-DoF); orientation = identity.** The median is what discards the 53–99 % edge flying-pixel population — the crux of the whole thesis.
5. Detections matched to ground truth by **greedy nearest-neighbour** (a deliberate, documented simplification — NOT Hungarian).
- ⚠ **Code-honesty fix:** the `pose_estimator.py` docstring still falsely claims "RANSAC plane segmentation (Open3D)" — must be corrected so code and thesis agree.

### The four objects (used in every phase)
| Object | Geometry | Width | Height | Jaw capture tol = (55−w)/2 | Character |
|---|---|---|---|---|---|
| **coke_can** | slim primitive cylinder | 52 mm | 47.3 mm | **1.5 mm** | largest/tallest; fills the jaw |
| **banana** | re-centred solid-yellow mesh | 35 mm | 26.1 mm | 10 mm | curved/elongated (12 cm), 120 g |
| **golf_ball** | primitive sphere | 43 mm | 29.6 mm | 6 mm | small, round, smooth, 46 g |
| **blue_box** | primitive box | 40 mm | 28.5 mm | 7.5 mm | sharp-edged — the only robust object |

---

# PART 2 — PHASES 1–4B (results before today)

## Phase 1 — Simulation infrastructure (DONE)
Full ROS 2 Humble / Gazebo Classic 11 stack: xArm URDF, overhead RGB-D camera (640×480, 60° FOV, top-down 0.68 m), tabletop world with the 4 objects, parallel-jaw gripper, perception→evaluator→CSV pipeline. The closed-loop demo was honestly **deferred into Phase 4A**.
**Struggles:** diagnosing why the sim wouldn't boot took ~a day and uncovered **5 separate bugs** all masquerading as "controllers stuck": unset `GAZEBO_MODEL_PATH`; `gzserver` hanging on the online model DB (fixed `GAZEBO_MODEL_DATABASE_URI=''`); a broken local patch to `gazebo_ros2_control`; the arm spawning free-floating and falling off the table (fixed with a `world` fixed joint at z=0.76); a controller-load race (fixed with `OnProcessExit` event chains).

## Phase 2 — Real ToF noise characterization (DONE) — SQ1
**Sensor:** OAK-D SR PoE; 200 frames/capture; **raw filters FPPN/optical/wiggle ON, temporal/median OFF** (on-camera smoothing disabled to expose raw noise).

**(a) Planar protocol** (5 distances 0.40–0.80 m, 200 frames, matte fronto-parallel target): axial precision is **sub-mm and essentially flat — no Kinect-style range growth**:

| Distance | 0.40 | 0.50 | 0.60 | 0.70 | 0.80 m |
|---|---|---|---|---|---|
| **axial σ** | 0.87 | 0.80 | 0.68 | 0.55 | 0.69 mm |

Bias within ±3 mm. **Nguyen's quadratic range model does NOT apply** — fitted `σ_z(z)=a+b·z²` gives **a = 0.91 mm, b = −0.000490 m⁻¹ (negative).** (n = 33 920 px/distance.)

**(b) Object protocol** — body vs a 6-px edge ring (the real finding): body noise stays quiet; the **edge mixed-pixel (flying-pixel) rate is huge on all objects**:

| Object | body σ | dropout | edge flying-pixel rate |
|---|---|---|---|
| coke_can | 0.70 mm | 0 % | **53.1 %** (re-capture 62.5 %, n=2) |
| banana | 0.62 mm | 0 % | **98.7 %** |
| golf_ball | 0.65 mm | 0 % | **98.2 %** |
| blue_box | 0.71 mm | 0 % | **94.9 %** |

**Findings:** the OAK-D's multi-frequency unwrapping gives Kinect-beating bulk depth (0.6–0.7 mm vs 2–10 mm); the pose-relevant noise lives **at edges, not the body** → motivates the median-centroid estimator + an edge noise term.
**Caveats:** the "sharp→low / curved→high" story is too simple (the sharp box is still 94.9 %); coke is low mainly because it is the **largest** object; the rate is computed over a **fixed 6-px ring → partly size-sensitive** (report qualitatively + a ring-width caveat). Coke's gloss adds a small specular dropout (centroid unaffected).

## Phase 3 — Noise injection + pose evaluation (DONE) — SQ2
**Injector** (`depth_noise_injector`) implements **four literature terms** from measured YAML: axial Gaussian σ_z(z) (Nguyen Eq.3) = **0.9 mm flat (b=0)**; lateral jitter (Nguyen Eq.1) = **0.8 px** (literature value, not separately measured); edge mixed-pixels (Foix/Sarbolandi) = **p_edge 0.70**; dropout (Sarbolandi) = **0 % measured**. Conditions: `sim_clean`, `sim_noisy_oakd`, `sim_noisy_nguyen` + an axial_scale ablation (×1,×2,×5,×10,×20). Re-run **2026-06-07 on the current clean models**, ~435 frames/condition, 100 % detection.

**Pose RMSE (mm), planar / 3D — from `phase3_pose_table.csv`:**
| axial_scale | coke | banana | golf | blue_box |
|---|---|---|---|---|
| 0 (clean) | 3.31 / 4.85 | 8.71 / 10.67 | 4.77 / 7.89 | 3.06 / 3.06 |
| ×1 (measured) | 3.45 / 4.99 | 8.60 / 10.64 | 4.66 / 7.94 | 3.44 / 3.44 |
| ×5 | 3.53 / 6.03 | 8.68 / 11.12 | 4.53 / 8.55 | 3.63 / 3.71 |
| ×10 | 3.47 / 7.41 | 9.08 / 12.19 | 4.39 / 10.48 | 3.44 / 4.03 |
| **×20** | **9.55 / 18.43** | **22.43 / 24.53** | **8.58 / 19.08** | **8.68 / 10.21** |

**Findings:** (1) measured (×1) ≈ clean ≈ nguyen — **no effect through ×10**; planar XY stays flat ~3.5 mm; the small 3D creep is the axial/Z component. (2) **Robustness threshold ≈ ×10** (σ ≈ 9 mm), all four collapse together at ×20. The old "noise regularises coke" claim was an **artefact of the off-centre textured mesh** — gone with the clean slim can (coke 80→~5 mm, banana 39→~11 mm 3D RMSE).

## Phase 4A — Simulated pick-and-place (DONE) — SQ3 (sim)
**Metric = analytic `grasp_feasible`:** a grasp succeeds **iff the perceived centre is within `capture_tol` of the true centre** (open jaws would straddle it). **No physical contact scored** — Gazebo Classic cannot reliably contact-grasp small/curved objects (proven: finger collisions pass *through*; object pose bit-identical before/after closing). Renamed `pick_success`→`grasp_feasible` to avoid overclaiming. **Gripper corrected 72→55 mm** (real measured).
**Experiment — varied-placement sweep, 480 analytic trials:** each object alone at **20 seeded positions+orientations** over the reachable annulus (r 0.12–0.30 m, full yaw, IK-validated; coke lying), × **6 noise conditions** = 6×4×20.

**Success rate (%) — from `phase4a_success_table.csv` (N=20/cell):**
| Condition | coke (1.5mm) | banana (10mm) | golf (6mm) | blue_box (7.5mm) | overall |
|---|---|---|---|---|---|
| clean | 0 % | 65 % | 65 % | 100 % | 57.5 % |
| ×1 measured | 5 % | 65 % | 65 % | 100 % | 58.75 % |
| ×10 | 0 % | 65 % | 75 % | 100 % | 60 % |
| **×20** | 5 % | **30 %** | **20 %** | **50 %** | **26.25 %** |

**Findings:** all 4 detected 20/20 (accuracy, not detection, is the bottleneck); the 3 graspable objects flat clean→×10, collapse at ×20 — the manipulation echo of the pose threshold. Per-object governed by **raw clean error vs jaw tolerance**: blue_box 3.5 vs 7.5 → 100 %; golf 5.2 vs 6.0 → 65 % (on the boundary); banana 9.3 vs 10 → 65 % (on the boundary); coke 4.9 vs **1.5** → 0 % (floored — at the gripper size limit, regardless of noise).
**Struggles:** the banana was 0 % at every level — a **model artefact, not noise**: (1) Gazebo ignored the SDF material on the textured mesh → HSV labeller failed → swapped to a solid-yellow mesh; (2) the visual mesh was **~28 mm off-centre from its link origin** (an offset that rotates with yaw, so no fixed bias fixes it) → fixed with a link-frame `<pose>` = −C_mean → error 28.7→10.6 mm, rate 0→65 %. Plus the documented Gazebo grasp-realism work (contact-triggered weld, effort fingers, fingertip pads, physics tuning) and an intermittent **GUI-only banana grab-fling** (an ODE solver-timing effect at joint formation; not reproducible headless; cosmetic — data is analytic).

## Phase 4B — Real-world perception / sim-to-real pose gap (DONE) — SQ3 (real perception)
**Setup (offline route):** fiducials detected on the **OAK-D RGB camera (CAM_B)**, not the ToF amplitude (the soft amplitude gave 0/24 corners) → composed into the depth frame via the **factory CAM_A↔CAM_B extrinsic**. Camera→table via a printed **7×5 ChArUco board (DICT_5X5_100, 100 mm squares)**: direct `solvePnP` (**no hand-eye cal**), 20/24 corners @ 0.26 px reproj; height 0.683 m vs depth-plane fit 0.684 m (1.4 mm agreement). 5 real scenes, **same** perception pipeline, matched to ArUco truth. **Metric = planar (table-XY) error** (flat marker gives XY truth, no top-Z).

**Planar pose RMSE (mm) — from `phase4b_gap_table.csv`:**
| Condition | coke | banana | golf | blue_box |
|---|---|---|---|---|
| sim_clean (n=20) | 10.44 | 13.27 | 6.15 | 3.78 |
| sim_noisy_oakd (n=20) | 10.35 | 13.24 | 6.02 | 3.54 |
| **real (n=4)** | 9.38 | 12.13 | 4.75 | 7.42 |
| **GAP (real − sim_clean)** | **−1.1** | **−1.1** | **−1.4** | **+3.6** |

**Findings:** the real-vs-sim planar pose gap is just **−1.4 to +3.6 mm across all four objects — sim ≈ real**; `sim_clean ≈ sim_noisy_oakd` still holds → **the negative result carries end-to-end on real data.**
**Limitation:** real is **n=4/object** — in 4 of 5 scenes one object drifted into the ToF edge zone and failed to segment (the ToF reliable area is smaller than the RGB marker FOV).

---

# PART 3 — PHASE 4C: REAL MANIPULATION (today's work, in full)

> This is the part documented nowhere else. It answers SQ3 on the **real arm** (does the pose-null propagate to a real grasp rate?) and the engineering is **baseline/Methods/Appendix**, not the contribution. The story matters because the *struggle* (and how it was overcome) is itself a documented sim-to-real finding.

### Starting point
The committed real executor (`real_grasp_executor.py`, HEAD `6e592ce`) used the **perceived PCA grasp axis**. To make the real and sim grasp recipes identical (so the measured gap is pure *execution*, not a perception confound), it was switched to always use the **predetermined per-object nominal grasp angle** (same as sim) — a one-line change (`minor_table = d.get("minor_axis")`) plus a re-added constant **`--off-x/--off-y` aim-nudge** flag. The sim path (`grasp_executor.py`, `grasp_common`, `arm_ik`) was left byte-identical.

### The problem that triggered everything: a ~4 cm working zone
On the bench, the 40 mm `blue_box` (needs the grasp centred within **±7.5 mm**) grasped reliably only in a **~4 cm × 4 cm patch** of the table — despite the arm's ~20 cm reach. The data (`real_manip_results.csv`) showed: detection ✓, IK feasible ✓, jaws close ✓ — but the jaws closed to ~17 mm on empty air (the box is 40 mm) → they were **missing the box body**, only ~3/15 holding.

### The misdiagnosis trail (documented because it's instructive)
1. **Not the grasp angle** — a wrong jaw angle would stall the jaws near 55 mm (can't close across the 60 mm length), not at 17 mm. 17 mm = empty jaws = a *positional* miss.
2. **The `--off-x` nudge was correcting the WRONG axis.** The table→arm calibration rotation `R_arm_from_table` ≈ **85°** (`[[0.094,−0.996],[0.996,0.094]]`), so an *arm-frame* `--off-x` moved the aim along the **table-Y** axis (the box's closing axis, ±7.5 mm tolerance) and did nothing for the table-X miss the operator saw — `--off-x −10` was actually a ~13 mm table-Y perturbation **hurting** the grip. **Fix:** apply the offset in the **table frame** (before `table_to_arm`). Verified by dry-run: +50 mm table-X → +38 mm arm-Y, matching `scale·R·[0.05,0]`.
3. **Then: a position-dependent error a constant offset cannot fix.** With table-X corrected, the operator reported the Y miss *flipped sign across the table* (left tip hits on the left side, right tip on the right). A single offset can't push one way on the left and the other on the right → the error is **position-dependent**.

### The honest dead-end, and the user's correct push-back
At this point the assessment was "this is the open-loop accuracy floor of a cheap arm; measure it honestly." The student rightly pushed back: a ~4 cm zone on an arm with ~20 cm reach is not plausible for a thesis, and a reference video proves the hardware *can* be precise. **This was the turning point** — the 4 cm zone was an artefact of *our own shortcut*, not a hardware limit.

### The multi-agent investigation (19 agents) — root cause, verified
A research+redesign workflow (web research across GitHub/Scholar/StackOverflow + redesign + adversarial critique + a project-manager synthesis) verified the root cause **directly against the live data**:
- **The error is an unidentifiable, position-dependent kinematic-model error.** The real arm over-reaches the URDF/`arm_ik` model by ~27 %. The global-scale workaround (`arm = scale·R@table + t`) leaves **14.1 mm RMS / 24.9 mm max on all 6 touch points** (the published 7.94 mm only survives by dropping a point to n=5). 14.1 mm > the box's ±7.5 mm tolerance → only a narrow band works.
- **6 touch points cannot identify a 13-parameter kinematic model** (overfits, no holdout). The "fit the per-servo gain first" hypothesis was **falsified** (gain-only fits leave 27–37 mm, *worse*).
- The **approach-tilt limit is edge-only** (vertical ~88° holds to r≈0.19 m; tilts to 85→75→65° past r≈0.20 m) — not the dominant cause.
- ⇒ **Any open-loop calibration is capped at ~8–14 mm and cannot clear ±7.5 mm workspace-wide. So: close the loop.**

### The solution: closed-loop visual nudge (FK-independent)
Tape one **ArUco marker (DICT_4X4_50, id 4** — the object markers use ids 0–3) flat on the gripper so the overhead OAK-D sees it. Before descending, iterate, **in the table frame**:
```
err = obj_xy − (gripper_marker_xy + tip_offset)
target += K · err          (K ≈ 0.6)
```
until `|err| < 3 mm`, then descend + close. **Because `err` is a difference of two points measured through the SAME camera in the SAME frame, the wrong FK and the 0.777 scale cancel as common-mode bias** — they only need to be good enough to converge. Grasp accuracy becomes **camera-bound (~1–4 mm), workspace-wide, independent of the broken kinematic model.** This is the same idea as a chess-playing arm that verifies against known points rather than trusting a perfect model — and it ties straight into the sim-to-real theme (*open-loop calibration error is the dominant real-world gap; one visual-feedback correction closes it within its working zone*).
*(A dense camera-measured thin-plate-spline correction field — REPLAB, arXiv:1905.07447 — was designed as a citable open-loop coarse stage + fallback, but was deferred: the closed loop alone delivered the result.)*

### Implementation (all UNCOMMITTED; `arm_ik` constants deliberately untouched because they are shared with the committed sim Phases 2–4A)
- **`xarm1s_manipulation/xarm1s_manipulation/visual_servo.py`** — the pure, hardware-free convergence core `servo_align(obj_xy, observe, command, …)`. Returns `{converged, measured, misses, final_target, history}`. An offline self-test with a synthetic position-dependent bias proves it: **500/500 random objects converge 9.2 mm → 1.85 mm (≤4 mm 100 %) in ~2.7 iterations.**
- **`tof_perception/scripts/detect_gripper_marker.py`** — RGB→table-frame gripper-marker detector, reusing the proven `ingest_real_scene` path (`detect_markers → marker_pose(solvePnP) → T_table_rgb @ pose`). Validated on a real scene (4 object markers → correct table-XY) and on the live gripper marker (`found:true`).
- **`real_grasp_executor.py --servo`** — wires the loop into `run_one` between the pre-grasp hover and the descent; additive (open-loop path byte-identical with `--servo` off). Flags: `--servo`, `--servo-id 4`, `--servo-len 0.05`, `--servo-gain 0.6`, `--servo-thresh 3`, `--servo-iters`, `--tip-dx/--tip-dy`. The bench observer captures an RGB (Win py-3.14 subprocess) → detects (WSL python3 subprocess), with a freshness guard (delete-before-capture + mtime/returncode check) and capture retries (the OAK-D drops X_LINK on rapid re-opens). A synthetic observer drives the full path under `--dry-run`.

### Adversarial code review (10 agents) — 3 real bench bugs found and fixed
1. **Detection-miss faked convergence** — a missed marker returned `err=0`, which the loop read as "converged" and descended at the uncorrected aim. **Fixed:** a miss now returns `None` → the loop holds aim, retries, and never counts a miss as convergence.
2. **Stale `_servo_rgb.png` re-detected as fresh** — a failed capture re-used the previous image. **Fixed:** delete-before-capture + mtime/returncode freshness guard + capture retry-with-delay.
3. **`--off-x/--off-y` corrupted the logged provenance under `--servo`** (didn't affect the grasp). **Fixed:** gated on `servo is None`.
*(A 4th finding — the predetermined-angle change is not byte-identical to HEAD — was the intended sim-parity change, not a bug.)*

### The bench journey to a clean result
- First real `--servo` grasps at the *tilted far edge* (tilt 77–82°) **diverged** (the high/offset marker swings as the arm reconfigures).
- A camera X_LINK drop on a later step exposed the miss-convergence bug (now fixed).
- **In the vertical zone (tilt ~88°), with `--servo-iters 8`, it converges and grasps.** Example: `err 19.1 → 0.96 mm conv`, gripped *even with the box deliberately rotated*.

### The result — Phase-4C blue_box formal run (5 placements × 3 = 15 trials)
**`blue_box 11/15 = 73 % lift-and-hold (Wilson 95 % CI 48–89 %).`** Clean labeled data: **`evaluation/results/real_manip_4c_formal.csv`**; Results 5.5 artifacts: `evaluation/results/phase4c_real_table.{txt,csv}` + `phase4c_real_vs_pred.png`.

| placement | reach r | servo err (mm) | 3 reps |
|---|---|---|---|
| p1 (0.35,0.16) | 0.20 m | 20→2, 23→2, 23→2 | ✅✅✅ |
| p2 (0.31,0.22) | 0.22 m | 26→7, 31→8, 24→2 | ✅❌✅ |
| p3 (0.37,0.29) | 0.16 m | 16→2, 17→2, 16→3 | ✅✅✅ |
| p4 (0.30,0.34) | 0.23 m | 44→17, 40→11, 40→11 | ❌❌❌ |
| p5 (0.37,0.37) | 0.19 m | 34→2, 31→2, 33→3 | ✅✅✅ |

- **Every failure is at r ≥ 0.22 m.** **Within the validated working zone r ≤ 0.20 m (p1, p3, p5): 9/9 = 100 %.**
- **Sim-to-real (the SQ3 datapoint):** sim analytic feasibility **100 %** → real closed-loop **73 % (100 % in-zone)**. The Phase-4B predictor (planar err **7.4 mm** vs jaw tol **7.5 mm** = "marginal") called it: open-loop would land on the cliff (≈0 % across a spread, the old 4 cm zone); the closed loop pushed it to 73 %/100 %-in-zone.
- **The before/after IS the Results 5.5 figure:** open-loop ~4 cm patch → closed-loop near-workspace.

### Limitations + the known fix
- **Far-reach falloff (r ≳ 0.20 m):** the loop gain collapses because the marker is mounted **high and behind the jaws** — at extended reach it swings instead of tracking the fingertips. **The fix (deferred): mount the marker low and directly over the jaw gap** (so it tracks the fingertip ~1:1 → converges full-reach). The near-zone result already proves the method.
- **Camera flakiness:** the OAK-D drops X_LINK / throws crash-dumps on rapid re-opens; the retry + freshness guard absorb it, but a persistent-camera architecture would be cleaner.
- **Other objects (predicted, not yet run):** **banana** (elongated like the box, ±10 mm tol — *more* forgiving) should work closed-loop in the zone — nearly free; **golf** (4.8 < 6 mm; closed-loop fixes the *aim* that was its wall, round-slip the only residual risk) worth a try; **coke = a SIZE WALL** (sim-feasible 0 %, 52 mm body in a 55 mm jaw) — closed-loop **cannot** fix it; document, don't chase.
- **Data-hygiene caveats:** the formal trials in the main `real_manip_results.csv` are mislabeled `placement_id=1, repeat=1` (the run template's flags weren't bumped), and `summarize_phase4c.read_real` counts **every** row with no placement filter → **always summarize the clean file** `real_manip_4c_formal.csv` (`summarize_phase4c.py --csv real_manip_4c_formal.csv`).

---

# PART 4 — OVERALL FINDINGS & NARRATIVE

1. **The negative result** (headline): measured ToF noise has no measurable effect on pose RMSE (Δ<0.1 mm) or grasp success — the per-axis median discards the 53–99 % edge flying-pixel outliers. **Refutes the proposal hypothesis.**
2. **Pose threshold ≈ ×10** (σ≈9 mm); **manipulation collapse ≈ ×20**; both far above reality. The extra factor before manipulation breaks exists because grasp is gated by object-vs-gripper tolerance.
3. **The injector is verified** (×20 collapse) → the null is sound, not a broken pipeline.
4. **Per-object spectrum (size vs shape):** blue_box robust (margin to spare); golf/banana borderline ~65 % (error sits *on* the tolerance); coke floored 0 % — a **size** limit (52 mm in 55 mm jaw), not curvature.
5. **The phases connect:** Phase 2 reframed the problem (real ToF is quiet in the bulk, noisy at edges) → Phase 3 (negative result) → Phase 4A (same shape in sim manipulation) → Phase 4B (sim≈real on real data, −1.4..+3.6 mm) → **Phase 4C (real manipulation rate via closed-loop; the open-loop calibration error, not depth noise, is the real-world bottleneck).** **Synthesis: for a robust median-centroid estimator the sim-to-real gap is NOT governed by depth-noise-model fidelity — it lives in calibration, lighting, and segmentation.**

### Limitations / open issues (thesis-level)
- Gazebo Classic cannot reliably contact-grasp small/curved objects → the sim success metric is **analytic/geometric** (a key supervisor discussion item).
- Noise model omits **spatial correlation and multipath**; lateral jitter uses the Nguyen literature value (0.8 px), not separately measured.
- Phase-4A CIs are **Wald** (collapse to ±0 at 0 %/100 %) — **Wilson** would be more honest (Phase 4C already uses Wilson).
- Real Phase-4B is n=4/object (ToF reliable zone < RGB marker FOV).
- Phase-4C real manipulation is currently **blue_box only**, in a near-zone (r≤0.20 m); the far-reach limit is a documented marker-mount issue.
- Gazebo Classic is end-of-life (Jan 2025).

### Future work
Run banana + golf with the closed loop; remount the marker low for full-reach convergence; re-solve the arm-base calibration with more points / a learned correction field; (out of scope) model spatial correlation + multipath; close the loop fully (visual servoing) as the principled accuracy ceiling.

---

# PART 5 — THESIS WRITING GUIDE

**Template:** official RUG **BSc-AI Overleaf template** (`bachelorproject.sty`, two-column, FSE-branded). Written in Overleaf (not in this repo). **Citations IEEE-numbered** (`\bibliographystyle{IEEEtran}` — the AI template defaults to APA/`apacite`; confirm with Kasaei). Add a **declaration of originality / AI-use**. *Two open confirmations with the supervisor: (1) AI vs CS program/template; (2) APA vs IEEE.*

| Ch | Title | Contents | Writable now? |
|---|---|---|---|
| 1 | Introduction | reality gap, RQ + 3 SQs, hypothesis, contributions, outline | ✅ |
| 2 | Related Work | sim-to-real/domain randomization; ToF/depth noise; tabletop perception; 6D pose eval; grasping (cite MVGrasp) | ✅ |
| 3 | System & Methods | sim/URDF/overhead RGB-D; noise-char protocol; injection model; depth-seg perception (median, translation-only); analytic IK; Phase-4B ChArUco extrinsic; **Phase-4C closed-loop servo (baseline/appendix)** | ✅ |
| 4 | Experimental Setup | hardware, 4 objects, conditions, trial counts, metrics (translation RMSE/MAE, grasp-feasible, Wilson) | ✅ |
| 5 | Results | 5.1 noise char · 5.2 injection (Δ<0.1 mm) · 5.3 ablation (≈×10) · 5.4 sim manipulation · **5.5 real pose gap (4B) + real manipulation (4C: 73 %/100 %-in-zone)** | 5.1–5.4 ✅; 5.5 mostly in |
| 6 | Discussion | negative result valid; median filter eats realistic noise; gap = calibration/lighting; **open-loop FK error is the real-world bottleneck (Phase-4C); closing the loop fixes it**; Gazebo grasp limitation | draft now |
| 7 | Conclusion & Future Work | — | after remaining 4C |

**Recommended writing order:** Ch.3 → Ch.4 → Ch.5.1–5.4 → Ch.2 → Ch.1 → Ch.6 draft → Ch.5.5 + Ch.7.
**Plug numbers straight from:** `phase3_pose_table.{txt,csv}`, `phase4a_success_table.csv`, `phase4b_gap_table.csv`, and `summarize_phase4c.py --csv real_manip_4c_formal.csv`. Figures already exist (σ-vs-z, edge overlays, ablation curve, `phase3_pose_vs_noise.png`, `phase4c_real_vs_pred.png`).

---

# PART 6 — REFERENCES (`thesis_writing/references.bib`)

**Used by the code/experiments (PART 1 — safe to cite):**
- `nguyen2012modeling` (axial+lateral injector eqs; the range model Phase 2 refutes); `khoshelham2012accuracy` (planar protocol); `sarbolandi2015kinect` (edge mixed-pixels + dropout); `foix2011lockin` (lock-in ToF / flying pixels).
- `tobin2017domain` (domain randomization / seeded RNG ablations); `hodan2018bop` (BOP — cite ONLY for the visible-top-surface alignment, NOT ADD/VSD).
- `garridojurado2014aruco`, `romeroramirez2018charuco` (ArUco/ChArUco truth + extrinsic); `foote2013tf` (tf2); `craig2005robotics` (closed-form IK).
- `koenig2004gazebo`, `macenski2022ros2`, `bradski2000opencv` (Gazebo / ROS 2 / OpenCV).

**Related-work framing (PART 2):** `jakobi1995noise` (origin of "reality gap"); `kasaei2021mvgrasp` (supervisor's MVGrasp); `bohg2014grasp` (grasp survey — positions the predefined grasp, not implemented); `rusu2008towards` (classic RANSAC-plane pipeline — cite as **contrast** only); `pinto2024flexx2` (PMD Flexx2 ToF — corroboration). If Methods calls the median "outlier-robust," add **Huber, *Robust Statistics* (1981)** (median 50 % breakdown).

**Software/assets (PART 3):** `gazebo_ros2_control`, `link_attacher` (PAL/IFRA), `luxonis_oakd`, `depthai_docs`. **TODO (only the user can recover):** `grabcad_xarm`, `sketchfab_cokecan`, `sketchfab_banana` — verify CC-BY licenses; see `src/ATTRIBUTION.md`.

**⛔ DO NOT CITE (PART 4 — would be false claims about the implementation):** `kuhn1955hungarian` (eval is greedy NN), `tsai1989handeye` (direct solvePnP, no hand-eye), `zhang2000calibration` (factory intrinsics), `hinterstoisser2012add` (translation-only pose), `fischler1981ransac` + `zhou2018open3d` (median-depth threshold, not RANSAC/Open3D), `olson2011apriltag` (ArUco chosen), `kumra2020antipodal` (learned grasping not implemented), `belhedi2015` (drop).

---

# PART 7 — DATA, FILES & HONESTY CHECKLIST

**Results tables/figures (`src/evaluation/results/`):** `phase3_pose_table.{csv,txt}`, `phase4a_success_table.csv`, `phase4b_gap_table.{csv,txt}`, `real_manip_4c_formal.csv` (clean Phase-4C), `phase4c_real_table.{csv,txt}` + `phase4c_real_vs_pred.png`, `phase3_pose_vs_noise.png`.
**Phase-2 params:** `src/tof_perception/scripts/fit_results_truly_raw/noise_params.json`; per-object `object_results/object_noise.json`.
**Notes:** `src/xarm1s_manipulation/{PHASE4A_NOTES.md, PHASE4A_RERUN_NOTES.md, GRASP_REALISM_NOTES.md, GRASP_RESEARCH.md, PHASE4C_NOTES.md, PHASE4C_RUNBOOK.md, PHASE4C_VISUAL_SERVO_PLAN.md}`; the 25-pp `thesis_ws/PROJECT_STATUS_REPORT.{md,pdf}`.
**Core code:** `tof_perception/{pose_estimator.py, scripts/ingest_real_scene.py, scripts/detect_gripper_marker.py}`, `noise_injection/depth_noise_injector.py`, `evaluation/{pose_evaluator.py, scripts/summarize_phase4c.py}`, `xarm1s_manipulation/xarm1s_manipulation/{grasp_executor.py, grasp_common.py, arm_ik.py, visual_servo.py, real_arm_driver.py}`, `xarm1s_manipulation/scripts/real_grasp_executor.py`, `xarm1s_grasp_plugin/src/link_attacher.cpp`.

**Code-honesty fixes before submission:**
1. `pose_estimator.py` docstring falsely says "RANSAC plane segmentation (Open3D)" — correct it (it's a median-depth threshold).
2. Verify the arm DOF (5-DOF IK vs the report's "6-DoF" file label) against `xarm.urdf`.
3. Fill the 3 TODO asset citations in `references.bib` (GrabCAD arm, Sketchfab coke/banana) — see `ATTRIBUTION.md`.
4. The Phase-4C closed-loop code is **uncommitted** (the student handles all git).

---

*End of brief. Everything needed to write the thesis is above; the Phase-4C narrative and the don't-cite list are the two things you won't find in the older `PROJECT_STATUS_REPORT.md`.*
