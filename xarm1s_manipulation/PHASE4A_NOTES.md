# Phase 4A — Simulated Pick-and-Place: Design, Issues, and Open Problem

**Project:** Investigating the Sim-to-Real Gap in RGB-D Based Tabletop Pose
Estimation and Manipulation (BSc thesis, University of Groningen)
**Component:** Phase 4A — closed-loop pick-and-place in simulation (Sub-Question 3)
**Stack:** ROS 2 Humble + Gazebo Classic 11 (note: Gazebo Classic is **end-of-life**),
`gazebo_ros2_control`, ODE physics.
**Date of this note:** 2026-05-31
**Status:** Pipeline works end-to-end; **one open cosmetic problem** (an
intermittent visual glitch when the *banana* is grasped) that does **not** affect
any thesis data. Bringing to supervisor for a second opinion.

---

## 1. What Phase 4A is supposed to produce

For each of the four objects (coke can, banana, golf ball, blue box) under each
perception condition (`sim_clean`, `sim_noisy_oakd`, ×10, ×20), produce:

- **Pick-success rate (%)**
- **Placement error (cm)** with confidence intervals across trials.

These feed Sub-Question 3 (how perception error propagates into manipulation).

---

## 2. The arm and why grasping is hard here

- **xArm 1S, 5-DOF**, *table-mounted*. The shoulder pivot sits only ~3 cm above
  the objects, and the "hand" segment is long relative to the upper/forearm. We
  solve the arm with an **analytical closed-form IK** (pure NumPy, no MoveIt/KDL).
- The gripper is a small anti-mirrored parallel jaw.

**Key constraint we proved empirically (2026-05-30):** *reliable physical
parallel-jaw grasping is not achievable in Gazebo Classic with this gripper.*
We instrumented a real contact grasp (vendored the standard `gazebo_grasp_fix`
plugin, widened the jaw, swept grasp height × close angle) and measured the
result: the finger collisions pass **through** the objects, the contact solver
never produces a usable grasp, and the object's pose is bit-identical before and
after closing. This is a well-documented Gazebo Classic limitation for
small/curved objects with thin mesh-collision fingers.

---

## 3. Grasp realization — two deliberately separate layers

Because real contact is infeasible (and because the proposal §4.4 asks only for a
"stable, repeatable **predefined** grasp," not contact physics), the grasp is an
**abstraction**, split into two independent layers:

### 3a. The measurement layer (this is what the thesis reports) — pure geometry, no physics
- **Pick success** = the perceived target XY is within a per-object capture
  tolerance of the object's true XY (i.e. the jaws would straddle it).
- **Placement error** = the perception offset at grasp time, carried rigidly to
  the drop point.
- **Both are computed analytically from perceived-vs-true pose at grasp.** They
  do **not** depend on the object physically moving. This is the important point
  for Section 5 below.

### 3b. The visualization layer (the on-screen demo) — physics
So the demo *looks* like a pick-and-place, the object is physically carried using
a **small custom Gazebo world plugin** (`xarm1s_grasp_plugin`, MIT-licensed,
written by us). It follows the community-standard "fixed-joint attach" pattern
(`pal-robotics/gazebo_ros_link_attacher`, `IFRA-Cranfield/IFRA_LinkAttacher`):

- On `/grasp/attach <object>` it creates a **revolute joint with both limits
  pinned to 0** (the reliable rigid constraint in Gazebo Classic; the native
  "fixed" joint type is historically unstable) between the gripper link and the
  object, so the object is carried rigidly.
- On `/grasp/detach <object>` it removes the joint.
- The object is then set onto its plate spot via the `gazebo_ros_state`
  `/set_entity_state` service.

All four objects are placed on a single flush, visual-only green plate.

---

## 4. Issues encountered this work period — and how each was resolved

| # | Symptom | Root cause | Resolution |
|---|---------|-----------|------------|
| 1 | **Whole arm + all objects glitch; an object "picked up" even when the arm was nowhere near it** | We were pinning *placed* objects to the **static table** with a runtime joint. A joint between a dynamic body and a static model destabilises ODE's **whole-world** solver (documented: PAL `gazebo_ros_link_attacher` issue #17). | **Removed the table-pin entirely.** No object is ever joined to the static table. Placement is done with `/set_entity_state` only. Whole-arm glitch gone. |
| 2 | Banana **flung on release** | The banana's long collision box stored elastic energy along the rigid joint and ejected on detach. | Shrank the banana collision box (0.18→0.12 m) and added a short `set_entity_state` "set-down" that absorbs release motion. |
| 3 | Code had dead/duplicate paths from earlier soft-attach design | Iterative development | Cleaned up: removed dead soft-attach state, timers, helpers, unused imports; rewrote docstrings. Build + import verified. |
| 4 | **Banana intermittently flies the moment it is grasped, and the whole sim "tweaks"** (only the banana, some runs only) | **OPEN — see Section 6.** | Best-effort robustness measure applied; root cause is a real-time ODE effect we could not reproduce offline. |

Items 1–3 are **resolved and verified**: in automated headless tests all four
objects are picked and placed on the plate every run, with no glitch and no
fling.

---

## 5. Why none of this affects the thesis results

The numbers Phase 4A reports (pick-success %, placement error) come from the
**measurement layer (3a)** — they are computed from the perceived-vs-true pose at
grasp time. They are **independent of whether the physical carry succeeds.**

Therefore the open problem in Section 6 is a **demo/visualization cosmetic
issue**, not a data-integrity issue. The Sub-Question 3 experiment (running the
noise conditions to get the headline result) is **not blocked** by it.

---

## 6. The open problem: intermittent banana grab-fling

### Symptom (observed in the GUI, real-time)
When grasping the **banana**, *intermittently* (some runs only), at the moment
the gripper closes/attaches — "right spot, then the banana flies away and the
whole sim tweaks until the banana is down on the table." Only the banana. The
coke / golf ball / blue box are always clean.

### What the diagnosis established
- **Not reproducible headless.** Across **16 automated runs**, including a loop
  of **15 consecutive banana grabs** using the **live camera's real (jittery)
  grasp point**, the grab and carry were smooth **every time** and the banana
  placed correctly every time — **zero flings caught.** It only happens in the
  real-time GUI, which points to an **ODE solver-timing effect at the instant the
  rigid joint forms** (headless runs faster than real-time, so the solver hits
  that step differently).
- **"The whole sim tweaks"** is the signature of an **intermittently
  ill-conditioned constraint** when the joint is created — the same *family* of
  problem as issue #1, but now the legitimate gripper↔object joint itself,
  occasionally.
- **It is banana-specific because the banana is the only elongated object**
  (12 cm), so it has large transverse inertia hanging at full arm reach when the
  rigid joint snaps shut. **It is not a mass effect** — the banana (120 g) is in
  fact the *heaviest* object; the lighter coke (50 g) and golf ball (46 g) never
  misbehave.

### Fixes tried this session (the ones that did NOT work — for the supervisor's reference)
Each was reverted because it either had no effect or **regressed another object**:

| Attempt | Idea | Outcome |
|--------|------|---------|
| Zero object velocity at **detach** | release at rest | **Flung the coke 16 m** — `SetLinearVel` raced `/set_entity_state` on the same body. Reverted. |
| Anchor the joint at the gripper link | shorten the constraint lever | No effect. Reverted. |
| Disable gravity while held | remove gravity torque | No effect. Reverted. |
| Detach **before** opening the jaws | release before the place-pose hold | **Flung the coke** — placement teleports it into still-closed fingers (coke is 52 mm; the jaws only frame to 42 mm). Reverted. |
| Carry the object **down to the table** before release (lower release height) | avoid the placement teleport | **Flung coke 59 m and banana to the floor** — pressing a rigidly-joined object into the table is a contact-vs-constraint explosion. Object must be released *above* the table. Reverted. |
| Open the jaws **during** the carry | remove the static gripper-open hold | No effect on the residual transient. Reverted. |

### Fix currently applied (safe, but unverified against the real-time symptom)
- The plugin now **zeroes the object's velocity the instant before the joint is
  created** ("form the constraint on a body at rest" — the standard robustness
  measure for joint-formation impulses). This is at *attach*, where there is no
  `/set_entity_state` race, so it is safe — verified all four still place
  correctly. Whether it reduces the *real-time* intermittent fling can only be
  judged by re-running in the GUI (we cannot reproduce it offline).

### Candidate next steps to discuss with the supervisor
1. **Joint compliance (CFM/ERP softening)** on the rigid joint — the textbook fix
   for an ill-conditioned / "whole-sim-tweak" LCP. Not yet tried (we did not want
   to ship an unverifiable change blindly).
2. **Shrink the banana collision box further** (0.12 → ~0.08 m) so it behaves more
   like the compact, stable objects (less transverse inertia at reach). The visual
   mesh and the analytic metric are unaffected.
3. **Add the canonical `Attach()` + `SetModel()` calls** that our plugin currently
   omits (some link-attacher forks include them for a more robust joint).
4. **ODE real-time settings** — increase solver iterations / reduce step size /
   cap the real-time factor, to make the real-time solver behave like the (stable)
   headless one.
5. **Question worth raising:** since the thesis metric is analytic (Section 5), is
   a physically rigid carry even necessary for the deliverable, or is a simpler
   visualization (or accepting the occasional banana glitch) acceptable? This may
   be the most pragmatic answer given Gazebo Classic is end-of-life.

---

## 7. How to reproduce / test (for completeness)

Headless automated check (WSL, no GUI):
```bash
cd ~/thesis_ws && source install/setup.bash
colcon build --packages-select xarm1s_grasp_plugin xarm1s_manipulation
# bring up sim + perception, then a ground-truth all-four pick-place demo
```
GUI check (two terminals):
```bash
# Terminal 1: sim + perception (Gazebo GUI)
ros2 launch tof_perception sim_pose_estimation.launch.py
# Terminal 2: ground-truth all-four pick-and-place
ros2 run xarm1s_manipulation grasp_executor --ros-args \
  -p use_sim_time:=true -p target_source:=ground_truth -p n_trials:=1 \
  -p execute_motion:=true -p move_time:=2.0
```
The intermittent banana glitch is only visible in the GUI (Terminal-1 + Terminal-2
real-time run); it does not appear in the faster-than-real-time headless runs.

---

## 8. One-paragraph summary for the meeting

The Phase-4A pick-and-place pipeline is complete and the **measured results
(pick-success, placement error) are sound** because they are computed
analytically from perception error, independent of the physics carry. Real
parallel-jaw contact is infeasible in Gazebo Classic (proven empirically), so the
object is carried by a rigid joint purely for the on-screen demo. We resolved a
whole-arm solver instability (caused by pinning objects to the static table) and a
release-time fling. The **one remaining issue** is that the *banana* — the only
elongated object — **intermittently flies when grasped, but only in the real-time
GUI**; we could not reproduce it in 16 automated runs, which indicates a real-time
ODE solver-timing effect at joint formation. Six attempted fixes either had no
effect or destabilised other objects and were reverted; one safe robustness
measure (forming the joint on a zeroed-velocity body) is in place. Because this is
cosmetic and the thesis data is unaffected, the noise-condition experiment can
proceed regardless. We would value the supervisor's view on the candidate fixes in
Section 6.5 — in particular whether a rigid physical carry is even required for the
deliverable.
