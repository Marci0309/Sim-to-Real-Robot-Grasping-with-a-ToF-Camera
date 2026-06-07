# Phase-4A grasp realism — contact-triggered grasp + effort control (2026-06-04)

Companion to `PHASE4A_NOTES.md` / `PHASE4A_RERUN_NOTES.md`. This documents the
rework of the demonstration grasp from a *blind* fixed-joint attach into a
**contact-triggered** grasp with **effort-controlled** fingers. Research basis:
`GRASP_RESEARCH.md` (GazeboGraspFix / Gazebo `physics::Gripper`).

## What was wrong

The old grasp looked fake: the gripper closed on **air** and the object was
welded regardless of contact (the `xarm1s_link_attacher` welded whenever the
script published `/grasp/attach`). The fingers were **position-controlled**, so
commanding them toward a closed angle drove them *into* the object → an ODE
penetration impulse that flung light objects (the coke/banana fling).

## What changed (4 pieces + the executor)

1. **Contact-triggered weld** (`xarm1s_grasp_plugin/src/link_attacher.cpp`).
   `/grasp/attach <model>` now only **arms** the object; the plugin reads the
   world `ContactManager` each physics step (`SetNeverDropContacts`, at
   `WorldUpdateEnd`) and welds the object to the gripper only once **both
   fingers are in sustained contact** with it (`grip_time`, default 0.05 s). So
   the gripper must genuinely close on the object first — no grip-on-air, no
   teleport-attach. A `fallback_timeout` (default 1.0 s; set 0 to require
   genuine contact) welds anyway if no contact is detected, for demo
   robustness, and logs that it did. On detach the object's velocity is zeroed
   (kills any residual contact-spring impulse). All joint creation/removal
   happens on the physics thread; the ROS callbacks only set intent.

2. **Effort-controlled gripper** (`xarm.urdf` + `controllers.yaml`). The two
   finger joints now expose the `effort` command interface (the arm stays on
   `position`); `gripper_controller` is a `JointTrajectoryController` with PID
   gains (`p=8, d=0.3`), so it tracks the commanded jaw position by emitting
   **bounded torque**. Closing onto an object therefore applies a soft squeeze
   that **stalls at the surface** instead of penetrating it. Finger
   `<limit effort>` lowered 15→5 N·m and `<dynamics damping/friction>` added.

3. **Fingertip contact pads** (`xarm.urdf`). ODE's trimesh collider does **not**
   reliably generate contacts for the thin finger STL (verified: jaws closed to
   34 mm inside a 40 mm box produced *zero* finger contacts). Each finger got a
   small **primitive box pad** flush with its inner face (measured from the STL:
   left inner edge x=−1.5 mm, right +1.5 mm, tip z≈+58.7 mm) extending outward
   and kept **short, at the very tip** (z 50–59 mm) so it does not swing inward
   on the finger's pivot arc when the jaws are open. This is what the plugin
   actually senses on.

4. **Grasp-friendly ODE physics** (`tabletop.world`). Added a `<physics>` block:
   `contact_max_correcting_vel=1.0` (default 100 → stops contact explosions),
   `iters=150`, `contact_surface_layer=0.001`, `max_step_size=0.001`. Plus
   per-object `<surface>` friction (`mu=1.5`) and soft contact (`kp=1e5, kd=10`)
   on the four objects and the fingertips; torsional friction on the sphere/
   cylinder.

   *Sink note (2026-06-04):* the light banana dips a few mm into the table when
   the jaws press it near the surface. A "fix" (object `kp=1e6` + a stiff table)
   was tried, but stiffening the contact **ejected** the light banana the instant
   the jaws closed (a flick-away — far worse than the dip), so it was **reverted**
   to `kp=1e5`. The slight dip is accepted as a Gazebo-Classic contact limitation;
   it doesn't affect the data (which is analytic). Chasing no-dip *and* no-flick
   for a curved/light object is finicky ODE tuning — better noted as a limitation.

5. **Executor** (`grasp_executor.py`). Closes the jaws **onto** the object
   (`grasp_width − 6 mm` squeeze) then **arms after closing** (so a transient
   open-jaw touch can't weld early). Placement replaces the teleport with a
   **gentle release** (open jaws → detach → object drops the last cm and
   settles under friction).

## Result (headless ground-truth pick-and-place, N=1/object)

| object    | grasp           | placed vs plate spot |
|-----------|-----------------|----------------------|
| blue_box  | contact-triggered | 1 mm |
| golf_ball | contact-triggered | 5 mm |
| banana    | contact-triggered | 21 mm |
| coke_can  | contact-triggered | 0 mm (set-down, see below) |

All four close on the object, weld on real two-finger contact, carry, and
release without flinging.

## The coke is the documented size-limit case

The 52 mm coke fills the 55 mm jaw, so once the fingertip pads are added the
**open** jaws can no longer fully clear it (they still penetrate ~2.5 mm). A
free release then lets the residual pad contact spring it off the table. The
**pickup is still a genuine contact-triggered grasp**; only the *placement* is a
gentle set-down on its spot (`place_settle=True` in the `OBJECTS` table → a
zero-velocity `set_entity_state` after the gripper has retreated clear). This is
consistent with the earlier finding that the coke is "at the gripper's size
limit." The other three objects use the fully physical free-drop release.

## Thesis framing (honest, defensible)

The grasp is a **contact-triggered kinematic grasp constraint** following
GazeboGraspFix (Buehler) and Gazebo's own `physics::Gripper`, standing in for
Gazebo Classic's unreliable *sustained* parallel-jaw friction on small/round
objects. The **pick-success metric is unchanged**: it is computed analytically
from the perceived-vs-true object centre at grasp time and does **not** depend on
the physical carry, so the fast statistical sweep (`execute_motion:=false`,
Sub-Question 3) is unaffected by any of this. The rework concerns only the
visual/physical fidelity of the *demonstration*.

## How to run / verify

* Demo (GUI): `bash scripts/start_sim.sh` then `bash scripts/run_demo.sh`.
* Headless smoke test: `bash scripts/test_grasp_headless.sh 'blue_box coke_can
  golf_ball banana'` — brings up gzserver (no GUI), runs the ground-truth
  pick-and-place, and reports the grasp events + final placement.
* Static check after editing URDF/world/SDF/controllers:
  `python3 test/validate_grasp_assets.py`.
