# Gazebo Classic Grasping — Research & Realism Plan (2026-06-04)

> **STATUS: IMPLEMENTED (2026-06-04).** The recommended path below was built and
> verified headless — all four objects now close on the object, weld on real
> two-finger contact, carry, and release without flinging. See
> `GRASP_REALISM_NOTES.md` for what was done and the per-object results. (Coke is
> the size-limit exception: real contact-triggered pickup, but a set-down release
> because the pads cannot clear the 52 mm can in the 55 mm jaw.)

Two web-research agents. Goal: replace the "grip air → object teleports" demo grasp
with something believable AND thesis-defensible.

## Verdict
- **Real friction grasping in Gazebo Classic (ODE) is possible but fragile.** It works
  for **flat-faced objects (our blue_box)** with heavy tuning + effort control, but is
  **unreliable for small/round/light objects** (slim can 52 mm, golf ball 43 mm, banana).
  This is documented community consensus — not a personal failing. *You were right that
  it isn't "impossible" — it's just unreliable for round/light parts.*
- **The fixed-joint attach approach is STANDARD, accepted practice — not a shameful hack.**
  Gazebo's own `physics::Gripper` uses the same zero-range-joint trick, and xArm's official
  `xarm_ros` repo explicitly tells users to install `gazebo_grasp_plugin` (GazeboGraspFix).

## Why our current demo looks outrageous
Our `xarm1s_grasp_plugin` follows the **PAL `link_attacher` pattern = a BLIND,
sequence-triggered attach** (it welds the object whenever the script says to, regardless of
whether the gripper is touching it). So the jaws close on air and the object snaps/teleports.
That blind-attach is the bad-looking failure mode.

## The fix: contact-TRIGGERED attach (GazeboGraspFix)
- **`GazeboGraspFix` (JenniferBuehler/gazebo-pkgs)** welds the object **only after detecting
  two opposing contact forces** on it for N cycles → **the gripper must genuinely close on
  the object before it attaches** → believable, no teleport. Detaches on open.
- This is the recommended, defensible path (it's what xArm's own docs use).

## Also required regardless of approach (this is our fling/explosion cause)
1. **Switch the fingers (ideally whole arm) to EFFORT control, not position control.**
   Position control commands the finger *into* the object → ODE penetration-correction
   impulse → explosion/fling. This is the coke-fling and banana-fling root cause.
2. **Realistic inertia tensors** on gripper links + objects.
3. **High friction on BOTH object and fingertips** (Gazebo uses the MIN of the two) + a
   high-friction fingertip pad.

## Real-friction tuning recipe (for an OPTIONAL box-grasp demo)
Per-collision `<surface>` on fingertips AND object:
```
mu, mu2: 100–1000 (default 1)     kp: 1e4–1e5 (default 1e12 = too hard for light parts)
kd: 10–100 (default 1)            max_vel: 0     min_depth: 0.001     max_contacts: 20
round objects also need torsional friction: <torsional><coefficient>100</coefficient>
```
World `<physics><ode>`:
```
solver quick, iters 100–200, sor 1.3     contact_max_correcting_vel: lower to 1–10 (stops explosions)
contact_surface_layer 0.001              max_step_size 0.001 (or 5e-4 for light objects)
```
Caveat (Imperial DE3 writeup): `max_vel=0` can make objects sink through the floor / vibrate —
tune carefully; their own values were "extremely optimistic, didn't reflect reality."

## Recommended path for the thesis
1. **Primary grasp = GazeboGraspFix (contact-triggered)** on all objects → believable,
   defensible, no teleport. Replace our blind-attach plugin with it (or make ours
   contact-triggered the same way).
2. **Switch fingers to effort control + fix inertias** → removes the fling/explosion.
3. **Optional rigor:** show ONE real-friction grasp of the blue_box (flat faces give a real
   contact patch) to prove you understand the contact params; state plainly that round/light
   objects use the attach constraint.
4. **Honest thesis framing:** the grasp is a *kinematic grasp constraint* standing in for
   unreliable ODE contact dynamics — exactly what Gazebo's own `physics::Gripper` does. Cite it.

## Key sources
- GazeboGraspFix wiki: https://github.com/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin
- Gazebo physics settings (grasp profile): https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Gazebo-physics-settings
- PAL link_attacher ("generalized grasp hack"; uses Gazebo's own Gripper): https://github.com/pal-robotics/gazebo_ros_link_attacher
- xArm uses GazeboGraspFix: https://github.com/xArm-Developer/xarm_ros
- ur5_gazebo_simulations (cuboids on friction; plugin added for cylinders/spheres): https://github.com/rohanNkhaire/ur5_gazebo_simulations
- Friction tutorial: https://classic.gazebosim.org/tutorials?tut=friction
- Physics params: https://classic.gazebosim.org/tutorials?tut=physics_params
- ROS Answers — can't grasp with position control (use effort): https://answers.ros.org/question/352107/
- Gazebo Answers — mu/kp/kd tuning: http://answers.gazebosim.org/question/22303/ , http://answers.gazebosim.org/question/19381/ , http://answers.gazebosim.org/question/22215/
