# Phase 4A Re-run — Realistic Gripper, Results, and Open Issues

**For supervisor discussion · 2026-06-04**
Companion to `PHASE4A_NOTES.md` (which covers the pick-and-place pipeline and the
Gazebo-Classic contact-grasp limitation).

---

## UPDATE — 2026-06-05: full varied-placement sweep (supersedes §4)

The earlier headline (§4) kept each object at **one fixed pose** and re-sampled
only perception noise there. At the supervisor's request the sweep was redone with
**full reachable-workspace coverage**: each object is run **one at a time** at
**20 different positions _and_ orientations** spanning the reachable annulus
(radius 0.12–0.30 m, every pose validated reachable by the IK; full yaw; the coke
lying on its side), in each of the six noise settings. The 20 poses per object are
**fixed across all six conditions** (seeded), so each noise comparison is *paired*
— only the depth noise changes between conditions, not the placements.

Two deliberate methodology choices (agreed beforehand):
- **Bias correction OFF.** The per-object mesh-centroid bias was tuned for one
  spot/orientation; under varied yaw that offset *rotates*, so a fixed correction
  is simply wrong. We report the **raw** perception error — the honest signal.
- **Metric unchanged:** the analytic `grasp_feasible` (perceived centre within the
  per-object jaw `capture_tol`); no arm motion (the Gazebo-Classic contact
  limitation, §2 / `PHASE4A_NOTES.md`).

**480 analytic trials** (6 conditions × 4 objects × 20 placements). Tooling:
`scripts/rerun_4a_sweep.sh 20` (now passes `vary_placement:=true
bias_correction:=false`); table + figure by `scripts/summarize_4a_rates.py`.

### Success rate (%) ± 95% CI — grasp-feasible, varied placements, bias off

| condition | coke (1.5 mm) | banana (10 mm) | golf (6 mm) | blue_box (7.5 mm) | overall |
|---|---|---|---|---|---|
| clean         | 0 ± 0  | 65 ± 21 | 65 ± 21 | 100 ± 0 | 57 ± 11 |
| ×1 (measured) | 5 ± 10 | 65 ± 21 | 65 ± 21 | 100 ± 0 | 59 ± 11 |
| ×2            | 5 ± 10 | 60 ± 21 | 65 ± 21 | 100 ± 0 | 57 ± 11 |
| ×5            | 0 ± 0  | 70 ± 20 | 65 ± 21 | 100 ± 0 | 59 ± 11 |
| ×10           | 0 ± 0  | 65 ± 21 | 75 ± 19 | 100 ± 0 | 60 ± 11 |
| **×20**       | 5 ± 10 | **30 ± 20** | **20 ± 18** | **50 ± 22** | **26 ± 10** |

Header = per-object jaw tolerance; N = 20/cell, 80/condition. CI is the Wald
interval `1.96·√(p(1−p)/n)` (collapses to ±0 at exactly 0 %/100 %; a Wilson interval
would be more honest at those boundary cells). The noisy-condition rates wobble a
few points between re-runs — the injected depth noise is realised fresh each run (the
20 placements are seeded, the noise is not); clean is deterministic. Figure:
`evaluation/results/phase4a_rate_vs_axial_scale.png`; numbers in
`phase4a_success_table.{txt,csv}`.

> **Banana re-centered (2026-06-05, later same day).** The banana was 0 % at *every*
> noise level because its **visual mesh was off-center from its link origin** (the
> ground-truth reference): the depth-footprint centroid sat ~28 mm from the origin,
> an offset that **rotates with yaw**, so no fixed world-frame bias could cancel it.
> Fix = a link-frame `<pose>` on the banana `<visual>` (= −C, the measured body-frame
> offset) so the footprint centroid lands on the link origin at *every* yaw, plus
> zeroing the now-obsolete banana `bias`. Clean banana error dropped **28.7 → 10.6 mm
> mean** (median 27.9 → 9.3) and the rate **0 → 65 %**. The ~9 mm residual is genuine
> — a curved object's silhouette-median does not transform as a rigid point under
> yaw. The table above is the **post-fix** sweep.

### What this says

1. **All four objects are detected 20/20 in every condition** — detection is not the
   bottleneck; **perception _accuracy_ is.** Three of the four (banana, golf,
   blue_box) have tolerance headroom; only coke is floored (below).
2. **The noise threshold is clean and consistent.** The three graspable objects are
   **flat from clean through ×10** and **collapse together only at ×20** (blue_box
   100→50 %, banana 65→30 %, golf 65→20 %). Measured OAK-D noise (×1) is
   indistinguishable from clean — the same honest negative as §4, consistent with the
   Phase-3 *pose* threshold (~×10). Manipulation tolerates a little more noise than
   pose before breaking, but the order of magnitude agrees.
3. **Raw (uncorrected) clean error per object** — why each lands where it does:

   | object | clean mean / median capture error | jaw tol | clean rate |
   |---|---|---|---|
   | blue_box  | 3.5 / 3.5 mm   | 7.5 mm | 100 % |
   | golf_ball | 5.4 / 5.2 mm   | 6.0 mm | 65 % (on the boundary) |
   | banana    | 10.6 / 9.3 mm  | 10 mm  | 65 % (boundary, after re-centering) |
   | coke_can  | 6.4 / 4.9 mm   | **1.5 mm** | 0 % |

   **coke is the only floored object** — tolerance-limited: a 52 mm can in a 55 mm
   jaw gives only 1.5 mm, smaller than its own ~6 mm mesh-centroid error, so it can
   never pass regardless of noise (a genuine "at the gripper's size limit" result,
   §5). **golf and banana sit *on* their tolerance** (5.2 and 9.3 mm median vs 6 and
   10 mm), which is why both rates *wobble* sub-threshold and neither reaches 100 %:
   at some poses the perceived centre lands a hair over tolerance, and small noise is
   as likely to nudge it back under as past (verified per-trial). The banana's
   residual is the *yaw-dependent* part of the curved-object silhouette centroid (the
   large *systematic* part was removed by re-centering, above).
4. **Report the per-object curve, not one averaged number.** Averaging hides the
   structure: blue_box robust to ×10, golf and banana borderline (~65 %), coke
   floored by the gripper size limit independent of noise. The thesis-honest framing
   is the per-object rate vs noise, with the **×20 collapse of the three graspable
   objects** as the manipulation-side echo of the Phase-3 perception threshold.

---

## 1. What changed, and why

The simulated gripper was opening to **~72 mm** (an idealized/"widened" URDF
value). The **real Hiwonder xArm 1S gripper measures 55 mm** (measured directly —
the manufacturer publishes no max-grip-width spec). The 72 mm value was inflating
simulated grasp success.

We corrected the sim to match reality:
- gripper opening set to **55 mm** (`xarm.urdf` finger limits + `JAW_OPEN = -0.459`
  in `grasp_executor.py`);
- the per-object **capture tolerance recomputed** as `(55 mm − object_width)/2`.

**This sim/real gripper mismatch (72 vs 55 mm) is itself a sim-to-real gap source
and should be disclosed in the write-up.**

## 2. The pick-success metric (for transparency)

A grasp succeeds **iff the perceived object _centre_ is within `capture_tol` of the
true centre** (the open jaws would straddle the object), with
`capture_tol = (gripper_opening − object_width)/2`. This is **analytic /
perception-accuracy-based** — no physical-contact grasp is simulated, because
Gazebo Classic cannot do reliable parallel-jaw contact for these small/curved
objects (see `PHASE4A_NOTES.md`). It is defensible under the proposal's
"predefined grasp" allowance. (Note: the CSV's `target_x/y` is the on-body grasp
point, a *different* quantity from the centre used by the metric.)

## 3. New per-object tolerances (55 mm gripper)

| object | width | capture_tol |
|---|---|---|
| coke_can | 52 mm | **1.5 mm** |
| banana | 35 mm | 10 mm |
| golf_ball | 43 mm | 6 mm |
| blue_box | 40 mm | 7.5 mm |

## 4. Headline result (preliminary — N = 10 validation; full N = 100 pending)

**Noise injection degrades manipulation only at `axial_scale ≈ 20`, not at
measured/realistic noise — consistent with the Phase-3 _pose_ threshold (~×10).**
Pose and manipulation now tell the same story.

Per-condition pick rate (n/d = object not detected):

| condition | coke | banana | golf | blue | overall |
|---|---|---|---|---|---|
| sim_clean | 0% | n/d | 100% | 100% | 50% |
| sim_noisy_oakd (×1) | 0% | n/d | 100% | 100% | 50% |
| ×2 / ×5 / ×10 | 0% | n/d | 100% | 100% | 50% |
| **×20** | 0% | n/d | **78%** | **90%** | 40% |

The noise is applied correctly (perceived-x spread grows with scale; injector log
confirmed `axial_scale = 20.0`):

| condition | golf spread | blue spread |
|---|---|---|
| sim_clean | 0.00 mm | 0.00 mm |
| ×1 / ×2 / ×5 | 0.4–0.9 mm | 0.6–0.9 mm |
| ×10 | 1.2 mm | 2.3 mm |
| ×20 | **21.9 mm** | **12.8 mm** |

The spread stays under the tolerances (6–7.5 mm) until ×20, hence no failures
until then. **Honest negative:** the earlier hypothesis that a tighter gripper
would make _measured_ noise matter did **not** hold.

## 5. Open issue A — the coke can is at the gripper's size limit

52 mm (a 250 ml slim can) in a 55 mm gripper → **1.5 mm tolerance**. It fails
consistently because of a **constant ~3.5 mm pull-tab mesh residual** that exceeds
the tolerance even in clean sim. Reading: coke is effectively a "too large /
mesh-bias-limited" object. **Keep as a documented hard case, or drop — to discuss.**

## 6. Open issue B — the banana is not detected (the main item to discuss)

> **RESOLVED (2026-06-05) — two fixes.** (1) *Detection:* the banana now uses a
> **solid-yellow mesh** (`meshes/banana_solid.obj`), so it detects **20/20 in every
> condition** (option (a) of the original diagnosis below). (2) *Accuracy:* that
> exposed a second issue — the banana was still **0 %** because its visual mesh was
> **off-center from its link origin**, putting the depth-footprint centroid ~28 mm
> off the ground-truth reference, an offset that **rotates with yaw** (so the old
> fixed bias couldn't help). Fixed by adding a link-frame `<pose>` to the banana
> `<visual>` (= −C, the measured body-frame offset) that re-centers the footprint
> centroid onto the link origin at every yaw, and zeroing the obsolete banana
> `bias`. Result: clean error **28.7 → 10.6 mm mean**, rate **0 → 65 %** (see the
> UPDATE banner). The ~9 mm residual is the genuine yaw-dependent wander of a curved
> object's silhouette centroid. The original detection diagnosis is kept below.

Banana = **0/10 detected in every condition.** Root cause: it is the only
**textured-mesh** object without a solid detectable colour. Gazebo Classic renders
textured meshes from their own material/texture and **ignores the SDF `<material>`**
— the primitives (golf = sphere, blue_box = box) take their SDF colours and detect
fine; the meshes (coke, banana) render their texture (coke's is red-enough,
banana's realistic photo is not). An attempted SDF-material fix was therefore inert.

Options:
- (a) give the banana mesh a **solid-yellow texture** so the HSV labeler can tag it
  (most reliable, needs one visual check in Gazebo); or
- (b) **report banana as a detection/texture limitation** and use the three
  reliably-detected objects.
**Undecided — want supervisor input.**

## 7. Data-integrity disclosure

The original `manip_results.csv` was **append-accumulated across many runs**, mixing
stale rows from different configs; an early offline recompute also read the wrong
column. **Both are corrected**: results are now regenerated by an automated sweep
(`scripts/rerun_4a_sweep.sh`) that archives the old file and writes a clean CSV.
Any earlier ad-hoc manipulation numbers should be disregarded.

## 8. Status / next

- **DONE (2026-06-05): full varied-placement sweep, 480 trials** — see the UPDATE
  section at the top (which supersedes the §4 fixed-pose preliminary). Banana
  detection resolved (§6). Artifacts in `evaluation/results/`:
  `manip_results.csv` (480 rows, varying true poses), `phase4a_success_table.txt`,
  `phase4a_success_table.csv`, `phase4a_rate_vs_axial_scale.png`.
- Tooling: `scripts/rerun_4a_sweep.sh [N]` — builds, archives the old CSV, then
  loops all six conditions (`execute_motion:=false vary_placement:=true
  bias_correction:=false`, analytic capture) into a fresh CSV with a per-condition
  summary; `scripts/summarize_4a_rates.py` regenerates the table + figure from the
  CSV. A standalone, Gazebo-free check of the placement sampler lives in
  `test/check_reachable_placements.py`.
- For the supervisor: the metric is still analytic `grasp_feasible` (a perception
  /geometry check, not a physical grasp — the key discussion item) and the
  Wald-vs-Wilson CI choice at the 0 %/100 % cells is open.
