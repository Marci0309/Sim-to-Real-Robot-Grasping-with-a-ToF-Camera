# Thesis — Per-Section Instructions (the "verify your doing" spec)

**Thesis:** *Investigating the Sim-to-Real Gap in RGB-D Based Tabletop Pose Estimation and Manipulation*
**Author:** Marcell Mor Szokedencsi (s5173191) · **Supervisor:** Dr. H. Kasaei · RUG BSc AI ·
WBAI901-15 (15 ECTS) · Overleaf, IEEE-numbered, two-column.

> **How to use this file.** One block per thesis subsection. Each block lists: **GOAL** (what the
> subsection must accomplish), **INCLUDE** (a content checklist), **NUMBERS** (exact figures that must
> appear, verbatim from the CSVs), **ASSETS** (tables/figures that belong here), **CITE** (the
> `references.bib` keys allowed here), **LENGTH**, and **PITFALLS** (honesty/over-claim traps). Tick
> each INCLUDE item against your written draft. The current `_0_5in.pdf` already satisfies most of
> these — treat the spec as a **review checklist** against that draft, not a blank-page guide.
>
> **Golden rules (apply everywhere):**
> 1. Every numeric claim must trace to a CSV in `evaluation/results/` or a JSON in
>    `tof_perception/scripts/`. No number that isn't in a file.
> 2. Only cite keys present in `references.bib` **PART 1–3**. Never cite a **PART 4 (EXCLUDED)** key —
>    that is the plagiarism/false-claim trap (see `THESIS_REFERENCE_AUDIT.md`).
> 3. The **noise result is the contribution**; the **manipulation system is the baseline**. Never let
>    Phase-4C engineering read as the deliverable.
> 4. Write the negative result as a **strength**, with the verified ×10/×20 failure mode as proof it
>    is sound, not a broken pipeline.

> **FORMAT CONFORMANCE (verified — see `SUPERVISOR_FORMAT_FINDINGS.md`):**
> - **Citation style DECIDED → APA author-year (`apacite`)**, the official RUG `bachelorproject.sty`
>   template default — no `.bst` swap. **Convert the draft's numeric `[n]` cites to author-year**
>   (`\cite{}` → *(Nguyen et al., 2012)*, `\citeA{}` → *Nguyen et al. (2012)*) and **reword number-leaning
>   prose** (*"as in [5]"* → *"as shown by Nguyen et al. (2012)"*). Every CITE key below is unchanged;
>   only the rendered form changes.
> - **Add Acknowledgements** to the front matter (his house style includes it).
> - **Label the axial-scale sweep explicitly as an "ablation study"** (§5.3) — his convention.
> - **Conclusion shape:** lead §7 with a **"Summary of Main Contributions"** paragraph then Future Work
>   (matches his "6.1 Summary of Main Contributions / 6.2 Future Work" pattern).
> - **Discussion chapter:** keeping a separate §6 Discussion is fine (recommended for a negative result);
>   his students often fold it into Results+Limitations instead — either passes.
> - **Title block:** supervisor = Dr. H. Kasaei only — **no second assessor** (confirmed by student).

---

## FRONT MATTER

### Title block
- **GOAL:** identify the work and author per the RUG template.
- **INCLUDE:** title (verbatim above); author `Marcell Mor Szokedencsi, s5173191,
  m.m.szokedencsi@student.rug.nl`; `Supervisor: Dr. H. Kasaei`; affiliation line *University of
  Groningen, Bernoulli Institute, Faculty of Science and Engineering*.
- **PITFALLS:** use the exact name spelling the programme expects; match the template's author block.

### Acknowledgements  *(short, optional but recommended — his house style includes it)*
- **GOAL:** thank the supervisor (Dr. Kasaei / IRL Lab) and anyone who helped with hardware/access.
- **PITFALLS:** keep it brief and professional; place before or after the abstract per the template.

### Abstract  *(~200 words, one paragraph)*
- **GOAL:** state problem → method → the negative headline → real-world confirmation → takeaway, so a
  reader knows the whole story without reading on.
- **INCLUDE:** sim-to-real / sensor-mismatch motivation; the setup (OAK-D SR PoE ToF + xArm 1S, Gazebo
  Classic + ROS 2); the 4 phases; the **headline negative result**; the verification (breaks only when
  scaled); the real confirmation; the one-line conclusion (gap = calibration/lighting, not noise).
- **NUMBERS:** pose RMSE effect **< 0.1 mm** vs clean; threshold **≈ ×10**; real-vs-sim gap
  **−1.4 to +3.6 mm**; real grasp **57 %** (34/60).
- **CITE:** none (abstract carries no citations in IEEE style).
- **PITFALLS:** don't bury the negative result; don't oversell manipulation; keep the
  "home setting vs hazardous >×10" nuance to the body, not the abstract.

---

## 1. INTRODUCTION  *(~1–1.5 columns)*

### 1.0 (opening, unnumbered)
- **GOAL:** motivate the reality gap; narrow from "domain randomization is blunt" to "we isolate one
  measurable cause (ToF depth noise) in a constrained top-down setup."
- **INCLUDE:** why sim is used (real robot time slow/fragile/expensive); reality-gap term; domain
  randomization as the blunt-but-blanket alternative; the narrow quantitative question; an explicit
  **forward spoiler that the answer is "no"** and why that is the useful result.
- **CITE:** `jakobi1995noise` (reality gap), `tobin2017domain` (domain randomization).
- **PITFALLS:** state the constrained scope (fixed overhead camera ⇒ depth is the dominant uncertainty,
  not viewpoint/hand-eye) so the negative result can't be dismissed as "you tested the easy case."

### 1.1 Research Questions
- **GOAL:** one main RQ + three SQs, each mapped to a phase.
- **INCLUDE (verbatim):** Main RQ (*how does modelling realistic ToF noise affect the sim-to-real
  gap…*); SQ1 noise→Phase 2; SQ2 pose→Phase 3; SQ3 manipulation→Phases 4A (sim)/4B (real perception)/4C
  (real manipulation).
- **PITFALLS:** keep the SQ→phase mapping identical everywhere it appears (here, §3 intro, §5, §7).

### 1.2 Hypothesis
- **GOAL:** state the proposal's directional hypothesis and that it is **not confirmed**.
- **INCLUDE (verbatim):** *"injecting measured ToF noise into simulation will reduce the sim-to-real
  gap"*; one sentence that the refutation is treated as a finding and the later phases test whether the
  null carries to manipulation.

### 1.3 Contributions  *(numbered list of 5)*
- **INCLUDE:** (1) measured ToF characterization (bulk sub-mm, edge flying-pixels dominate);
  (2) a 4-term injection module with axial-scale control; (3) **the controlled negative result**
  (no effect at ×1; degradation only ≈×10 pose / ×20 manip; injector verified); (4) end-to-end real
  validation (−1.4..+3.6 mm gap; 34/60 closed-loop; grip-mechanics finding); (5) a reproducible
  Gazebo/ROS 2 xArm-1S tabletop setup.
- **INCLUDE (scope paragraph):** the pick-and-place is the proposal's authorised **predefined-grasp
  baseline**; Phase-4C engineering is **methods/appendix, not a results contribution**.
- **PITFALLS:** contribution 3 must pair the null with the verification in the same breath.

### 1.4 Thesis Outline
- **GOAL:** one short paragraph mapping §2–§7.
- **PITFALLS:** keep tense/section numbers consistent with the final structure.

---

## 2. RELATED WORK  *(~1.5 columns; thematic funnel, ending at "we do X differently")*

### 2.1 The sim-to-real gap and domain randomization
- **INCLUDE:** reality gap origin; noise's role; domain randomization as blanket strategy that doesn't
  attribute the residual gap to a cause; **our complementary analytic route (isolate one cause)**.
- **CITE:** `jakobi1995noise`, `tobin2017domain`.

### 2.2 ToF and depth-sensor noise modelling
- **INCLUDE:** structured-light vs ToF differences; axial+lateral range/angle model; Kinect-class
  empirical precision/bias; flying/mixed pixels at discontinuities; framed as the **expected behaviour
  our Phase-2 data tests** (and partly refutes).
- **CITE:** `sarbolandi2015kinect`, `foix2011lockin`, `nguyen2012modeling`, `khoshelham2012accuracy`.
- **PITFALLS:** present Nguyen's range model as a *hypothesis to test*, not as adopted — Phase 2 finds
  it inapplicable (flat, negative curvature).

### 2.3 Tabletop perception and pose estimation
- **INCLUDE:** classical RANSAC-plane + clustering pipeline; **our deliberate simplification** to a
  depth threshold for a known fronto-parallel plane (cite RANSAC only as contrast); BOP's
  visible-top-surface alignment (we use *only* that, not ADD/VSD — pose is translation-only).
- **CITE:** `rusu2008towards` (CONTRAST only), `hodan2018bop` (alignment convention only).
- **PITFALLS:** never imply you ran RANSAC or computed ADD. The estimator is median-depth + 2D CC.

### 2.4 Fiducial markers for ground truth
- **INCLUDE:** ArUco robustness; ChArUco board for the camera→table extrinsic via direct solvePnP;
  per-object markers for GT.
- **CITE:** `garridojurado2014aruco`, `romeroramirez2018charuco`.
- **PITFALLS:** do NOT cite AprilTag (`olson2011apriltag`, EXCLUDED) unless you explicitly justify
  choosing ArUco over it.

### 2.5 Grasping
- **INCLUDE:** data-driven grasp synthesis survey; learned multi-view grasping as host-lab context;
  **our scope: a fixed predefined grasp for known objects, used only to test pose-error propagation**.
- **CITE:** `bohg2014grasp`, `kasaei2021mvgrasp` (the supervisor's lab work — include it).
- **PITFALLS:** do NOT cite learned-grasping methods you didn't implement (`kumra2020antipodal`,
  EXCLUDED) except as a named alternative.

---

## 3. SYSTEM AND METHODS  *(longest section; ~2.5–3 columns)*

### 3.1 Simulation and robot model
- **INCLUDE:** Gazebo Classic 11 + ROS 2 Humble; ros2_control (JTC arm + effort/PID gripper) via
  vendored `gazebo_ros2_control`; arm fixed to world at **z = 0.76 m**; xArm 1S as a **5-DOF** chain
  (base yaw + planar 3R + wrist roll) + parallel-jaw gripper, tool point **+0.09 m**; URDF from GrabCAD
  (→ Appendix A); OAK-D SR PoE ToF overhead at **0.68 m**, RGB-D 640×480, ~60° FOV, DepthAI, **factory
  intrinsics (no Zhang calibration)**.
- **ASSETS:** F10 (system overview), F4 (clean vs noisy depth) optionally here.
- **CITE:** `koenig2004gazebo`, `macenski2022ros2`, `gazebo_ros2_control`, `luxonis_oakd`,
  `depthai_docs`.
- **PITFALLS:** say **5-DOF** and footnote the proposal's "6-DoF" mislabel (reconcile vs `xarm.urdf`);
  state factory intrinsics explicitly (this is why Zhang is NOT cited).

### 3.2 Analytic inverse kinematics
- **INCLUDE:** planar-3R ⇒ closed-form analytic IK in NumPy (no MoveIt/KDL except optional FK check);
  removes solver convergence as a confound; `feasible_approach` searches steepest reachable slanted
  approach (tilt ~88° near base → 70–80° far) because a vertical top-down grasp is infeasible at most
  reaches.
- **CITE:** `craig2005robotics` (ch.4).
- **PITFALLS:** the far-reach tilt degradation must be introduced here because it explains the
  Phase-4C near-zone limit later.

### 3.3 Real ToF noise characterisation
- **INCLUDE:** filters FPPN/optical/wiggle **ON**, temporal/median **OFF** (expose raw noise); 200
  frames/capture; two protocols — **planar** (matte fronto-parallel target, 5 distances 0.40–0.80 m,
  per-pixel temporal σ + bias) and **object** (body vs 6-px edge ring).
- **NUMBERS:** n ≈ 33 920 px/distance.
- **ASSETS:** none here (results in 5.1); protocol diagram optional.
- **CITE:** `khoshelham2012accuracy` (planar protocol), `foix2011lockin`/`sarbolandi2015kinect`
  (edge motivation), `nguyen2012modeling`/`khoshelham2012accuracy` (models tested).
- **PITFALLS:** describe the 6-px ring as a *fixed-width* choice (size-sensitive) — a stated caveat.

### 3.4 Depth-noise injection model
- **INCLUDE:** added to the **clean simulated depth before perception** (no Gazebo internals touched);
  four terms — axial Gaussian σ_z(z) (Nguyen Eq.3), lateral jitter **0.8 px (literature value, not
  separately measured)**, edge mixed-pixels (Foix/Sarbolandi), dropout (Sarbolandi, **measured 0**);
  axial_scale ∈ {1,2,5,10,20}; seeded RNG; three named conditions `sim_clean`, `sim_noisy_oakd`,
  `sim_noisy_nguyen`.
- **ASSETS:** F4 (clean / ×1 / ×10 depth).
- **CITE:** `nguyen2012modeling`, `sarbolandi2015kinect`, `foix2011lockin`, `tobin2017domain` (seeded
  ablation spirit).
- **PITFALLS:** disclose the lateral jitter is a literature value (also a limitation in §6.6).

### 3.5 Perception pipeline
- **INCLUDE:** HSV masks **label only**, never geometry; **depth table-removal** (≥5 mm closer than
  median valid depth = object) **deliberately replacing RANSAC**; `cv2.connectedComponents` → one blob;
  **per-axis median centroid** → table-frame position; **translation-only (3-DoF), orientation =
  identity**; greedy nearest-neighbour GT matching (NOT Hungarian).
- **INCLUDE (the crux sentence):** the per-axis median has a **50 % breakdown point** and discards the
  53–99 % edge flying-pixel population — *this is why realistic noise is invisible to the estimator*.
- **ASSETS:** F12 (perception strip).
- **CITE:** `bradski2000opencv` (OpenCV ops), `huber1981robust` (median breakdown — **must be added to
  references.bib**), `rusu2008towards` (contrast for the replaced RANSAC).
- **PITFALLS:** ⚠️ **code-honesty:** the `pose_estimator.py` docstring still says "RANSAC plane
  segmentation (Open3D)" — fix the code so it matches this text. Do NOT cite `fischler1981ransac` or
  `zhou2018open3d` (EXCLUDED). Do NOT cite `kuhn1955hungarian` (you use greedy).

### 3.6 Grasp pipeline and feasibility metric
- **INCLUDE:** fixed predefined per-object nominal grasp angle (identical sim & real); **analytic
  `grasp_feasible`** metric (perceived centre within jaw capture tol of true centre) because Gazebo
  Classic can't reliably contact-grasp small/curved objects (fingers pass through; pose bit-identical
  before/after close); renamed from `pick_success` to avoid overclaiming; MIT contact-attach plugin
  (PAL/IFRA pattern) **for visualisation only**.
- **CITE:** `bohg2014grasp` (predefined-grasp positioning), `link_attacher`.
- **PITFALLS:** be explicit the metric is geometric, an **upper bound** on physical grasp; never
  compare it naively to the real 4C rate.

### 3.7 Real-world extrinsic calibration (Phase 4B)
- **INCLUDE:** fiducials on **RGB (CAM_B)** not ToF amplitude (soft amplitude gave 0/24 corners),
  composed into depth via factory RGB↔ToF extrinsic; camera→table from a **7×5 ChArUco board
  (DICT_5X5_100, 100 mm squares)** by **direct solvePnP, NO hand-eye**; **20/24 corners @ 0.26 px**;
  board height **0.683 m** vs depth-plane **0.684 m** (**1.4 mm** agreement).
- **ASSETS:** F8 (extrinsics overlay).
- **CITE:** `garridojurado2014aruco`, `romeroramirez2018charuco`, `foote2013tf` (tf2),
  `bradski2000opencv` (solvePnP).
- **PITFALLS:** stress **no hand-eye / no Zhang** (so `tsai1989handeye`, `zhang2000calibration` stay
  EXCLUDED).

### 3.8 Real arm control and closed-loop visual servo  *(baseline/methods — keep it that way)*
- **INCLUDE:** USB-HID bridge (VID 0x0483/PID 0x5750), ROS-free Python; the two real-to-model
  mismatches — gripper opens **55 mm** (URDF 72 mm) and the arm **over-reaches its model ~27–28 %**;
  global similarity-scale (~0.777) leaves **~14 mm RMS** position-dependent residual (an
  **unidentifiable** kinematic error, so no fixed offset/low-param recal fixes it workspace-wide); the
  **closed-loop fix** — gripper ArUco (DICT_4X4_50, id 4; objects 0–3), `err = obj − (marker + tip)`,
  `target += K·err` (K≈0.6) until `‖err‖<3 mm`, then descend+close; **common-mode cancellation** makes
  aim **camera-bound ~1–4 mm**, model-independent; a REPLAB-style dense correction field was prepared
  as a citable fallback but **not needed**.
- **ASSETS:** F11 (servo schematic — make it).
- **CITE:** `yang2019replab` (REPLAB — **must be added to references.bib**), optionally `foote2013tf`.
- **PITFALLS:** keep the framing "this is engineering so the *recipe* matches sim; only its *outcome*
  is a result." Don't narrate the debugging saga (→ §6.4 one paragraph + Appendix D).

---

## 4. EXPERIMENTAL SETUP  *(~1 column)*

### 4.1 Hardware and compute
- **INCLUDE:** xArm 1S + overhead OAK-D at 0.68 m; **Windows/WSL split** (ROS 2 + perception + analysis
  in WSL; real camera DepthAI + real arm USB-HID on Windows, because the PoE link + USB-HID aren't
  cleanly reachable from WSL).
- **CITE:** none new.

### 4.2 Objects
- **INCLUDE:** the 4 objects span grasp difficulty; jaw capture tol `(55−w)/2`.
- **ASSETS:** **T1** (objects table).
- **NUMBERS:** coke 52 mm/1.5 mm tol; banana 35 mm/10 mm; golf 43 mm/6 mm; box 40 mm/7.5 mm.
- **PITFALLS:** footnote 55 mm = measured real opening.

### 4.3 Conditions
- **INCLUDE:** `sim_clean`, `sim_noisy_oakd`, `sim_noisy_nguyen`; axial_scale sweep {1,2,5,10,20}, ×1 =
  measured.

### 4.4 Trials and metrics
- **INCLUDE:** Phase 3 ≈ 435 frames/condition, 100 % detection; Phase 4A = **480 analytic trials**
  (4 objects × 20 placements × 6 conditions, N=20/cell, annulus r 0.12–0.30 m, can lying); Phase 4B = 5
  scenes → **n=4 usable/object**; Phase 4C = 4 objects × 5 placements × 3 = **60 trials**. Metrics:
  translation RMSE/MAE (mm), analytic grasp-feasible %, real lift-and-hold with **Wilson 95 %**;
  BOP visible-top-surface alignment; greedy NN association.
- **CITE:** `hodan2018bop` (alignment).
- **PITFALLS:** state n=4 honestly here (it's a limitation, not hidden).

---

## 5. RESULTS  *(~2.5 columns; report numbers, interpret lightly — save deep interpretation for §6)*

### 5.1 Real ToF noise characterisation (SQ1)
- **INCLUDE:** axial σ sub-mm and flat (no Kinect range growth); fit `a=0.91 mm, b=−4.9e-4` (negative
  ⇒ Nguyen range model **does not apply**); bulk 0.6–0.7 mm (vs Kinect 2–10 mm); bias ±3 mm; edge
  flying-pixel **53–99 %**; the 3 caveats (box still 94.9 %; coke low = size; 6-px ring size-sensitive).
- **ASSETS:** **T2**, **T3**, **F2** (σ-vs-z), **F3** (object edge overlays).
- **NUMBERS:** σ 0.87/0.80/0.68/0.55/0.69; body σ 0.70/0.62/0.65/0.71; edge 53.1/98.7/98.2/94.9 %.
- **CITE:** `nguyen2012modeling` (model refuted), `khoshelham2012accuracy`, `foix2011lockin`,
  `sarbolandi2015kinect`.

### 5.2 Noise injection and pose accuracy (SQ2)
- **INCLUDE:** ×1 indistinguishable from clean and from nguyen; planar RMSE flat **~3.5 mm** through
  ×10; only the 3D (Z) component creeps; planar Δ(clean→×1) **< 0.1 mm**.
- **ASSETS:** **T4**, **F5** (pose RMSE vs scale).
- **CITE:** `nguyen2012modeling`.
- **PITFALLS:** keep "< 0.1 mm" exact — it's the headline.

### 5.3 Axial-scale ablation
- **INCLUDE:** degradation only ≈×10 (σ≈9 mm); all four collapse at ×20 — **the verification that the
  null is real**, an order of magnitude above reality.
- **ASSETS:** same **T4**/**F5** (it is the ablation).
- **PITFALLS:** explicitly tie ×20 collapse → "injector works → null is sound."

### 5.4 Simulated manipulation (SQ3, simulation)
- **INCLUDE:** all 4 detected every trial (accuracy not detection is the bottleneck); 3 graspable
  objects flat clean→×10, collapse at ×20; per-object governed by clean error vs jaw tol (box 3.5/7.5
  → 100 %; golf 5.2/6.0 → 65 %; banana 9.3/10 → 65 %; coke 4.9/1.5 → 0 %, a **size** floor).
- **ASSETS:** **T5**, **F6** (rate vs scale), **F9** (optional perception error).
- **CITE:** none new.
- **PITFALLS:** coke 0 % is a size limit, **not** a noise effect — say so.

### 5.5 Real pose gap and real manipulation (SQ3, real)
- **INCLUDE (5.5a pose gap):** real-vs-sim planar gap **−1.4 to +3.6 mm**; `sim_clean ≈ sim_noisy_oakd`
  on real data; blue_box +3.6 mm is the one positive outlier (matters for grasping). **Add the
  §D-note-1 sentence** explaining sim_clean here = varied-placement sweep (≠ Table 4).
- **INCLUDE (5.5b real manip):** 60 trials closed-loop; blue 11/15 (73 %), banana 10/15 (67 %), coke
  10/15 (67 %), golf 3/15 (20 %); overall **34/60 = 57 %**; rates follow **neither** the 4A ceiling nor
  the 4B predictor, diverging in **both** directions; aim driven to camera floor (~1–3 mm) ⇒ residual =
  **grip mechanics**; coke 0 %→67 % (firm close on compliant near-jaw-width can); golf 65 %→20 % (sphere
  rolls out of flat jaws); blue_box 9/9 = 100 % in the near zone (r≤0.20 m), every failure at r≥0.22 m.
- **ASSETS:** **T6**, **T7**, **T8**, **F7** (sim-to-real gap), **F1** (real vs predicted), F13 (opt).
- **NUMBERS:** all of the above verbatim from `phase4b_gap_table.csv`, `phase4c_real_table.csv`,
  `real_manip_4c_formal.csv`.
- **CITE:** none required (interpretation citations go in §6).
- **PITFALLS:** report operator-scored success honestly; don't let 4C engineering read as a result —
  only the *rate* is the result.

---

## 6. DISCUSSION  *(~1.5–2 columns; this is where citations interpret)*

### 6.1 A valid negative result, not a broken pipeline
- **INCLUDE:** restate null (Δ<0.1 mm pose, no grasp change); refutes proposal hypothesis; sound
  because the same injector breaks perception at ×10 / manip at ×20; structural reason (edge noise +
  median).
- **CITE:** (optional) none, or `nguyen2012modeling`.

### 6.2 Why the estimator absorbs realistic noise
- **INCLUDE:** edge flying-pixels (53–99 %) vs quiet body; median 50 % breakdown discards them; axial
  Gaussian perturbs body pixels the median already handles; reframes Nguyen's range model as
  inapplicable to this multi-frequency sensor.
- **CITE:** `huber1981robust`, `nguyen2012modeling`, `foix2011lockin`, `sarbolandi2015kinect`.

### 6.3 Where the gap actually lives
- **INCLUDE:** Phase-4B puts the end-to-end gap at −1.4..+3.6 mm; residual = calibration + lighting +
  segmentation, not sensor stochasticity (single solvePnP, factory intrinsics, illumination-sensitive
  boundary); echoes the DR lesson that the dominant transfer factors aren't always the randomised ones.
- **CITE:** `tobin2017domain`, `jakobi1995noise`.

### 6.4 Manipulation: the real bottleneck is kinematic, and closing the loop removes it
- **INCLUDE (one honest paragraph of the 4C story):** sim feasibility = perceived-centre vs jaw tol;
  on the real arm the dominant error is the **~27–28 % kinematic over-reach** (~14 mm residual > box
  tol) → ~4 cm open-loop zone; the visual loop cancels it as common-mode → camera-bound aim → blue_box
  9/9 in-zone, 11/15 overall; **then** grip mechanics dominate (coke up, golf down); consistent with
  the grasp literature's reachability-vs-stability distinction.
- **CITE:** `bohg2014grasp`, `yang2019replab`, `jakobi1995noise`.
- **PITFALLS:** compress, don't narrate; keep "engineering = baseline."

### 6.5 Simulation realism and the analytic metric
- **INCLUDE:** Gazebo Classic can't contact-grasp small/curved objects; analytic feasibility is an
  **upper bound**; renamed to avoid overclaiming; the attach plugin is visualisation-only.
- **CITE:** `koenig2004gazebo`, `link_attacher`.

### 6.6 Limitations
- **INCLUDE (each one sentence):** analytic sim metric; noise model omits spatial correlation +
  multipath, lateral jitter is a literature 0.8 px; real perception n=4/object; real manip near-zone
  only (marker-mount issue); Phase-4A CIs Wald (Wilson is more honest); 5-DOF reconciliation; Gazebo
  Classic EOL Jan 2025. End with: **none of these affect the central negative result** (independent in
  Phases 2–3, corroborated in 4B).
- **CITE:** `nguyen2012modeling` (jitter value).

---

## 7. CONCLUSION AND FUTURE WORK  *(~0.75 column)*

### 7.0 Conclusion
- **HOUSE STYLE:** open with a **"Summary of Main Contributions"** paragraph (Kasaei's 6.1 pattern) —
  restate the 5 contributions compactly — then the synthesis below.
- **INCLUDE:** answer the main RQ (for a robust median-centroid estimator, modelling realistic ToF
  noise does **not** measurably reduce the gap); recap the 5-phase chain; SQ1/SQ2/SQ3 answers;
  synthesis (gap = calibration/lighting/segmentation; on the real arm, open-loop kinematic error is the
  single largest source, removed by one visual correction).
- **NUMBERS:** <0.1 mm; ×10/×20; −1.4..+3.6 mm; 34/60 (57 %); 100 % in-zone blue_box.

### 7.1 Future work
- **INCLUDE:** gripper better matched to round/near-jaw objects (compliant pads/cradle); remount marker
  low for full-reach convergence; re-solve arm-base calibration with more points or a learned
  correction field (REPLAB); extend noise model to spatial correlation + multipath; compare the median
  baseline to a learned estimator (MVGrasp); a contact-capable physics backend to validate the analytic
  metric against scored physical grasps.
- **CITE:** `yang2019replab`, `kasaei2021mvgrasp`.

---

## REFERENCES
- **APA author-year (`apacite`, template default).** Only PART 1–3 keys (the two additions
  `huber1981robust`, `yang2019replab` are now in the `.bib`). The corrected `references.bib` is already
  applied; see `THESIS_REFERENCE_AUDIT.md` for what changed and the **EXCLUDED** list to never cite.
  ⚠️ Fill the `grabcad_xarm` author (uploader name) before final compile — it is a cited entry.

## APPENDICES
- A Assets & Attribution · B Declaration of Originality / AI use · C Reproducibility & Repository ·
  D (opt) Servo detail · E (opt) full 60-trial log. See `THESIS_APPENDIX_PLAN.md`.

---

### FINAL PRE-SUBMISSION CHECKLIST (run against the compiled PDF)
- [ ] Every number traces to a CSV/JSON (spot-check T4, T6, T7).
- [ ] No PART-4 EXCLUDED key cited anywhere (grep the `.tex` for each key).
- [ ] `huber1981robust` + `yang2019replab` present in `references.bib` and compile without `?`.
- [ ] `pose_estimator.py` RANSAC/Open3D docstring fixed (code-honesty).
- [ ] 5-DOF stated; proposal's "6-DoF" reconciled in a footnote.
- [ ] GrabCAD provenance closed in Appendix A; CC-BY strings for coke + banana pasted verbatim.
- [ ] T6-vs-T4 sim_clean discrepancy explained in §5.5.
- [ ] Declaration + AI-use disclosure **names the specific tool(s) + version** and the purposes
      (UG policy Rule 2.1/2.2); signed/dated. Confirm with Kasaei if a separate signed form is required.
- [ ] Figure count 7–9, table count 8; all figures referenced from the text; captions self-contained.
- [ ] Citation style = **APA `apacite`** (decided — template default, no swap). Numeric `[n]` cites
      converted to author-year; number-leaning prose reworded. Programme = BSc AI. **No second assessor.**
- [ ] Compiled in the official RUG `bachelorproject.sty` two-column template; Acknowledgements added.
- [ ] `references.bib` corrected (applied 2026-06-27); `grabcad_xarm` uploader name filled before compile.
