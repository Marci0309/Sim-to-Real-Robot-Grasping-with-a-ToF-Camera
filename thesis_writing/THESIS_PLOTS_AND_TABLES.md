# Thesis — Plots & Tables Plan

**Thesis:** *Investigating the Sim-to-Real Gap in RGB-D Based Tabletop Pose Estimation and Manipulation*
**Author:** M. Szokedencsi (s5173191) · **Supervisor:** Dr. H. Kasaei · RUG BSc AI · IEEE two-column

> Purpose: a single checklist of **every table and figure** the thesis needs, what data feeds
> each, whether the asset already exists, and the exact command to (re)generate it. Work top to
> bottom; tick `[x]` as each is placed in Overleaf.
>
> **Status legend:** ✅ EXISTS (file on disk, drop straight in) · ♻️ REGEN (re-run a script to
> refresh from current CSVs) · 🆕 MAKE (no asset yet — you must create/screenshot it).
>
> The current 11-page draft (`_0_5in.pdf`) already has **8 tables + 1 figure**. It is
> **table-heavy and figure-poor**: 7 strong figures already sit on disk unused. Adding them is the
> single highest-value, lowest-effort improvement — they are in the defense deck and persuasive.

All paths are relative to `~/thesis_ws/src/`. Regen environment: **WSL `python3`** unless noted.
Regen commands are copied from `thesis_writing/VISUALIZATION_COOKBOOK.md` (verified against code).

---

## A. TABLES (8 — all already in the draft; verify data + 2 fixes)

| # | Table (caption gist) | Section | Data source (exact) | Status | Action / note |
|---|---|---|---|---|---|
| **T1** | The four test objects: geometry, width, height, jaw tol `(55−w)/2`, character | 4.2 Objects | hand-built; widths/tols from `THESIS_HANDOFF_BRIEF.md` PART 1 | ✅ in draft | Numbers correct. Footnote that 55 mm = **measured real** gripper opening (URDF was 72 mm). |
| **T2** | Planar axial precision σ vs distance (0.40–0.80 m) | 5.1 (SQ1) | `tof_perception/scripts/fit_results_truly_raw/noise_params.json` | ✅ in draft | σ = 0.87/0.80/0.68/0.55/0.69 mm; add fit `a=0.91 mm, b=−4.9e-4`, n≈33 920 px/dist. |
| **T3** | Per-object body σ, dropout, edge flying-pixel rate | 5.1 (SQ1) | `tof_perception/scripts/object_results/` (per-object `object_noise.json`) | ✅ in draft | coke 0.70/0%/53.1; banana 0.62/0%/98.7; golf 0.65/0%/98.2; box 0.71/0%/94.9. Keep the 3 caveats (6-px ring is size-sensitive; coke low = size; box still 94.9%). |
| **T4** | Pose RMSE (planar / 3D) by object × axial scale {0,×1,×5,×10,×20} | 5.2 + 5.3 (SQ2) | `evaluation/results/phase3_pose_table.csv` | ✅ in draft | Matches CSV exactly. Optionally also show the `nguyen` row (3.6/8.7/4.7/3.6) to make "measured ≈ literature ≈ clean" explicit. |
| **T5** | Analytic `grasp_feasible` success % (N=20/cell), jaw tol in header | 5.4 (SQ3 sim) | `evaluation/results/phase4a_success_table.csv` | ✅ in draft | Matches. Caption must say metric = **analytic feasibility**, not physical contact. |
| **T6** | Real-vs-sim planar pose RMSE + gap (real − sim_clean) | 5.5 (SQ3 real) | `evaluation/results/phase4b_gap_table.csv` (+`.txt`) | ✅ in draft | ⚠️ **See §D note 1** — `sim_clean` here ≠ T4 clean (different placement set). Add one sentence so the examiner isn't blindsided. |
| **T7** | Phase-4C real lift-and-hold + sim-feas + pose-err/tol predictor + Wilson CI | 5.5 (SQ3 real) | `evaluation/results/phase4c_real_table.csv` (+`.txt`) | ✅ in draft | Matches. Order rows however 5.5 prose reads (draft uses blue/banana/coke/golf). |
| **T8** | Phase-4C blue-box servo convergence by placement (err before→after) | 5.5 (SQ3 real) | `evaluation/results/real_manip_4c_formal.csv` (blue_box rows) | ✅ in draft | Values from `note` column (`19.75->2.39mm`…). Confirms r≥0.22 m = every failure. |

**No new tables strictly required.** Optional adds (only if you want them): a *conditions* table (sim_clean/sim_noisy_oakd/sim_noisy_nguyen + axial_scale sweep) in §4.3 — but this is already prose and probably not worth a table at 2-column width.

---

## B. FIGURES — already on disk, NOT yet in the draft (add these — biggest win)

> Every file below already exists under `evaluation/results/` or `tof_perception/.../`. These are
> the figures from the defense deck. Pulling them into the thesis fixes the "figure-poor" problem.

| # | Figure (caption gist) | Section | File on disk | Status | Regen command |
|---|---|---|---|---|---|
| **F1** | Phase-4C real vs predicted (lift-success vs 4A ceiling; pose-err vs jaw tol) | 5.5 | `evaluation/results/phase4c_real_vs_pred.png` | ✅ (this is the draft's only figure) | `python3 evaluation/scripts/summarize_phase4c.py --csv evaluation/results/real_manip_4c_formal.csv --out-dir evaluation/results` |
| **F2** | Phase-2 axial σ vs distance (sub-mm, flat — Nguyen range model fails) | 5.1 | `tof_perception/scripts/fit_results_truly_raw/sigma_vs_z.png` | ✅ | `python3 tof_perception/scripts/fit_noise_model.py --in-dir ./tof_perception/scripts/captures_truly_raw --out-dir ./tof_perception/scripts/fit_results_truly_raw` |
| **F2b** | Phase-2 bias vs distance (within ±3 mm) | 5.1 | `tof_perception/scripts/fit_results_truly_raw/bias_vs_z.png` | ✅ | (same as F2) — *optional companion; or combine F2+F2b into one 2-panel figure.* |
| **F3** | Phase-2 object edge overlays — green body / **red** flying-pixel ring, per object | 5.1 | `tof_perception/scripts/object_results/{coke_can,banana,golf_ball,blue_box}_overlay.png` | ✅ | `python3 tof_perception/scripts/analyze_object_noise.py --in-dir ./tof_perception/scripts/object_captures --out-dir ./tof_perception/scripts/object_results` |
| **F4** | Clean vs noisy simulated depth (injector at clean / ×1 / ×10) | 3.4 or 5.2 | `evaluation/results/clean_vs_noisy_depth.png` | ✅ | `cd thesis_writing && python3 make_noise_demo.py` |
| **F5** | Phase-3 pose RMSE vs axial scale (flat to ×10, collapse at ×20) | 5.2/5.3 | `evaluation/results/phase3_pose_vs_noise.png` | ✅ | `python3 evaluation/scripts/summarize_phase3.py` |
| **F6** | Phase-4A grasp-feasible rate vs axial scale | 5.4 | `evaluation/results/phase4a_rate_vs_axial_scale.png` | ✅ | `python3 xarm1s_manipulation/scripts/summarize_4a_rates.py evaluation/results/manip_results.csv` |
| **F7** | Phase-4B sim-to-real pose gap (clean sim / noisy sim / real bars per object) | 5.5 | `evaluation/results/phase4b_sim_to_real_gap.png` | ✅ | `python3 evaluation/scripts/compute_sim_to_real_gap.py` |
| **F8** | ChArUco extrinsic overlay (board reprojection on the real scene) | 3.7 | `tof_perception/board_4b/extrinsics_overlay.png` | ✅ | `python3 tof_perception/scripts/solve_table_extrinsics.py --name board` |
| **F9** | Phase-4A perceived error vs jaw-tolerance rings (per object) | 5.4 | `evaluation/results/phase4a_perception_error.png` | ✅ (optional) | `python3 xarm1s_manipulation/scripts/plot_perception_error.py sim_clean` |

**Recommended minimum figure set** to fix the figure-poverty: **F1 (have it) + F2 + F3 + F5 + F7**, plus one
system/setup figure (F10) and the servo schematic (F11) below. That is ~7 figures for an 11-page two-column
thesis — appropriate density for a Kasaei-supervised perception thesis.

---

## C. FIGURES — need MAKING or capturing (no asset yet)

| # | Figure | Section | Status | How to make it |
|---|---|---|---|---|
| **F10** | **System pipeline:** clean-sim / noisy-sim / real → shared perception → IK → grasp, host-colour-coded | 3.1 / 4.1 | ✅ **MADE** | `evaluation/results/system_pipeline.png` (regen `python3 thesis_writing/make_thesis_schematics.py`) **+ TikZ** in `thesis_writing/thesis_schematics_tikz.tex`. *Optional complement:* a sim‖real **photo** composite (reuse `deck_qa/slide_01.png`/`slide_04.png` or screenshot Gazebo + photograph the rig — needs your images). |
| **F11** | **Closed-loop visual-servo schematic:** object + gripper markers seen by one camera; err vector; `target += K·err` loop; common-mode cancellation | 3.8 | ✅ **MADE** | `evaluation/results/servo_schematic.png` (regen as above) **+ TikZ** in `thesis_schematics_tikz.tex`. The Phase-4C "money" figure. |
| **F12** | Perception pipeline strip: RGB → table-removed depth → connected-component blobs → median centroid + grasp axis | 3.5 | 🆕 MAKE (reuse) | *Needs your images.* Reuse deck slide 6 panel, or screenshot `live_perception_view.py` ("FILTER" red blob + "CENTERS" green-X/magenta-axis windows) — see cookbook §0. |
| **F13** | *(optional)* Phase-4C reach-vs-outcome scatter: success/fail vs reach `r`, showing the r≤0.20 m near-zone cliff | 5.5 | 🆕 MAKE (optional) | Plot from `real_manip_4c_formal.csv`: x = reach r (from arm_x/arm_y), color = lift_success, per object. Strengthens the "all failures at r≥0.22 m" claim beyond blue-box T8. |

> **F11 + F10 are DONE** (PNG rendered + verified, plus paste-ready TikZ). Only **F12** still needs a
> screenshot/photo from you. F13 optional.

---

## D. Data-integrity notes to resolve BEFORE the figures/tables are final

1. **T6 "sim_clean" ≠ T4 "clean" — explain it.** Table 4 clean (coke 3.31, banana 8.71 mm) is the
   **fixed central** Phase-3 scene set; Table 6 sim_clean (coke 10.44, banana 13.27 mm) is the
   **varied-placement annulus sweep** (Phase-4A placements, r 0.12–0.30 m, off-centre + slanted
   views inflate planar RMSE). Same word "clean", different placement distribution. **Add one
   sentence in §5.5** (e.g. *"the simulated reference here is the varied-placement sweep of
   Section 5.4, whose wider pose spread raises planar RMSE relative to the central scenes of
   Table 4"*) so an examiner does not read it as an inconsistency. This is the most likely
   defense question on the results.
2. **Re-run all regen commands once** so every figure reflects the *current* clean models
   (slim coke + re-centred banana). The CSVs are already current; just confirm the PNGs were
   regenerated after 2026-06-07 (Phase-3 de-pollution) and 2026-06-18 (Phase-4C final run).
3. **Wilson vs Wald CIs:** Phase-4A (T5) intervals are Wald; Phase-4C (T7) are Wilson. If you add CIs
   to any 4A bar chart, say "Wald" in the caption (and note Wilson is the more honest choice — already
   a stated limitation). Don't silently mix.
4. **Figure legibility at 2-column width:** F2/F5/F6/F7 are line/bar plots — check axis font size after
   `\includegraphics[width=\linewidth]`. Regenerate with a larger `rcParams` font if cramped.
5. **Colour-blind safety:** F3 uses green/red (body/edge). Keep it (it's the deck's signature visual)
   but state "green = body, red = edge ring" in the caption so it survives greyscale printing.

---

## E. Figure → Section placement summary (the target layout)

- **§3 Methods:** F4 (injector), F8 (ChArUco), F11 (servo schematic), F12 (perception strip), F10 (system).
- **§5.1 (SQ1):** F2 (+F2b), F3.  · **§5.2/5.3 (SQ2):** F5 (and T4).  · **§5.4 (SQ3 sim):** F6 (and T5, F9 opt).
- **§5.5 (SQ3 real):** F7 (gap), F1 (real vs pred), (F13 opt), with T6/T7/T8.

Roughly **7–9 figures + 8 tables**. That is the right density; don't exceed it — two-column space is tight.
