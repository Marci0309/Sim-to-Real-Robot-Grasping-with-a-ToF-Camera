# Thesis — Appendix Structure (what to include, what to leave out)

**Principle for a 15-ECTS BSc thesis:** appendices hold material that is *necessary for
reproducibility or integrity* but would break the flow of the main text. They are **not** a dumping
ground. Keep them lean. The main argument (the negative result + the SQ3 chain) must stand entirely
on the 7 numbered sections; an examiner should be able to grade the thesis without opening an
appendix. Everything in an appendix must be *referenced from the main text* at least once
(`see Appendix X`) or it does not belong.

The current draft already has **Appendix A (Assets & Attribution)** and **Appendix B (Declaration of
Originality / AI use)**. Both are correct and should stay. Below is the recommended full set.

---

## ✅ INCLUDE

### Appendix A — Assets and Attribution  *(KEEP — already drafted; close one TODO)*
- 3D models (xArm URDF/STLs, coke, banana), simulation primitives (golf, box), and all third-party
  software with licenses. Source of truth: `src/ATTRIBUTION.md`.
- **Status:** coke (`sebastian204`, "Soda Can 250ml", **CC-BY-4.0**) and banana
  (`Lassi Kaukonen / @thesidekick`, "Banana", **CC-BY-4.0**) are **resolved** — paste the required
  CC-BY attribution strings verbatim (they are in `ATTRIBUTION.md` §"Required attribution").
- **One open action:** the **GrabCAD xArm 1S** model URL is now in the `.bib`
  (https://grabcad.com/library/hiwonder-x-arm-1); only the **uploader name** remains — read it off the
  page and confirm reuse terms. This is the *only* asset gap left.
- Keep the three "action items to close before submission" (incl. confirming 5-DOF vs the proposal's
  "6-DoF" against `xarm.urdf`).

### Appendix B — Declaration of Originality and Use of AI Tools  *(KEEP — already drafted; RUG requires it)*
- Originality statement + explicit AI-tool-use disclosure (LLM as writing/coding assistant; all
  experimental design, data, analysis, and results are the author's). Signature + date + student no.
- ⚠️ Confirm the **exact wording/format** the FSE/AI programme mandates for 2026 (the format-research
  findings file will say if there is a required template — adopt that wording over the draft's if so).

### Appendix C — Reproducibility and Repository  *(ADD — short, high value)*
- One paragraph + a small table: repo URL (`github.com/Marci0309/thesis_wsl`), the 6 first-party ROS 2
  packages, key entry-point scripts (noise injector, pose estimator, evaluators, summarizers), and the
  software stack versions (Gazebo Classic 11, ROS 2 Humble, DepthAI v3.5, Python).
- Point to the result CSVs that back each table (don't reprint them): `phase3_pose_table.csv`,
  `phase4a_success_table.csv`, `phase4b_gap_table.csv`, `phase4c_real_table.csv`,
  `real_manip_4c_formal.csv`. This is what makes the negative result auditable.
- Keep to ≤ half a column.

### Appendix D — Closed-loop visual servo: implementation detail  *(ADD only if §3.8 feels cramped)*
- The Phase-4C engineering is **explicitly baseline/Methods, not a contribution** (per the proposal
  and handoff brief). §3.8 should carry the *idea* (common-mode cancellation, `target += K·err`,
  `‖err‖<3 mm`); this appendix can hold the *mechanics* an examiner might ask about: the ArUco id
  scheme (object ids 0–3, gripper id 4, DICT_4X4_50), the gain/threshold/iteration flags, the
  Windows-camera ↔ WSL-detection subprocess split, and the offline self-test result (500/500 synthetic
  objects converge 9.2 mm → 1.85 mm in ~2.7 iters).
- **Do NOT** narrate the full multi-agent debugging saga (the 4 cm zone, the misdiagnosis trail). That
  lives in `THESIS_HANDOFF_BRIEF.md`; in the thesis it compresses to *one* honest paragraph in §6.4
  (the open-loop ~14 mm cap → close the loop). Keep the appendix factual, not a story.

---

## 🟡 OPTIONAL (include only if space and only if referenced)

- **Appendix E — Full Phase-4C per-trial log (60 rows).** The trial-level table with per-trial servo
  error and outcome notes (from `real_manip_4c_formal.csv`). Adds transparency for the 34/60 headline
  and the grip-mechanics claim (the notes literally say "rolled out", "pushed together until good
  grasp"). **Recommendation:** include as a compact table *only if* you have the page budget; otherwise
  point to the CSV in Appendix C. The blue-box subset is already T8 in the main text.
- **Appendix — Noise model parameters.** The measured YAML (`noise_params.json`): axial `a/b`, lateral
  jitter 0.8 px, `p_edge` 0.70, dropout 0. One small table. Useful for exact reproduction; skip if §3.4
  already states all four values inline (it largely does).

---

## ⛔ DO NOT INCLUDE

- **Raw source code listings.** Point to the repo (Appendix C). A BSc thesis is not a code dump.
- **Large raw data tables** (per-frame depth stats, all 480 Phase-4A trials, all 33 920-px distributions).
  Cite the CSV/JSON files instead.
- **The full engineering narrative / session notes** (`PHASE4C_NOTES.md`, `THESIS_STATE.md`, the
  19-agent / 10-agent investigation story). Internal artefacts — not thesis material.
- **The defense slide deck or speaker notes.** Different medium.
- **A glossary/acronym list** unless the programme template requires one — at 11 pages it is overhead;
  define ToF, RMSE, IK, DOF, FOV, IRL inline on first use instead.
- **Anything from the `references.bib` PART 4 "EXCLUDED" list** (Hungarian, hand-eye, Zhang, ADD,
  RANSAC/Open3D, AprilTag, learned grasping). These describe methods you did **not** implement;
  putting them anywhere (even an appendix) re-introduces the false-claim/plagiarism risk the audit
  is designed to prevent.
- **Proofs / derivations of the analytic IK.** Cite Craig ch.4; a full derivation is out of scope for
  a study whose contribution is the noise result, not the kinematics.

---

## Ordering & cross-references (checklist)
- [ ] Order: A (Assets) → B (Declaration) → C (Reproducibility) → D (Servo detail, if used) → E (opt).
- [ ] Every appendix is cited from the body (`Appendix A` from §3.1/§4.2; `Appendix C` from §4.1/§5;
      `Appendix D` from §3.8; `Appendix B` is referenced from the front matter / by faculty rule).
- [ ] Appendices come **after** the bibliography (IEEE style) unless the RUG template dictates otherwise
      — confirm against the template (see format-findings file).
- [ ] GrabCAD provenance TODO closed in A. · 5-DOF vs 6-DoF reconciled (A action item + §3.2 footnote).
