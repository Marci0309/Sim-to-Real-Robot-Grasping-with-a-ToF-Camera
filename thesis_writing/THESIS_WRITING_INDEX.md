# Thesis Writing — Master Index & 2-Day Plan

Five planning deliverables were produced (2026-06-27) to take *Investigating the Sim-to-Real Gap in
RGB-D Based Tabletop Pose Estimation and Manipulation* from its near-complete 11-page draft
(`_0_5in.pdf`) to a submission-ready thesis. Read them in this order.

| # | File | What it gives you |
|---|---|---|
| 1 | **[SUPERVISOR_FORMAT_FINDINGS.md](SUPERVISOR_FORMAT_FINDINGS.md)** | Evidence-backed answer to "what format does Kasaei want": IEEE (not APA), official two-column `bachelorproject.sty`, his exact chapter pattern, example theses to copy, AI-disclosure rules. |
| 2 | **[THESIS_SECTION_INSTRUCTIONS.md](THESIS_SECTION_INSTRUCTIONS.md)** | The "verify your doing" spec — every subsection's GOAL / content checklist / exact numbers / which tables-figures / allowed citations / pitfalls + a pre-submission checklist. |
| 3 | **[THESIS_PLOTS_AND_TABLES.md](THESIS_PLOTS_AND_TABLES.md)** | Every table (8) + figure mapped to its CSV source and regen command; the 7 ready-made figures to add; the 3 figures you must make. |
| 4 | **[THESIS_REFERENCE_AUDIT.md](THESIS_REFERENCE_AUDIT.md)** | Web-verification of all 20 references (zero hallucinations) + the fixes, the 2 missing entries to add, usage-overclaim flags, asset licenses, and the EXCLUDED-list confirmation. |
| 5 | **[THESIS_APPENDIX_PLAN.md](THESIS_APPENDIX_PLAN.md)** | What goes in the appendices (A Assets, B Declaration, C Reproducibility, opt. D/E) and an explicit DO-NOT-INCLUDE list. |

Existing inputs they build on: `THESIS_HANDOFF_BRIEF.md` (full project record), `draft_results55_discussion.tex`
(paste-ready §5.5 + §6), `VISUALIZATION_COOKBOOK.md` (figure regen), `../thesis_writing/references.bib`,
`../ATTRIBUTION.md`, and the result CSVs in `../evaluation/results/`.

---

## The big picture (what I found)
- **Your "mock-up" is actually a strong, near-complete IEEE draft.** These docs are a *review + fix +
  polish* checklist against it, not a from-scratch guide. The hard writing is largely done.
- **Format (decided):** **APA `apacite`** (template default — no `.bst` swap; convert the draft's `[n]`
  cites to author-year); port into the official RUG two-column template; add Acknowledgements; consider a
  "Summary of Main Contributions" lead-in to the Conclusion (Kasaei's house style). No second assessor.
- **References:** **no hallucinations** — all 20 are real. But 2 cited works are missing from the .bib
  (Huber, REPLAB), ~6 have metadata errors, 3 have wording-level overclaim risks, and `pinto2024flexx2`
  has a fake author ("Pinto") and must be fixed or dropped.
- **Figures:** the draft has 1 figure but **7 strong figures already sit unused on disk** — adding them
  is the highest-value, lowest-effort win.
- **One results trap:** Table 6 "sim_clean" ≠ Table 4 "clean" (different placement sets) — must be
  explained in §5.5 or an examiner will flag it.

---

## Recommended 2-day execution order
**Day 1 — structure, references, figures (the mechanical wins):**
1. Set up the official RUG BSc-AI Overleaf template; **keep `apacite` (APA)** — no `.bst` swap; paste in
   your draft and **convert the `[n]` cites to author-year** (apacite `\cite`/`\citeA`).
2. ✅ `references.bib` fixes already applied (Huber + REPLAB added; nguyen/hodan/macenski/link_attacher
   fixed; pinto→`cai2024flexx2`; coke/banana CC-BY + GrabCAD URL filled). **You:** fill the `grabcad_xarm`
   uploader name; then recompile and verify the list renders from the .bib (not the old hand-typed list).
3. Generate/refresh all figures (`THESIS_PLOTS_AND_TABLES.md` §B regen commands) and place the 7
   ready-made ones (F2, F3, F5, F7 + the others). Make F11 (servo schematic) and F10 (system) — F12 is
   reusable from the deck.
4. Add the §5.5 sentence explaining the Table 6-vs-Table 4 sim_clean difference.

**Day 2 — prose polish, honesty, appendices, submit:**
5. Walk the whole draft against `THESIS_SECTION_INSTRUCTIONS.md`, ticking each INCLUDE item; reword the
   3 overclaim risks (Tobin ablation, REPLAB correction-field, Khoshelham 5-vs-10 distances).
6. Code-honesty fixes: `pose_estimator.py` RANSAC/Open3D docstring; 5-DOF footnote; close the GrabCAD
   provenance TODO.
7. Finalize appendices per `THESIS_APPENDIX_PLAN.md` (A Assets w/ CC-BY strings, B Declaration naming the
   AI tool+version, C Reproducibility). Add Acknowledgements.
8. Run the pre-submission checklist at the end of `THESIS_SECTION_INSTRUCTIONS.md`. Compile, proofread,
   submit.

**Open items (mostly closed):** ✅ no second assessor; ✅ GrabCAD URL filled (https://grabcad.com/library/hiwonder-x-arm-1).
Remaining: read the **GrabCAD uploader name** off that page into the `.bib`, and (optional) confirm with
Kasaei whether the programme wants a separate signed AI declaration — your Appendix B already includes one.
