================================================================================
 THESIS OVERLEAF PROJECT  —  self-contained, compiles out of the box
 "Investigating the Sim-to-Real Gap in RGB-D Based Tabletop Pose Estimation
  and Manipulation"  —  M. M. Szokedencsi (s5173191), RUG BSc AI
================================================================================

WHAT THIS IS
  A complete, compilable LaTeX project containing the full thesis: abstract, all
  7 sections, 8 tables, 9 figures, the bibliography (APA), and Appendices A/B/C.
  It is deliberately SELF-CONTAINED (standard two-column `article` + apacite +
  booktabs + graphicx + subcaption + siunitx) so it builds in Overleaf with no
  extra setup. You can later merge it into the official RUG `bachelorproject.sty`
  template if you wish (see "PORTING" below).

HOW TO USE IN OVERLEAF
  1. overleaf.com -> New Project -> Upload Project -> select this .zip.
  2. Menu (top-left) -> Compiler: pdfLaTeX.  Main document: main.tex.
  3. Click "Recompile".  (Overleaf runs pdfLaTeX -> BibTeX -> pdfLaTeX x2.)
  That's it — you should see all plots, tables, and appendices.

FILES  (proper multi-file project — one chapter per .tex, included via \input)
  main.tex                  preamble, title, abstract, \input of the chapters, bibliography
  sections/                 one file per chapter:
                              01_introduction.tex   02_related_work.tex
                              03_methods.tex        04_experimental_setup.tex
                              05_results.tex        06_discussion.tex
                              07_conclusion.tex
  appendix/                 one file per appendix:
                              A_assets.tex  B_declaration.tex  C_reproducibility.tex
  references.bib            corrected, verified bibliography (APA/apacite)
  figures/                  all 15 figures (PNG); the thesis uses 13 of them
  README.txt                this file
  Edit a chapter in its own file under sections/ ; main.tex just wires them together.
  (TikZ vector versions of the two schematics live in the repo at
   src/thesis_writing/thesis_schematics_tikz.tex — optional; the PNGs here are the
   authoritative, reviewed versions and need no compilation setup.)

CITATION STYLE
  APA author-year via apacite (the official RUG BSc-AI template default) — your
  choice. In text: \cite{key} -> (Author, year); \citeA{key} -> Author (year).
  To switch to IEEE numbered instead: remove \usepackage{apacite}, set
  \bibliographystyle{IEEEtran}, and replace \citeA{} with \cite{} (IEEEtran has
  no \citeA). The .bib entries work for either style unchanged.

FIGURES — status
  Included & used (13): system_pipeline, servo_schematic, clean_vs_noisy_depth,
    sigma_vs_z, {coke_can,banana,golf_ball,blue_box}_overlay, phase3_pose_vs_noise,
    phase4a_rate_vs_axial_scale, phase4b_sim_to_real_gap, phase4c_real_vs_pred,
    extrinsics_overlay (the 7x5 ChArUco extrinsic board imaged by the OAK-D RGB
    camera, Methods 3.7).
  Staged but NOT placed (optional adds): bias_vs_z (Phase-2 bias),
    phase4a_perception_error (per-object error-vs-tolerance scatter). Add with
    \includegraphics if you want them.
  The two schematics (system_pipeline, servo_schematic) were authored here and
    reviewed by a panel of agents; servo_schematic passed "ship". The four Phase-2
    object overlays are usable but the object is small in frame — for sharper print
    you can re-crop them via analyze_object_noise.py (optional).

STILL ON YOUR PLATE (small)
  - Provide the F12 "perception pipeline strip" figure (a live screenshot of
    live_perception_view.py or deck slide 6) if you want it in Methods 3.5.
  - Optional: add Acknowledgements; confirm with Dr. Kasaei whether the programme
    wants a separate signed AI declaration (Appendix B already includes one).
  - The bibliography compiles from references.bib; verify the rendered list once.

WHAT WAS ALREADY FIXED FOR YOU (vs the original draft)
  - APA citations throughout; numeric [n] converted to \cite/\citeA.
  - references.bib corrected & verified (no hallucinations): added Huber 1981 and
    REPLAB (Yang 2019); fixed Nguyen/BOP/Macenski/link_attacher metadata; replaced
    the fake "pinto2024flexx2" with the real cai2024flexx2; filled the coke/banana
    CC-BY and the GrabCAD (Yair Casas) asset entries.
  - Honesty fixes: the seeded ablation is presented as our own design (not Tobin);
    REPLAB cited for the low-cost-arm idea, not as the origin of a correction field;
    Khoshelham's protocol phrased as the practice we follow.
  - The Table-6 vs Table-4 "sim_clean" difference is explained in Section 5.5.
  - 5-DOF stated with a reconciling footnote; the analytic grasp metric is named
    honestly (grasp_feasible).

PORTING TO THE OFFICIAL RUG TEMPLATE (optional)
  Open the official "University of Groningen - BSc AI - Bachelor Thesis Template"
  on Overleaf, then move the body of main.tex (everything from \section{Introduction}
  to before \bibliography) into the template's section files, copy references.bib
  and figures/, and keep the template's \bibliographystyle{apacite}.
================================================================================
