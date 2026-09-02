# Supervisor / RUG Format Findings — what Dr. Kasaei prefers

**How this was found:** three research agents inspected (a) the official RUG BSc-AI Overleaf template,
(b) the RUG/FSE + UG-wide rules, and (c) **actual theses Kasaei supervised** (opened the PDFs) plus his
own papers. Every claim below carries a source. Confidence is marked where it matters.

---

## 0. THE TWO DECISIONS — resolved

### Citation style: **APA author-year (`apacite`) — DECIDED (2026-06-27).**
- **The student chose APA** — the official RUG BSc-AI template's default (`\usepackage{apacite}` +
  `\bibliographystyle{apacite}`). Rationale: the verified IEEE-using exemplars were Kasaei's **MSc**
  theses; for the **BSc** the safest position is to follow the official template default unchanged.
- **What this costs you (do this):** the current draft uses numeric IEEE `[1]…[23]`. Converting to APA
  means: (1) keep `\usepackage{apacite}` + `\bibliographystyle{apacite}` as shipped (no `.bst` swap);
  (2) every in-text cite becomes **author-year** — `\cite{nguyen2012modeling}` →
  *(Nguyen et al., 2012)*, and `\citeA{nguyen2012modeling}` → *Nguyen et al. (2012)* for textual use;
  (3) **reword prose that leans on numbers** — *"as in [5]"* → *"as shown by Nguyen et al. (2012)"*,
  *"the literature [3, 4]"* → *"the literature (Sarbolandi et al., 2015; Foix et al., 2011)"*.
- The `.bib` entries themselves are **style-agnostic** — the corrected file works for APA as-is.
- *Evidence for the record (in case it ever comes up):* Kasaei's own papers and his **MSc** theses
  (Mein 2025, Zhang 2025, Parisotto 2021) use IEEE numeric; the BSc **template** defaults to APA. Both
  are defensible; you picked APA = template default. A one-line confirmation to Kasaei never hurts.

### Programme/template: **BSc Artificial Intelligence — official `bachelorproject.sty` template.** ✅
- Kasaei is AI faculty (Bernoulli Institute, Dept. of AI); your deck + draft already say BSc AI.
- Use the **official "University of Groningen – BSc AI – Bachelor Thesis Template"** (Overleaf, current
  March 2026): `\documentclass[10pt,a4paper,twocolumn]{article}` + `\usepackage{bachelorproject}`,
  single-column title block + abstract, then two-column body. License CC-BY-4.0.
- Activate RUG institutional Overleaf Pro (overleaf.com/edu/rug). Your draft is already two-column IEEE,
  so porting into this template is mostly the title block + `.bst` swap.

---

## 1. Kasaei's house chapter structure (from his real theses)

Verified identical across **Mein 2025**, **Zhang 2025**, and (variant) **Parisotto 2021**:

```
Acknowledgements · Abstract · [List of Figures]
1 Introduction        → ends with explicit "Research Questions" + "Thesis Outline" subsections
2 Related Work
3 Methods
4 Experimental Setup   ← SEPARATE chapter (tools/hardware/datasets/hyperparameters)
5 Results              ← one subsection per experiment; often ENDS with a "Limitations" subsection
6 Conclusion           → "6.1 Summary of Main Contributions" + "6.2 Future Work"
Bibliography · Appendices (A, B, …)
```
Plus: **figure- and table-heavy**, with an **explicitly-titled "Ablation" section** benchmarking against
named baselines, and **qualitative robot/Gazebo panels**. (all high confidence)

### How your current draft compares
Your draft: `1 Intro (RQ/Hyp/Contrib/Outline) · 2 Related Work · 3 System & Methods · 4 Experimental
Setup · 5 Results · 6 Discussion · 7 Conclusion+Future Work`.

✅ **Already matches** on: separate Experimental Setup chapter, explicit Research Questions + Thesis
Outline subsections, IEEE citations, the ablation (your axial-scale sweep — **label it "ablation"
explicitly** to hit his convention), and a Future Work section.

🔸 **The one structural difference — a separate "Discussion" chapter.** Kasaei's students typically have
**no standalone Discussion**: they fold interpretation into Results, end Results with **Limitations**,
and put a 2-part **Conclusion (Summary of Main Contributions + Future Work)**. You have a 7-chapter form
with Discussion split out. **Recommendation — your call, both defensible:**
   - **Keep the separate Discussion (recommended).** A *negative-result* thesis genuinely benefits from a
     dedicated interpretive chapter, and 7 chapters is standard scientific structure. To still hit his
     house style: **rename §7 to lead with "Summary of Main Contributions"** then "Future Work", and make
     sure your §6.6 Limitations reads as the Results-adjacent limitations he expects.
   - **OR match exactly:** merge §6 Discussion into §5 Results (each results subsection gets its
     interpretation), end §5 with "Limitations", and keep §6 Conclusion = Contributions + Future Work →
     a 6-chapter thesis.
   Either passes; pick keep-Discussion unless he has said otherwise. Don't agonize — it's cosmetic.

🔸 **Front matter:** his MSc theses (report-class) include **Acknowledgements** and a **List of Figures**.
The BSc *two-column article* template typically omits a List of Figures. Add **Acknowledgements** (easy
goodwill); a List of Figures only if the template supports it cleanly — optional at 11 pages.

🔸 **Title block:** his theses list **supervisor + often a second evaluator/co-supervisor**. Check whether
your BSc project needs a **second assessor** named on the title page (ask Kasaei / the project coordinator).

---

## 2. Length & required components
- **No public hard page/word limit** was found for the BSc-AI project; the IMRaD structure (abstract →
  intro+RQs → related work → methods → results → discussion → conclusion → refs → appendices) is a
  strong convention, not a published rule. (medium confidence)
- Practical target: the two-column AI style runs ~**10–14 pages**; your draft at 11 pages is on target.
- WBAI901-15 = Bachelor's Project, 15 ECTS, Bernoulli Institute. (high confidence)

## 3. AI-use disclosure (UG policy, 1 Dec 2023) — important for your Appendix B
- UG **requires disclosing** any non-trivial GenAI use: **(2.1) tool name + version** and **(2.2) the
  purpose / how it was used** (e.g. in Methods or an acknowledgement). (high confidence)
- There is **no UG-wide mandatory *signed* "declaration" form** — disclosure via an in-text/appendix
  statement suffices; **but individual programmes/supervisors may require more.** (high confidence)
- Every thesis gets a **plagiarism scan** (Model TER Art. 9.18.3); AI-detector scores may **not** be used
  as proof of cheating; submitting raw GenAI output as your own is never allowed. (high confidence)
- **Action on your Appendix B:** it already declares AI use — strengthen it to satisfy Rule 2.1 by
  **naming the specific tool(s) + version** (e.g. "Claude / GPT-class LLM assistant, 2025–2026") and the
  concrete purposes (drafting/revising text, structuring/debugging code, organizing notes), which you
  already list. Confirm with Kasaei whether the programme wants the separate signed declaration (your
  draft includes one — keep it; it can only help).

---

## 4. Example theses to open for calibration (Kasaei-supervised)
Read 2–3 of these to copy his exact look/structure:
- **Mein 2025 — *Dual-Arm Grasp Generation…*** (clearest exemplar of the steered template)
  https://fse.studenttheses.ub.rug.nl/34581/ ·
  PDF: https://fse.studenttheses.ub.rug.nl/34581/1/MasterThesisAnneJanMeinUniversityofGroningen.pdf
- **Zhang 2025 — *Low-Cost Imitation Learning…*** (Results ends with 5.4 Limitations)
  https://fse.studenttheses.ub.rug.nl/36404/
- **Parisotto 2021 — *MORE: Multi-View 3D Recognition & Pose Estimation*** (closest topic to yours —
  pose estimation; also an arXiv paper) https://fse.studenttheses.ub.rug.nl/24092/
- Repository root (browse more AI theses): https://fse.studenttheses.ub.rug.nl/
- His lab/pubs for the figure/ablation house style: https://www.ai.rug.nl/irl-lab/ · https://hkasaei.github.io/

> Note: these exemplars are **MSc** theses (report-class, single-column, List of Figures). The **BSc**
> deliverable uses the two-column article template — so copy his *structure, citation style, ablation
> framing, and figure density*, but follow the BSc template for layout/front-matter.

---

## 5. Sources
- Official BSc-AI template: https://www.overleaf.com/latex/templates/university-of-groningen-bsc-ai-bachelor-thesis-template/skczmjhsqmbq
- WBAI901-15: https://ocasys.rug.nl/current/catalog/course/WBAI901-15
- UG AI-in-teaching policy (1 Dec 2023, PDF): https://www.rug.nl/about-ug/organization/quality-assurance/education/eng-rug-beleid-ai-in-onderwijs-2023-def.pdf
- Kasaei papers (house style): MVGrasp arXiv:2103.10997 · OrthographicNet arXiv:1902.03057
- Example theses: Mein 34581, Zhang 36404, Parisotto 24092 (URLs above).
