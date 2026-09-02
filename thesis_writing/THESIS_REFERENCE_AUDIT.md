# Thesis — Reference Audit (anti-hallucination + honest-usage check)

**Method:** every scholarly/software entry in `references.bib` (plus the two entries the draft cites
but the .bib is missing) was independently web-verified by a dedicated agent against the authoritative
record — DOI landing page, IEEE Xplore / Springer / Elsevier / MDPI, arXiv, DBLP, or CrossRef. Asset
entries were resolved from `src/ATTRIBUTION.md`.

> **STATUS (2026-06-27): all `.bib` fixes in §1, §3, §4 below are now APPLIED** to
> `thesis_writing/references.bib` (added Huber + REPLAB; fixed nguyen/hodan/macenski/link_attacher;
> replaced pinto→`cai2024flexx2`; filled coke/banana CC-BY + the GrabCAD URL). Style switched to APA
> (`apacite`). The **only** thing left in the `.bib` is the `grabcad_xarm` **uploader name** (read it off
> the GrabCAD page) and the two prose-overclaim rewordings in §2 (done in the `.tex`, not the `.bib`).

## ✅ HEADLINE: no hallucinations
**All 20 web-checked references are real, findable publications.** None is fabricated. Your
plagiarism exposure is **not** from invented citations — it is from (a) two cited works missing from
the .bib, (b) ~6 metadata errors, and (c) three places where the *prose* could overclaim what a real
source says. All are fixed below.

---

## 1. Must-fix BEFORE compiling (paste the corrected BibTeX)

### 1a. ADD — cited in the draft, absent from `references.bib` (would compile as `?`)

**`huber1981robust`** — *real* (Huber, *Robust Statistics*, Wiley 1981; ISBN 0-471-41805-6). Used for
the median's 50 % breakdown point in §3.5/§6.2.
```bibtex
@book{huber1981robust,
  author    = {Huber, Peter J.},
  title     = {Robust Statistics},
  series    = {Wiley Series in Probability and Mathematical Statistics},
  publisher = {John Wiley \& Sons},
  address   = {New York},
  year      = {1981},
  isbn      = {0-471-41805-6}
}
```

**`yang2019replab`** (REPLAB) — *real* (ICRA 2019, pp. 8691–8697). **Third author is Jesse Zhang, not
"Jeff".** Used in §3.8/§7.
```bibtex
@inproceedings{yang2019replab,
  author    = {Yang, Brian and Jayaraman, Dinesh and Zhang, Jesse and Levine, Sergey},
  title     = {{REPLAB}: A Reproducible Low-Cost Arm Benchmark for Robotic Learning},
  booktitle = {2019 International Conference on Robotics and Automation (ICRA)},
  pages     = {8691--8697},
  year      = {2019},
  publisher = {IEEE},
  doi       = {10.1109/ICRA.2019.8794390},
  note      = {Extended version: arXiv:1905.07447}
}
```

### 1b. FIX metadata — real papers, wrong details

| key | fix | corrected entry |
|---|---|---|
| **`nguyen2012modeling`** | third author is **David R. Lovell** (missing middle initial) | `author = {Nguyen, Chuong V. and Izadi, Shahram and Lovell, David R.}` (rest unchanged) |
| **`hodan2018bop`** | pages **19–34 → 19–35**; carry the **full 16-author** list (not 4+et al.); diacritics | see block below |
| **`macenski2022ros2`** | first name **Steven → Steve**; add `pages = {eabm6074}` | `author = {Macenski, Steve and Foote, Tully and Gerkey, Brian and Lalancette, Chris and Woodall, William}, ... pages = {eabm6074}` |
| **`link_attacher`** | the entry **conflates two different repos** — fix to the one you actually follow (IFRA-Cranfield, ROS 2, fixed-joint) | see block below |
| **`pinto2024flexx2`** | author is **NOT Pinto** — real authors Cai, Plozza, Marty, Joseph, Magno; title wrong. **Rename or drop** (it is uncited) | see §3 |

```bibtex
@inproceedings{hodan2018bop,
  author    = {Hod{\v{a}}n, Tom{\'a}{\v{s}} and Michel, Frank and Brachmann, Eric and Kehl, Wadim
               and Buch, Anders Glent and Kraft, Dirk and Drost, Bertram and Vidal, Joel
               and Ihrke, Stephan and Zabulis, Xenophon and Sahin, Caner and Manhardt, Fabian
               and Tombari, Federico and Kim, Tae-Kyun and Matas, Ji{\v{r}}{\'\i} and Rother, Carsten},
  title     = {{BOP}: Benchmark for {6D} Object Pose Estimation},
  booktitle = {Computer Vision -- ECCV 2018},
  series    = {Lecture Notes in Computer Science}, volume = {11214},
  pages     = {19--35}, year = {2018}, publisher = {Springer},
  doi       = {10.1007/978-3-030-01249-6_2}
}

@misc{link_attacher,
  author       = {Bueno Viso, Mikel and Asif, Seemal and Webb, Phil},
  title        = {{IFRA\_LinkAttacher}: A {ROS}2 {Gazebo} Plugin for Creating a Fixed Joint Between Two Links},
  howpublished = {GitHub, IFRA-Cranfield, Centre for Robotics and Assembly, Cranfield University},
  year         = {2023},
  note         = {Apache-2.0. Accessed 2026-06-27},
  url          = {https://github.com/IFRA-Cranfield/IFRA_LinkAttacher}
}
```
> `link_attacher` detail: the old entry credited *"PAL Robotics and IFRA-Cranfield contributors"* with
> the **PAL** URL, while the draft text cites the **IFRA-Cranfield** URL. They are two distinct projects
> (PAL = Sam Pfeiffer, BSD, ROS 1, *virtual revolute joint*; IFRA = Cranfield, Apache-2.0, ROS 2,
> *fixed joint*). You use the ROS 2 fixed-joint pattern → **IFRA-Cranfield is correct.** Make the
> in-text reference [19] author match (drop "PAL Robotics" from the author string).

### 1c. VERIFIED — real and correctly cited (cosmetic-only notes)

`khoshelham2012accuracy`, `sarbolandi2015kinect`, `foix2011lockin`, `tobin2017domain`, `foote2013tf`,
`craig2005robotics`, `garridojurado2014aruco`, `romeroramirez2018charuco`, `koenig2004gazebo`,
`bradski2000opencv`, `jakobi1995noise`, `kasaei2021mvgrasp`, `bohg2014grasp`, `rusu2008towards`.

Minor polish (optional, IEEEtran mostly handles casing): restore diacritics in
`garridojurado2014aruco` (**Muñoz-Salinas, Marín-Jiménez**) and in author names generally; `bohg` title
uses an em-dash *Synthesis—A Survey*. `kasaei2021mvgrasp` also has a journal version (Robotics and
Autonomous Systems 160:104313, 2022) if you prefer citing the peer-reviewed version over arXiv.

---

## 2. Honest-usage flags (prose, not BibTeX — fix the wording so you don't overclaim)

These are the real plagiarism/integrity risks now that the entries are confirmed real:

1. **`tobin2017domain` — don't attribute your ablation to it.** Tobin motivates *randomization*; it does
   **not** provide a "seeded independent-RNG ablation methodology." Present the seeded ablation as
   **your own design**, citing Tobin only for the randomization motivation. Update the .bib comment too.
2. **`yang2019replab` (REPLAB) — don't claim it invented a dense correction field.** REPLAB is a
   low-cost reproducible *arm/benchmark platform*; it does **not** present a "dense camera-measured
   thin-plate-spline correction field." §3.8 currently says "a dense … correction field **in the style
   of REPLAB**" — reword to cite REPLAB for the **low-cost reproducible-arm / benchmarking** idea, and
   describe the TPS correction field as **your own planned fallback** (don't imply REPLAB did TPS).
3. **`khoshelham2012accuracy` — the "five distances" are yours, not theirs.** Khoshelham sampled **ten**
   distances (0.5–5.0 m) with RANSAC plane fitting. Your "five distances 0.40–0.80 m" is *your* protocol
   "following the practice of [6]." Keep the phrasing as *"following standard practice [6], we image at
   five distances…"* — do **not** write "[6] uses five distances." (Current §3.3 wording is fine; just
   don't drift.)
4. **`romeroramirez2018charuco` — it is the ArUco *detection* backend, not the ChArUco-board paper.**
   ChArUco = ArUco markers + a chessboard (an OpenCV construction). Cite [10] as the fast ArUco detector
   underpinning OpenCV's ChArUco; don't imply it introduced ChArUco boards. (Current §3.7 is acceptable.)
5. **`rusu2008towards` / `bohg2014grasp` — contrast/framing only (already correct).** Keep citing Rusu as
   *"unlike the classic RANSAC-plane pipeline [7], we use a median-depth threshold"* and Bohg as the
   grasp taxonomy. Never as a method you implemented.

---

## 3. The one near-"hallucination": `pinto2024flexx2`

The placeholder `author = {Pinto, et al.}` is **wrong** — there is **no author named Pinto**. The real
paper (arXiv:2412.15040, also IEEE COINS 2024) is **"Noise Analysis and Modeling of the PMD Flexx2
Depth Camera for Robotic Applications" by Cai, Plozza, Marty, Joseph, Magno.** It is **uncited** in the
draft. **Two clean options:**
- **(a) Drop it** entirely (simplest — it adds nothing the draft relies on), or
- **(b) Keep it** as ToF-noise corroboration, but rename the key to `cai2024flexx2` and use:
```bibtex
@inproceedings{cai2024flexx2,
  author    = {Cai, Yuke and Plozza, Davide and Marty, Steven and Joseph, Paul and Magno, Michele},
  title     = {Noise Analysis and Modeling of the {PMD} {Flexx2} Depth Camera for Robotic Applications},
  booktitle = {2024 IEEE International Conference on Omni-layer Intelligent Systems (COINS)},
  pages     = {422--427}, year = {2024},
  doi       = {10.1109/COINS61597.2024.10622644},
  note      = {Preprint: arXiv:2412.15040}
}
```
> Leaving a mis-keyed, mis-titled, fake-author entry in the file is exactly the kind of thing a careful
> examiner greps for. Fix it or delete it — do not leave `Pinto, et al.` in the .bib.

---

## 4. Assets (from `ATTRIBUTION.md` — two of three TODOs are already closed)

| .bib key | status | action |
|---|---|---|
| `sketchfab_cokecan` | ✅ **RESOLVED** — "Soda Can 250ml" by **sebastian204**, **CC-BY-4.0** | fill author/title/URL into the .bib + paste the CC-BY string in Appendix A (both already in `ATTRIBUTION.md`) |
| `sketchfab_banana` | ✅ **RESOLVED** — "Banana" by **Lassi Kaukonen (@thesidekick)**, **CC-BY-4.0** | same; author to do a final visual check it is the exact model |
| `grabcad_xarm` | 🟡 **URL filled** (https://grabcad.com/library/hiwonder-x-arm-1) | only the **uploader name** remains — read it off the page (GrabCAD blocks scrapers, 403) and confirm academic reuse |
| `gazebo_ros2_control`, `luxonis_oakd`, `depthai_docs` | low-risk software/product URLs | confirm the URLs still resolve; entries are fine as-is |

CC-BY-4.0 requires attribution — the verbatim strings are in `ATTRIBUTION.md §"Required attribution"`;
put them in **Appendix A**. (CC-BY-4.0 permits commercial + thesis use, so no licensing problem.)

---

## 5. EXCLUDED list — confirmed clean ✅

The current draft's reference list `[1]–[23]` was cross-checked against the **PART 4 "DO NOT CITE"**
keys. **None of them appear** — `kuhn1955hungarian`, `tsai1989handeye`, `zhang2000calibration`,
`hinterstoisser2012add`, `fischler1981ransac`, `zhou2018open3d`, `olson2011apriltag`,
`kumra2020antipodal`, `belhedi2015` are all correctly absent. Keep them commented out. Do not
reintroduce any of them (each corresponds to a method you did **not** implement — citing it = a false
claim). The matching code-honesty fix (the `pose_estimator.py` "RANSAC/Open3D" docstring) still needs
doing so code and thesis agree.

---

## 6. ⚠️ Compile from the corrected `.bib`, not from the draft's rendered list

The reference list in `_0_5in.pdf` looks hand/LLM-generated, not BibTeX-compiled — e.g. its `[7]` Rusu
renders **"Rusu, Blodow, Marton, Beetz"** (4 authors, wrong order, Dolha dropped) while the **.bib is
correct** (Rusu, Marton, Blodow, Dolha, Beetz). Once you `apacite`-compile from the *corrected*
`references.bib`, renderings fix themselves — **but verify** the final PDF's reference list against the
.bib (spot-check Rusu's author order and BOP's 16 authors / pp. 19–35).

---

## ACTION CHECKLIST
- [x] Add `huber1981robust` and `yang2019replab` to `references.bib`. *(applied 2026-06-27)*
- [x] Apply the 4 metadata fixes (`nguyen`, `hodan`, `macenski`, `link_attacher`). *(applied)*
- [x] Fix or delete `pinto2024flexx2` (→ `cai2024flexx2`). *(replaced)*
- [x] Fill `sketchfab_cokecan` + `sketchfab_banana` from `ATTRIBUTION.md`; add the `grabcad_xarm` URL. *(applied)*
- [ ] **`grabcad_xarm` uploader name** — read it off https://grabcad.com/library/hiwonder-x-arm-1 and
      replace the `>>> TODO` in the `author` field (it is a cited entry, so it WILL show in the bibliography).
- [ ] Reword the 2 overclaim risks in the `.tex` (Tobin ablation; REPLAB correction-field) + the
      Khoshelham "five distances" phrasing.
- [ ] Keep all PART-4 EXCLUDED keys commented; fix the `pose_estimator.py` RANSAC/Open3D docstring.
- [ ] `apacite`-compile and verify the rendered list against the .bib; convert in-text `[n]`→author-year.
