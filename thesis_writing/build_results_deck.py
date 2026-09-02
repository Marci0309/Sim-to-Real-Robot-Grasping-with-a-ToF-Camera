#!/usr/bin/env python3
"""Presentation-style results & findings PDF for the supervisor meeting (v2).

One landscape slide per phase + visual slides (clean-vs-noisy depth, why
closed-loop, perception/calibration visuals). Pure matplotlib.
Outputs PDF to repo + Windows Downloads, dumps per-slide PNGs for QA.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages

BASE = os.path.expanduser("~/thesis_ws/src")
FIG = lambda *p: os.path.join(BASE, *p)
QA = FIG("thesis_writing", "deck_qa")
os.makedirs(QA, exist_ok=True)
SHOTS = "/mnt/c/Users/user/Downloads/shots"     # drop your bench/sim screenshots here
SHOT = lambda n: os.path.join(SHOTS, n)

NAVY, BLUE, GREEN, RED, ORANGE = "#0B3D5C", "#1565C0", "#2E7D32", "#C62828", "#EF6C00"
GREY, LIGHT, INK = "#5A6B73", "#ECEFF1", "#1A2329"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13})

_N = {"i": 1, "total": 0}   # footer auto-counter (title = slide 1, no footer)


def new_slide():
    fig = plt.figure(figsize=(13.33, 7.5), dpi=150); fig.patch.set_facecolor("white"); return fig


def title_bar(fig, title, tag=None):
    fig.add_artist(Rectangle((0, 0.885), 1, 0.115, transform=fig.transFigure, color=NAVY, zorder=0))
    fig.add_artist(Rectangle((0, 0.877), 1, 0.008, transform=fig.transFigure, color=ORANGE, zorder=0))
    fig.text(0.035, 0.942, title, color="white", fontsize=22, fontweight="bold", va="center")
    if tag:
        fig.text(0.965, 0.942, tag, color="#9FC4DE", fontsize=13, va="center", ha="right", style="italic")


def footer(fig, label):
    _N["i"] += 1
    fig.add_artist(Rectangle((0, 0.060), 1, 0.003, transform=fig.transFigure, color=LIGHT, zorder=0))
    fig.text(0.035, 0.038, label, color=GREY, fontsize=10, va="center")
    fig.text(0.965, 0.038, f"{_N['i']} / {_N['total']}", color=GREY, fontsize=10, va="center", ha="right")


def bullets(fig, items, x=0.04, y=0.80, dy=0.066, size=15):
    yy = y
    for text, lvl, col in items:
        mark = "▶  " if lvl == 0 else "        –  "
        fig.text(x, yy, mark + text, color=col, va="top", ha="left",
                 fontsize=size if lvl == 0 else size - 1.5,
                 fontweight="bold" if lvl == 0 else "normal")
        yy -= dy
    return yy


def add_image(fig, path, rect, caption=None, placeholder=None):
    l, b, w, h = rect
    try:
        ax = fig.add_axes([l, b, w, h]); ax.imshow(mpimg.imread(path)); ax.axis("off")
    except Exception:                                          # noqa: BLE001
        ax = fig.add_axes([l, b, w, h]); ax.axis("off")
        ax.add_patch(Rectangle((0.02, 0.02), 0.96, 0.96, fill=False, ls="--", ec=GREY, lw=1.5))
        ax.text(0.5, 0.5, placeholder or "[figure unavailable]", ha="center", va="center",
                color=GREY, fontsize=10)
    if caption:
        fig.text(l + w / 2, b - 0.016, caption, ha="center", va="top", fontsize=9.5, color=GREY, style="italic")


def add_table(fig, rect, cols, rows, fontsize=12, hi_rows=None):
    l, b, w, h = rect
    ax = fig.add_axes([l, b, w, h]); ax.axis("off")
    tbl = ax.table(cellText=rows, colLabels=cols, loc="center", cellLoc="center")
    tbl.auto_set_font_size(False); tbl.set_fontsize(fontsize); tbl.scale(1, 1.5)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#CFD8DC")
        if r == 0:
            cell.set_facecolor(NAVY); cell.set_text_props(color="white", fontweight="bold")
        else:
            if hi_rows and r in hi_rows:
                cell.set_facecolor("#FFF3E0")
            elif r % 2 == 0:
                cell.set_facecolor("#F7F9FA")
            if c == 0:
                cell.set_text_props(fontweight="bold")


def takeaway(fig, text, color=GREEN):
    fig.add_artist(Rectangle((0, 0.072), 1, 0.066, transform=fig.transFigure, color=LIGHT, zorder=0))
    fig.add_artist(Rectangle((0, 0.072), 0.010, 0.066, transform=fig.transFigure, color=color, zorder=1))
    fig.text(0.028, 0.105, "★  " + text, color=INK, fontsize=13, va="center", fontweight="bold")


# ----------------------------------------------------------------- slides
def s_title(fig):
    fig.add_artist(Rectangle((0, 0), 1, 1, transform=fig.transFigure, color=NAVY))
    fig.add_artist(Rectangle((0, 0.31), 1, 0.012, transform=fig.transFigure, color=ORANGE))
    fig.text(0.5, 0.76, "Investigating the Sim-to-Real Gap", color="white", fontsize=33, fontweight="bold", ha="center")
    fig.text(0.5, 0.685, "in RGB-D Tabletop Pose Estimation & Manipulation", color="white", fontsize=22, ha="center")
    fig.text(0.5, 0.52, "Results & Findings   ·   Supervisor Meeting", color="#9FC4DE", fontsize=18, ha="center", style="italic")
    fig.text(0.5, 0.41, "Marcell Mor Szokedencsi  (s5173191)  ·  BSc Artificial Intelligence", color="white", fontsize=15, ha="center")
    fig.text(0.5, 0.365, "University of Groningen   ·   Supervisor: Dr. Hamidreza Kasaei (IRL Lab)", color="white", fontsize=15, ha="center")
    fig.text(0.5, 0.19, "HEADLINE  —  a validated NEGATIVE RESULT", color=ORANGE, fontsize=17, ha="center", fontweight="bold")
    fig.text(0.5, 0.13, "modelling realistic ToF depth-noise does NOT move pose-estimation accuracy", color="white", fontsize=15, ha="center")


def s_question(fig):
    title_bar(fig, "Research Question & Approach", "Overview")
    bullets(fig, [
        ("Main RQ:  how does modelling realistic Time-of-Flight (ToF)", 0, INK),
        ("noise affect the sim-to-real gap in RGB-D tabletop pose", 1, INK),
        ("estimation and manipulation?", 1, INK),
        ("SQ1  ToF noise statistics          →  Phase 2", 0, BLUE),
        ("SQ2  pose-estimation accuracy   →  Phase 3", 0, BLUE),
        ("SQ3  manipulation propagation  →  Phase 4A / 4B / 4C", 0, BLUE),
        ("Hypothesis: injecting measured noise REDUCES the gap.", 0, RED),
        ("→  NOT confirmed — the negative result (a strength).", 1, RED),
    ], y=0.81, dy=0.075, size=15)
    add_table(fig, [0.63, 0.42, 0.34, 0.33], ["Setup", ""],
              [["Camera", "OAK-D SR PoE ToF @ 0.68 m"], ["Arm", "Hiwonder xArm 1S (5-DOF)"],
               ["Sim", "Gazebo Classic + ROS 2"], ["Objects", "coke / banana / golf / box"]], fontsize=11.5)
    takeaway(fig, "Five connected phases isolate ONE variable: depth-noise-model fidelity.", BLUE)
    footer(fig, "Question & Approach")


def s_findings(fig):
    title_bar(fig, "Four Headline Findings", "Summary")
    bullets(fig, [
        ("1.  NEGATIVE RESULT — at measured noise, pose RMSE moves", 0, GREEN),
        ("Δ < 0.1 mm vs clean; no effect on grasp success.", 1, INK),
        ("2.  POSE THRESHOLD ≈ ×10  (σ ≈ 9 mm) — 10× above reality.", 0, BLUE),
        ("3.  MANIPULATION COLLAPSE ≈ ×20 — gated by object-vs-", 0, ORANGE),
        ("gripper tolerance, not perception alone.", 1, INK),
        ("4.  REAL CONFIRMS SIM — 4B pose gap −1.4..+3.6 mm;", 0, NAVY),
        ("4C real grasp 34/60 = 57 % on hardware.", 1, INK),
    ], y=0.81, dy=0.082, size=15.5)
    add_table(fig, [0.64, 0.40, 0.33, 0.36], ["Why the null\nis VALID", ""],
              [["@ ×1", "below noise floor"], ["@ ×10", "pose degrades"],
               ["@ ×20", "grasping collapses"], ["⇒", "injector verified"]], fontsize=11.5)
    takeaway(fig, "The per-axis median discards the 53–99 % edge flying-pixels that ARE the real noise.")
    footer(fig, "Headline Findings")


def s_phase2(fig):
    title_bar(fig, "Phase 2  —  Real ToF Noise  (SQ1)", "characterisation")
    bullets(fig, [
        ("Axial precision sub-mm and FLAT with range:", 0, INK),
        ("0.40 m → 0.87 mm,   0.80 m → 0.69 mm.", 1, INK),
        ("Nguyen quadratic range model does NOT apply (b < 0).", 0, RED),
        ("Body quiet (0.6–0.7 mm); pose-relevant noise is at EDGES", 0, ORANGE),
        ("— flying-pixel rate 53–99 % on all four objects.", 1, INK),
        ("→ motivates the median-centroid estimator + edge term.", 0, BLUE),
    ], y=0.80, dy=0.072, size=14.5)
    add_image(fig, FIG("tof_perception", "scripts", "fit_results_truly_raw", "sigma_vs_z.png"),
              [0.55, 0.50, 0.42, 0.31], "Axial σ vs distance (flat, sub-mm)")
    add_image(fig, FIG("tof_perception", "scripts", "object_results", "blue_box_overlay.png"),
              [0.55, 0.175, 0.42, 0.29], "Edge ring = flying / mixed pixels")
    takeaway(fig, "Real ToF is quiet in the body, noisy only at edges — the whole study turns on this.")
    footer(fig, "Phase 2 · SQ1 noise statistics")


def s_noise_demo(fig):
    title_bar(fig, "The ToF Noise Model  —  Clean vs Noisy Depth", "what we inject")
    bullets(fig, [
        ("The injector adds 4 literature terms (axial Gaussian, lateral jitter, edge flying-pixels, dropout)", 0, INK),
        ("to clean simulated depth. At measured (×1) level the body is barely touched; ×10 degrades it.", 1, INK),
    ], y=0.82, dy=0.052, size=13.5)
    add_image(fig, FIG("evaluation", "results", "clean_vs_noisy_depth.png"),
              [0.05, 0.30, 0.90, 0.40], "Synthetic clean tabletop depth + the measured OAK-D noise model (same pipeline as the experiments)")
    takeaway(fig, "The median centroid ignores the ×1 edge noise — exactly why injecting realistic noise changes nothing.")
    footer(fig, "Noise model · clean vs noisy")


def s_phase3(fig):
    title_bar(fig, "Phase 3  —  Pose Accuracy  (SQ2)", "the negative result")
    bullets(fig, [
        ("Measured (×1) ≈ clean ≈ literature model; planar RMSE flat", 0, GREEN),
        ("~3.5 mm through ×10,  Δ(clean,×1) < 0.1 mm.  Threshold ≈ ×10.", 1, INK),
    ], y=0.82, dy=0.055, size=14)
    add_table(fig, [0.04, 0.205, 0.45, 0.47], ["axial", "coke", "banana", "golf", "blue"],
              [["0 clean", "3.3/4.9", "8.7/10.7", "4.8/7.9", "3.1/3.1"],
               ["×1", "3.5/5.0", "8.6/10.6", "4.7/7.9", "3.4/3.4"],
               ["×10", "3.5/7.4", "9.1/12.2", "4.4/10.5", "3.4/4.0"],
               ["×20", "9.6/18.4", "22.4/24.5", "8.6/19.1", "8.7/10.2"]], fontsize=11.5, hi_rows=[4])
    add_image(fig, FIG("evaluation", "results", "phase3_pose_vs_noise.png"),
              [0.55, 0.18, 0.42, 0.55], "Pose RMSE (mm) vs axial-noise scale")
    takeaway(fig, "Realistic noise sits an order of magnitude below the estimator's noise floor.  Table: planar / 3D RMSE.")
    footer(fig, "Phase 3 · SQ2 pose accuracy")


def s_phase4a(fig):
    title_bar(fig, "Phase 4A  —  Simulated Manipulation  (SQ3, sim)", "480 trials")
    bullets(fig, [
        ("Analytic grasp-feasible; all 4 detected 20/20. Flat clean→×10,", 0, INK),
        ("collapse @ ×20. Per-object = clean error vs jaw tolerance.", 1, INK),
    ], y=0.82, dy=0.055, size=14)
    add_table(fig, [0.04, 0.22, 0.45, 0.44], ["cond.", "coke", "banana", "golf", "blue"],
              [["clean", "0%", "65%", "65%", "100%"], ["×1", "5%", "65%", "65%", "100%"],
               ["×10", "0%", "65%", "75%", "100%"], ["×20", "5%", "30%", "20%", "50%"]], fontsize=12, hi_rows=[4])
    add_image(fig, FIG("evaluation", "results", "phase4a_rate_vs_axial_scale.png"),
              [0.55, 0.18, 0.42, 0.55], "Grasp-feasible rate vs axial scale")
    takeaway(fig, "coke floored 0 % = a SIZE limit (52 mm in 55 mm jaw), not noise;  box 100 %, golf/banana ~65 %.")
    footer(fig, "Phase 4A · SQ3 simulated manipulation")


def s_phase4b(fig):
    title_bar(fig, "Phase 4B  —  Real Pose Gap  (SQ3, real perception)", "5 real scenes")
    bullets(fig, [
        ("Same pipeline on real OAK-D scenes. Real-vs-sim planar pose", 0, GREEN),
        ("gap −1.4..+3.6 mm; sim_clean ≈ sim_noisy on real data too.", 1, INK),
    ], y=0.82, dy=0.055, size=14)
    add_table(fig, [0.04, 0.22, 0.45, 0.44], ["", "coke", "banana", "golf", "blue"],
              [["sim_clean", "10.4", "13.3", "6.2", "3.8"], ["sim_noisy", "10.4", "13.2", "6.0", "3.5"],
               ["real n=4", "9.4", "12.1", "4.8", "7.4"], ["GAP", "−1.1", "−1.1", "−1.4", "+3.6"]], fontsize=12, hi_rows=[4])
    add_image(fig, FIG("evaluation", "results", "phase4b_sim_to_real_gap.png"),
              [0.55, 0.18, 0.42, 0.55], "Sim vs real planar pose error (mm)")
    takeaway(fig, "Sim ≈ real at the millimetre level — the negative result carries end-to-end onto hardware.")
    footer(fig, "Phase 4B · SQ3 real perception")


def s_why_closedloop(fig):
    title_bar(fig, "Why Closed-Loop Visual Servo?  (not open-loop)", "the method choice")
    bullets(fig, [
        ("Open-loop problem: the real arm over-reaches its URDF model by ~28 %", 0, RED),
        ("— an unidentifiable, position-dependent kinematic error.", 1, INK),
        ("6 touch-cal points can't fit a 13-param model; the global-scale hack", 0, INK),
        ("leaves 14 mm RMS / 25 mm max  >  the box's ±7.5 mm tolerance.", 1, INK),
        ("⇒ open-loop reliable only in a ~4 cm patch of the ~20 cm reach.", 1, RED),
        ("Better calibration can't save it — open-loop is capped at ~8–14 mm.", 0, ORANGE),
        ("Fix — CLOSE THE LOOP:  one ArUco marker on the gripper + the SAME", 0, GREEN),
        ("overhead camera see gripper AND object; nudge target by K·(obj − gripper).", 1, INK),
        ("The wrong FK + scale cancel as COMMON-MODE bias → aim camera-bound", 0, GREEN),
        ("~1–3 mm, FK-independent.  Zone:  ~4 cm  →  the whole near workspace.", 1, INK),
    ], y=0.81, dy=0.066, size=13.5)
    add_table(fig, [0.66, 0.40, 0.31, 0.30], ["", "open", "closed"],
              [["zone", "~4 cm", "near-WS"], ["aim err", "8–14 mm", "~1–3 mm"], ["bound by", "arm model", "camera"]], fontsize=11.5)
    takeaway(fig, "Same idea as a chess-arm verifying against known points — not trusting a perfect model.", NAVY)
    footer(fig, "Phase 4C · why closed-loop")


def s_phase4c(fig):
    title_bar(fig, "Phase 4C  —  Real Manipulation  (SQ3, real)", "60 trials · NEW")
    bullets(fig, [
        ("Closed-loop visual servo makes aim camera-bound ~1–3 mm, independent of the", 0, INK),
        ("broken arm model.   4 objects × 5 placements × 3 = 60 real lift-and-hold trials.", 1, INK),
    ], y=0.82, dy=0.055, size=14)
    add_table(fig, [0.20, 0.20, 0.60, 0.50], ["object", "lift", "rate", "95% CI", "sim-feas"],
              [["blue_box", "11/15", "73 %", "[48, 89]", "100 %"], ["banana", "10/15", "67 %", "[42, 85]", "65 %"],
               ["coke_can", "10/15", "67 %", "[42, 85]", "0 %"], ["golf_ball", "3/15", "20 %", "[7, 45]", "65 %"],
               ["OVERALL", "34/60", "57 %", "[44, 68]", "—"]], fontsize=13, hi_rows=[5])
    takeaway(fig, "blue_box 73 % · banana 67 % · coke 67 % · golf 20 %   →   OVERALL 34/60 = 57 % lift-and-hold.")
    footer(fig, "Phase 4C · SQ3 real manipulation (results)")


def s_phase4c_fig(fig):
    title_bar(fig, "Phase 4C  —  Real vs Predicted", "the key figure")
    add_image(fig, FIG("evaluation", "results", "phase4c_real_vs_pred.png"),
              [0.07, 0.33, 0.86, 0.49],
              "Left: real lift vs sim feasibility (grey).     Right: real pose error vs jaw tolerance.")
    takeaway(fig, "Coke 0 % feasible → 67 % real (beats sim).  Golf 65 % feasible → 20 % real.  Real diverges from BOTH predictors.", ORANGE)
    footer(fig, "Phase 4C · the key figure")


def s_insight(fig):
    title_bar(fig, "Phase 4C  —  Why the Rates Diverge", "the new insight")
    bullets(fig, [
        ("Once the loop fixes AIM (camera-bound ~1–3 mm everywhere),", 0, INK),
        ("grasp success is set by GRIP MECHANICS, not perception.", 1, INK),
        ("coke:  predicted ~0 % (1.5 mm rigid tol)  →  67 % real", 0, GREEN),
        ("firm close + slight can compliance — REAL BEATS SIM.", 1, INK),
        ("golf:  predicted 'likely' (4.8 < 6 mm)  →  20 % real", 0, RED),
        ("smooth sphere rolls out of flat jaws despite < 2 mm aim.", 1, INK),
        ("→ neither the sim-feasible ceiling nor the pose-error", 0, BLUE),
        ("predictor orders the real rates (they diverge both ways).", 1, INK),
    ], y=0.80, dy=0.078, size=15.5)
    takeaway(fig, "Real grasp rate = object shape × jaw force/compliance.  Perception / pose-error is NOT the bottleneck.", NAVY)
    footer(fig, "Phase 4C · grip-mechanics finding")


def s_perception_visuals(fig):
    title_bar(fig, "Perception & Calibration  —  Visuals", "extra visuals")
    bullets(fig, [
        ("Live demo available: red object blobs + green grasp-X + magenta jaw-axis", 0, INK),
        ("via  live_perception_view.py  (overhead OAK-D).  See the cookbook.", 1, BLUE),
    ], y=0.82, dy=0.052, size=13.5)
    add_image(fig, FIG("tof_perception", "board_4b", "extrinsics_overlay.png"),
              [0.05, 0.20, 0.43, 0.50], "ChArUco camera→table extrinsic (real)")
    add_image(fig, FIG("evaluation", "results", "phase4a_perception_error.png"),
              [0.53, 0.20, 0.43, 0.50], "Perceived error vs jaw-tolerance ring (per object)")
    takeaway(fig, "Calibration + per-object jaw-tolerance rings explain the grasp pass/fail boundary.")
    footer(fig, "Perception & calibration visuals")


def s_synth(fig):
    title_bar(fig, "Synthesis  —  Answer to the RQ", "conclusion")
    bullets(fig, [
        ("Phase 2:  real ToF quiet in the bulk, noisy at edges (SQ1).", 0, INK),
        ("Phase 3:  a robust median discards that edge noise → NULL (SQ2).", 0, INK),
        ("Phase 4A:  same null in simulated manipulation (SQ3 sim).", 0, INK),
        ("Phase 4B:  sim ≈ real on real perception (−1.4..+3.6 mm).", 0, INK),
        ("Phase 4C:  real grasp rate = grip mechanics, not noise.", 0, INK),
    ], y=0.80, dy=0.080, size=16)
    fig.text(0.04, 0.30, "The gap lives in CALIBRATION (the ~28 % arm-model error, fixed by closing the loop),",
             fontsize=14, color=INK, va="center")
    fig.text(0.04, 0.255, "lighting, and segmentation — NOT in depth-noise-model fidelity.", fontsize=14, color=INK, va="center")
    takeaway(fig, "For a robust median-centroid estimator, realistic ToF noise does NOT measurably reduce the sim-to-real gap.")
    footer(fig, "Synthesis · the negative result")


def s_limits(fig):
    title_bar(fig, "Limitations & Future Work", "honest caveats")
    fig.text(0.04, 0.80, "Limitations", fontsize=16, fontweight="bold", color=NAVY)
    bullets(fig, [
        ("Sim grasp metric is ANALYTIC (Gazebo can't reliably contact-grasp).", 0, GREY),
        ("Noise model omits spatial correlation + multipath; lateral jitter = lit. value.", 0, GREY),
        ("Phase 4B is n = 4 / object;  Phase 4C is near-zone (r ≤ 0.20 m).", 0, GREY),
        ("Far-edge fall-off = gripper-marker mount, not the method.", 0, GREY),
    ], y=0.73, dy=0.066, size=14)
    fig.text(0.04, 0.40, "Future work", fontsize=16, fontweight="bold", color=NAVY)
    bullets(fig, [
        ("Remount the marker low over the jaws → full-reach convergence.", 0, BLUE),
        ("Compliant pads / cradle for round + near-jaw-width objects.", 0, BLUE),
        ("Learned correction field (REPLAB) + learned estimator (MVGrasp).", 0, BLUE),
    ], y=0.33, dy=0.066, size=14)
    footer(fig, "Limitations & Future Work")


def s_real_camera(fig):
    title_bar(fig, "Perception — Real OAK-D (live)", "your screenshots")
    add_image(fig, SHOT("real_depth.png"), [0.025, 0.28, 0.305, 0.50], "Raw ToF depth (table ~0.69 m)",
              placeholder="save the 'ToF Depth'\nscreenshot to\nDownloads\\shots\\real_depth.png")
    add_image(fig, SHOT("real_filter.png"), [0.345, 0.28, 0.305, 0.50], "FILTER — red = segmented object",
              placeholder="save the '1) FILTER'\nscreenshot to\nDownloads\\shots\\real_filter.png")
    add_image(fig, SHOT("real_centers.png"), [0.665, 0.28, 0.305, 0.50], "CENTERS — green grasp-X + jaw axis",
              placeholder="save the '2) CENTERS'\nscreenshot to\nDownloads\\shots\\real_centers.png")
    takeaway(fig, "Live on the real overhead OAK-D: red object blobs, green grasp-X in table mm, magenta jaw axis.")
    footer(fig, "Perception · real OAK-D (live)")


def s_sim_views(fig):
    title_bar(fig, "Simulation — Overhead RGB-D + Perception", "Gazebo")
    add_image(fig, SHOT("sim_rgb.png"), [0.025, 0.28, 0.305, 0.50], "RGB scene  (/overhead_rgb)",
              placeholder="save sim RGB →\nDownloads\\shots\\sim_rgb.png")
    add_image(fig, SHOT("sim_depth.png"), [0.345, 0.28, 0.305, 0.50], "Clean depth  (/overhead_depth)",
              placeholder="save sim depth →\nDownloads\\shots\\sim_depth.png")
    add_image(fig, SHOT("sim_debug.png"), [0.665, 0.28, 0.305, 0.50], "Perception: green=blob, dot=centroid, X=grasp",
              placeholder="save debug_view →\nDownloads\\shots\\sim_debug.png")
    takeaway(fig, "Same pipeline in sim: RGB-D scene → labelled segmentation → centroid + grasp-X, at the real 0.68 m height.")
    footer(fig, "Simulation · RGB-D + perception")


SLIDES = [s_title, s_question, s_findings, s_phase2, s_noise_demo, s_phase3, s_phase4a,
          s_phase4b, s_why_closedloop, s_phase4c, s_phase4c_fig, s_insight,
          s_perception_visuals, s_real_camera, s_sim_views, s_synth, s_limits]
_N["total"] = len(SLIDES)

OUT_REPO = FIG("thesis_writing", "thesis_results_summary.pdf")
OUT_WIN = "/mnt/c/Users/user/Downloads/thesis_results_summary.pdf"

_N["i"] = 1
with PdfPages(OUT_REPO) as pdf:
    for idx, fn in enumerate(SLIDES):
        fig = new_slide(); fn(fig); pdf.savefig(fig)
        if idx in (13, 14):                            # QA the two screenshot slides
            fig.savefig(os.path.join(QA, f"slide_{idx:02d}.png"), dpi=90)
        plt.close(fig)

import shutil
try:
    shutil.copyfile(OUT_REPO, OUT_WIN); print("wrote:", OUT_REPO, "AND", OUT_WIN)
except Exception as e:                                          # noqa: BLE001
    print("wrote:", OUT_REPO, "(Downloads copy failed:", e, ")")
print("slides:", _N["total"])
