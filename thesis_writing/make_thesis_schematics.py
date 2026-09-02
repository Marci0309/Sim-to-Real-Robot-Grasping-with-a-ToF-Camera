#!/usr/bin/env python3
"""Render the two 'must-make' thesis schematics (F11 servo loop, F10 system pipeline).

Outputs PNGs to evaluation/results/. These are SCHEMATICS (no experiment data), so
they are safe to regenerate any time. A paste-ready TikZ version of each also lives in
thesis_writing/thesis_schematics_tikz.tex (preferred for the final Overleaf build).
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle, Circle

OUT = os.path.join(os.path.dirname(__file__), "..", "evaluation", "results")
os.makedirs(OUT, exist_ok=True)

# muted, print-safe palette
WIN = "#2c6fbb"    # Windows host
WSL = "#2e8b57"    # WSL host
GREY = "#444444"
LIGHT = "#eef2f7"
OBJ = "#1a9850"    # object point (green)
GRIP = "#e08214"   # gripper point (orange)
ERR = "#d73027"    # error vector (red)


def rbox(ax, x, y, w, h, text, ec=GREY, fc="white", fs=9, lw=1.4, tc="black"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                ec=ec, fc=fc, lw=lw, zorder=2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            color=tc, zorder=3, linespacing=1.25)
    return (x + w / 2, y + h / 2), (x, y, w, h)


def arrow(ax, p1, p2, color=GREY, lw=1.6, style="-|>", ls="-", rad=0.0, z=2):
    ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle=style, mutation_scale=15,
                                 lw=lw, color=color, ls=ls, zorder=z,
                                 connectionstyle=f"arc3,rad={rad}"))


# =====================================================================
# F11 — Closed-loop visual-servo schematic
# =====================================================================
def make_servo():
    fig, ax = plt.subplots(figsize=(8.4, 6.0))
    ax.set_xlim(0, 10); ax.set_ylim(0, 8.2); ax.axis("off")

    def plabel(text, xy, xytext, color):
        ax.annotate(text, xy=xy, xytext=xytext, fontsize=11, color=color, ha="center",
                    va="center", zorder=10,
                    bbox=dict(boxstyle="round,pad=0.22", fc="white", ec=color, lw=1.0),
                    arrowprops=dict(arrowstyle="-", color=color, lw=1.1, shrinkA=2, shrinkB=4))

    # ---------------- TOP HALF: the loop concept ----------------
    cam_c, _ = rbox(ax, 3.9, 6.95, 2.2, 0.72, "Overhead OAK-D SR PoE (ToF)\nsingle camera",
                    ec=WIN, fs=9.5)
    # field-of-view cone down to the table
    ax.add_patch(plt.Polygon([(3.9, 6.95), (6.1, 6.95), (8.8, 1.2), (1.2, 1.2)],
                             closed=True, fc="#fff8e1", ec="#e8d9a0", lw=0.8, alpha=0.32, zorder=0))

    # visual-servo loop box (left) -- a clean numbered list (matched size with insight box)
    rbox(ax, 0.35, 4.5, 3.75, 2.1,
         "Visual-servo loop\n"
         "1.  measure  $\\mathbf{e}$  through the camera\n"
         "2.  $\\mathrm{target} \\leftarrow \\mathrm{target} + K\\,\\mathbf{e}$   ($K\\approx0.6$)\n"
         "3.  repeat until  $\\|\\mathbf{e}\\| < 3$ mm\n"
         "4.  descend and close the jaws",
         ec=GREY, fs=9.5)
    arrow(ax, (2.2, 4.5), (3.18, 1.62), color=GREY, lw=1.6, rad=-0.18)

    # key-insight box (right) -- trimmed; numeric details (FK over-reach, 0.777) live in the caption
    rbox(ax, 5.90, 4.5, 3.75, 2.1,
         "Both points are seen by the SAME\n"
         "camera in the SAME frame, so the\n"
         "arm's calibration errors cancel as\n"
         "common-mode bias\n"
         r"$\Rightarrow$ aim is camera-bound" + "\n(~1–4 mm), model-independent",
         ec=WIN, fc="#f3f8fd", fs=9.2)

    # ---------------- BOTTOM HALF: the physical scene ----------------
    # table
    ax.add_patch(Rectangle((0.8, 0.6), 8.4, 0.5, fc=LIGHT, ec=GREY, lw=1.3, zorder=1))
    ax.text(0.95, 0.85, "table frame", fontsize=8.5, color=GREY, va="center")

    # gripper (parallel jaws) on the LEFT
    for gx in (2.72, 3.40):
        ax.add_patch(Rectangle((gx, 1.10), 0.16, 0.72, fc="#bdbdbd", ec=GREY, lw=1.0, zorder=2))
    ax.add_patch(Rectangle((2.72, 1.82), 0.84, 0.18, fc="#9e9e9e", ec=GREY, lw=1.0, zorder=2))
    ax.add_patch(Rectangle((2.96, 2.06), 0.36, 0.36, fc="white", ec="black", lw=1.1, zorder=3))
    ax.add_patch(Rectangle((3.05, 2.15), 0.18, 0.18, fc="black", zorder=3))
    ax.text(3.14, 2.66, "gripper marker (id 4)", fontsize=7.8, color=GREY, ha="center",
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.85), zorder=4)
    p_grip = (3.14, 1.32)
    ax.add_patch(Circle(p_grip, 0.085, fc=GRIP, ec="black", lw=0.7, zorder=6))

    # object on the RIGHT
    ax.add_patch(Rectangle((6.55, 1.10), 0.72, 0.55, fc="#9ecae1", ec=GREY, lw=1.1, zorder=2))
    ax.add_patch(Rectangle((6.73, 1.65), 0.36, 0.36, fc="white", ec="black", lw=1.1, zorder=3))
    ax.add_patch(Rectangle((6.82, 1.74), 0.18, 0.18, fc="black", zorder=3))
    ax.text(6.91, 2.25, "object marker (id 0–3)", fontsize=7.8, color=GREY, ha="center",
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.85), zorder=4)
    p_obj = (6.91, 1.32)
    ax.add_patch(Circle(p_obj, 0.085, fc=OBJ, ec="black", lw=0.7, zorder=6))

    # error vector between the two points -- the HERO element (thick, saturated)
    ax.add_patch(FancyArrowPatch(p_grip, p_obj, arrowstyle="-|>", mutation_scale=26,
                                 lw=3.4, color=ERR, zorder=7))
    ax.text((p_grip[0] + p_obj[0]) / 2, 1.66,
            r"$\mathbf{e}=p_{\mathrm{obj}}-(p_{\mathrm{marker}}+t_{\mathrm{tip}})$",
            fontsize=11, color=ERR, ha="center", zorder=8, weight="bold",
            bbox=dict(boxstyle="round,pad=0.22", fc="white", ec=ERR, lw=1.0))

    # point labels with white bbox + leader lines (now unambiguous)
    plabel(r"$p_{\mathrm{obj}}$", p_obj, (8.25, 2.95), OBJ)
    plabel(r"$p_{\mathrm{marker}}+t_{\mathrm{tip}}$", p_grip, (1.55, 3.25), GRIP)

    # single shared lens apex so both rays visibly come from ONE camera; rays end at the markers
    lens = (5.0, 6.95)
    ax.add_patch(Circle(lens, 0.05, fc=GREY, ec="black", lw=0.5, zorder=4))
    arrow(ax, lens, (6.91, 2.05), color=OBJ, lw=1.0, style="-", ls=(0, (4, 3)), z=1)   # -> object marker
    arrow(ax, lens, (3.14, 2.46), color=GRIP, lw=1.0, style="-", ls=(0, (4, 3)), z=1)  # -> gripper marker

    # No baked-in title -- the LaTeX \caption carries the title + "Phase 4C, baseline" framing.
    fig.tight_layout()
    out = os.path.join(OUT, "servo_schematic.png")
    fig.savefig(out, dpi=200, bbox_inches="tight"); plt.close(fig)
    print("wrote", os.path.normpath(out))


# =====================================================================
# F10 — System / experimental pipeline
# =====================================================================
def make_pipeline():
    fig, ax = plt.subplots(figsize=(8.4, 4.2))
    ax.set_xlim(0, 14); ax.set_ylim(0, 7); ax.axis("off")

    # two depth sources
    rbox(ax, 0.3, 4.7, 3.0, 1.4,
         "Gazebo Classic 11 twin\n(clean sim depth)\n+ ROS 2 Humble", ec=WSL, fs=8.3)
    rbox(ax, 0.3, 2.7, 3.0, 1.1,
         "Depth-noise injector\naxial · lateral · edge · dropout", ec=WSL, fc="#eafaf0", fs=8.3)
    rbox(ax, 0.3, 0.7, 3.0, 1.3,
         "Real OAK-D SR PoE\n(ToF, overhead 0.68 m)", ec=WIN, fs=8.3)

    # perception
    perc_c, perc = rbox(ax, 4.6, 2.3, 3.6, 2.2,
                        "Perception\n(median-centroid)\n\nHSV label-only ·\ndepth table-removal (>5 mm) ·\n"
                        "connected components ·\nper-axis median centroid", ec=WSL, fs=8.2)
    ax.text(perc[0] + perc[2] / 2, perc[1] - 0.28, "→ table-frame (x, y), translation-only",
            fontsize=8, color=GREY, ha="center", style="italic")

    # IK
    ik_c, _ = rbox(ax, 9.0, 2.8, 2.3, 1.3,
                   "Analytic IK\n(5-DOF, closed-form)\nslanted approach", ec=WSL, fs=8.3)
    # grasp
    grasp_c, _ = rbox(ax, 11.6, 2.55, 2.1, 1.8,
                      "Grasp\n(predefined angle)\n\nsim: analytic\nfeasibility |\nreal: closed-loop\nvisual servo",
                      ec=WIN, fs=8.0)

    # arrows
    arrow(ax, (3.3, 5.4), (4.6, 3.9), color=GREY, rad=-0.1)         # sim depth -> injector path
    arrow(ax, (3.3, 3.25), (4.6, 3.4), color=GREY)                  # injector -> perception
    arrow(ax, (3.3, 1.35), (4.6, 3.0), color=GREY, rad=0.1)         # real depth -> perception
    arrow(ax, (1.8, 4.7), (1.8, 3.8), color=GREY, lw=1.2)          # sim -> injector
    arrow(ax, (8.2, 3.4), (9.0, 3.45), color=GREY)                 # perception -> IK
    arrow(ax, (11.3, 3.45), (11.6, 3.45), color=GREY)              # IK -> grasp

    # host legend
    ax.add_patch(FancyBboxPatch((0.3, 6.25), 0.3, 0.3, boxstyle="round,pad=0.02", ec=WIN, fc="white", lw=1.6))
    ax.text(0.75, 6.4, "Windows host (camera + arm)", fontsize=8, va="center", color=WIN)
    ax.add_patch(FancyBboxPatch((6.0, 6.25), 0.3, 0.3, boxstyle="round,pad=0.02", ec=WSL, fc="white", lw=1.6))
    ax.text(6.45, 6.4, "WSL host (perception · simulation · analysis)", fontsize=8, va="center", color=WSL)

    ax.set_title("System pipeline: clean sim / noisy sim / real share one perception+grasp path",
                 fontsize=10.5, weight="bold")
    fig.tight_layout()
    out = os.path.join(OUT, "system_pipeline.png")
    fig.savefig(out, dpi=200, bbox_inches="tight"); plt.close(fig)
    print("wrote", os.path.normpath(out))


if __name__ == "__main__":
    make_servo()
    make_pipeline()
