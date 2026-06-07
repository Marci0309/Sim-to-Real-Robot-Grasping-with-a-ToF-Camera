#!/usr/bin/env python3
"""Phase 4A perception-error overlay -- the *why* behind each object's grasp-
feasible rate. For one noise condition it plots, per object, the per-trial
perceived-vs-true centre error (mm, world frame) as a scatter against that
object's jaw capture-tolerance ring. A point inside the ring = the analytic
grasp_feasible test passes; outside = miss. This visualises directly why coke
(1.5 mm ring) and banana (~29 mm offset) sit at 0 % while blue_box (all inside
7.5 mm) is 100 %.

    python3 plot_perception_error.py [condition_tag]   # default sim_clean

Reads evaluation/results/manip_results.csv; writes phase4a_perception_error.png.
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.normpath(os.path.join(HERE, "..", "..", "evaluation", "results"))
CSV = os.path.join(RESULTS, "manip_results.csv")

# per-object jaw capture tolerance (mm) = (55 mm gripper - object width)/2
TOL_MM = {"coke_can": 1.5, "banana": 10.0, "golf_ball": 6.0, "blue_box": 7.5}
ORDER = ["coke_can", "banana", "golf_ball", "blue_box"]


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else "sim_clean"
    rows = list(csv.DictReader(open(CSV)))
    fig, axes = plt.subplots(2, 2, figsize=(10, 9))
    for ax, obj in zip(axes.ravel(), ORDER):
        tol = TOL_MM[obj]
        dxs, dys, ok = [], [], 0
        for r in rows:
            if r["tag"] != tag or r["object"] != obj or r["perceived_x"] == "":
                continue
            dx = (float(r["perceived_x"]) - float(r["true_x"])) * 1000.0
            dy = (float(r["perceived_y"]) - float(r["true_y"])) * 1000.0
            dxs.append(dx)
            dys.append(dy)
            if math.hypot(dx, dy) <= tol:
                ok += 1
        n = len(dxs)
        rate = 100.0 * ok / n if n else 0.0
        # colour: green = inside tol (feasible), red = outside (miss)
        cols = ["tab:green" if math.hypot(x, y) <= tol else "tab:red"
                for x, y in zip(dxs, dys)]
        lim = max(tol, max((math.hypot(x, y) for x, y in zip(dxs, dys)),
                           default=tol)) * 1.25
        ax.add_patch(Circle((0, 0), tol, fill=True, color="tab:green",
                            alpha=0.10, zorder=0))
        ax.add_patch(Circle((0, 0), tol, fill=False, color="tab:green",
                            lw=2.0, label=f"jaw tol = {tol:g} mm"))
        ax.scatter(dxs, dys, c=cols, s=40, edgecolors="k", linewidths=0.4,
                   zorder=3)
        ax.plot(0, 0, "k+", ms=12, mew=2, zorder=4)   # true centre
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.axhline(0, color="k", lw=0.5, alpha=0.4)
        ax.axvline(0, color="k", lw=0.5, alpha=0.4)
        ax.set_title(f"{obj}   tol={tol:g} mm   "
                     f"feasible {ok}/{n} = {rate:.0f}%", fontsize=11)
        ax.set_xlabel("perceived - true  E-W (mm)")
        ax.set_ylabel("perceived - true  N-S (mm)")
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle(
        f"Phase 4A perception error vs jaw tolerance  [{tag}, bias off]\n"
        "green ring = capture tolerance; a point inside = grasp-feasible, "
        "outside = miss", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = os.path.join(RESULTS, "phase4a_perception_error.png")
    fig.savefig(out, dpi=130)
    print(f"wrote -> {out}")


if __name__ == "__main__":
    main()
