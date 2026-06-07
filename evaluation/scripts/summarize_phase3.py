#!/usr/bin/env python3
"""Phase-3 sim pose-accuracy summary (CURRENT models) from run.csv.

Reads evaluation/results/run.csv (sim_* tags from rerun_phase3_sweep.sh) and
computes, per condition and per object:
  * PLANAR RMSE = sqrt(mean(dx^2 + dy^2))   -- the robust table-XY centroid
  * 3D RMSE     = sqrt(mean(norm^2))         -- incl. the BOP visible-top dz

Emits a table (phase3_pose_table.{txt,csv}) and a figure
(phase3_pose_vs_noise.png) of pose RMSE vs axial-noise scale, which shows the
flat-until-~x10-then-collapse-at-x20 behaviour: the median-centroid absorbs
measured ToF noise, so pose accuracy is unchanged at realistic noise levels --
the SQ2 negative result, now on the current slim-coke + re-centred-banana models.

Author: Marcell Mor Szokedencsi (s5173191), University of Groningen
"""
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))
RUN = os.path.join(RES, "run.csv")
OBJECTS = ["coke_can", "banana", "golf_ball", "blue_box"]

# tag -> (display label, axial_scale)   scale=None for the Nguyen literature model
COND = [
    ("sim_clean",          "clean",    0.0),
    ("sim_noisy_nguyen",   "nguyen",   None),
    ("sim_noisy_oakd",     "oakd x1",  1.0),
    ("sim_noisy_oakd_x2",  "oakd x2",  2.0),
    ("sim_noisy_oakd_x5",  "oakd x5",  5.0),
    ("sim_noisy_oakd_x10", "oakd x10", 10.0),
    ("sim_noisy_oakd_x20", "oakd x20", 20.0),
]


def rmse(a):
    a = np.asarray(a, float)
    return float(np.sqrt(np.mean(a ** 2))) if len(a) else float("nan")


def planar_3d(rs, obj):
    pl, nm = [], []
    for r in rs:
        if r.get(obj + "_norm", "") in ("", "nan"):
            continue
        pl.append(1000.0 * math.hypot(float(r[obj + "_dx"]), float(r[obj + "_dy"])))
        nm.append(1000.0 * float(r[obj + "_norm"]))
    return rmse(pl), rmse(nm), len(nm)


def main():
    rows = {}
    for r in csv.DictReader(open(RUN)):
        rows.setdefault(r["tag"], []).append(r)

    # ---------- table ----------
    L = ["Phase-3 sim pose accuracy -- CURRENT models (slim 250 ml coke + re-centred banana)",
         "Per condition: PLANAR RMSE / 3D RMSE (mm).  Median-centroid perception.", ""]
    hdr = f"{'condition':<12}" + "".join(f"{o:>15}" for o in OBJECTS)
    L += [hdr, "-" * len(hdr)]
    csv_rows = []
    for tag, label, scale in COND:
        rs = rows.get(tag, [])
        if not rs:
            continue
        cells = []
        for o in OBJECTS:
            pl, nm, n = planar_3d(rs, o)
            cells.append(f"{pl:.1f}/{nm:.1f}")
            csv_rows.append([tag, label, "" if scale is None else scale, o, n,
                             round(pl, 2), round(nm, 2)])
        L.append(f"{label:<12}" + "".join(f"{c:>15}" for c in cells))
    txt = "\n".join(L)
    print(txt)
    open(os.path.join(RES, "phase3_pose_table.txt"), "w").write(txt + "\n")
    with open(os.path.join(RES, "phase3_pose_table.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tag", "label", "axial_scale", "object", "n",
                    "planar_rmse_mm", "rmse3d_mm"])
        w.writerows(csv_rows)

    # ---------- figure: RMSE vs axial scale (clean = scale 0) ----------
    series = [(t, s) for t, _, s in COND if s is not None]
    xs = [s for _, s in series]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True)
    for ax, (metric, idx) in zip(axes, [("planar RMSE", 0), ("3D RMSE", 1)]):
        for o in OBJECTS:
            ys = [planar_3d(rows.get(t, []), o)[idx] for t, _ in series]
            ax.plot(xs, ys, "o-", label=o)
        ax.axvline(10, ls="--", color="grey", alpha=0.6)
        ax.set_xlabel("axial noise scale  (x measured OAK-D;  0 = clean)")
        ax.set_ylabel(f"pose {metric} (mm)")
        ax.set_title(metric)
        ax.grid(alpha=0.3)
    axes[0].legend()
    fig.suptitle("Phase-3 pose accuracy vs ToF noise scale (current models)\n"
                 "flat until ~x10 (median-centroid absorbs noise), collapses at x20 "
                 "-- the SQ2 negative result", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "phase3_pose_vs_noise.png"), dpi=130)
    print("\nwrote -> phase3_pose_table.{txt,csv} + phase3_pose_vs_noise.png  in", RES)


if __name__ == "__main__":
    main()
