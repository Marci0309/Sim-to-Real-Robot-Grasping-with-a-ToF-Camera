#!/usr/bin/env python3
"""Phase-4B sim-to-real gap on pose (PLANAR), CURRENT models.

SIM baseline = the Phase-4A varied-placement sweep (evaluation/results/manip_results.csv),
which used the CURRENT models (slim red coke cylinder + re-centred solid-yellow banana)
and the same depth-segmentation perception, sampled over 20 varied poses/object -- the
closest match to the real protocol (varied placements). We use it rather than the
Phase-3 run.csv sim rows because Phase-4A is varied-placement; Phase-3 is a single static
scene. (run.csv's sim rows were re-run on the current models on 2026-06-07 and are no
longer stale -- see rerun_phase3_sweep.sh -- but remain static-scene, less comparable.)

REAL = the Phase-4B ingested scenes (run.csv, tag `real`).

Metric: PLANAR error sqrt(dx^2+dy^2) for both.
  * sim:  perceived_xy - true_xy  from manip_results.csv (bias OFF, as captured)
  * real: dx,dy                   from run.csv

    python3 compute_sim_to_real_gap.py

Writes evaluation/results/: phase4b_gap_table.{txt,csv}, phase4b_sim_to_real_gap.png
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
MANIP_CSV = os.path.join(RES, "manip_results.csv")     # current-model SIM (Phase-4A sweep)
RUN_CSV = os.path.join(RES, "run.csv")                 # REAL rows (Phase-4B)

OBJECTS = ["coke_can", "banana", "golf_ball", "blue_box"]
SIM_TAGS = ["sim_clean", "sim_noisy_oakd"]             # current-model sim conditions
SHOW = ["sim_clean", "sim_noisy_oakd", "real"]


def sim_planar(rows, tag, obj):
    """planar mm from manip_results (perceived_xy - true_xy)."""
    out = []
    for r in rows:
        if r.get("tag") != tag or r.get("object") != obj:
            continue
        px, py = r.get("perceived_x", ""), r.get("perceived_y", "")
        if px == "" or py == "":
            continue
        out.append(math.hypot(float(px) - float(r["true_x"]),
                              float(py) - float(r["true_y"])) * 1000.0)
    return out


def real_planar(rows, obj, tag="real"):
    """planar mm from run.csv (dx,dy)."""
    out = []
    for r in rows:
        if r.get("tag") != tag:
            continue
        dx, dy = r.get(f"{obj}_dx", ""), r.get(f"{obj}_dy", "")
        if dx == "" or dy == "":
            continue
        out.append(math.hypot(float(dx), float(dy)) * 1000.0)
    return out


def stats(es):
    if not es:
        return None
    a = np.asarray(es, float)
    return dict(n=len(es), rmse=float(np.sqrt(np.mean(a ** 2))),
                mae=float(np.mean(a)), med=float(np.median(a)))


def main():
    manip = list(csv.DictReader(open(MANIP_CSV))) if os.path.exists(MANIP_CSV) else []
    run = list(csv.DictReader(open(RUN_CSV))) if os.path.exists(RUN_CSV) else []

    table = {}
    for o in OBJECTS:
        for t in SIM_TAGS:
            table[(t, o)] = stats(sim_planar(manip, t, o))
        table[("real", o)] = stats(real_planar(run, o))

    # ---------- text table ----------
    L = ["Phase-4B sim-to-real gap -- PLANAR pose RMSE (mm)  [CURRENT models]",
         "sim = Phase-4A varied-placement sweep (slim coke + re-centred banana); "
         "real = Phase-4B scenes.", ""]
    hdr = f"{'condition':<16}" + "".join(f"{o:>13}" for o in OBJECTS)
    L += [hdr, "-" * len(hdr)]
    for t in SHOW:
        cells = []
        for o in OBJECTS:
            s = table[(t, o)]
            cells.append("--" if s is None else f"{s['rmse']:.1f}(n={s['n']})")
        L.append(f"{t:<16}" + "".join(f"{c:>13}" for c in cells))
    L += ["", "GAP  (real - sim_clean) planar RMSE, mm:"]
    for o in OBJECTS:
        rr, cc = table[("real", o)], table[("sim_clean", o)]
        if rr and cc:
            L.append(f"  {o:<10} real {rr['rmse']:4.1f}  sim {cc['rmse']:4.1f}  "
                     f"gap {rr['rmse'] - cc['rmse']:+.1f}")
    txt = "\n".join(L)
    print(txt)
    open(os.path.join(RES, "phase4b_gap_table.txt"), "w").write(txt + "\n")

    with open(os.path.join(RES, "phase4b_gap_table.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["condition", "object", "n", "planar_rmse_mm", "planar_mae_mm",
                    "planar_median_mm"])
        for t in SHOW:
            for o in OBJECTS:
                s = table[(t, o)]
                if s:
                    w.writerow([t, o, s["n"], round(s["rmse"], 2),
                                round(s["mae"], 2), round(s["med"], 2)])

    # ---------- figure: one panel, all 4 objects, 3 conditions ----------
    x = np.arange(len(OBJECTS))
    width = 0.8 / len(SHOW)
    fig, ax = plt.subplots(figsize=(9.5, 5.2))
    for i, t in enumerate(SHOW):
        vals = [(table[(t, o)]["rmse"] if table[(t, o)] else 0.0) for o in OBJECTS]
        ax.bar(x + i * width - 0.4 + width / 2, vals, width, label=t)
    ax.set_xticks(x)
    ax.set_xticklabels(OBJECTS)
    ax.set_ylabel("planar pose RMSE (mm)")
    ax.set_title("Phase-4B sim-to-real gap (current models): planar pose RMSE per object\n"
                 "sim = Phase-4A varied-placement sweep; real = Phase-4B scenes", fontsize=11)
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "phase4b_sim_to_real_gap.png"), dpi=130)
    print(f"\nwrote -> phase4b_gap_table.{{txt,csv}} + phase4b_sim_to_real_gap.png  in {RES}")


if __name__ == "__main__":
    main()
