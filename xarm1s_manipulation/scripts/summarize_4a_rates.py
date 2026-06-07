#!/usr/bin/env python3
"""Phase-4A varied-placement success-rate summary (table + figure).

Reads the manipulation sweep CSV (``manip_results.csv``) produced by
``rerun_4a_sweep.sh`` in ``vary_placement`` mode -- one analytic grasp-feasible
trial per row, tagged by noise condition (``tag``) and object -- and emits a
thesis-ready summary:

  * ``phase4a_success_table.txt``  -- aligned condition x object table, each cell
    ``rate% +-CI`` with the trial count, plus an overall column.
  * ``phase4a_success_table.csv``  -- the same numbers machine-readable
    (one row per tag x object: n, k, rate, ci, mean placement error).
  * ``phase4a_rate_vs_axial_scale.png`` -- grasp-feasible rate vs the depth-noise
    ``axial_scale`` (clean=0, then x1..x20), one line per object + overall, with
    95% Wald CI error bars.

Success metric = ``grasp_feasible`` (the analytic perceived-centre-within-jaw
test; see grasp_executor.py). The 95% CI is the Wald interval
``1.96*sqrt(p(1-p)/n)`` -- the same formula the executor's summarise() prints.

Usage:
    python3 summarize_4a_rates.py [CSV] [--outdir DIR]
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

# Condition -> (axial_scale for the x-axis, short display label). sim_clean has
# NO injector, plotted at scale 0; the rest are the measured-OAK-D noise scaled
# by the ablation factor. Order = the sweep's condition order.
COND = [
    ("sim_clean",          0.0,  "clean"),
    ("sim_noisy_oakd",     1.0,  "x1"),
    ("sim_noisy_oakd_x2",  2.0,  "x2"),
    ("sim_noisy_oakd_x5",  5.0,  "x5"),
    ("sim_noisy_oakd_x10", 10.0, "x10"),
    ("sim_noisy_oakd_x20", 20.0, "x20"),
]
OBJECTS = ["coke_can", "banana", "golf_ball", "blue_box"]


def wald_ci(k: int, n: int) -> float:
    """95% Wald half-width for k successes in n trials (0 if n==0)."""
    if n == 0:
        return 0.0
    p = k / n
    return 1.96 * math.sqrt(max(p * (1.0 - p), 0.0) / n)


def load(csv_path):
    """(tag, object) -> dict(n, k, place_errs[]). Reads grasp_feasible, falling
    back to the legacy pick_success column for older CSVs."""
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        sys.exit(f"no rows in {csv_path}")
    agg = {}
    for r in rows:
        key = (r["tag"], r["object"])
        a = agg.setdefault(key, {"n": 0, "k": 0, "errs": []})
        a["n"] += 1
        ok = int(r.get("grasp_feasible") or r.get("pick_success") or 0)
        a["k"] += ok
        pe = r.get("place_err_mm", "")
        if ok and pe not in ("", None):
            a["errs"].append(float(pe))
    return agg, len(rows)


def fmt_cell(a):
    if a is None or a["n"] == 0:
        return "    --     "
    p = 100.0 * a["k"] / a["n"]
    ci = 100.0 * wald_ci(a["k"], a["n"])
    return f"{p:5.1f}+-{ci:4.1f}"


def write_table_txt(agg, path):
    present = [(t, s, lab) for (t, s, lab) in COND
               if any((t, o) in agg for o in OBJECTS)]
    lines = []
    lines.append("Phase 4A grasp-feasible success rate (%) +- 95% CI   "
                 "[varied placements, bias correction OFF]")
    lines.append("metric = analytic grasp_feasible (perceived centre within the "
                 "per-object jaw capture tolerance)")
    lines.append("")
    header = f"{'condition':<16}{'scale':>6}   " + "".join(
        f"{o[:9]:>13}" for o in OBJECTS) + f"{'OVERALL':>15}{'N':>6}"
    lines.append(header)
    lines.append("-" * len(header))
    for t, s, lab in present:
        tot_n = sum(agg.get((t, o), {"n": 0})["n"] for o in OBJECTS)
        tot_k = sum(agg.get((t, o), {"k": 0})["k"] for o in OBJECTS)
        scale = "clean" if s == 0 else f"x{s:g}"
        row = f"{t:<16}{scale:>6}   "
        for o in OBJECTS:
            row += f"{fmt_cell(agg.get((t, o))):>13}"
        ov = {"n": tot_n, "k": tot_k}
        row += f"{fmt_cell(ov):>15}{tot_n:>6}"
        lines.append(row)
    text = "\n".join(lines) + "\n"
    open(path, "w").write(text)
    return text


def write_table_csv(agg, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tag", "axial_scale", "object", "n", "k", "rate",
                    "ci95", "mean_place_err_mm"])
        for t, s, _lab in COND:
            for o in OBJECTS + ["OVERALL"]:
                if o == "OVERALL":
                    n = sum(agg.get((t, oo), {"n": 0})["n"] for oo in OBJECTS)
                    k = sum(agg.get((t, oo), {"k": 0})["k"] for oo in OBJECTS)
                    errs = [e for oo in OBJECTS
                            for e in agg.get((t, oo), {"errs": []})["errs"]]
                else:
                    a = agg.get((t, o))
                    if a is None:
                        continue
                    n, k, errs = a["n"], a["k"], a["errs"]
                if n == 0:
                    continue
                rate = k / n
                me = (sum(errs) / len(errs)) if errs else ""
                w.writerow([t, s, o, n, k, f"{rate:.4f}",
                            f"{wald_ci(k, n):.4f}",
                            f"{me:.2f}" if me != "" else ""])


def make_figure(agg, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7.0, 4.6))
    markers = {"coke_can": "o", "banana": "s", "golf_ball": "^",
               "blue_box": "D"}
    # evenly-spaced ordinal x (clean, x1, x2, x5, x10, x20): the scales (0,1,2,5,
    # 10,20) crowd the low end on a linear axis, so plot per-condition positions
    # and label each with its scale -- clearer for a 6-point threshold plot.
    pos = {t: i for i, (t, _s, _l) in enumerate(COND)}
    for o in OBJECTS:
        xs, ys, es = [], [], []
        for t, _s, _lab in COND:
            a = agg.get((t, o))
            if a is None or a["n"] == 0:
                continue
            xs.append(pos[t])
            ys.append(100.0 * a["k"] / a["n"])
            es.append(100.0 * wald_ci(a["k"], a["n"]))
        if xs:
            ax.errorbar(xs, ys, yerr=es, marker=markers[o], capsize=3,
                        linewidth=1.6, markersize=6, label=o)
    # overall (dashed, heavier)
    oxs, oys, oes = [], [], []
    for t, _s, _lab in COND:
        n = sum(agg.get((t, o), {"n": 0})["n"] for o in OBJECTS)
        k = sum(agg.get((t, o), {"k": 0})["k"] for o in OBJECTS)
        if n == 0:
            continue
        oxs.append(pos[t])
        oys.append(100.0 * k / n)
        oes.append(100.0 * wald_ci(k, n))
    if oxs:
        ax.errorbar(oxs, oys, yerr=oes, marker="*", color="k",
                    linestyle="--", linewidth=2.0, markersize=10,
                    capsize=3, label="overall", zorder=5)

    ax.set_xlabel("depth-noise axial scale  (clean = no injector, "
                  "x1 = measured OAK-D)")
    ax.set_ylabel("grasp-feasible success rate (%)")
    ax.set_title("Phase 4A: manipulation success vs depth noise\n"
                 "(varied placements over the reachable workspace, bias off)")
    ax.set_xticks(range(len(COND)))
    ax.set_xticklabels(["clean" if s == 0 else f"x{s:g}"
                        for _t, s, _l in COND])
    ax.set_xlim(-0.3, len(COND) - 0.7)
    ax.set_ylim(-3, 105)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left", fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_csv = os.path.normpath(os.path.join(
        here, "..", "..", "evaluation", "results", "manip_results.csv"))
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default=default_csv)
    ap.add_argument("--outdir", default=None,
                    help="output dir (default = the CSV's directory)")
    a = ap.parse_args()
    csv_path = os.path.expanduser(a.csv)
    if not os.path.exists(csv_path):
        sys.exit(f"CSV not found: {csv_path}")
    outdir = a.outdir or os.path.dirname(csv_path)
    os.makedirs(outdir, exist_ok=True)

    agg, nrows = load(csv_path)
    txt = write_table_txt(agg, os.path.join(outdir, "phase4a_success_table.txt"))
    write_table_csv(agg, os.path.join(outdir, "phase4a_success_table.csv"))
    make_figure(agg, os.path.join(outdir, "phase4a_rate_vs_axial_scale.png"))

    print(txt)
    print(f"read {nrows} rows from {csv_path}")
    print("wrote -> " + outdir)
    for f in ("phase4a_success_table.txt", "phase4a_success_table.csv",
              "phase4a_rate_vs_axial_scale.png"):
        print("   " + f)


if __name__ == "__main__":
    main()
