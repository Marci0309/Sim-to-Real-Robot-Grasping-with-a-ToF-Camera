#!/usr/bin/env python3
"""Phase 4C results summary (thesis Results 5.5) from real_manip_results.csv.

For each object computes the REAL physical lift-success rate (k/N) with a Wilson
95% CI (honest at the 0%/100% boundary, unlike Wald), and places it beside the
two model-linked predictors so the comparison is transparent:
  * sim analytic grasp-feasible % (Phase 4A sim_clean)  -- the necessary-condition ceiling
  * real planar pose error mm (Phase 4B `real`) vs the jaw capture tolerance -- the predictor

Writes evaluation/results/phase4c_real_table.{csv,txt} + phase4c_real_vs_pred.png.

    python3 summarize_phase4c.py
    python3 summarize_phase4c.py --csv /tmp/synth.csv --out-dir /tmp   # dry-test
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))
# capture tolerances are single-sourced in grasp_common (manipulation package)
import sys
sys.path.insert(0, os.path.abspath(os.path.join(
    HERE, "..", "..", "xarm1s_manipulation", "xarm1s_manipulation")))
try:
    import grasp_common as gc
    CAPTURE_TOL_MM = {o: gc.OBJECTS[o]["capture_tol"] * 1000 for o in gc.OBJECTS}
    OBJECTS = list(gc.OBJECTS)
except Exception:                                   # noqa: BLE001 (fallback if not importable)
    CAPTURE_TOL_MM = {"coke_can": 1.5, "banana": 10.0, "golf_ball": 6.0, "blue_box": 7.5}
    OBJECTS = list(CAPTURE_TOL_MM)


def wilson(k: int, n: int, z: float = 1.96):
    """Wilson score interval -> (rate, lo, hi) as fractions."""
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    p = k / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, centre - half), min(1.0, centre + half)


def read_real(csv_path):
    """{object: (k, n)} from real_manip_results.csv (skip dry-run / blank rows)."""
    agg = {o: [0, 0] for o in OBJECTS}
    if not os.path.exists(csv_path):
        return agg
    for r in csv.DictReader(open(csv_path)):
        o = r.get("object")
        ls = r.get("lift_success", "")
        if o in agg and ls not in ("", None):
            agg[o][1] += 1
            agg[o][0] += int(ls)
    return agg


def read_4a_feasible(path):
    out = {}
    if os.path.exists(path):
        for r in csv.DictReader(open(path)):
            if r["tag"] == "sim_clean" and r["object"] in OBJECTS:
                out[r["object"]] = float(r["rate"]) * 100
    return out


def read_4b_err(path):
    out = {}
    if os.path.exists(path):
        for r in csv.DictReader(open(path)):
            if r["condition"] == "real" and r["object"] in OBJECTS:
                out[r["object"]] = float(r["planar_rmse_mm"])
    return out


def predict(err_mm, tol_mm):
    if err_mm is None:
        return "n/a"
    if err_mm > 1.5 * tol_mm:
        return "~0% (err >> tol)"
    if err_mm > tol_mm:
        return "marginal/fail"
    if err_mm > 0.8 * tol_mm:
        return "marginal"
    return "likely (err < tol)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(RES, "real_manip_results.csv"))
    ap.add_argument("--out-dir", default=RES)
    args = ap.parse_args()

    real = read_real(args.csv)
    feas = read_4a_feasible(os.path.join(RES, "phase4a_success_table.csv"))
    err = read_4b_err(os.path.join(RES, "phase4b_gap_table.csv"))

    rows, tot_k, tot_n = [], 0, 0
    for o in OBJECTS:
        k, n = real[o]
        tot_k += k; tot_n += n
        rate, lo, hi = wilson(k, n)
        rows.append(dict(object=o, n=n, k=k, rate=rate, lo=lo, hi=hi,
                         sim_feasible=feas.get(o), pose_err=err.get(o),
                         tol=CAPTURE_TOL_MM[o],
                         predicted=predict(err.get(o), CAPTURE_TOL_MM[o])))
    orate, olo, ohi = wilson(tot_k, tot_n)

    # ---- text + csv ----
    L = ["Phase-4C real manipulation -- physical lift-and-hold success (Results 5.5)",
         "real lift = operator-scored; sim_feasible = Phase-4A analytic ceiling; "
         "pose_err = Phase-4B real planar RMSE vs jaw tolerance (the predictor).", ""]
    hdr = (f"{'object':<10} {'N':>3} {'lift':>7} {'rate%':>6} {'95% CI':>13} "
           f"{'sim_feas%':>9} {'err_mm':>7} {'tol_mm':>7}  predicted")
    L += [hdr, "-" * len(hdr)]
    for r in rows:
        ci = (f"[{r['lo']*100:4.0f},{r['hi']*100:4.0f}]"
              if r["n"] else "   --   ")
        L.append(f"{r['object']:<10} {r['n']:>3} {r['k']:>3}/{r['n']:<3} "
                 f"{r['rate']*100 if r['n'] else float('nan'):>5.0f} {ci:>13} "
                 f"{(r['sim_feasible'] if r['sim_feasible'] is not None else float('nan')):>9.0f} "
                 f"{(r['pose_err'] if r['pose_err'] is not None else float('nan')):>7.1f} "
                 f"{r['tol']:>7.1f}  {r['predicted']}")
    L += ["", f"OVERALL real lift-success: {tot_k}/{tot_n} = "
              f"{orate*100 if tot_n else float('nan'):.0f}% "
              f"(95% CI [{olo*100:.0f},{ohi*100:.0f}])" if tot_n else
              "OVERALL: no real trials yet (run the experiment)."]
    txt = "\n".join(L)
    print(txt)
    os.makedirs(args.out_dir, exist_ok=True)
    open(os.path.join(args.out_dir, "phase4c_real_table.txt"), "w").write(txt + "\n")
    with open(os.path.join(args.out_dir, "phase4c_real_table.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["object", "n", "k", "real_lift_rate", "ci95_lo", "ci95_hi",
                    "sim_feasible_pct", "real_pose_err_mm", "capture_tol_mm", "predicted"])
        for r in rows:
            w.writerow([r["object"], r["n"], r["k"],
                        round(r["rate"], 4) if r["n"] else "",
                        round(r["lo"], 4) if r["n"] else "",
                        round(r["hi"], 4) if r["n"] else "",
                        r["sim_feasible"], r["pose_err"], round(r["tol"], 2), r["predicted"]])

    # ---- figure: real lift vs sim feasible (left) ; pose err vs tol (right) ----
    x = np.arange(len(OBJECTS)); w = 0.38
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(12, 5))
    real_pct = [rows[i]["rate"] * 100 if rows[i]["n"] else 0 for i in range(len(OBJECTS))]
    yerr = [[(rows[i]["rate"] - rows[i]["lo"]) * 100 if rows[i]["n"] else 0 for i in range(len(OBJECTS))],
            [(rows[i]["hi"] - rows[i]["rate"]) * 100 if rows[i]["n"] else 0 for i in range(len(OBJECTS))]]
    sim_pct = [rows[i]["sim_feasible"] or 0 for i in range(len(OBJECTS))]
    axL.bar(x - w/2, sim_pct, w, label="sim feasible (4A)", color="#90a4ae")
    axL.bar(x + w/2, real_pct, w, yerr=yerr, capsize=4, label="real lift (4C)", color="#1565c0")
    axL.set_xticks(x); axL.set_xticklabels(OBJECTS); axL.set_ylim(0, 105)
    axL.set_ylabel("success rate (%)"); axL.legend()
    axL.set_title("Real lift-success vs sim analytic feasibility")
    errs = [rows[i]["pose_err"] or 0 for i in range(len(OBJECTS))]
    tols = [rows[i]["tol"] for i in range(len(OBJECTS))]
    axR.bar(x - w/2, errs, w, label="real pose err (4B)", color="#ef6c00")
    axR.bar(x + w/2, tols, w, label="jaw capture tol", color="#2e7d32")
    axR.set_xticks(x); axR.set_xticklabels(OBJECTS)
    axR.set_ylabel("mm"); axR.legend()
    axR.set_title("The predictor: real pose error vs jaw tolerance")
    fig.suptitle("Phase 4C: real manipulation vs the sim-feasible ceiling and the "
                 "4B pose-error-vs-tolerance predictor", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out_dir, "phase4c_real_vs_pred.png"), dpi=130)
    print(f"\nwrote phase4c_real_table.{{txt,csv}} + phase4c_real_vs_pred.png in {args.out_dir}")


if __name__ == "__main__":
    main()
