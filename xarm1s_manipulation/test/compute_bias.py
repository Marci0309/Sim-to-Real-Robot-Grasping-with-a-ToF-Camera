#!/usr/bin/env python3
"""Per-object mean perception offset (detected - truth) for each run tag in
run.csv. The sim_clean means are the constant Sketchfab-mesh biases used as the
default sim-only grasp_bias_correction (coke/banana) for Phase 4A.
"""
import collections
import csv
import os
import statistics as st

CSV = os.path.expanduser("~/thesis_ws/src/evaluation/results/run.csv")
OBJ = ["coke_can", "banana", "golf_ball", "blue_box"]

per_tag = collections.defaultdict(lambda: collections.defaultdict(list))
with open(CSV) as f:
    for row in csv.DictReader(f):
        tag = row["tag"]
        for o in OBJ:
            for ax in ("dx", "dy", "dz"):
                try:
                    per_tag[tag][(o, ax)].append(float(row[f"{o}_{ax}"]))
                except (ValueError, KeyError):
                    pass

for tag in sorted(per_tag):
    print(f"\n=== {tag} ===")
    for o in OBJ:
        xs = per_tag[tag][(o, "dx")]
        if not xs:
            continue
        dx = st.mean(per_tag[tag][(o, "dx")]) * 1000
        dy = st.mean(per_tag[tag][(o, "dy")]) * 1000
        dz = st.mean(per_tag[tag][(o, "dz")]) * 1000
        print(f"  {o:<10} n={len(xs):5d}  dx={dx:+7.1f} dy={dy:+7.1f} "
              f"dz={dz:+7.1f} mm")
