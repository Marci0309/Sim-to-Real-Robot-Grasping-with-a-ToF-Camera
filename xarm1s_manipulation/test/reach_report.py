#!/usr/bin/env python3
"""Geometric sanity report for the per-object grasp solutions (no ROS)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from xarm1s_manipulation import arm_ik as ik  # noqa: E402

TARGETS = {
    "coke_can":  (0.25, 0.15, 0.816),
    "banana":    (0.25, -0.15, 0.820),
    "golf_ball": (-0.25, 0.15, 0.793),
    "blue_box":  (-0.25, -0.15, 0.780),
    "placement": (0.30, 0.0, 0.80),
}
TABLE_Z = 0.75
ORDER = ["base", "J6", "shoulder", "elbow", "wrist", "tip"]

for clearance in (None, TABLE_Z):
    tag = "no clearance" if clearance is None else f"clearance>={TABLE_Z}"
    print(f"\n################  {tag}  ################")
    for name, t in TARGETS.items():
        res = ik.feasible_approach(t, min_clearance=clearance)
        if res is None:
            print(f"{name:<10} UNREACHABLE")
            continue
        tilt, cfg = res
        pts = ik.link_world_positions(cfg)
        minz = min(pts[k][2] for k in ("shoulder", "elbow", "wrist", "tip"))
        # horizontal distance of elbow from base axis (folds behind base?)
        print(f"{name:<10} tilt={math.degrees(tilt):4.0f}deg  "
              f"min_link_z={minz:.3f}")
        for k in ORDER:
            p = pts[k]
            print(f"    {k:<9} x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f}")
