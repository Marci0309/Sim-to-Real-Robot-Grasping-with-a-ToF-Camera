#!/usr/bin/env python3
"""Probe how steep (top-down) an approach the arm can achieve at various object
layouts, to choose positions where the gripper can descend ONTO the object.
feasible_approach returns the STEEPEST reachable tilt below horizontal
(90 deg = perfectly vertical/top-down)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from xarm1s_manipulation import arm_ik as ik  # noqa: E402

# object grasp heights = object CENTER (table top 0.75 + half-height)
ZC = {"coke_can": 0.783, "banana": 0.77, "golf_ball": 0.771, "blue_box": 0.765}
NAMES = ["coke_can", "banana", "golf_ball", "blue_box"]

GRIDS = {
    "current (r=0.29)": [(0.25, 0.15), (0.25, -0.15), (-0.25, 0.15), (-0.25, -0.15)],
    "r=0.21": [(0.16, 0.13), (0.16, -0.13), (-0.16, 0.13), (-0.16, -0.13)],
    "r=0.18": [(0.14, 0.11), (0.14, -0.11), (-0.14, 0.11), (-0.14, -0.11)],
    "r=0.16": [(0.12, 0.10), (0.12, -0.10), (-0.12, 0.10), (-0.12, -0.10)],
    "r=0.14": [(0.10, 0.10), (0.10, -0.10), (-0.10, 0.10), (-0.10, -0.10)],
}

for gname, pts in GRIDS.items():
    print(f"\n=== {gname} ===")
    for nm, (x, y) in zip(NAMES, pts):
        res = ik.feasible_approach((x, y, ZC[nm]), min_clearance=0.75)
        if res is None:
            print(f"  {nm:10} UNREACHABLE")
        else:
            tilt, _ = res
            print(f"  {nm:10} steepest tilt = {math.degrees(tilt):4.0f} deg "
                  f"below horizontal  (90 = straight down)")
