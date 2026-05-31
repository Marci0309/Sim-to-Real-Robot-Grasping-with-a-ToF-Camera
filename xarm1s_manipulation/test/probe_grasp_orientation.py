#!/usr/bin/env python3
"""Check the wrist-roll math: after computing J2 for each object's minor axis,
the gripper closing axis (local X) should be ALIGNED with that minor axis
(align ~1.0 = jaws close across it; ~0 = perpendicular = wrong)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from xarm1s_manipulation import arm_ik as ik  # noqa: E402

OBJ = {
    "coke_can":  ((0.14, 0.11, 0.783), (1.0, 0.0)),
    "banana":    ((0.14, -0.11, 0.770), (0.0, 1.0)),
    "golf_ball": ((-0.14, 0.11, 0.771), None),
    "blue_box":  ((-0.14, -0.11, 0.765), (0.0, 1.0)),
}
for name, (tgt, minor) in OBJ.items():
    tilt, cfg = ik.feasible_approach(tgt, min_clearance=0.75)
    j2 = 0.0
    if minor is not None:
        j2 = ik.wrist_roll_for_axis(cfg, minor)
        cfg["xarm_2_joint"] = j2
    closing = ik.forward_kinematics(cfg)[:3, 0]          # gripper local X (world)
    ch = closing[:2] / (np.linalg.norm(closing[:2]) + 1e-9)
    if minor is None:
        print(f"{name:10} tilt={math.degrees(tilt):3.0f}  J2=  0 (sphere)  "
              f"closing_xy=({ch[0]:+.2f},{ch[1]:+.2f})")
    else:
        m = np.array(minor) / np.linalg.norm(minor)
        align = abs(float(ch @ m))
        print(f"{name:10} tilt={math.degrees(tilt):3.0f}  J2={math.degrees(j2):+4.0f}deg  "
              f"closing_xy=({ch[0]:+.2f},{ch[1]:+.2f})  align_with_minor={align:.3f}")
