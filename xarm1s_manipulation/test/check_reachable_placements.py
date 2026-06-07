#!/usr/bin/env python3
"""Standalone pre-flight for vary_placement sampling (no Gazebo needed).

Instantiates the real GraspExecutor node offline and exercises
_reachable_placements for every object, checking that:
  * the module imports cleanly after the vary_placement edits,
  * each object yields the full n requested reachable poses,
  * positions vary (full workspace coverage, not a fixed spot),
  * radii stay inside [0.12, 0.30] and every pose passes feasible_approach,
  * per-object orientation rules hold (coke lies on its side roll=pi/2).
"""
import math
import sys

import numpy as np
import rclpy

from xarm1s_manipulation.grasp_executor import GraspExecutor, OBJECTS
from xarm1s_manipulation import arm_ik as ik

rclpy.init()
node = GraspExecutor()
N, SEED = 20, 0
ok = True
print(f"{'object':<10} {'got/req':>8} {'r_min':>7} {'r_max':>7} "
      f"{'uniq_xy':>8} {'roll':>6} {'all_reachable':>14}")
for label in OBJECTS:
    poses = node._reachable_placements(label, N, SEED)
    rs = [math.hypot(p[0], p[1]) for p in poses]
    uniq = len({(round(p[0], 4), round(p[1], 4)) for p in poses})
    rolls = {round(p[3], 4) for p in poses}
    reach = all(
        ik.feasible_approach((p[0], p[1], OBJECTS[label]["grasp_z"]),
                             min_clearance=0.75) is not None
        for p in poses)
    exp_roll = round(math.pi / 2.0, 4) if label == "coke_can" else 0.0
    good = (len(poses) == N and uniq == N
            and min(rs) >= 0.12 - 1e-6 and max(rs) <= 0.30 + 1e-6
            and reach and rolls == {exp_roll})
    ok = ok and good
    print(f"{label:<10} {len(poses):>4}/{N:<3} {min(rs):>7.3f} {max(rs):>7.3f} "
          f"{uniq:>8} {sorted(rolls)!s:>6} {str(reach):>14}  "
          f"{'OK' if good else 'FAIL'}")

# determinism: same seed -> identical samples
again = node._reachable_placements("golf_ball", N, SEED)
det = again == node._reachable_placements("golf_ball", N, SEED)
print(f"\ndeterministic (same seed -> same poses): {det}")
ok = ok and det

node.destroy_node()
rclpy.shutdown()
print("\nRESULT:", "ALL OK" if ok else "FAILURES PRESENT")
sys.exit(0 if ok else 1)
