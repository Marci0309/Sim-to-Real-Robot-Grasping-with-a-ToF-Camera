#!/usr/bin/env python3
"""Headless validation of arm_ik (no ROS, no Gazebo).

Run directly:   python3 test/test_ik.py
Or via pytest:  python3 -m pytest test/test_ik.py -v

Checks:
  * FK∘IK round-trip: whenever IK returns a solution, the full-URDF forward
    kinematics lands within a few mm of the requested target (the planar solver
    ignores ~3 mm of lateral joint offset, so a small residual is expected).
  * A purely vertical top-down grasp is *infeasible* at the object positions
    (the headline kinematic finding).
  * All four objects + the placement target are reachable with a slanted
    approach, within joint limits.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from xarm1s_manipulation import arm_ik as ik  # noqa: E402

# Grasp targets (fingertip goal = visible top-centre of each object), from
# tabletop.world + pose_evaluator half-heights.
OBJECT_TARGETS = {   # tighter grid (r~0.18); fingertip ~5mm above table so the
                     # prongs descend AROUND the body (soft-attach hold)
    "coke_can":  (0.14, 0.11, 0.755),
    "banana":    (0.14, -0.11, 0.755),
    "golf_ball": (-0.14, 0.11, 0.755),
    "blue_box":  (-0.14, -0.11, 0.755),
}
PLACEMENT_TARGET = (0.27, 0.07, 0.79)  # farthest green-plate spot, release height

ROUNDTRIP_TOL_MM = 6.0   # planar-solver residual budget vs full FK


def test_fk_ik_roundtrip():
    """IK solutions must be confirmed by the full forward kinematics."""
    max_err = 0.0
    n_ok = 0
    for q in np.linspace(0.12, 0.30, 10):
        for z in np.linspace(0.78, 0.95, 8):
            for tilt in np.linspace(0.0, math.radians(50), 6):
                target = (float(q), 0.0, float(z))   # azimuth 0
                cfg = ik.inverse_kinematics(target, float(tilt))
                if cfg is None:
                    continue
                err = np.linalg.norm(ik.fk_position(cfg) - np.array(target))
                max_err = max(max_err, err)
                n_ok += 1
    assert n_ok > 50, f"too few reachable samples ({n_ok})"
    assert max_err * 1000 < ROUNDTRIP_TOL_MM, \
        f"round-trip error {max_err*1000:.2f} mm exceeds {ROUNDTRIP_TOL_MM} mm"


def test_topdown_feasible():
    """With the corrected fingertip TCP (shorter hand) and the closer object
    layout, a near-vertical top-down grasp is now reachable for every object
    (it was NOT with the old phantom 0.090 m gripper_tip_link)."""
    for name, tgt in OBJECT_TARGETS.items():
        cfg = ik.inverse_kinematics(tgt, math.radians(88.0))
        assert cfg is not None, f"{name} not reachable near-vertical"


def test_objects_reachable_slanted():
    """Every object + placement is reachable with some slanted approach."""
    for name, tgt in {**OBJECT_TARGETS, "placement": PLACEMENT_TARGET}.items():
        res = ik.feasible_approach(tgt)
        assert res is not None, f"{name} unreachable at {tgt}"
        phi, cfg = res
        err = np.linalg.norm(ik.fk_position(cfg) - np.array(tgt))
        assert err * 1000 < ROUNDTRIP_TOL_MM, \
            f"{name} FK error {err*1000:.2f} mm"


def test_grasp_geometry_clears_table():
    """Every grasp solution keeps all links above the table (no collision)."""
    for name, tgt in {**OBJECT_TARGETS, "placement": PLACEMENT_TARGET}.items():
        res = ik.feasible_approach(tgt, min_clearance=0.75)
        assert res is not None, f"{name} unreachable with table clearance"
        _, cfg = res
        pts = ik.link_world_positions(cfg)
        minz = min(pts[k][2] for k in ("shoulder", "elbow", "wrist", "tip"))
        assert minz >= 0.75, f"{name} dips to z={minz:.3f} (below table)"


def main() -> int:
    print("=" * 74)
    print("arm_ik headless validation")
    print("=" * 74)
    print(f"Planar links: a1={ik.A1:.5f}  a2={ik.A2:.5f}  a3={ik.A3:.5f}  "
          f"(a1+a2={ik.A1+ik.A2:.4f})   shoulder z={ik.Z_PIVOT:.5f}")
    print()
    print(f"{'object':<10} {'reach?':<7} {'tilt°↓':<8} "
          f"{'FK err(mm)':<11} joints (J2..J6, deg)")
    print("-" * 74)
    all_targets = {**OBJECT_TARGETS, "placement": PLACEMENT_TARGET}
    for name, tgt in all_targets.items():
        res = ik.feasible_approach(tgt)
        if res is None:
            print(f"{name:<10} {'NO':<7}")
            continue
        tilt, cfg = res
        err = np.linalg.norm(ik.fk_position(cfg) - np.array(tgt)) * 1000
        degs = [math.degrees(cfg[n]) for n in ik.ARM_JOINT_ORDER]
        joints = " ".join(f"{d:+6.1f}" for d in degs)
        print(f"{name:<10} {'yes':<7} {math.degrees(tilt):<8.1f} "
              f"{err:<11.2f} {joints}")
    print()
    # Vertical (90°) top-down feasibility per object (now reachable w/ TCP fix).
    print("Vertical (90° tilt) top-down feasibility per object:")
    for name, tgt in OBJECT_TARGETS.items():
        cfg = ik.inverse_kinematics(tgt, math.radians(90.0))
        print(f"   {name:<10} {'REACHABLE' if cfg else 'infeasible'}")
    print()

    # Run the asserts.
    failures = []
    for fn in (test_fk_ik_roundtrip, test_topdown_feasible,
               test_objects_reachable_slanted, test_grasp_geometry_clears_table):
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failures.append((fn.__name__, str(e)))
            print(f"FAIL  {fn.__name__}: {e}")
    print("=" * 74)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
