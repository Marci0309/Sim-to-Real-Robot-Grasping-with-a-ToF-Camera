#!/usr/bin/env python3
"""Phase 4C real pick-and-place executor for the Hiwonder xArm 1S.

Runs on the **Windows** real-arm host (plain CPython, no ROS). For one object +
placement + repeat it:

  1. reads the PERCEIVED object pose in the TABLE frame (from
     ``perceive_scene_4c.py`` -> ``captures_4c/<scene>_perceived.json``);
  2. transforms it to the ARM base frame via ``config/arm_base_extrinsics.json``
     (table->arm rigid transform from ``solve_arm_base_extrinsics.py``);
  3. plans the pick-and-place waypoints with the SAME geometry as the sim
     (``grasp_common.plan_pick_place`` + ``arm_ik``);
  4. drives the real arm through HOME -> pre -> grasp -> close -> lift -> place ->
     open -> retreat via ``RealArm`` (open-loop, no live feedback);
  5. prompts the operator for the PHYSICAL outcome (lift-and-hold y/n) and appends
     a row to ``evaluation/results/real_manip_results.csv``.

Always validate with ``--dry-run`` first (no device, no prompts):
    python real_grasp_executor.py --object golf_ball --scene scene01 --dry-run
Real run (after calibration + arm-base extrinsic exist):
    python real_grasp_executor.py --object golf_ball --scene scene01 --placement-id 1 --repeat 1
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from typing import Dict, Optional, Tuple

import numpy as np

# Make the package importable whether run standalone (Windows) or in-tree.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, "..", "xarm1s_manipulation")
sys.path.insert(0, os.path.abspath(_PKG))
try:
    from xarm1s_manipulation import grasp_common as gc, arm_ik as ik
    from xarm1s_manipulation.real_arm_driver import RealArm
except ImportError:
    import grasp_common as gc            # type: ignore
    import arm_ik as ik                  # type: ignore
    from real_arm_driver import RealArm  # type: ignore

_CFG = os.path.abspath(os.path.join(_HERE, "..", "config"))
_RES = os.path.abspath(os.path.join(_HERE, "..", "..", "evaluation", "results"))
_CAP = os.path.abspath(os.path.join(_HERE, "..", "..", "tof_perception", "captures_4c"))
CSV_PATH = os.path.join(_RES, "real_manip_results.csv")
CSV_FIELDS = ["tag", "object", "placement_id", "repeat", "perceived_x",
              "perceived_y", "arm_x", "arm_y", "tilt_deg", "ik_feasible",
              "gripper_closed", "lift_success", "place_success", "note"]

# Real-bench fingertip grasp/place height. Last session's table-height finding:
# the closing fingers swing down ~4 cm, so commanding the fingertip to ~0.82 m
# grips the body with the tips clear of the (higher-than-model) real table. The
# sim/model uses 0.755. The APPROACH TILT is searched at the model seed height
# (TILT_SEED_Z) then applied at this height -- the validated demo recipe. Tunable
# per run via --grasp-z.
DEFAULT_GRASP_Z = 0.82
TILT_SEED_Z = 0.755


def load_arm_base(path: Optional[str]) -> Tuple[np.ndarray, np.ndarray, float, bool]:
    """Return (R 3x3, t 3, scale, calibrated?). Falls back to identity (dry-run only).

    ``scale`` (default 1.0 = rigid) absorbs the real-arm FK reach-scale error
    measured in Phase 4C: the real xArm 1S reaches ~28% farther than its URDF/
    arm_ik model, so the table->arm map is a similarity, not a rigid transform
    (see fit_arm_base_extrinsics.py). Applied BEFORE IK; because the same FK/IK
    runs end to end, the real fingertip still lands at the true target.
    """
    p = path or os.path.join(_CFG, "arm_base_extrinsics.json")
    if os.path.exists(p):
        d = json.load(open(p))
        return (np.array(d["R_arm_from_table"], float),
                np.array(d["t_arm_from_table"], float),
                float(d.get("scale", 1.0)), True)
    return np.eye(3), np.zeros(3), 1.0, False


def table_to_arm(xy_table, R: np.ndarray, t: np.ndarray, scale: float = 1.0) -> np.ndarray:
    p = scale * (R @ np.array([float(xy_table[0]), float(xy_table[1]), 0.0])) + t
    return p[:2]


def load_perceived(scene: str) -> Dict[str, dict]:
    p = os.path.join(_CAP, f"{scene}_perceived.json")
    if not os.path.exists(p):
        raise SystemExit(f"perceived poses not found: {p}\n"
                         f"  run perceive_scene_4c.py --name {scene} first.")
    return json.load(open(p))["objects"]


def append_row(row: dict, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


def _score(dry_run: bool) -> Tuple[str, str, str]:
    """Operator-scored physical outcome. Skipped in dry-run."""
    if dry_run or not sys.stdin.isatty():
        return "", "", "dry-run"
    lift = input("  lift-and-hold success? [y/N]: ").strip().lower().startswith("y")
    place = input("  placed on target? [y/N]: ").strip().lower().startswith("y")
    note = input("  note (optional): ").strip()
    return str(int(lift)), str(int(place)), note


def run_one(label: str, scene: str, placement_id: int, repeat: int,
            arm: RealArm, R: np.ndarray, t: np.ndarray, scale: float,
            dry_run: bool, grasp_z: float = DEFAULT_GRASP_Z,
            pick_only: bool = False, close_mm: Optional[float] = None) -> dict:
    d = gc.OBJECTS[label]
    perceived = load_perceived(scene)
    if label not in perceived:
        print(f"  {label} not detected in {scene}; recording as a miss.")
        row = dict(tag="real", object=label, placement_id=placement_id,
                   repeat=repeat, perceived_x="", perceived_y="", arm_x="",
                   arm_y="", tilt_deg="", ik_feasible=0, gripper_closed=0,
                   lift_success=0, place_success=0, note="not detected")
        return row

    xy_table = perceived[label]["table_xy"]
    arm_xy = table_to_arm(xy_table, R, t, scale)

    # tilt from the steepest feasible approach, searched at the model SEED height
    # then applied at the real grasp_z below (matches the validated demo recipe)
    res = ik.feasible_approach((arm_xy[0], arm_xy[1], TILT_SEED_Z), min_clearance=0.75)
    row = dict(tag="real", object=label, placement_id=placement_id, repeat=repeat,
               perceived_x=round(float(xy_table[0]), 5),
               perceived_y=round(float(xy_table[1]), 5),
               arm_x=round(float(arm_xy[0]), 5), arm_y=round(float(arm_xy[1]), 5),
               tilt_deg="", ik_feasible=0, gripper_closed=0,
               lift_success=0, place_success=0, note="")
    if res is None:
        row["note"] = "IK infeasible at this placement"
        print(f"  {label}: IK infeasible at arm XY {arm_xy.round(3)} -- recorded.")
        return row
    tilt, _ = res
    row["tilt_deg"] = round(math.degrees(tilt), 1)

    # Prefer the PERCEIVED minor axis (PCA on the depth blob) over the object's
    # nominal one, then rotate it into the ARM frame via R (the yaw half of the
    # transform). Symmetric/missing axes (None) leave the wrist roll untouched.
    minor_table = perceived[label].get("minor_axis") or d.get("minor_axis")
    minor_arm = None
    if minor_table is not None:
        v = R @ np.array([float(minor_table[0]), float(minor_table[1]), 0.0])
        minor_arm = (float(v[0]), float(v[1]), float(v[2]))

    wps = gc.plan_pick_place(label, arm_xy, tilt, grasp_z=grasp_z, minor_axis=minor_arm)
    if wps is None:
        row["note"] = "waypoint IK failed"
        print(f"  {label}: a waypoint was unreachable -- recorded.")
        return row
    row["ik_feasible"] = 1

    # Grip force = how far PAST the object width the jaws are commanded (they stall
    # on the object). The nominal grasp_width was too wide here, so allow a tighter
    # absolute close via --close-mm for a firm pinch.
    close_w = (close_mm / 1000.0) if close_mm else max(0.006, d["grasp_width"] - gc.GRIP_SQUEEZE)
    print(f"  {label}: table{np.round(xy_table,3).tolist()} -> arm"
          f"{arm_xy.round(3).tolist()}, tilt {row['tilt_deg']} deg, "
          f"grasp_z {grasp_z:.3f}, close {close_w*1000:.0f}mm")

    # --- the pick-and-place motion (mirrors grasp_executor.run_trial) ---
    arm.set_gripper(0.055)                              # open
    arm.move_joints(gc.HOME_CFG); arm.move_joints(wps["pre"]); arm.move_joints(wps["grasp"])
    arm.set_gripper(close_w)                            # close ONTO the object
    row["gripper_closed"] = 1
    arm.move_joints(wps["grasp"]); arm.move_joints(wps["pre"])      # lift
    if pick_only:
        print("  [pick-only] holding the lifted object at the pre-grasp pose "
              "(skipping carry/place/retreat).")
    else:
        arm.move_joints(wps["place_pre"]); arm.move_joints(wps["place"])  # carry
        arm.set_gripper(0.055)                          # release
        arm.move_joints(wps["place"]); arm.move_joints(wps["place_pre"]); arm.move_joints(gc.HOME_CFG)

    lift, place, note = _score(dry_run)
    row["lift_success"], row["place_success"], row["note"] = lift, place, note
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description="Phase 4C real pick-and-place")
    ap.add_argument("--object", required=True, choices=list(gc.OBJECTS))
    ap.add_argument("--scene", required=True, help="perceived scene name (e.g. scene01)")
    ap.add_argument("--placement-id", type=int, default=0)
    ap.add_argument("--repeat", type=int, default=0)
    ap.add_argument("--calib", default=os.path.join(_CFG, "servo_calib.json"),
                    help="servo_calib.json path (defaults to config/servo_calib.json "
                         "so the real run never falls back to the NOMINAL map)")
    ap.add_argument("--extrinsics", default=None, help="arm_base_extrinsics.json path")
    ap.add_argument("--grasp-z", type=float, default=DEFAULT_GRASP_Z,
                    help="real fingertip grasp/place height (m); baked from the "
                         "validated phase4c demo (real table sits higher than the model)")
    ap.add_argument("--pick-only", action="store_true",
                    help="stop after the lift (skip carry/place) -- safe for first grasps")
    ap.add_argument("--close-mm", type=float, default=None,
                    help="absolute jaw width to command at the grasp (mm); command "
                         "well past the object width for a firm pinch (overrides grasp_width)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    R, t, scale, calibrated = load_arm_base(args.extrinsics)
    if not calibrated:
        print("[exec] WARNING: no arm_base_extrinsics.json -- using IDENTITY "
              "table->arm transform (dry-run only; not a real grasp frame).")
    # Dry-runs go to a separate CSV so they never pollute the real results.
    out_csv = CSV_PATH.replace(".csv", ".dryrun.csv") if args.dry_run else CSV_PATH
    arm = RealArm(calib_path=args.calib, dry_run=args.dry_run)
    try:
        row = run_one(args.object, args.scene, args.placement_id, args.repeat,
                      arm, R, t, scale, args.dry_run, grasp_z=args.grasp_z,
                      pick_only=args.pick_only, close_mm=args.close_mm)
        append_row(row, out_csv)
        print(f"[exec] recorded -> {out_csv}\n        {row}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
