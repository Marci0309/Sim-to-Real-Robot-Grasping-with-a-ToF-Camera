#!/usr/bin/env python3
"""Phase 4C arm-base <-> table extrinsic (Windows, powered).

The perception (Phase 4B) reports object poses in the TABLE frame (the ChArUco
board origin); the IK (arm_ik) works in the ARM base frame. In sim these coincide
by construction; on the real bench the board sits at an arbitrary offset from the
arm, so we must MEASURE the table->arm rigid transform.

Method (touch known points -- no extra fixturing): for >=4 points whose
table-frame XY is known (e.g. ChArUco corners at k*square_length, or marker
centres), relax the arm, touch the closed fingertip TCP to the physical point,
and read the joints; arm_ik.forward_kinematics then gives the fingertip XY in the
arm frame. A 2D Kabsch solve on the (table, arm) pairs yields R, t and an RMS
residual.

Writes ``config/arm_base_extrinsics.json``:
    {"R_arm_from_table": 3x3, "t_arm_from_table": 3, "residual_mm": ..., "n": N}

    python solve_arm_base_extrinsics.py            # interactive
    python solve_arm_base_extrinsics.py --calib ../config/servo_calib.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "xarm1s_manipulation")))
try:
    from xarm1s_manipulation import servo_calib, arm_ik as ik
except ImportError:
    import servo_calib            # type: ignore
    import arm_ik as ik           # type: ignore

CFG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
OUT = os.path.join(CFG_DIR, "arm_base_extrinsics.json")


def kabsch_2d(table_xy: np.ndarray, arm_xy: np.ndarray):
    """Best-fit 2D rigid transform mapping table_xy -> arm_xy (no scale)."""
    ca, cb = table_xy.mean(0), arm_xy.mean(0)
    A, B = table_xy - ca, arm_xy - cb
    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R2 = Vt.T @ np.diag([1.0, d]) @ U.T
    t2 = cb - R2 @ ca
    resid = np.sqrt(np.mean(np.sum((arm_xy - (table_xy @ R2.T + t2)) ** 2, axis=1)))
    return R2, t2, resid


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", default=os.path.join(CFG_DIR, "servo_calib.json"))
    args = ap.parse_args()
    calib = servo_calib.load_calib(args.calib)
    if not calib.get("calibrated", False):
        print("WARNING: servo_calib is NOMINAL -- run calibrate_servos.py first, "
              "or the FK from read-back joints will be wrong.")

    import xarm
    ctl = xarm.Controller("USB")

    # Close the gripper and KEEP it powered so the closed fingertips meet at the
    # modeled TCP (arm_ik.FINGERTIP_DZ); relax only the 5 ARM servos so the arm is
    # hand-posable. servoOff is per-servo (gripper = id 1, arm joints = ids 2..6);
    # bus servos hold by friction when relaxed (no flop) but re-stiffen after idle,
    # so we re-relax before every touch.
    def relax_arm():
        try:
            for j in servo_calib.ARM_JOINTS:
                ctl.servoOff(servo_calib.JOINT_TO_SERVO[j])
        except Exception:                      # noqa: BLE001  (older lib: relax all)
            try:
                ctl.servoOff()
            except Exception:                  # noqa: BLE001
                pass

    try:
        ctl.setPosition([[calib["gripper"]["servo"],
                          servo_calib.gripper_width_to_units(0.0, calib)]],
                        duration=600, wait=True)
    except Exception:                          # noqa: BLE001
        pass
    relax_arm()
    print("Gripper CLOSED + held; arm relaxed (hand-posable). Touch the CLOSED "
          "fingertip to each known table point.\n"
          "Enter >=4 points spread across the workspace. Blank x to finish.")

    table_pts, arm_pts, raw_pts = [], [], []
    while True:
        s = input(f"\nPoint {len(table_pts)+1}  table-frame x (m) [blank=done]: ").strip()
        if not s:
            break
        x = float(s)
        y = float(input("            table-frame y (m): ").strip())
        relax_arm()                            # re-relax: bus servos re-stiffen when idle
        input("  Touch the fingertip to that point, hold, press Enter...")
        units = {j: int(ctl.getPosition(servo_calib.JOINT_TO_SERVO[j]))
                 for j in servo_calib.ARM_JOINTS}
        cfg = {j: servo_calib.units_to_radians(j, units[j], calib)
               for j in servo_calib.ARM_JOINTS}
        tip = ik.forward_kinematics(cfg)[:3, 3]
        table_pts.append([x, y])
        arm_pts.append([float(tip[0]), float(tip[1])])
        raw_pts.append({"table": [x, y], "units": units,
                        "fk": [float(tip[0]), float(tip[1]), float(tip[2])]})
        print(f"  table=({x:.3f},{y:.3f})  ->  arm=({tip[0]:.3f},{tip[1]:.3f})  "
              f"units={[units[j] for j in servo_calib.ARM_JOINTS]}")

    json.dump({"calib_upr": calib["arm"]["xarm_2_joint"]["units_per_rad"],
               "points": raw_pts},
              open(os.path.join(CFG_DIR, "arm_base_touch_raw.json"), "w"), indent=2)
    print(f"  raw units+positions saved -> arm_base_touch_raw.json  ({len(raw_pts)} pts)")

    if len(table_pts) < 4:
        raise SystemExit("need >=4 points for a reliable solve.")

    R2, t2, resid = kabsch_2d(np.array(table_pts), np.array(arm_pts))
    R = np.eye(3); R[:2, :2] = R2
    t = np.array([t2[0], t2[1], 0.0])
    out = {"R_arm_from_table": R.tolist(), "t_arm_from_table": t.tolist(),
           "residual_mm": round(float(resid) * 1000, 2), "n": len(table_pts),
           "method": "touch >=4 known table points + 2D Kabsch"}
    os.makedirs(CFG_DIR, exist_ok=True)
    json.dump(out, open(OUT, "w"), indent=2)
    print(f"\nRMS residual = {out['residual_mm']} mm  (n={out['n']})")
    print(f"wrote {OUT}")
    if out["residual_mm"] > 8:
        print("[note] residual > 8 mm -- re-touch more carefully or use the "
              "fixed-offset fallback (board clamped at a measured pose).")


if __name__ == "__main__":
    main()
