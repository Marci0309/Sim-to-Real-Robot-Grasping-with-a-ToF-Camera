#!/usr/bin/env python3
"""Phase 4C manual jog / blind-grasp helper (Windows, real arm).

Moves the fingertip to a world (x, y, z) via arm_ik + the live servo_calib, and/or
sets the gripper width. Used for the blind grasp test and manual verification.
SAFE: aborts if IK is infeasible or any joint would exceed its calibrated unit
limit; commands one slow move.

    py -3.11 phase4c_manual_grasp.py --x 0.18 --y 0.0 --z 0.775          # move fingertip
    py -3.11 phase4c_manual_grasp.py --gripper-mm 37                     # close gripper
    py -3.11 phase4c_manual_grasp.py --x 0.18 --y 0.0 --z 0.875 --gripper-mm 55
    py -3.11 phase4c_manual_grasp.py --home                              # back to HOME
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "xarm1s_manipulation"))
import servo_calib            # noqa: E402
import arm_ik as ik           # noqa: E402

CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "servo_calib.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float)
    ap.add_argument("--gripper-mm", type=float, default=None)
    ap.add_argument("--duration", type=int, default=2500)
    ap.add_argument("--seed-z", type=float, default=0.755)
    ap.add_argument("--home", action="store_true")
    args = ap.parse_args()

    calib = servo_calib.load_calib(CAL)
    import xarm
    ctl = xarm.Controller("USB")

    if args.gripper_mm is not None:
        gu = servo_calib.gripper_width_to_units(args.gripper_mm / 1000.0, calib)
        ctl.setPosition([[calib["gripper"]["servo"], gu]], duration=600, wait=True)
        time.sleep(0.4)
        rb = ctl.getPosition(calib["gripper"]["servo"])
        contact = "STALLED ON OBJECT (contact!)" if abs(rb - gu) > 20 else "reached commanded (no contact)"
        print("gripper ->", args.gripper_mm, "mm (cmd unit", gu, ") | readback", rb, "->", contact)
        time.sleep(0.1)

    if args.home:
        home = {jn: 0.0 for jn in servo_calib.ARM_JOINTS}
        ctl.setPosition(servo_calib.cfg_to_servo_commands(home, calib),
                        duration=args.duration, wait=True)
        time.sleep(0.4)
        print("at HOME. readback", [ctl.getPosition(i) for i in range(2, 7)])
        return

    if args.x is None or args.z is None:
        print("nothing to move (need --x and --z, or --gripper-mm, or --home)")
        return

    tx, ty, tz = args.x, args.y, args.z
    rg = ik.feasible_approach((tx, ty, args.seed_z), min_clearance=0.75)
    if rg is None:
        print("seed IK infeasible at", (tx, ty, args.seed_z)); return
    tilt, seedcfg = rg
    cfg = ik.inverse_kinematics((tx, ty, tz), tilt, seed=seedcfg, min_clearance=0.75)
    if cfg is None:
        rr = ik.feasible_approach((tx, ty, tz), seed=seedcfg, min_clearance=0.75)
        cfg = rr[1] if rr else None
    if cfg is None:
        print("target IK infeasible at", (tx, ty, tz)); return

    for jn in cfg:                       # explicit limit check (abort, don't clamp)
        a = calib["arm"][jn]
        ru = a["zero_unit"] + a["dir"] * a["units_per_rad"] * cfg[jn]
        if ru < a["min_unit"] - 1 or ru > a["max_unit"] + 1:
            print("ABORT", jn, "would hit limit at unit", round(ru)); return

    cmds = servo_calib.cfg_to_servo_commands(cfg, calib)
    tip = ik.fk_position(cfg)
    print("target", (tx, ty, tz), "| tilt", round(math.degrees(tilt), 1),
          "deg | height above table", round(float(tip[2]) - 0.755, 3), "m")
    ctl.setPosition(cmds, duration=args.duration, wait=True)
    time.sleep(0.4)
    print("moved. readback", [ctl.getPosition(i) for i in range(2, 7)])


if __name__ == "__main__":
    main()
