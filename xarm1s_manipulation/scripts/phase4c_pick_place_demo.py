#!/usr/bin/env python3
"""Phase 4C full pick-and-place DEMO (blind, HOME -> pick -> place -> HOME).

Picks an object hand-placed at (px,py) and places it at (qx,qy), then returns
HOME. PRE-VERIFIES every waypoint's IK + servo limits before moving anything;
aborts cleanly if any is infeasible (places back at the pick spot if the place
spot is unreachable). Run on Windows py-3.11.

    py -3.11 phase4c_pick_place_demo.py
    py -3.11 phase4c_pick_place_demo.py --px 0.18 --py 0 --qx 0.18 --qy -0.10 --grip-mm 18 --grasp-z 0.80
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


def solve(tx, ty, tz):
    rg = ik.feasible_approach((tx, ty, 0.755), min_clearance=0.75)
    if rg is None:
        return None
    tilt, seed = rg
    cfg = ik.inverse_kinematics((tx, ty, tz), tilt, seed=seed, min_clearance=0.75)
    if cfg is None:
        rr = ik.feasible_approach((tx, ty, tz), seed=seed, min_clearance=0.75)
        cfg = rr[1] if rr else None
    return cfg


def in_limits(cfg, calib):
    for jn in cfg:
        a = calib["arm"][jn]
        ru = a["zero_unit"] + a["dir"] * a["units_per_rad"] * cfg[jn]
        if ru < a["min_unit"] - 1 or ru > a["max_unit"] + 1:
            return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--px", type=float, default=0.18)
    ap.add_argument("--py", type=float, default=0.0)
    ap.add_argument("--qx", type=float, default=0.18)
    ap.add_argument("--qy", type=float, default=-0.10)
    ap.add_argument("--hover-z", type=float, default=0.875)
    ap.add_argument("--grasp-z", type=float, default=0.82)   # grips box middle; closing tips clear the table
    ap.add_argument("--open-mm", type=float, default=55.0)
    ap.add_argument("--grip-mm", type=float, default=18.0)
    ap.add_argument("--pause", type=float, default=1.0)
    ap.add_argument("--dur-scale", type=float, default=1.0, help="multiply all move durations (>1 = slower/gentler, won't fling light objects)")
    args = ap.parse_args()
    calib = servo_calib.load_calib(CAL)
    home = {j: 0.0 for j in servo_calib.ARM_JOINTS}

    plan = [("hover_pick", args.px, args.py, args.hover_z),
            ("grasp",      args.px, args.py, args.grasp_z),
            ("lift",       args.px, args.py, args.hover_z),
            ("hover_place", args.qx, args.qy, args.hover_z),
            ("place",      args.qx, args.qy, args.grasp_z)]
    cfgs = {}
    for lab, x, y, z in plan:
        c = solve(x, y, z)
        if c is None or not in_limits(c, calib):
            if lab in ("hover_place", "place"):       # place unreachable -> place back at pick
                print("  place spot unreachable -> will place back at the pick spot")
                args.qx, args.qy = args.px, args.py
                cfgs["hover_place"] = cfgs["lift"]
                cfgs["place"] = cfgs["grasp"]
                break
            print("ABORT: %s IK/limits infeasible at (%.3f,%.3f,%.3f)" % (lab, x, y, z))
            return
        cfgs[lab] = c
    print("all waypoints verified feasible. running HOME -> pick -> place -> HOME ...")

    import xarm
    ctl = xarm.Controller("USB")

    def go(cfg, dur, lab):
        ctl.setPosition(servo_calib.cfg_to_servo_commands(cfg, calib),
                        duration=int(dur * args.dur_scale), wait=True)
        print("  ->", lab)
        time.sleep(args.pause)

    def grip(mm, dur=700):
        gu = servo_calib.gripper_width_to_units(mm / 1000.0, calib)
        ctl.setPosition([[calib["gripper"]["servo"], gu]], duration=dur, wait=True)
        time.sleep(0.5)
        rb = ctl.getPosition(calib["gripper"]["servo"])
        print("     gripper %g mm -> readback %d %s" % (mm, rb, "(GRIP/contact)" if abs(rb - gu) > 20 else "(open/free)"))

    go(home, 2500, "HOME (start)")
    grip(args.open_mm)
    go(cfgs["hover_pick"], 2500, "hover over PICK")
    go(cfgs["grasp"], 1800, "descend to grasp")
    grip(args.grip_mm)
    go(cfgs["lift"], 1800, "LIFT")
    go(cfgs["hover_place"], 2500, "carry to PLACE")
    go(cfgs["place"], 1800, "lower at place")
    grip(args.open_mm)
    go(cfgs["hover_place"], 1500, "retreat up")
    go(home, 2600, "HOME (end)")
    print("PICK-AND-PLACE COMPLETE (HOME -> HOME).")


if __name__ == "__main__":
    main()
