#!/usr/bin/env python3
"""Phase 4C per-servo calibration for the real Hiwonder xArm 1S (Windows, powered).

Produces ``config/servo_calib.json`` via a robust two-point fit per arm servo
(hobby servos have arbitrary horn-mounting offsets, so we do NOT trust the
nominal 0-1000 <-> +-125 deg map):

  * Pose A = URDF ZERO  (HOME: arm straight up, all joints 0)        -> zero_unit
  * Pose B = +90 deg (pi/2) on that joint, protractor-verified       -> slope, dir

The gripper is fitted from an OPEN (measure the 55 mm jaw) and a CLOSED
reference. Per-joint safe unit limits are derived from arm_ik.LIMITS. The arm is
relaxed (servo power off) so you can hand-pose it.

This is a HARDWARE script -- it opens the USB-HID device and reads servo
positions. Validate the rest of the pipeline with the dry-run tools first.

    python calibrate_servos.py                 # interactive, writes config/servo_calib.json
"""
from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "xarm1s_manipulation")))
try:
    from xarm1s_manipulation import servo_calib, arm_ik as ik
except ImportError:
    import servo_calib            # type: ignore
    import arm_ik as ik           # type: ignore

CFG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
OUT = os.path.join(CFG_DIR, "servo_calib.json")
UNIT_MARGIN = 15          # keep this many units clear of each derived joint limit


def read_unit(ctl, sid: int) -> float:
    return float(ctl.getPosition(sid))


def main() -> None:
    import xarm
    ctl = xarm.Controller("USB")
    print("Connected. Powering servos OFF so you can hand-pose the arm.")
    try:
        ctl.servoOff()
    except Exception as e:                      # noqa: BLE001
        print(f"  (servoOff not available: {e}; pose against gravity gently)")

    calib = {"calibrated": True, "note": "two-point per-servo fit", "arm": {}, "gripper": {}}

    print("\n=== ARM SERVOS (joints xarm_2..xarm_6) ===")
    print("For each: hold the joint at the reference pose, then press Enter.")
    for joint in servo_calib.ARM_JOINTS:
        sid = servo_calib.JOINT_TO_SERVO[joint]
        input(f"\n[{joint} / servo {sid}]  Pose A = URDF ZERO (HOME, arm straight "
              f"up). Hold and press Enter...")
        u0 = read_unit(ctl, sid)
        input(f"  Pose B = +90 deg (pi/2) on {joint} (protractor). Hold and press Enter...")
        u90 = read_unit(ctl, sid)
        slope = (u90 - u0) / (math.pi / 2.0)
        if abs(slope) < 1e-6:
            print("  !! zero slope -- re-do this joint (poses identical).")
            return
        direction = 1 if slope > 0 else -1
        upr = abs(slope)
        lo, hi = ik.LIMITS[joint]
        u_lo = u0 + direction * upr * lo
        u_hi = u0 + direction * upr * hi
        mn, mx = sorted((u_lo, u_hi))
        calib["arm"][joint] = {
            "servo": sid, "zero_unit": round(u0, 1),
            "units_per_rad": round(upr, 3), "dir": direction,
            "min_unit": int(round(max(0, mn + UNIT_MARGIN))),
            "max_unit": int(round(min(1000, mx - UNIT_MARGIN))),
        }
        print(f"  {joint}: zero={u0:.0f}  units/rad={upr:.1f}  dir={direction}  "
              f"limits=[{calib['arm'][joint]['min_unit']},{calib['arm'][joint]['max_unit']}]")

    print("\n=== GRIPPER (servo 1) ===")
    input("  Fully OPEN the jaws (measure ~55 mm). Press Enter...")
    ou = read_unit(ctl, servo_calib.GRIPPER_SERVO)
    ow = float(input("  measured open width in mm [55]: ") or "55") / 1000.0
    input("  CLOSE the jaws to a known reference (e.g. fingertips touching ~0 mm). Press Enter...")
    cu = read_unit(ctl, servo_calib.GRIPPER_SERVO)
    cw = float(input("  measured closed width in mm [0]: ") or "0") / 1000.0
    calib["gripper"] = {
        "servo": servo_calib.GRIPPER_SERVO,
        "open_unit": round(ou, 1), "open_width_m": ow,
        "closed_unit": round(cu, 1), "closed_width_m": cw,
        "min_unit": int(min(ou, cu)), "max_unit": int(max(ou, cu)),
    }
    print(f"  gripper: open {ow*1000:.0f}mm@{ou:.0f}  closed {cw*1000:.0f}mm@{cu:.0f}")

    os.makedirs(CFG_DIR, exist_ok=True)
    json.dump(calib, open(OUT, "w"), indent=2)
    print(f"\nwrote {OUT}")
    print("VERIFY: command a few known arm_ik poses and ruler-check the fingertip "
          "XY before trusting it (do this BEFORE any multi-joint trajectory).")


if __name__ == "__main__":
    main()
