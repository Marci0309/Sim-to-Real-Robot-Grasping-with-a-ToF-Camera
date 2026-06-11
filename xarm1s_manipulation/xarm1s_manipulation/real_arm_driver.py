#!/usr/bin/env python3
"""Real Hiwonder xArm 1S control bridge (Phase 4C) -- the ONLY module that imports
the ``xarm`` USB-HID library. Everything else stays hardware-agnostic and
testable via ``--dry-run`` (which never opens the device).

The xArm 1S enumerates as a USB-HID device (VID 0x0483 / PID 0x5750) and is
driven by ``pip install xarm`` (the ccourson library):
    ctl = xarm.Controller('USB')
    ctl.setPosition([[servo_id, unit], ...], duration_ms, wait)   # unit 0..1000
    ctl.getPosition(servo_id)                                     # -> unit
Run this on **Windows** (co-located with DepthAI); WSL HID passthrough is brittle.

Joint angles (radians, arm_ik convention) are converted to servo units by
``servo_calib`` using ``config/servo_calib.json``. Method names of the underlying
library should be confirmed against the installed version during the Phase-0 smoke
test; the dry-run path is fully exercised without it.

Usage:
    python real_arm_driver.py --dry-run            # print unit commands, no device
    python real_arm_driver.py --dry-run --calib ../config/servo_calib.json
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Optional

# Make the package dir importable whether run as a module (ROS/WSL) or as a
# plain script on Windows (the real-arm host).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from xarm1s_manipulation import servo_calib, arm_ik as ik
except ImportError:
    import servo_calib            # type: ignore
    import arm_ik as ik           # type: ignore


class RealArm:
    """Thin wrapper over the xArm 1S HID controller. ``dry_run=True`` prints the
    servo commands instead of sending them, so the full perceive->IK->command
    pipeline can be validated with the arm unplugged."""

    def __init__(self, calib_path: Optional[str] = None, dry_run: bool = False,
                 default_move_ms: int = 1500):
        self.calib = servo_calib.load_calib(calib_path)
        self.dry_run = dry_run
        self.default_move_ms = default_move_ms
        self.ctl = None
        if not self.calib.get("calibrated", False):
            print("[RealArm] WARNING: using NOMINAL (uncalibrated) servo map -- "
                  "run calibrate_servos.py before a real grasp.")
        if not dry_run:
            import xarm                       # noqa: imported lazily on purpose
            self.ctl = xarm.Controller("USB")
            print("[RealArm] connected to xArm 1S over USB-HID.")

    # ---- motion -----------------------------------------------------------
    def move_joints(self, cfg_rad: Dict[str, float],
                    duration_ms: Optional[int] = None, wait: bool = True) -> None:
        """Move the 5 arm servos to a joint config (radians)."""
        dur = int(duration_ms or self.default_move_ms)
        cmds = servo_calib.cfg_to_servo_commands(cfg_rad, self.calib)
        if self.dry_run:
            pretty = "  ".join(f"S{sid}->{u}" for sid, u in cmds)
            print(f"[dry-run] move_joints {dur}ms: {pretty}")
            return
        # xarm.Controller.setPosition(servos, position=None, duration=1000, wait=False)
        # -- pass duration/wait by KEYWORD (positional would map dur->position).
        self.ctl.setPosition(cmds, duration=dur, wait=wait)

    def set_gripper(self, width_m: float, duration_ms: int = 600,
                    wait: bool = True) -> None:
        """Open/close the gripper to a jaw width (m)."""
        g = self.calib["gripper"]
        unit = servo_calib.gripper_width_to_units(width_m, self.calib)
        if self.dry_run:
            print(f"[dry-run] set_gripper {width_m*1000:.0f}mm -> S{g['servo']}={unit}")
            return
        self.ctl.setPosition([[g["servo"], unit]], duration=duration_ms, wait=wait)

    def read_joints(self) -> Dict[str, float]:
        """Read the 5 arm servos back as a joint config (radians)."""
        if self.dry_run:
            print("[dry-run] read_joints (no device) -> {}")
            return {}
        out: Dict[str, float] = {}
        for j in servo_calib.ARM_JOINTS:
            sid = self.calib["arm"][j]["servo"]
            out[j] = servo_calib.units_to_radians(j, self.ctl.getPosition(sid), self.calib)
        return out

    def go_home(self, duration_ms: int = 2000) -> None:
        self.move_joints({j: 0.0 for j in servo_calib.ARM_JOINTS}, duration_ms)

    def relax(self) -> None:
        """Power the servos off so the arm is back-drivable (for calibration).
        The exact call may differ by library version -- confirm at Phase 0."""
        if self.dry_run:
            print("[dry-run] relax (servo power off)")
            return
        try:
            self.ctl.servoOff()             # ccourson xarm: all servos off
        except Exception as e:               # noqa: BLE001
            print(f"[RealArm] relax() not available on this lib build: {e}")

    def close(self) -> None:
        if self.ctl is not None:
            try:
                self.ctl = None
            except Exception:                # noqa: BLE001
                pass


def _demo(calib_path: Optional[str]) -> None:
    """Dry-run smoke: HOME + a sample reachable grasp pose -> servo commands."""
    arm = RealArm(calib_path=calib_path, dry_run=True)
    print("\n-- HOME --")
    arm.go_home()
    print("\n-- sample grasp (blue_box-ish target 0.18,0.0,0.755 @ ~80 deg) --")
    res = ik.feasible_approach((0.18, 0.0, 0.755), min_clearance=0.75)
    if res is None:
        print("  (no IK solution for the sample target)")
    else:
        tilt, cfg = res
        print(f"  tilt={math.degrees(tilt):.0f} deg")
        arm.set_gripper(0.055)
        arm.move_joints(cfg)
        arm.set_gripper(0.040)
    print("\n-- back HOME --")
    arm.go_home()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="xArm 1S control bridge (Phase 4C)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print servo commands without opening the device")
    ap.add_argument("--calib", default=None, help="path to servo_calib.json")
    args = ap.parse_args()
    if not args.dry_run:
        print("Refusing to open the device outside --dry-run from this CLI. "
              "Import RealArm(dry_run=False) from the executor instead.")
        sys.exit(2)
    _demo(args.calib)
