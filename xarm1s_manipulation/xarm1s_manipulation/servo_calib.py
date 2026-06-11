#!/usr/bin/env python3
"""Servo calibration model for the real Hiwonder xArm 1S (Phase 4C).

Pure module (no ``xarm`` import) that converts between the URDF joint angles the
analytical IK (``arm_ik``) produces (radians, with arm_ik's sign/zero
conventions) and the xArm 1S's HID servo **position units** (integers 0-1000,
mapping to roughly +-125 deg of mechanical travel; unit ~500 ~ 0 deg).

The per-servo numbers come from ``calibrate_servos.py`` (a two-point fit:
URDF-zero pose -> ``zero_unit``; a protractor-verified +90 deg -> slope) and live
in ``config/servo_calib.json``. Until that is run, ``DEFAULT_CALIB`` below holds
NOMINAL values (servo centre 500, the library's nominal 0-1000 <-> +-125 deg
slope, gripper guesses) so the whole pipeline can be dry-run before the arm is
even powered. **Do not trust the nominal gripper/zero values for a real grasp --
run the calibration first.**

Physical servo numbering (xArm 1S): gripper = ID 1, then ID 2..6 from the wrist
out to the base. The URDF chain is base(J6) -> J5 -> J4 -> J3 -> J2(wrist), so
``xarm_N_joint`` maps to servo ID ``N``. This mapping is a HYPOTHESIS confirmed
during calibration (each joint's ``servo`` id is stored in the JSON and can be
overridden there).
"""
from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional

# URDF joint -> physical servo ID (confirmed/overridable in servo_calib.json).
JOINT_TO_SERVO: Dict[str, int] = {
    "xarm_6_joint": 6,   # base yaw
    "xarm_5_joint": 5,   # shoulder pitch
    "xarm_4_joint": 4,   # elbow pitch
    "xarm_3_joint": 3,   # wrist pitch
    "xarm_2_joint": 2,   # wrist roll
}
GRIPPER_SERVO = 1
ARM_JOINTS = ["xarm_2_joint", "xarm_3_joint", "xarm_4_joint", "xarm_5_joint", "xarm_6_joint"]

# Nominal: the xarm library maps units 0..1000 to about -125..+125 deg
# (250 deg span). units_per_rad = 1000 / radians(250).
_NOMINAL_UPR = 1000.0 / math.radians(250.0)   # ~= 229.18 units/rad

DEFAULT_CALIB: Dict = {
    "calibrated": False,        # set True by calibrate_servos.py
    "note": "NOMINAL values -- run calibrate_servos.py before any real grasp",
    "arm": {
        j: {"servo": JOINT_TO_SERVO[j], "zero_unit": 500.0,
            "units_per_rad": _NOMINAL_UPR, "dir": 1,
            "min_unit": 0, "max_unit": 1000}
        for j in ARM_JOINTS
    },
    # Gripper: linear between an open and a closed reference (units<->width m).
    "gripper": {"servo": GRIPPER_SERVO,
                "open_unit": 600.0, "open_width_m": 0.055,
                "closed_unit": 120.0, "closed_width_m": 0.0,
                "min_unit": 0, "max_unit": 1000},
}


def load_calib(path: Optional[str] = None) -> Dict:
    """Load servo_calib.json, or return the NOMINAL DEFAULT_CALIB if absent."""
    if path and os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return json.loads(json.dumps(DEFAULT_CALIB))   # deep copy


def clamp_units(unit: float, lo: float, hi: float) -> int:
    return int(round(max(lo, min(hi, unit))))


def radians_to_units(joint: str, theta_rad: float, calib: Dict) -> int:
    """Convert a URDF joint angle (rad) to a clamped integer servo unit."""
    a = calib["arm"][joint]
    unit = a["zero_unit"] + a["dir"] * a["units_per_rad"] * theta_rad
    return clamp_units(unit, a["min_unit"], a["max_unit"])


def units_to_radians(joint: str, unit: float, calib: Dict) -> float:
    """Inverse of radians_to_units (for read-back / the touch-solve FK)."""
    a = calib["arm"][joint]
    return (unit - a["zero_unit"]) / (a["dir"] * a["units_per_rad"])


def gripper_width_to_units(width_m: float, calib: Dict) -> int:
    """Linear map of a jaw opening (m) to a clamped integer gripper servo unit."""
    g = calib["gripper"]
    ow, cw = g["open_width_m"], g["closed_width_m"]
    ou, cu = g["open_unit"], g["closed_unit"]
    if abs(ow - cw) < 1e-9:
        return clamp_units(ou, g["min_unit"], g["max_unit"])
    frac = (width_m - cw) / (ow - cw)            # 0 at closed, 1 at open
    unit = cu + frac * (ou - cu)
    return clamp_units(unit, g["min_unit"], g["max_unit"])


def cfg_to_servo_commands(cfg_rad: Dict[str, float], calib: Dict
                          ) -> List[List[int]]:
    """A joint-config dict (xarm_2..xarm_6, rad) -> [[servo_id, unit], ...] for
    the 5 arm servos, ready for xarm Controller.setPosition()."""
    out: List[List[int]] = []
    for j in ARM_JOINTS:
        if j in cfg_rad:
            out.append([calib["arm"][j]["servo"],
                        radians_to_units(j, cfg_rad[j], calib)])
    return out
