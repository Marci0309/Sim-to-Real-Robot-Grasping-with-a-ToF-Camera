#!/usr/bin/env python3
"""Pure (ROS-free) shared grasp data + geometry for the xArm 1S pick-and-place.

This is the SINGLE SOURCE OF TRUTH for the per-object grasp data, the gripper jaw
model, and the pick-and-place waypoint geometry. Both the Gazebo sim node
(``grasp_executor.py``) and the real-arm executor
(``scripts/real_grasp_executor.py``, Phase 4C) import from here, so the objects'
capture tolerances / grasp widths / minor axes and the IK waypoint math are
guaranteed identical across sim and real.

Imports only ``math`` / ``numpy`` + ``arm_ik`` (also pure) -- NO ROS -- so it can
be imported on a plain-Python (no ROS) Windows host driving the real arm. The
``arm_ik`` import works both as a package (ROS/WSL) and standalone (the real
scripts put this directory on ``sys.path``).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

try:                                    # package context (ROS / WSL)
    from xarm1s_manipulation import arm_ik as ik
except ImportError:                     # standalone context (Windows real arm)
    import arm_ik as ik

# ---------------------------------------------------------------------------
# Per-object grasp data. See grasp_executor.py's original docstring for the full
# rationale of each field. Detections are matched to objects by nearest nominal
# XY, so /detected_objects order does not matter.
#   nominal     : world XY used to pick the right detection / yaw branch.
#   grasp_z     : fingertip target z (~5 mm above the 0.75 m table top).
#   capture_tol : max horizontal perceived-vs-true error for a feasible grasp
#                 = (gripper opening 55 mm - object width)/2.
#   bias        : sim-only mesh-bias correction (0 for the current clean models).
#   minor_axis  : world direction the jaws should close ACROSS (None = symmetric).
#   grasp_width : object width across the minor axis (how far to close the jaws).
#   place       : this object's own spot on the drop plate.
#   half_h      : object half-height -> resting z = 0.75 + half_h.
#   place_settle: set the object down (vs free drop) -- the 52 mm coke fills the
#                 55 mm jaw, so a free release springs it off.
# ---------------------------------------------------------------------------
OBJECTS: Dict[str, dict] = {
    "coke_can":  dict(model="coke_can",  nominal=(0.14, 0.11), grasp_z=0.755,
                      capture_tol=0.0015, bias=(0.0, 0.0), minor_axis=(1.0, 0.0),
                      grasp_width=0.052, place=(0.27, 0.07), half_h=0.026,
                      place_settle=True),  # (55-52)/2, at the jaw size limit
    "banana":    dict(model="banana",    nominal=(0.14, -0.11), grasp_z=0.755,
                      capture_tol=0.010, bias=(0.0, 0.0), minor_axis=(0.0, 1.0),
                      grasp_width=0.035, place=(0.27, -0.07), half_h=0.015),  # (55-35)/2
    "golf_ball": dict(model="golf_ball", nominal=(-0.14, 0.11), grasp_z=0.755,
                      capture_tol=0.006, bias=(0.0, 0.0), minor_axis=None,
                      grasp_width=0.043, place=(0.19, 0.05), half_h=0.0214),  # (55-43)/2
    "blue_box":  dict(model="blue_box",  nominal=(-0.14, -0.11), grasp_z=0.755,
                      capture_tol=0.0075, bias=(0.0, 0.0), minor_axis=(0.0, 1.0),
                      grasp_width=0.040, place=(0.19, -0.05), half_h=0.015),  # (55-40)/2
}

# ---------------------------------------------------------------------------
# Gripper jaw model. Fingers rotate about Y: gap(lj) = R*cos(lj + delta), with
# (PX, PZ) the inner-face fingertip in the gripper frame (measured from meshes).
# ---------------------------------------------------------------------------
GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
GRIPPER_PX, GRIPPER_PZ = 0.0015, 0.059
JAW_OPEN = -0.459              # left-finger angle for 55 mm open (real gripper)
GRIP_SQUEEZE = 0.006          # close this much tighter than the object width
GRIP_OPEN = [JAW_OPEN, -JAW_OPEN]


def _finger_angle_for_width(width: float) -> float:
    """Left-finger joint angle whose jaw gap equals ``width`` m (right = -this)."""
    R = 2.0 * math.hypot(GRIPPER_PX, GRIPPER_PZ)
    delta = math.atan2(GRIPPER_PZ, GRIPPER_PX)
    return -delta + math.acos(max(-1.0, min(1.0, width / R)))


def grip_positions(width: float) -> List[float]:
    """Gripper [left, right] joint positions that close to a jaw gap of ``width``."""
    lj = _finger_angle_for_width(width)
    return [lj, -lj]


HOME_CFG = {n: 0.0 for n in ik.ARM_JOINT_ORDER}   # arm straight up, clear
PRE_DIST = 0.08      # pre-grasp / retreat offset back along the approach axis
PLATE_XY = (0.23, 0.0)
PLACE_Z = 0.79


# ---------------------------------------------------------------------------
# Pick-and-place geometry (shared by the sim and real executors)
# ---------------------------------------------------------------------------
def approach_dir(xy, tilt: float) -> np.ndarray:
    """Unit approach direction toward ``xy`` (world XY), tilted ``tilt`` below
    horizontal. Used to offset the pre-grasp/retreat point back along the line."""
    xy = np.asarray(xy, float)
    r = float(np.linalg.norm(xy))
    if r < 1e-6:
        return np.array([0.0, 0.0, -1.0])
    u = xy / r
    return np.array([u[0] * math.cos(tilt), u[1] * math.cos(tilt), -math.sin(tilt)])


def ik_waypoint(xyz, tilt: float, seed: Optional[Dict[str, float]] = None,
                min_clearance: float = 0.75) -> Optional[Dict[str, float]]:
    """IK for a single waypoint: try the given tilt, else fall back to the
    steepest feasible approach. Mirrors grasp_executor._ik_waypoint."""
    xyz = tuple(float(v) for v in xyz)
    cfg = ik.inverse_kinematics(xyz, tilt, min_clearance=min_clearance, seed=seed)
    if cfg is not None:
        return cfg
    res = ik.feasible_approach(xyz, min_clearance=min_clearance, seed=seed)
    return res[1] if res else None


def plan_pick_place(label: str, grasp_xy, tilt: float,
                    grasp_z: Optional[float] = None,
                    minor_axis=None,
                    ) -> Optional[Dict[str, Dict[str, float]]]:
    """Build the ordered pick-and-place waypoint configs for ``label`` with the
    fingertip grasp at world ``grasp_xy`` (2-vec) and approach ``tilt``.

    Faithful extraction of grasp_executor.run_trial's waypoint block: solve the
    grasp pose first, then SEED the pre/place waypoints from it so the whole
    motion stays in ONE IK branch (no mid-carry arm flip), and rotate the wrist
    roll J2 so the jaws close ACROSS the object's minor axis. Returns
    ``{"pre", "grasp", "place_pre", "place"}`` configs, or ``None`` if any
    waypoint is unreachable.

    ``grasp_z`` (real bench only): override the fingertip pick height. The sim
    leaves it ``None`` and uses the per-object ``grasp_z`` (~0.755 over a 0.75 m
    table) for the pick and ``PLACE_Z`` for the drop. On the real bench the table
    sits higher and the closing fingers swing down, so the real executor passes a
    single measured height (~0.82 m) used for BOTH pick and place -- matching the
    validated ``phase4c_pick_place_demo``.

    ``minor_axis`` (real bench only): override the world/ARM-frame direction the
    jaws close across. The per-object ``minor_axis`` is in the TABLE frame, which
    equals the arm frame only in sim (R = I); the real executor passes the nominal
    axis already rotated into the arm frame by the arm-base<->table extrinsic R
    (the yaw half of the supervisor's pose transform). ``None`` keeps the default.
    """
    d = OBJECTS[label]
    gz = float(grasp_z) if grasp_z is not None else d["grasp_z"]
    place_z = float(grasp_z) if grasp_z is not None else PLACE_Z
    grasp = np.array([float(grasp_xy[0]), float(grasp_xy[1]), gz])
    pre = grasp - approach_dir(grasp[:2], tilt) * PRE_DIST
    place_xy = np.array(d["place"])
    place = np.array([place_xy[0], place_xy[1], place_z])
    place_pre = place - approach_dir(place_xy, tilt) * PRE_DIST

    cfg_grasp = ik_waypoint(grasp, tilt)
    cfg_pre = ik_waypoint(pre, tilt, seed=cfg_grasp)
    cfg_place_pre = ik_waypoint(place_pre, tilt, seed=cfg_grasp)
    cfg_place = ik_waypoint(place, tilt, seed=cfg_place_pre or cfg_grasp)
    if None in (cfg_pre, cfg_grasp, cfg_place_pre, cfg_place):
        return None

    minor = minor_axis if minor_axis is not None else d.get("minor_axis")
    if minor is not None:
        j2 = ik.wrist_roll_for_axis(cfg_grasp, minor)
        for c in (cfg_pre, cfg_grasp, cfg_place_pre, cfg_place):
            c["xarm_2_joint"] = j2
    return {"pre": cfg_pre, "grasp": cfg_grasp,
            "place_pre": cfg_place_pre, "place": cfg_place}
